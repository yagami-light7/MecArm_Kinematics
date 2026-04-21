'''
本文件用于验证 TCP 6D 位姿闭环控制律。

控制对象：
    TCP 位姿 = [p_tcp, R_tcp]

控制流程：
    1.q向量（关节电机反馈）
    2.FK得到当前EE位姿  几何法Jacobian得到当前EE雅可比
    3.根据TCP偏置得到TCP位姿 和 TCP的6d雅可比
    4.根据笛卡尔空间误差 得到期望twist向量
    5.基于DLS阻尼最小二乘 计算qdot向量
    6.欧拉积分更新q

'''

from pathlib import Path
import sys
import numpy as np
import pinocchio as pin

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from robot_loader import load_robot, build_q, EE_FRAME_NAME
from fk_jacobian import forward_kinematics, frame_jacobian
from tcp_frame import TCP_FRAME_NAME, extract_tcp_pose, tcp_jacobian_from_ee_jacobian
from pose_error import pos_error_6d
from task6d_controller import step_pose_6d_control
from task6d_controller import task_twist_from_pose_error, solve_dls_6d, clip_qdot

from datetime import datetime
import csv
import matplotlib.pyplot as plt


# 初始关节角
INITIAL_THETA = np.array([0.0, -0.0, 0.0, 0.0, 1.57, 0.0], dtype=float)

# TCP 目标位置增量，基坐标系表达，单位 m
TARGET_POSITION_OFFSET = np.array([0.3, -0.15, 0.15], dtype=float)

# TCP 目标姿态，世界坐标系表达，单位 rad
TARGET_RPY_WORLD_ABS = np.array([0.1,-0.1,0], dtype=float)

DT = 0.001              # 控制步长
KP_POS = 10.0           # 位置控制增益
KP_ROT = 10.0           # 姿态控制增益
DAMPING = 0.05          # 阻尼系数
QDOT_LIMIT = 2.5        # 关节速度限制
MOVE_TIME = 1.5         # 直线轨迹运动时间，单位 s
SETTLE_TIME = 0.5       # 到达终点后的收敛保持时间，单位 s
MAX_STEPS = int((MOVE_TIME + SETTLE_TIME) / DT) # 最大步数
POS_ERROR_TOL = 1e-4    # 位置误差容限
ROT_ERROR_TOL = 1e-3    # 姿态误差容限


# 关节空间限幅
def clamp_q_to_limits(model, q):
    lower = np.asarray(model.lowerPositionLimit[:model.nq], dtype=float)
    upper = np.asarray(model.upperPositionLimit[:model.nq], dtype=float)

    lower = np.where(np.isfinite(lower), lower, -np.inf)
    upper = np.where(np.isfinite(upper), upper, np.inf)

    return np.clip(q, lower, upper)


# 计算TCP位姿和雅可比矩阵
def compute_tcp_state(model, data, frame_id, q):
    # 计算末端位姿和雅可比矩阵
    M_ee, p_ee, R_ee = forward_kinematics(model, data, frame_id, q)
    J_ee = frame_jacobian(model, data, frame_id, q)

    # 通过末端位姿和雅可比 转化为TCP位姿和雅可比
    M_tcp, p_tcp, R_tcp = extract_tcp_pose(M_ee)
    J_tcp = tcp_jacobian_from_ee_jacobian(J_ee, p_ee, p_tcp)

    return p_tcp, R_tcp, J_tcp

# 五次项插值优化笛卡尔空间轨迹
def min_jerk_time_scaling(t, total_time):
    tau = np.clip(t / total_time, 0.0, 1.0)

    s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    s_dot = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / total_time

    return s, s_dot

# 计算五次项插值后的笛卡尔空间运动轨迹
def cartesian_pose_reference(t, p_start, p_goal, R_start, R_goal,total_time):
    p_start = np.asarray(p_start, dtype=float).reshape(3)
    p_goal = np.asarray(p_goal, dtype=float).reshape(3)
    R_start = np.asarray(R_start, dtype=float).reshape(3, 3)
    R_goal = np.asarray(R_goal, dtype=float).reshape(3, 3)

    s, s_dot = min_jerk_time_scaling(t, total_time)

    dp = p_goal - p_start
    
    # 生成笛卡尔位置轨迹
    p_ref = p_start + dp * s
    v_ff = s_dot * dp   # 笛卡尔空间参考轨迹线速度 即 前馈线速度

    # 生成笛卡尔姿态轨迹
    R_delta = R_start.T @ R_goal    # 计算相对旋转矩阵

    rotvec_local_total = pin.log3(R_delta) # 使用log3计算SO3旋转量
    
    R_ref = R_start @ pin.exp3(s * rotvec_local_total)# 指数映射生成基坐标系下的参考姿态

    w_ff_local = s_dot * rotvec_local_total# 前馈角速度：先得到局部角速度，再转到基坐标系下表达
    w_ff = R_ref @ w_ff_local   # 笛卡尔空间参考轨迹角速度 即 前馈角速度

    return p_ref, R_ref, v_ff, w_ff


def create_result_dir():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_dir = PROJECT_ROOT / "results" / "tcp_pose_6d_demo" / timestamp
    result_dir.mkdir(parents=True, exist_ok=True)
    return result_dir


def save_trajectory_csv(result_dir, records):
    csv_path = result_dir / "trajectory.csv"

    fieldnames = [
        "step", "time",
        "pos_err_norm", "rot_err_norm", "err_6d_norm", "qdot_max",
        "px", "py", "pz",
        "roll", "pitch", "yaw",
        "r11", "r12", "r13",
        "r21", "r22", "r23",
        "r31", "r32", "r33",
        "epx", "epy", "epz", "erx", "ery", "erz",
        "q1", "q2", "q3", "q4", "q5", "q6",
        "qdot1", "qdot2", "qdot3", "qdot4", "qdot5", "qdot6",
    ]

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)

    return csv_path


def save_pose_error_svg(result_dir, records):
    steps = np.array([row["step"] for row in records], dtype=float)
    pos_err_mm = np.array([row["pos_err_norm"] for row in records], dtype=float) * 1000.0
    rot_err_deg = np.array([row["rot_err_norm"] for row in records], dtype=float) * 180.0 / np.pi
    qdot_max = np.array([row["qdot_max"] for row in records], dtype=float)

    plt.rcParams.update({
        "font.family": "Times New Roman",
        "font.weight": "bold",
        "axes.labelweight": "bold",
        "axes.titleweight": "bold",
        "axes.unicode_minus": False,
    })

    fig, axes = plt.subplots(3, 1, figsize=(8.5, 8.0), sharex=True)

    colors = {
        "pos": "#1F4E79",
        "rot": "#C55A11",
        "qdot": "#548235",
        "grid": "#BFBFBF",
    }

    axes[0].plot(steps, pos_err_mm, color=colors["pos"], linewidth=2.2)
    axes[0].set_ylabel("Position Error [mm]")
    axes[0].grid(True, linestyle="--", linewidth=0.7, alpha=0.45, color=colors["grid"])

    axes[1].plot(steps, rot_err_deg, color=colors["rot"], linewidth=2.2)
    axes[1].set_ylabel("Rotation Error [deg]")
    axes[1].grid(True, linestyle="--", linewidth=0.7, alpha=0.45, color=colors["grid"])

    axes[2].plot(steps, qdot_max, color=colors["qdot"], linewidth=2.2)
    axes[2].set_xlabel("Step")
    axes[2].set_ylabel("Max |qdot| [rad/s]")
    axes[2].grid(True, linestyle="--", linewidth=0.7, alpha=0.45, color=colors["grid"])

    fig.suptitle("TCP 6D Pose Control Error", fontsize=15, fontweight="bold")
    fig.tight_layout()

    svg_path = result_dir / "03_pose_error.svg"
    png_path = result_dir / "03_pose_error.png"

    fig.savefig(svg_path, format="svg", bbox_inches="tight")
    fig.savefig(png_path, format="png", dpi=450, bbox_inches="tight")
    plt.close(fig)

    return svg_path


def save_joint_position_svg(result_dir, records):
    time = np.array([row["time"] for row in records], dtype=float)
    q_mat = np.array(
        [[row[f"q{i}"] for i in range(1, 7)] for row in records],
        dtype=float,
    )

    plt.rcParams.update({
        "font.family": "Times New Roman",
        "font.weight": "bold",
        "axes.labelweight": "bold",
        "axes.titleweight": "bold",
        "axes.unicode_minus": False,
    })

    colors = ["#0B1F3A", "#1F77B4", "#2CA02C", "#D62728", "#9467BD", "#8C564B"]
    fig, axes = plt.subplots(6, 1, figsize=(9.0, 10.5), sharex=True, facecolor="white")

    for i in range(6):
        ax = axes[i]

        ax.plot(
            time,
            q_mat[:, i],
            linewidth=2.3,
            color=colors[i],
            label=f"q{i + 1}",
        )

        ax.set_ylabel(f"q{i + 1} [rad]")
        ax.grid(True, linestyle="--", linewidth=0.7, alpha=0.45, color="#BFBFBF")
        ax.legend(
            loc="upper right",
            frameon=True,
            facecolor="white",
            edgecolor="#D3D8DE",
            fontsize=9,
        )

    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("Joint Position History", fontsize=15, fontweight="bold")
    fig.tight_layout()

    svg_path = result_dir / "01_joint_position.svg"
    png_path = result_dir / "01_joint_position.png"
    fig.savefig(svg_path, format="svg", bbox_inches="tight")
    fig.savefig(png_path, format="png", dpi=450, bbox_inches="tight")
    plt.close(fig)

    return svg_path



def save_tcp_trajectory_3d_svg(result_dir, records, p_start, p_des, p_final):
    from matplotlib.ticker import MaxNLocator, FormatStrFormatter

    plt.rcParams.update({
        "font.family": "Times New Roman",
        "font.weight": "bold",
        "axes.labelweight": "bold",
        "axes.titleweight": "bold",
        "mathtext.default": "regular",
        "axes.unicode_minus": False,
    })

    traj = np.array([[row["px"], row["py"], row["pz"]] for row in records], dtype=float)
    p_start = np.asarray(p_start, dtype=float).reshape(3)
    p_des = np.asarray(p_des, dtype=float).reshape(3)
    p_final = np.asarray(p_final, dtype=float).reshape(3)

    all_pts = np.vstack([traj, p_start, p_des, p_final])
    mins = all_pts.min(axis=0)
    maxs = all_pts.max(axis=0)
    spans = np.maximum(maxs - mins, 1e-9)

    pad = np.maximum(0.10 * spans, np.array([0.01, 0.002, 0.01]))
    xlim = (mins[0] - pad[0], maxs[0] + pad[0])
    ylim = (mins[1] - pad[1], maxs[1] + pad[1])
    zlim = (mins[2] - pad[2], maxs[2] + pad[2])

    fig = plt.figure(figsize=(9.5, 7.2), facecolor="white")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("white")

    try:
        ax.set_proj_type("ortho")
    except Exception:
        pass

    c_traj = "#0B1F3A"
    c_ref = "#7A869A"
    c_start = "#008B8B"
    c_final = "#B8860B"
    c_target = "#B22222"

    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_zlim(*zlim)

    vis_spans = np.array([xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0]], dtype=float)
    vis_spans = vis_spans / vis_spans.max()
    vis_spans = np.clip(vis_spans, 0.28, None)
    ax.set_box_aspect(vis_spans)

    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2],
            color=c_traj, linewidth=2.5, label="TCP trajectory")

    ax.plot([p_start[0], p_des[0]],
            [p_start[1], p_des[1]],
            [p_start[2], p_des[2]],
            color=c_ref, linewidth=1.8, linestyle=(0, (6, 3)),
            label="Reference line")

    ax.scatter([p_start[0]], [p_start[1]], [p_start[2]],
            s=58, color=c_start, edgecolors="white", linewidths=0.8, label="Start")
    ax.scatter([p_final[0]], [p_final[1]], [p_final[2]],
            s=58, color=c_final, edgecolors="white", linewidths=0.8, label="Final")
    ax.scatter([p_des[0]], [p_des[1]], [p_des[2]],
            s=110, marker="*", color=c_target, edgecolors="white", linewidths=0.8, label="Target")

    # 每隔若干步画一次 TCP 坐标轴三叉。
    # 由于当前轨迹的 y 轴范围很小，直接用同一个数据长度会让 y 轴视觉上过长；
    # 这里按 3D box 的显示尺度修正每根轴的长度，使三根轴在图中近似等长。
    sample_count = 9
    sample_indices = np.linspace(0, len(records) - 1, sample_count, dtype=int)
    axis_ranges = np.array([xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0]], dtype=float)
    visual_scale = vis_spans / axis_ranges
    visual_axis_len = 0.045

    def equal_visual_axis_vec(direction):
        direction = np.asarray(direction, dtype=float).reshape(3)
        direction = direction / np.linalg.norm(direction)
        data_len = visual_axis_len / np.linalg.norm(visual_scale * direction)
        return data_len * direction

    for k, idx in enumerate(sample_indices):
        row = records[idx]
        p = np.array([row["px"], row["py"], row["pz"]], dtype=float)

        R = np.array([
            [row["r11"], row["r12"], row["r13"]],
            [row["r21"], row["r22"], row["r23"]],
            [row["r31"], row["r32"], row["r33"]],
        ], dtype=float)

        labels = ("TCP x-axis", "TCP y-axis", "TCP z-axis") if k == 0 else (None, None, None)
        axis_vecs = [equal_visual_axis_vec(R[:, axis_idx]) for axis_idx in range(3)]

        ax.quiver(p[0], p[1], p[2],
                axis_vecs[0][0], axis_vecs[0][1], axis_vecs[0][2],
                color="#D62728", linewidth=1.0, arrow_length_ratio=0.20, label=labels[0])

        ax.quiver(p[0], p[1], p[2],
                axis_vecs[1][0], axis_vecs[1][1], axis_vecs[1][2],
                color="#2CA02C", linewidth=1.0, arrow_length_ratio=0.20, label=labels[1])

        ax.quiver(p[0], p[1], p[2],
                axis_vecs[2][0], axis_vecs[2][1], axis_vecs[2][2],
                color="#1F77B4", linewidth=1.0, arrow_length_ratio=0.20, label=labels[2])

    ax.view_init(elev=22, azim=-48)

    ax.xaxis.set_major_locator(MaxNLocator(5))
    ax.yaxis.set_major_locator(MaxNLocator(3))
    ax.zaxis.set_major_locator(MaxNLocator(5))
    ax.xaxis.set_major_formatter(FormatStrFormatter("%.3f"))
    ax.yaxis.set_major_formatter(FormatStrFormatter("%.4f"))
    ax.zaxis.set_major_formatter(FormatStrFormatter("%.3f"))

    ax.set_title("TCP 3D Trajectory With Orientation Frames", fontsize=15, pad=16)
    ax.set_xlabel("X [m]", fontsize=11, labelpad=10)
    ax.set_ylabel("Y [m]", fontsize=11, labelpad=8)
    ax.set_zlabel("Z [m]", fontsize=11, labelpad=8)

    for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
        axis.pane.fill = False
        axis.pane.set_edgecolor("#CDD3DA")
        axis._axinfo["grid"]["color"] = (0.72, 0.76, 0.80, 0.60)
        axis._axinfo["grid"]["linestyle"] = "--"
        axis._axinfo["grid"]["linewidth"] = 0.6

    ax.legend(
        loc="upper center",
        bbox_to_anchor=(0.5, 1.04),
        ncol=4,
        frameon=True,
        facecolor="white",
        edgecolor="#D3D8DE",
        fontsize=8.5,
    )

    fig.tight_layout()

    svg_path = result_dir / "02_tcp_trajectory_3d.svg"
    png_path = result_dir / "02_tcp_trajectory_3d.png"
    fig.savefig(svg_path, format="svg", bbox_inches="tight")
    fig.savefig(png_path, format="png", dpi=450, bbox_inches="tight")
    plt.close(fig)

    return svg_path




def main():
    np.set_printoptions(precision=6, suppress=True)
    result_dir = create_result_dir()
    records = []


    model, data, frame_id = load_robot()
    q = build_q(model, INITIAL_THETA)

    # 计算TCP位姿
    p_start, R_start, J_tcp = compute_tcp_state(model, data, frame_id, q)

    # 计算目标位姿
    p_des = p_start + TARGET_POSITION_OFFSET
    R_des = pin.rpy.rpyToMatrix(TARGET_RPY_WORLD_ABS)

    print("EE_FRAME_NAME =", EE_FRAME_NAME)
    print("TCP_FRAME_NAME =", TCP_FRAME_NAME)
    print("initial q =")
    print(q)
    print("p_start =")
    print(p_start)
    print("p_des =")
    print(p_des)
    print("start rpy world =")
    print(pin.rpy.matrixToRpy(R_start))
    print("target rpy world =")
    print(TARGET_RPY_WORLD_ABS)

    print("TARGET_POSITION_OFFSET =", TARGET_POSITION_OFFSET)
    print("TARGET_RPY_WORLD_ABS =", TARGET_RPY_WORLD_ABS)
    print("DT =", DT)
    print("KP_POS =", KP_POS)
    print("KP_ROT =", KP_ROT)
    print("DAMPING =", DAMPING)
    print("QDOT_LIMIT =", QDOT_LIMIT) 

    for step in range(MAX_STEPS):
        # UpdateFeedback 计算TCP位姿和雅可比
        p_cur, R_cur, J_tcp = compute_tcp_state(model, data, frame_id, q)

        # SetControl：设置控制量 计算误差 得到期望twist DLS得到期望qdot 欧拉积分得到q_next
        # q_next, qdot_target, e_6d, twist_target = step_pose_6d_control(
        #     q=q,
        #     p_des=p_des,
        #     R_des=R_des,
        #     p_cur=p_cur,
        #     R_cur=R_cur,
        #     J_tcp=J_tcp,
        #     dt=DT,
        #     kp_pos=KP_POS,
        #     kp_rot=KP_ROT,
        #     damping=DAMPING,
        #     qdot_limit=QDOT_LIMIT,
        # ) 

        '''优化为线性插值后轨迹生成 + 速度前馈 '''
        t = step * DT
        # 生成笛卡尔轨迹（包括参考位置和前馈速度）
        p_ref, R_ref, v_ff, w_ff = cartesian_pose_reference(t, p_start, p_des, R_start, R_des, MOVE_TIME)

        # 当前阶段先保持目标姿态不变，所以角速度前馈为 0
        R_ref = R_des
        w_ff = np.zeros(3, dtype=float)

        # 计算误差
        e_6d = pos_error_6d(p_ref, p_cur, R_ref, R_cur)

        # 计算后馈twist+前馈twist 生成期望twist
        twist_fb = task_twist_from_pose_error(e_6d, KP_POS, KP_ROT)
        twist_ff = np.hstack([v_ff, w_ff])
        twist_target = twist_ff + twist_fb

        # DLS求解期望qdot
        qdot_target = solve_dls_6d(J_tcp, twist_target, damping=DAMPING)

        # 关节速度限幅
        qdot_target = clip_qdot(qdot_target, QDOT_LIMIT)

        # 欧拉积分
        q_next = q + qdot_target * DT

        # 关节限幅
        # # SendCommand 控制量下发 完成闭环
        q = clamp_q_to_limits(model, q_next)

        # 分别计算位置误差和姿态误差范数
        pos_err_norm = np.linalg.norm(e_6d[0:3])
        rot_err_norm = np.linalg.norm(e_6d[3:6])
        err_6d_norm = np.linalg.norm(e_6d)
        qdot_abs_max = np.max(np.abs(qdot_target))

        rpy_cur = pin.rpy.matrixToRpy(R_cur)

        records.append({
            "step": step,
            "time": step * DT,
            "pos_err_norm": float(pos_err_norm),
            "rot_err_norm": float(rot_err_norm),
            "err_6d_norm": float(err_6d_norm),
            "qdot_max": float(qdot_abs_max),

            "px": float(p_cur[0]),
            "py": float(p_cur[1]),
            "pz": float(p_cur[2]),

            "roll": float(rpy_cur[0]),
            "pitch": float(rpy_cur[1]),
            "yaw": float(rpy_cur[2]),

            "r11": float(R_cur[0, 0]),
            "r12": float(R_cur[0, 1]),
            "r13": float(R_cur[0, 2]),
            "r21": float(R_cur[1, 0]),
            "r22": float(R_cur[1, 1]),
            "r23": float(R_cur[1, 2]),
            "r31": float(R_cur[2, 0]),
            "r32": float(R_cur[2, 1]),
            "r33": float(R_cur[2, 2]),

            "epx": float(e_6d[0]),
            "epy": float(e_6d[1]),
            "epz": float(e_6d[2]),
            "erx": float(e_6d[3]),
            "ery": float(e_6d[4]),
            "erz": float(e_6d[5]),

            "q1": float(q[0]),
            "q2": float(q[1]),
            "q3": float(q[2]),
            "q4": float(q[3]),
            "q5": float(q[4]),
            "q6": float(q[5]),

            "qdot1": float(qdot_target[0]),
            "qdot2": float(qdot_target[1]),
            "qdot3": float(qdot_target[2]),
            "qdot4": float(qdot_target[3]),
            "qdot5": float(qdot_target[4]),
            "qdot6": float(qdot_target[5]),
        })

        if step % 20 == 0 or step == MAX_STEPS - 1:
            print(
                f"step={step:04d} "
                f"pos_err={pos_err_norm:.6e} "
                f"rot_err={rot_err_norm:.6e} "
                f"err_6d={err_6d_norm:.6e} "
                f"qdot_max={qdot_abs_max:.6e} "
                f"p_cur={p_cur}"
            )

        # 退出判断
        if t >= MOVE_TIME and pos_err_norm < POS_ERROR_TOL and rot_err_norm < ROT_ERROR_TOL:
            print("TCP 6D pose control converged.")
            break

    # 迭代结束后重新计算最终 TCP 位姿
    p_final, R_final, _ = compute_tcp_state(model, data, frame_id, q)
    e_final = pos_error_6d(p_des, p_final, R_des, R_final)

    print("\n===== 迭代结果 =====")
    print("final q =")
    print(q)
    print("p_final =")
    print(p_final)
    print("p_des =")
    print(p_des)
    print("final position error =")
    print(e_final[0:3])
    print("final orientation error =")
    print(e_final[3:6])
    print("final pos err norm =", np.linalg.norm(e_final[0:3]))
    print("final rot err norm =", np.linalg.norm(e_final[3:6]))

    trajectory_csv_path = save_trajectory_csv(result_dir, records)
    joint_position_svg_path = save_joint_position_svg(result_dir, records)
    tcp_trajectory_3d_svg_path = save_tcp_trajectory_3d_svg(
        result_dir,
        records,
        p_start,
        p_des,
        p_final,
    )
    pose_error_svg_path = save_pose_error_svg(result_dir, records)

    print("result_dir =", result_dir)
    print("trajectory_csv_path =", trajectory_csv_path)
    print("joint_position_svg_path =", joint_position_svg_path)
    print("tcp_trajectory_3d_svg_path =", tcp_trajectory_3d_svg_path)
    print("pose_error_svg_path =", pose_error_svg_path)





if __name__ == "__main__":
    main()
