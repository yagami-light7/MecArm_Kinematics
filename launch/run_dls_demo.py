'''

本文件通过调用Pinocchio库实现FK和Jacobian，基于DLS阻尼最小二乘，实现笛卡尔空间的闭环控制
运行后自动生成机械臂轨迹和误差

后续迁移至MCU部署时 将FK和Jaccobian运算替换为手动矩阵FK+几何法Jacobian(已成功部署实现)

'''

from pathlib import Path
import sys
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from robot_loader import load_robot, build_q, EE_FRAME_NAME
from fk_jacobian import (
    forward_kinematics,
    frame_jacobian,
    numeric_position_jacobian,
    extract_position_jacobian,
)
from dls import step_position_dls
from datetime import datetime
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


# DLS起始姿态
INITIAL_THETA = np.array([0, 0, 0, 0, 0, 0], dtype=float)
# 目标笛卡尔空间位置增量
TARGET_OFFSET = np.array([0.5, 0.05, 0.2], dtype=float)

DT = 0.001   # 离散积分步长
KP = 10.0    # 比例增益
DAMPING = 0.05  # DLS阻尼因子
QDOT_LIMIT = 2.5    # 关节速度限幅
MAX_STEPS = 1000     # 最多迭代步数
ERROR_TOL = 1e-4    # 误差截断值


# 读取关节限位并对关节目标位置进行限幅
def clamp_q_to_limits(model, q):
    lower = np.asarray(model.lowerPositionLimit[:model.nq], dtype=float)
    upper = np.asarray(model.upperPositionLimit[:model.nq], dtype=float)

    lower = np.where(np.isfinite(lower), lower, -np.inf)
    upper = np.where(np.isfinite(upper), upper, np.inf)

    return np.clip(q, lower, upper)


# 创建离线验证文件夹
def create_result_dir():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_dir = PROJECT_ROOT / "results" / "dls_demo" / timestamp
    result_dir.mkdir(parents=True, exist_ok=True)
    return result_dir


# 保存DLS生成轨迹
def save_trajectory_csv(result_dir, records):
    csv_path = result_dir / "trajectory.csv"
    fieldnames = [
        "step",
        "error_norm",
        "px", "py", "pz",
        "q1", "q2", "q3", "q4", "q5", "q6",
        "qdot1", "qdot2", "qdot3", "qdot4", "qdot5", "qdot6",
    ]

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)

    return csv_path


# 保存轨迹误差曲线
def save_error_norm_svg(result_dir, records):
    steps = [row["step"] for row in records]
    error_norms = [row["error_norm"] for row in records]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(steps, error_norms, color="#1565C0", linewidth=2.0)

    ax.set_title("DLS Position Error Norm")
    ax.set_xlabel("Step")
    ax.set_ylabel("||e|| [m]")
    ax.grid(True, linestyle="--", alpha=0.4)

    fig.tight_layout()

    svg_path = result_dir / "error_norm.svg"
    fig.savefig(svg_path, format="svg")
    plt.close(fig)

    return svg_path


# 保存3D轨迹曲线
def save_trajectory_3d_svg(result_dir, records, p_start, p_des, p_final):
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator, FormatStrFormatter

    plt.rcParams.update({
        "font.family": "Times New Roman",
        "mathtext.default": "regular",
        "axes.unicode_minus": False,
    })

    p_start = np.asarray(p_start, dtype=float).reshape(3)
    p_des   = np.asarray(p_des,   dtype=float).reshape(3)
    p_final = np.asarray(p_final, dtype=float).reshape(3)

    traj = np.array([[row["px"], row["py"], row["pz"]] for row in records], dtype=float)
    x, y, z = traj[:, 0], traj[:, 1], traj[:, 2]

    all_pts = np.vstack([traj, p_start, p_des, p_final])
    mins = all_pts.min(axis=0)
    maxs = all_pts.max(axis=0)
    spans = np.maximum(maxs - mins, 1e-9)

    # 自适应边界
    pad = np.maximum(0.12 * spans, np.array([0.0012, 0.0003, 0.0012]))
    xlim = (mins[0] - pad[0], maxs[0] + pad[0])
    ylim = (mins[1] - pad[1], maxs[1] + pad[1])
    zlim = (mins[2] - pad[2], maxs[2] + pad[2])

    fig = plt.figure(figsize=(9.6, 7.2), constrained_layout=False, facecolor="white")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("white")

    # 尽量减小透视畸变
    try:
        ax.set_proj_type("ortho")
    except Exception:
        pass

    # 配色
    c_traj   = "#17324D"
    c_ref    = "#8A94A3"
    c_start  = "#1F9D8A"
    c_final  = "#C58B00"
    c_target = "#C23B55"

    # 主轨迹
    ax.plot(
        x, y, z,
        color=c_traj,
        linewidth=2.4,
        solid_capstyle="round",
        label="Actual trajectory",
        zorder=3,
    )

    # 参考线
    ax.plot(
        [p_start[0], p_des[0]],
        [p_start[1], p_des[1]],
        [p_start[2], p_des[2]],
        color=c_ref,
        linewidth=1.8,
        linestyle=(0, (6, 3)),
        label="Reference line",
        zorder=2,
    )

    # 关键点
    ax.scatter(
        [p_start[0]], [p_start[1]], [p_start[2]],
        s=52, color=c_start, edgecolors="white", linewidths=0.8,
        depthshade=False, label="Start", zorder=5
    )
    ax.scatter(
        [p_final[0]], [p_final[1]], [p_final[2]],
        s=52, color=c_final, edgecolors="white", linewidths=0.8,
        depthshade=False, label="Final", zorder=5
    )
    ax.scatter(
        [p_des[0]], [p_des[1]], [p_des[2]],
        s=95, marker="*", color=c_target, edgecolors="white", linewidths=0.8,
        depthshade=False, label="Target", zorder=6
    )

    # 坐标范围
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_zlim(*zlim)

    # 视角：比你现在那张更稳一点
    ax.view_init(elev=20, azim=-30)

    # 按显示范围自动设置 box_aspect
    vis_spans = np.array([
        xlim[1] - xlim[0],
        ylim[1] - ylim[0],
        zlim[1] - zlim[0],
    ], dtype=float)

    # 防止 y 太小导致画面极端压扁
    vis_spans = vis_spans / vis_spans.max()
    vis_spans = np.clip(vis_spans, 0.35, None)
    ax.set_box_aspect(vis_spans)

    # 刻度：减少数量，避免重叠
    ax.xaxis.set_major_locator(MaxNLocator(4))
    ax.yaxis.set_major_locator(MaxNLocator(3))
    ax.zaxis.set_major_locator(MaxNLocator(4))

    ax.xaxis.set_major_formatter(FormatStrFormatter("%.3f"))
    ax.yaxis.set_major_formatter(FormatStrFormatter("%.4f"))
    ax.zaxis.set_major_formatter(FormatStrFormatter("%.3f"))

    ax.tick_params(axis="x", which="major", pad=2, labelsize=9)
    ax.tick_params(axis="y", which="major", pad=2, labelsize=9)
    ax.tick_params(axis="z", which="major", pad=2, labelsize=9)

    # 标题和标签别太重
    ax.set_title(
        "Cartesian DLS Trajectory",
        fontsize=14,
        fontweight="semibold",
        pad=16,
    )
    ax.set_xlabel("X [m]", fontsize=11, fontweight="semibold", labelpad=10)
    ax.set_ylabel("Y [m]", fontsize=11, fontweight="semibold", labelpad=8)
    ax.set_zlabel("Z [m]", fontsize=11, fontweight="semibold", labelpad=8)

    # pane / grid
    for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
        axis.pane.fill = False
        axis.pane.set_edgecolor("#C8CED6")
        axis._axinfo["grid"]["color"] = (0.75, 0.78, 0.82, 0.60)
        axis._axinfo["grid"]["linestyle"] = "--"
        axis._axinfo["grid"]["linewidth"] = 0.6

    # 图例移到图外上方
    handles, labels = ax.get_legend_handles_labels()
    leg = fig.legend(
        handles, labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.98),
        ncol=4,
        frameon=True,
        facecolor="white",
        edgecolor="#D3D8DE",
        framealpha=0.97,
        fontsize=10,
        handlelength=2.2,
        columnspacing=1.4,
        borderpad=0.5,
    )
    leg.get_frame().set_linewidth(0.8)

    # 给上方图例留空间
    fig.subplots_adjust(top=0.84, left=0.04, right=0.98, bottom=0.06)

    svg_path = result_dir / "trajectory_3d.svg"
    png_path = result_dir / "trajectory_3d.png"

    fig.savefig(svg_path, format="svg", bbox_inches="tight")
    fig.savefig(png_path, format="png", dpi=450, bbox_inches="tight")
    plt.close(fig)

    return svg_path


def main():
    
    result_dir = create_result_dir()
    records = []

    np.set_printoptions(precision=6, suppress=True)

    model, data, frame_id = load_robot()
    q = build_q(model, INITIAL_THETA)

    _, p_start, _ = forward_kinematics(model, data, frame_id, q)
    p_des = p_start + TARGET_OFFSET

    print("EE_FRAME_NAME =", EE_FRAME_NAME)
    print("initial q =", q)
    print("start position =", p_start)
    print("desired position =", p_des)
    print("DT =", DT)
    print("KP =", KP)
    print("DAMPING =", DAMPING)
    print("QDOT_LIMIT =", QDOT_LIMIT)

    validated_block = None

    for step in range(MAX_STEPS):
        # FK计算当前位置向量
        _, p_cur, _ = forward_kinematics(model, data, frame_id, q)

        # 计算雅可比矩阵
        J6 = frame_jacobian(model, data, frame_id, q)

        # 提取线速度雅可比部分
        if validated_block is None:
            J_num = numeric_position_jacobian(model, data, frame_id, q)
            Jv, validated_block = extract_position_jacobian(J6, J_num)
            validation_err = np.linalg.norm(Jv - J_num)
            print("validated position Jacobian block =", validated_block)
            print("numeric validation error =", validation_err)
        elif validated_block == "top":
            Jv = J6[0:3, :]
        else:
            Jv = J6[3:6, :]
        
        # 利用DLS计算关节空间目标位置向量
        q_next, q_dot, err, v_des = step_position_dls(q, 
                                                    p_cur,
                                                    p_des,
                                                    Jv,
                                                    DT,
                                                    KP,
                                                    DAMPING,
                                                    QDOT_LIMIT
        )

        # 更新关节空间并进行关节限幅
        q = clamp_q_to_limits(model, q_next)

        # 计算误差范数
        error_norm = np.linalg.norm(err)

        # 保存信息
        records.append(
        {
            "step": step,
            "error_norm": float(error_norm),
            "px": float(p_cur[0]),
            "py": float(p_cur[1]),
            "pz": float(p_cur[2]),
            "q1": float(q[0]),
            "q2": float(q[1]),
            "q3": float(q[2]),
            "q4": float(q[3]),
            "q5": float(q[4]),
            "q6": float(q[5]),
            "qdot1": float(q_dot[0]),
            "qdot2": float(q_dot[1]),
            "qdot3": float(q_dot[2]),
            "qdot4": float(q_dot[3]),
            "qdot5": float(q_dot[4]),
            "qdot6": float(q_dot[5]),
        }
        )

        # 判断是否结束迭代
        if step % 10 == 0 or error_norm < ERROR_TOL or step == MAX_STEPS - 1:
            print(
                f"step={step:03d} error_norm={error_norm:.6e} "
                f"p_cur={p_cur} v_des={v_des}"
            )

        if error_norm < ERROR_TOL:
            print("DLS demo converged.")
            break
    
    # 迭代结束 打终点信息与误差
    _, p_final, _ = forward_kinematics(model, data, frame_id, q)

    print("final q =", q)
    print("final position =", p_final)
    print("final error =", p_des - p_final)

    # 保存轨迹
    trajectory_csv_path = save_trajectory_csv(result_dir, records)
    error_svg_path = save_error_norm_svg(result_dir, records)
    trajectory_3d_svg_path = save_trajectory_3d_svg(result_dir, records, p_start, p_des, p_final)


    print("result_dir =", result_dir)
    print("trajectory_csv_path =", trajectory_csv_path)
    print("error_svg_path =", error_svg_path)
    print("trajectory_3d_svg_path =", trajectory_3d_svg_path)



if __name__ == "__main__":

    main()
