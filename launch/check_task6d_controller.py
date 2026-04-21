'''
本文件用于验证 task6d_controller.py 的 6D 位姿控制律。

验证内容：
1. 当前 TCP 位姿获取是否正常
2. 零误差时 qdot 是否接近 0
3. 给定 6D 目标位姿后，是否能生成合理 twist_target
4. DLS 求解 qdot 后，单步积分是否让 TCP 误差下降

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
from task6d_controller import (
    expand_gain_3d,
    task_twist_from_pose_error,
    solve_dls_6d,
    step_pose_6d_control,
)

TEST_THETA = np.array([0.0, -0.2, 0.3, 0.0, 0.3, 0.2], dtype=float)

DT = 0.001
KP_POS = 10.0
KP_ROT = 4.0
DAMPING = 0.05
QDOT_LIMIT = 2.0

# 位姿增量
TARGET_DP = np.array([0.02, -0.01, 0.03], dtype=float)
TARGET_DRPY = np.array([0.0, 0.0, 0.15], dtype=float)


# 计算TCP位姿和雅可比矩阵
def compute_tcp_state(model, data, frame_id, q):
    # 计算末端位姿和雅可比矩阵
    M_ee, p_ee, R_ee = forward_kinematics(model, data, frame_id, q)
    J_ee = frame_jacobian(model, data, frame_id, q)

    # 通过末端位姿和雅可比 转化为TCP位姿和雅可比
    M_tcp, p_tcp, R_tcp = extract_tcp_pose(M_ee)
    J_tcp = tcp_jacobian_from_ee_jacobian(J_ee, p_ee, p_tcp)

    return p_tcp, R_tcp, J_tcp


def main():
    # 设置打印格式
    np.set_printoptions(precision=6, suppress=True)

    # 搭建机器人模型 计算q向量
    model, data, frame_id = load_robot()
    q = build_q(model,TEST_THETA)

    # 计算TCP位姿和雅可比矩阵
    p_cur, R_cur, J_tcp = compute_tcp_state(model, data, frame_id, q)

    print("EE_FRAME_NAME =", EE_FRAME_NAME)
    print("TCP_FRAME_NAME =", TCP_FRAME_NAME)
    print("q =")
    print(q)
    print("p_cur =")
    print(p_cur)
    print("R_cur =")
    print(R_cur)

    print("\n===== Case 1: 目标位姿=当前位姿的输出情况 =====")
    q_next_zero, qdot_zero, e_zero, twist_zero = step_pose_6d_control(
        q=q,
        p_des=p_cur,
        R_des=R_cur,
        p_cur=p_cur,
        R_cur=R_cur,
        J_tcp=J_tcp,
        dt=DT,
        kp_pos=KP_POS,
        kp_rot=KP_ROT,
        damping=DAMPING,
        qdot_limit=QDOT_LIMIT,
    )

    print("e_zero =")
    print(e_zero)
    print("twist_zero =")
    print(twist_zero)
    print("qdot_zero =")
    print(qdot_zero)
    print("q_next_zero - q =")
    print(q_next_zero - q)

    print("\n===== Case 2: 比较单步执行控制之后的误差是否缩小 =====")
    R_delta = pin.rpy.rpyToMatrix(TARGET_DRPY)

    p_des = p_cur + TARGET_DP
    R_des = R_cur @ R_delta

    e_before = pos_error_6d(p_des, p_cur, R_des, R_cur)
    err_before_norm = np.linalg.norm(e_before)

    q_next, qdot_target, e_6d, twist_target = step_pose_6d_control(
        q=q,
        p_des=p_des,
        R_des=R_des,
        p_cur=p_cur,
        R_cur=R_cur,
        J_tcp=J_tcp,
        dt=DT,
        kp_pos=KP_POS,
        kp_rot=KP_ROT,
        damping=DAMPING,
        qdot_limit=QDOT_LIMIT,
    )

    p_next, R_next, _ = compute_tcp_state(model, data, frame_id, q_next)
    e_after = pos_error_6d(p_des, p_next, R_des, R_next)
    err_after_norm = np.linalg.norm(e_after)

    twist_realized = J_tcp @ qdot_target
    twist_residual = twist_target - twist_realized

    print("p_des =")
    print(p_des)
    print("TARGET_DP =")
    print(TARGET_DP)
    print("TARGET_DRPY =")
    print(TARGET_DRPY)

    print("e_before =")
    print(e_before)
    print("twist_target =")
    print(twist_target)
    print("qdot_target =")
    print(qdot_target)
    print("max abs qdot =", np.max(np.abs(qdot_target)))

    print("twist_realized =")
    print(twist_realized)
    print("twist_residual =")
    print(twist_residual)
    print("twist residual norm =", np.linalg.norm(twist_residual))

    print("q_next =")
    print(q_next)
    print("p_next =")
    print(p_next)
    print("e_after =")
    print(e_after)

    print("err_before_norm =", err_before_norm)
    print("err_after_norm  =", err_after_norm)
    print("error decreased =", err_after_norm < err_before_norm)


if __name__ == "__main__":
    main()

