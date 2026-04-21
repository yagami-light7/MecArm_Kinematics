from pathlib import Path
import sys
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from robot_loader import load_robot, build_q, EE_FRAME_NAME
from fk_jacobian import forward_kinematics, frame_jacobian
from tcp_frame import (
    TCP_FRAME_NAME,
    TCP_TRANSLATION,
    TCP_RPY,
    extract_tcp_pose,
    tcp_jacobian_from_ee_jacobian,
    numeric_tcp_jacobian_6d,
)

# 测试角度
TEST_THETA = np.array([0.0, -0.2, 0.3, 0.0, 0.3, 0.2], dtype=float)


def main():
    # numpy打印设置
    np.set_printoptions(precision=6, suppress=True)

    # 构造机器人模型并计算q向量
    model, data, frame_id = load_robot()
    q = build_q(model, TEST_THETA)

    # 计算Empty Link6的位姿和雅可比矩阵
    M_ee, p_ee, R_ee = forward_kinematics(model, data, frame_id, q)
    J_ee = frame_jacobian(model, data, frame_id, q)

    # 计算TCP位姿
    M_tcp, p_tcp, R_tcp = extract_tcp_pose(M_ee)

    # 计算TCP雅可比矩阵
    J_tcp = tcp_jacobian_from_ee_jacobian(J_ee, p_ee, p_tcp)

    # 数值微分计算TCP雅可比矩阵 用作检验
    J_tcp_num = numeric_tcp_jacobian_6d(model, data, frame_id, q)

    # 分别评估线速度、角速度、完整 6D Jacobian 的误差
    err_v = np.linalg.norm(J_tcp[0:3, :] - J_tcp_num[0:3, :])
    err_w = np.linalg.norm(J_tcp[3:6, :] - J_tcp_num[3:6, :])
    err_6d = np.linalg.norm(J_tcp - J_tcp_num)

    print("EE_FRAME_NAME =", EE_FRAME_NAME)
    print("TCP_FRAME_NAME =", TCP_FRAME_NAME)
    print("q =", q)

    print("TCP_TRANSLATION =", TCP_TRANSLATION)
    print("TCP_RPY =", TCP_RPY)

    print("p_ee =")
    print(p_ee)

    print("p_tcp =")
    print(p_tcp)

    print("J_ee =")
    print(J_ee)

    print("J_tcp =")
    print(J_tcp)

    print("J_tcp_num =")
    print(J_tcp_num)

    print("tcp linear jacobian error =", err_v)
    print("tcp angular jacobian error =", err_w)
    print("tcp full 6D jacobian error =", err_6d)

if __name__ == "__main__":
    main()