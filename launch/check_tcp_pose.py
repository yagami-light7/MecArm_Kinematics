'''
本文件用于验证FK：TCP位姿计算的正确性

'''

from pathlib import Path
import sys
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from robot_loader import load_robot, build_q, EE_FRAME_NAME
from fk_jacobian import forward_kinematics
from tcp_frame import (
    TCP_FRAME_NAME,
    TCP_TRANSLATION,
    TCP_RPY,
    extract_tcp_pose,
)

# 测试角度数据
TEST_THETA = np.array([0.0, -0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)


def main():
    np.set_printoptions(precision=6, suppress=True)

    # 构建模型 构造q向量
    model, data, frame_id = load_robot()
    q = build_q(model, TEST_THETA)

    # 正向运动学 计算EmptyLink位姿
    M_ee, p_ee, R_ee = forward_kinematics(model, data, frame_id,q)

    # 得到 TCP 位姿
    M_tcp, p_tcp, R_tcp = extract_tcp_pose(M_ee)

    print("EE_FRAME_NAME =", EE_FRAME_NAME)
    print("TCP_FRAME_NAME =", TCP_FRAME_NAME)
    print("q =", q)

    print("TCP_TRANSLATION =", TCP_TRANSLATION)
    print("TCP_RPY =", TCP_RPY)

    print("p_ee =")
    print(p_ee)

    print("R_ee =")
    print(R_ee)

    print("p_tcp =")
    print(p_tcp)

    print("R_tcp =")
    print(R_tcp)

    print("M_tcp =")
    print(M_tcp)



if __name__ == "__main__":
    main()