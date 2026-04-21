'''

本文件用于对比 手动计算的FK和线速度雅可比矩阵 与 调用Pinocchio库的结果

'''
from pathlib import Path 
import sys
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from robot_loader import load_robot, build_q
from fk_jacobian import forward_kinematics, frame_jacobian
from manual_fk_jacobian import manual_forward_kinematics, manual_position_jacobian

TEST_THETA = np.array([0, 0, 0, 0, 0, 0], dtype=float)


def main():
    np.set_printoptions(precision=6, suppress=True)

    model, data, frame_id = load_robot()

    q_pin = build_q(model, TEST_THETA)
    _, p_pin, R_pin = forward_kinematics(model, data, frame_id, q_pin)
    Jv_pin = frame_jacobian(model, data, frame_id, q_pin)[0:3, :]

    T_man, p_man, R_man = manual_forward_kinematics(TEST_THETA)
    Jv_man = manual_position_jacobian(TEST_THETA)

    print("p_pin =")
    print(p_pin)
    print("p_man =")
    print(p_man)
    print("position error norm =", np.linalg.norm(p_man - p_pin))

    print("R_pin =")
    print(R_pin)
    print("R_man =")
    print(R_man)
    print("rotation matrix error norm =", np.linalg.norm(R_man - R_pin))

    print("Jv_pin =")
    print(Jv_pin)
    print("Jv_man =")
    print(Jv_man)
    print("jacobian error norm =", np.linalg.norm(Jv_man - Jv_pin))

    print("T_man =")
    print(T_man)


if __name__ == "__main__":
    main()