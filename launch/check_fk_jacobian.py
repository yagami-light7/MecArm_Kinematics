'''

本文件通过调用pinocchio的API实现FK、Jacobian计算
并使用数值微分+FK的方法验证Jacobian的正确性

'''

from pathlib import Path
import sys
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from robot_loader import load_robot, build_q, URDF_PATH, EE_FRAME_NAME
from fk_jacobian import (
    forward_kinematics, # 正向运动学
    frame_jacobian, # 计算末端雅可比
    numeric_position_jacobian, # 雅可比矩阵数值差分计算
    extract_position_jacobian # 提取位置雅可比
)

TEST_THETA = np.array([0.2, -0.3, 0.4, 0.1, 0.2, 0.3], dtype=float)

def main():
    np.set_printoptions(precision=6, suppress=True)

    model, data, frame_id = load_robot()
    q = build_q(model, TEST_THETA)

    M, p_cur, R_cur = forward_kinematics(model, data, frame_id, q)
    J6 = frame_jacobian(model, data, frame_id, q)
    J_num = numeric_position_jacobian(model, data, frame_id, q)
    Jv, block_name = extract_position_jacobian(J6, J_num)

    err_top = np.linalg.norm(J6[0:3] - J_num)
    err_bottom = np.linalg.norm(J6[3:6] - J_num)
    err_selected = np.linalg.norm(Jv - J_num)

    print("URDF_PATH =", URDF_PATH)
    print("EE_FRAME_NAME =", EE_FRAME_NAME)
    print("frame_id =", frame_id)
    print("q =", q)

    print("position =")
    print(p_cur)

    print("rotation =")
    print(R_cur)

    print("SE3 =")
    print(M)

    print("J6 =")
    print(J6)

    print("J_num =")
    print(J_num)

    print("selected block =", block_name)
    print("err_top =", err_top)
    print("err_bottom =", err_bottom)
    print("err_selected =", err_selected)


if __name__ == "__main__":
    main()  
