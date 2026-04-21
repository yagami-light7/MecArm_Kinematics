'''
本文件用于检验误差计算文件"pose_error.py"的正确性

'''

from pathlib import Path
import sys
import numpy as np
import pinocchio as pin

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from robot_loader import load_robot, build_q
from fk_jacobian import forward_kinematics
from tcp_frame import extract_tcp_pose
from pose_error import position_error, orientation_error, pos_error_6d

# 测试角度
TEST_THETA = np.array([0.0, -0.2, 0.3, 0.0, 0.3, 0.2], dtype=float)

def main():
    np.set_printoptions(precision=6, suppress=True)

    model, data, frame_id =load_robot()
    q = build_q(model, TEST_THETA)

    # 当前EElink6位姿
    M_ee, _, _ = forward_kinematics(model, data, frame_id, q)

    # 当前TCP位姿
    M_tcp, p_cur, R_cur = extract_tcp_pose(M_ee)

    print("q =", q)
    print("p_cur =")
    print(p_cur)
    print("R_cur =")
    print(R_cur)

    print("\n===== Case 1: same pose =====")
    e_pos_same = position_error(p_cur, p_cur)
    e_rot_same = orientation_error(R_cur, R_cur)
    e_6d_same = pos_error_6d(p_cur, p_cur, R_cur, R_cur)

    print("e_pos_same =")
    print(e_pos_same)
    print("e_rot_same =")
    print(e_rot_same)
    print("e_6d_same =")
    print(e_6d_same)


    print("\n===== Case 2: translation only =====")
    dp = np.array([0.02, -0.01, 0.03], dtype=float)

    p_des_trans = p_cur + dp
    R_des_trans = R_cur

    e_pos_trans = position_error(p_des_trans, p_cur)
    e_rot_trans = orientation_error(R_des_trans, R_cur)
    e_6d_trans = pos_error_6d(p_des_trans, p_cur, R_des_trans, R_cur)

    print("expected translation error =")
    print(dp)

    print("e_pos_trans =")
    print(e_pos_trans)
    print("e_rot_trans =")
    print(e_rot_trans)
    print("e_6d_trans =")
    print(e_6d_trans)


    print("\n===== Case 3: rotation only =====")
    delta_rpy = np.array([0.0, 0.0, 0.2], dtype=float)
    R_delta = pin.rpy.rpyToMatrix(delta_rpy)

    # 让目标姿态相对当前 TCP 姿态做一个“局部坐标系下”的旋转扰动
    R_des_rot = R_cur @ R_delta
    p_des_rot = p_cur.copy()

    e_pos_rot = position_error(p_des_rot, p_cur)
    e_rot_rot = orientation_error(R_des_rot, R_cur)
    e_6d_rot = pos_error_6d(p_des_rot, p_cur, R_des_rot, R_cur)

    # 理论上，这时的局部旋转误差就是 log3(R_delta)，
    # 再映射到世界表达应为 R_cur @ log3(R_delta)
    expected_rot_world = R_cur @ pin.log3(R_delta)

    print("expected_rot_world =")
    print(expected_rot_world)

    print("e_pos_rot =")
    print(e_pos_rot)
    print("e_rot_rot =")
    print(e_rot_rot)
    print("e_6d_rot =")
    print(e_6d_rot)


if __name__ == "__main__":
    main()
