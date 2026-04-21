'''
本文件用于笛卡尔空间误差计算，完成以下数学处理：

    平移误差
    姿态误差
    完整 6D 位姿误差

'''
import numpy as np
import pinocchio as pin

# 计算平移误差 在基坐标系下相减
def position_error(p_des, p_cur):
    p_des = np.asarray(p_des, dtype=float).reshape(3)
    p_cur = np.asarray(p_cur, dtype=float).reshape(3)

    return p_des - p_cur


# 计算姿态误差 思路同数值微分求角速度雅可比矩阵：
# 1.在末端坐标系下，使用log3求当前姿态和目标姿态的的转角
# 2.映射回基坐标系
def orientation_error(R_des, R_cur):
    R_des = np.asarray(R_des, dtype=float).reshape(3, 3)
    R_cur = np.asarray(R_cur, dtype=float).reshape(3, 3)

    # 使用log3求当前姿态和目标姿态的的转角
    R_real = R_cur.T @ R_des

    # 映射回基坐标系
    e_rot_local = pin.log3(R_real)
    e_rot_world = R_cur @ e_rot_local

    return e_rot_world


# 组合得到6D位姿误差向量
def pos_error_6d(p_des, p_cur, R_des, R_cur):
    e_pos = position_error(p_des, p_cur)
    e_rot = orientation_error(R_des, R_cur)
    e_6d = np.hstack([e_pos, e_rot])
    
    return e_6d