'''
本文件提供6d闭环控制接口：
    1. 由6d位姿误差生成末端期望twist
    2. 基于DLS & Weight-DLS 算法实现由期望twist生成期望qdot
    3. 模拟控制律
'''

import numpy as np
from pose_error import pos_error_6d


# 将三维增益转化为数组 可输入标量 或者 三维向量
def expand_gain_3d(gain, name):

    gain = np.asarray(gain, dtype = float).reshape(-1)

    # 标量
    if gain.size == 1:
        return np.repeat(gain.item(), 3)
    
    if gain.size != 3:
        raise ValueError(f"{name} must be a scalar or a 3-element array")
    
    return gain


# 由6d位姿误差生成末端期望twist:[vx, vy, vz, wx, wy, wz]
def task_twist_from_pose_error(e_6d, kp_pos, kp_rot):
    # 将误差转为列向量
    e_6d = np.asarray(e_6d, dtype = float).reshape(6)
    
    # 整理增益 
    kp_pos = expand_gain_3d(kp_pos, "kp_pos")
    kp_rot = expand_gain_3d(kp_rot, "kp_rot")

    # 计算期望twist
    v_target = kp_pos * e_6d[:3]
    w_target = kp_rot * e_6d[3:]

    twist_target = np.hstack([v_target, w_target])

    return twist_target

# 支持多类型输入的DLS权重矩阵
def expand_task_weight_6d(Wx, Wq):
    Wx = np.asarray(Wx, dtype=float).reshape(-1)
    Wq = np.asarray(Wq, dtype=float).reshape(-1)

    # 标量输入
    if Wx.size == 1:
        Wx = np.repeat(Wx.item(), 6)

    if Wq.size == 1:
        Wq = np.repeat(Wq.item(), 6)

    # 二元素输入
    if Wx.size == 2:
        Wx_pos = np.repeat(Wx[0], 3)
        Wx_rot = np.repeat(Wx[1], 3)
        Wx = np.hstack([Wx_pos, Wx_rot])

    # if Wq.size == 2:
    #     Wq_pos = np.repeat(Wq[0], 3)
    #     Wq_rot = np.repeat(Wq[1], 3)
    #     Wq = np.hstack([Wq_pos, Wq_rot])

    if Wx.size != 6:
        raise ValueError("Wx must be a scalar, a 2-element array, or a 6-element array")

    if Wq.size != 6:
        raise ValueError("Wq must be a scalar, or a 6-element array")

    return Wx, Wq

# 提取DLS算法的伪逆矩阵 J#_dls = (J^T J + λ^2 I)^-1 J^T
def dls_inverse_6d(J_6d, damping):
    J_6d = np.asarray(J_6d, dtype=float)

    if J_6d.shape[0] != 6:
        raise ValueError("J_6d must have 6 rows")

    n_joint = J_6d.shape[1] 

    I = np.eye(n_joint, dtype=float)

    A = J_6d.T @ J_6d + damping**2 * I
    b = J_6d.T

    return np.linalg.solve(A, b)


# 提取Weighted-DLS算法的伪逆矩阵 J#_wdls = (J^T Wx J + λ^2 Wq)^-1 J^T Wx
def wdls_inverse_6d(J_6d, damping, Wx, Wq):
    J_6d = np.asarray(J_6d, dtype=float)

    if J_6d.shape[0] != 6:
        raise ValueError("J_6d must have 6 rows")

    Wx, Wq = expand_task_weight_6d(Wx, Wq)

    if np.any(Wx <= 0):
        raise ValueError("All task weights in Wx must be positive")

    if np.any(Wq <= 0):
        raise ValueError("All joint weights in Wq must be positive")

    if J_6d.shape[1] != Wq.size:
        raise ValueError("Wq size must match the number of joints")
    
    Wx_diag = np.diag(Wx)
    Wq_diag = np.diag(Wq)


    A = J_6d.T @ Wx_diag @ J_6d + damping**2 * Wq_diag
    b = J_6d.T @ Wx_diag

    return np.linalg.solve(A, b)


# 使用6D Jacobian 完成DLS算法 求解期望关节空间速度
def solve_dls_6d(J_6d, twist_target, damping):
    J_6d = np.asarray(J_6d, dtype=float)
    twist_target = np.asarray(twist_target, dtype=float).reshape(6)

    if J_6d.shape[0] != 6:
        raise ValueError("J_6d must have 6 rows")
    
    I = np.eye(6, dtype=float)

    # 使用numpy的求解模块 实现DLS算法

    ''' 
        min  1/2 ||J qdot - twist||^2 + 1/2 λ^2 ||qdot||^2
        qdot = J#_dls twist
        J#_dls = (J^T J + λ^2 I)^-1 J^T
    '''
    J_dls_inv = dls_inverse_6d(J_6d, damping)
    qdot_target = J_dls_inv @ twist_target

    return qdot_target


# 使用6D Jacobian 完成Weight-DLS算法 求解期望关节空间速度
def solve_weighted_dls_6d(J_6d, twist_target, damping, Wx, Wq):
    J_6d = np.asarray(J_6d, dtype=float)
    twist_target = np.asarray(twist_target, dtype=float).reshape(6)

    if J_6d.shape[0] != 6:
        raise ValueError("J_6d must have 6 rows")
    
    J_wdls_inv = wdls_inverse_6d(
        J_6d=J_6d,
        damping=damping,
        Wx=Wx,
        Wq=Wq,
    )

    # 使用numpy的求解模块 实现DLS算法
    ''' 
        min  1/2 (J qdot - twist)^T Wx (J qdot - twist)+ 1/2 λ^2 qdot^T Wq qdot
        qdot = J#_wdls twist
        J#_wdls = (J^T Wx J + λ^2 Wq)^-1 J^T Wx
    '''
    qdot_target = J_wdls_inv @ twist_target

    return qdot_target


# 构造零空间投影矩阵
def nullspace_projector_6d(J_6d, J_dls_or_wdls_inv):
    J_6d = np.asarray(J_6d, dtype=float)
    J_dls_or_wdls_inv = np.asarray(J_dls_or_wdls_inv, dtype=float)

    n_joint = J_6d.shape[1]
    I = np.eye(n_joint, dtype=float)

    return I - J_dls_or_wdls_inv @ J_6d


# 代价函数：构造保持关节中心副任务 通过零空间投影实现机械臂运动优化
def joint_center_secondary_velocity(q, q_home, q_min, q_max, gain):
    q = np.asarray(q, dtype=float).reshape(-1)
    q_home = np.asarray(q_home, dtype=float).reshape(-1)
    q_min = np.asarray(q_min, dtype=float).reshape(-1)
    q_max = np.asarray(q_max, dtype=float).reshape(-1)

    q_range = q_max - q_min
    q_range = np.maximum(q_range, 1e-6)

    q_error_norm = (q - q_home) / q_range

    return -gain * q_error_norm


# 关节速度做限幅    可输入标量或6维向量
def clip_qdot(qdot, qdot_limit):
    qdot = np.asarray(qdot, dtype=float).reshape(-1)
    qdot_limit = np.asarray(qdot_limit, dtype=float).reshape(-1)

    # 标量
    if qdot_limit.size == 1:
        return np.clip(qdot, -qdot_limit.item(), qdot_limit.item())

    if qdot_limit.size != qdot.size:
        raise ValueError("qdot_limit must be a scalar or have the same size as qdot")

    return np.clip(qdot, -qdot_limit, qdot_limit)


# 模拟控制律 
def step_pose_6d_control(
    q,
    p_des,
    R_des,
    p_cur,
    R_cur,
    J_tcp,
    dt,
    kp_pos,
    kp_rot,
    damping,
    qdot_limit,
):
    
    q = np.asarray(q, dtype=float).reshape(-1)

    # 计算6d误差
    e_6d = pos_error_6d(p_des, p_cur, R_des, R_cur)
    # 根据误差生成期望twist
    twist_target = task_twist_from_pose_error(e_6d, kp_pos, kp_rot)
    # 根据期望twist 基于dls 计算期望qdot
    qdot_target = solve_dls_6d(J_tcp, twist_target, damping)
    # 对qdot进行限幅
    qdot_target = clip_qdot(qdot_target, qdot_limit)
    # 欧拉积分 得到目标关节角度 实际操作时直接使用qdot进行速度控制 效果更好
    q_next = q + qdot_target * dt

    return q_next, qdot_target, e_6d, twist_target

