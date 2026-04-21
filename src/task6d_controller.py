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


# 使用6D Jacobian 完成DLS算法 求解期望关节空间速度
def solve_dls_6d(J_6d, twist_target, damping):
    J_6d = np.asarray(J_6d, dtype=float)
    twist_target = np.asarray(twist_target, dtype=float).reshape(6)

    if J_6d.shape[0] != 6:
        raise ValueError("J_6d must have 6 rows")
    
    I = np.eye(6, dtype=float)

    # 使用numpy的求解模块 实现DLS算法
    qdot_target = J_6d.T @ np.linalg.solve(J_6d @ J_6d.T + damping**2 * I, twist_target)

    return qdot_target


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

