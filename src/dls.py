'''
本文件用于模拟MCU控制律，完成以下数学处理：

    DLS求解关节空间目标速度
    欧拉积分得到关节空间目标角度

DLS仅解决位置（平动速度）

'''

import numpy as np

# 基于DLS 求解关节空间目标速度
def solve_dls(Jv, v_des, damping):
    # 构造任务空间单位阵，维度等于末端位置维度
    I = np.eye(Jv.shape[0])
    # 用 solve(A, b) 代替 inv(A) @ b，更稳定  此处存在精确解 不使用lstsq
    return Jv.T @ np.linalg.solve(Jv @ Jv.T + (damping ** 2 ) * I, v_des)

# 控制律
def step_position_dls(q, p_cur, p_des, Jv, dt, kp, damping, qdot_limit):
    #计算笛卡尔空间误差
    err = p_des - p_cur
    # 比例控制计算期望速度
    v_des = kp * err
    # DLS求解关节速度
    q_dot = solve_dls(Jv, v_des, damping)
    # 关节速度限幅
    q_dot = np.clip(q_dot, -qdot_limit, qdot_limit)
    # 欧拉积分 更新下一时刻关节角度
    q_next = q + q_dot * dt

    return q_next, q_dot, err, v_des
