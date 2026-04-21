import numpy as np
import pinocchio as pin

from robot_loader import load_robot
from fk_jacobian import forward_kinematics

# TCP tool center point 定义名称 用于打印和日志输出
TCP_FRAME_NAME = "TCP"

# TCP相对Empty_link6的平移向量
TCP_TRANSLATION = np.array([0, 0, 0.15], dtype = float)

# TCP相对Empty_link6的Rotation  使得夹爪水平时 TCP坐标系姿态与基坐标系姿态相同
TCP_ROTATION = np.array([
    [ 0.014486555, -0.765812320,  0.642900949],
    [-0.017242311, -0.643064142, -0.765618190],
    [ 0.999746389,  0.000006072, -0.022520159],
], dtype=float)



# 构造Empty_link6到TCP的变换
# Empty_link6 -> TCP
def build_tcp_transform():
    # 计算旋转矩阵
    R_e_tcp = TCP_ROTATION
    # 构造齐次变换矩阵
    M_e_tcp = pin.SE3(R_e_tcp, TCP_TRANSLATION)
    return M_e_tcp


# 由Empty_link6的齐次变换矩阵计算TCP齐次变换矩阵
# M_e_tcp: Empty_link6 -> TCP
def tcp_pose_from_ee_pose(M_ee):
    # 构造Empty_link6到TCP的变换
    M_e_tcp = build_tcp_transform()
    # 得到基坐标到TCP的齐次变换矩阵
    # Base -> TCP = (Base -> Empty_link6) * (Empty_link6 -> TCP)
    M_tcp =  M_ee * M_e_tcp # SE3对象将 * 重载 不需要使用@ 

    return M_tcp


# 提取TCP在基坐标下的位置向量和旋转矩阵
def extract_tcp_pose(M_ee):
    # 计算TCP的齐次变换矩阵
    M_tcp = tcp_pose_from_ee_pose(M_ee)

    return M_tcp, M_tcp.translation.copy(), M_tcp.rotation.copy()


# 构造反对称矩阵 用于叉乘运算 ω × r = - [r]_x ω
def skew(v):
    x, y, z = np.asarray(v, dtype = float).reshape(3)
    return np.array([   [ 0, -z, +y], 
                        [+z,  0, -x], 
                        [-y, +x,  0]], dtype=float)


# 根据 Empty_Link6 的 6x6 Jacobian 和 TCP 相对位置，构造 TCP 的 6x6 Jacobian。
def tcp_jacobian_from_ee_jacobian(J_ee, p_ee, p_tcp):
    J_ee = np.asarray(J_ee, dtype=float)

    Jv_ee = J_ee[0:3, :]  # 线速度雅可比矩阵
    Jw_ee = J_ee[3:6, :]  # 角速度雅可比矩阵

    r = np.asarray(p_tcp, dtype=float) - np.asarray(p_ee, dtype=float)

    Jv_tcp = Jv_ee - skew(r) @ Jw_ee
    Jw_tcp = Jw_ee 

    J_tcp = np.vstack([Jv_tcp, Jw_tcp])

    return J_tcp


# 使用FK+数值微分验证线速度雅可比矩阵正确性
def numeric_tcp_position_jacobian(model, data, frame_id, q, eps = 1e-7):
    # 计算当前q下的TCP位置向量
    M_ee, _, _  = forward_kinematics(model, data, frame_id, q)
    _, p_tcp, _ = extract_tcp_pose(M_ee)

    J_num = np.zeros((3, model.nv))

    for i in range(model.nv):
        q_perturb = q.copy()
        q_perturb[i] += eps # 对第i个关节施加增量 这会影响Jacobian的第i列

        # 计算施加关节增量后的TCP位置向量
        M_ee_turb, _, _ = forward_kinematics(model, data, frame_id, q_perturb)
        _, p_tcp_turb, _ = extract_tcp_pose(M_ee_turb)

        J_num[:, i] = (p_tcp_turb - p_tcp) / eps

    return J_num


# 使用FK+数值微分验证角速度雅可比矩阵正确性
# 通过相对旋转的 log 映射求角速度增量
def numeric_tcp_angular_jacobian(model, data, frame_id, q, eps=1e-7):
    # 计算当前q下的TCP旋转矩阵
    M_ee, _, _ = forward_kinematics(model, data, frame_id, q)
    _, _, R_tcp = extract_tcp_pose(M_ee)

    Jw_num = np.zeros((3, model.nv))

    for i in range(model.nv):
        q_perturb = q.copy()
        q_perturb[i] += eps # 对第i个关节施加增量 这会影响Jacobian的第i列

        # 计算施加关节增量后的TCP旋转矩阵
        M_ee_turb, _, _ = forward_kinematics(model, data, frame_id, q_perturb)
        _, _, R_tcp_perturb = extract_tcp_pose(M_ee_turb) # 当前姿态到扰动姿态的相对旋转
        
        # 先在当前 TCP 局部坐标系下求小转角
        R_rel = R_tcp.T @ R_tcp_perturb

        # log3 给出 SO(3) 的小旋转向量
        dphi_local = pin.log3(R_rel)

        # 由于我们的雅可比矩阵是相对基坐标系的
        # 我们再将角速度增量转为基坐标系表达
        dphi_world = R_tcp @ dphi_local

        Jw_num[:, i] = dphi_world / eps

    return Jw_num


# 组合线速度雅可比矩阵和角速度雅可比矩阵，得到 TCP 的 6D 数值 Jacobian
def numeric_tcp_jacobian_6d(model, data, frame_id, q, eps=1e-7):
    Jv_num = numeric_tcp_position_jacobian(model, data, frame_id, q, eps)
    Jw_num = numeric_tcp_angular_jacobian(model, data, frame_id, q, eps)
    J_num_6d = np.vstack([Jv_num, Jw_num])
    return J_num_6d
