import numpy as np

JOINT_CHAIN = [
    {"name": "Joint1", "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0], "axis_sign": 1.0, "theta_bias": 0.0},
    {"name": "Joint2", "xyz": [0.0, 0.0324999999999998, 0.193], "rpy": [-1.57079632679489, -0.467936330721615, -3.14159250487207], "axis_sign": 1.0, "theta_bias": 0.0},
    {"name": "Joint3", "xyz": [0.304, 0.0, -0.0025], "rpy": [0.0, 0.0, -2.6737], "axis_sign": 1.0, "theta_bias": 0.0},
    {"name": "Joint4", "xyz": [0.32542, 0.0070546, 0.034], "rpy": [0.021675, 1.5708, 0.0], "axis_sign": 1.0, "theta_bias": -0.3967},
    {"name": "Joint5", "xyz": [-0.038577, -0.016159, 0.0995], "rpy": [1.5708, 0.0, 1.9675], "axis_sign": -1.0, "theta_bias": 1.44484},
    {"name": "Joint6", "xyz": [-0.0038378, 0.030308, 0.041825], "rpy": [-1.5708, 1.1723, 0.12596], "axis_sign": 1.0, "theta_bias": -0.3},
]

# 将roll轴转化为绕x轴的旋转矩阵
def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype = float)


# 将pitch轴转化为绕y轴的旋转矩阵
def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype = float)


# 将yaw轴转化为绕z轴的旋转矩阵
def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype = float)


# 将rpy转化为旋转矩阵
def rpy_2_rot(rpy):
    roll, pitch, yaw = rpy
    
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


# 将位置向量和旋转矩阵转化为齐次变换矩阵
def make_transform(xyz, rpy):
    T = np.eye(4, dtype=float)
    T[:3, :3] = rpy_2_rot(rpy)
    T[:3, 3] = np.asarray(xyz, dtype=float)

    return T


# 将绕z轴的旋转转化为齐次变换矩阵  用于表达关节旋转
def rotz_transform(theta):
    T = np.eye(4, dtype=float)
    T[:3, :3] = rot_z(theta)
    return T


# 正向运动学计算 求解各关节齐次变换矩阵、位置向量、关节轴位姿向量
def manual_fk_all(theta):
    theta = np.asarray(theta, dtype=float).reshape(-1)
    if theta.size != len(JOINT_CHAIN):
        raise ValueError(f"theta must have {len(JOINT_CHAIN)} elements")
    
    T = np.eye(4, dtype=float)
    T_list = []
    p_list = []
    z_list = []

    for i, prm in enumerate(JOINT_CHAIN):
        # 计算原始齐次变化矩阵
        T_const = make_transform(prm["xyz"], prm["rpy"])

        # 映射到joint frame 取joint的轴原点和方向
        T_joint = T @ T_const
        p_i = T_joint[:3, 3].copy() # Joint_i position
        z_i = T_joint[:3, :3] @ np.array([0.0, 0.0, 1.0], dtype=float) # Joint_i z-axis direction
        z_i = z_i / np.linalg.norm(z_i) # 归一化检查

        # 进行joint旋转 由于原始urdf零点存在偏置 因此需要叠加偏置
        theta_eff = prm["axis_sign"] * theta[i] + prm["theta_bias"]
        T = T_joint @ rotz_transform(theta_eff) # 得到baselink到link6的齐次变换矩阵

        T_list.append(T.copy())
        p_list.append(p_i)
        z_list.append(z_i)

    return T, T_list, p_list, z_list


# 运动学正解
def manual_forward_kinematics(theta):
    T_0E, _, _, _ = manual_fk_all(theta)
    return T_0E, T_0E[:3, 3].copy(), T_0E[:3, :3].copy()


# 计算雅可比矩阵（几何法）
# 此处只计算线速度雅可比
def manual_position_jacobian(theta):
    T_0E, _, p_list, z_list = manual_fk_all(theta)
    p_e = T_0E[:3, 3]

    Jv = np.zeros((3, len(JOINT_CHAIN)), dtype=float)
    for i, prm in enumerate(JOINT_CHAIN):
        Jv[:, i] = prm["axis_sign"] * np.cross(z_list[i], p_e - p_list[i])

    return Jv
