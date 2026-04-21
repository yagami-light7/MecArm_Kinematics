import numpy as np
import pinocchio as pin


def forward_kinematics(model, data, frame_id, q):
    # 正运动学解算
    pin.forwardKinematics(model, data, q)
    # 更新frame位姿
    pin.updateFramePlacements(model, data)
    # 取出frame的齐次变换矩阵
    M = data.oMf[frame_id]
    # 返回其次变换矩阵、位置矩阵、旋转矩阵
    return M, M.translation.copy(), M.rotation.copy() 


def frame_jacobian(model, data, frame_id, q):
    # 计算末端的雅可比矩阵 并且以世界坐标系的参考
    J6 = pin.computeFrameJacobian(model, data, q, frame_id, 
                                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    return J6.copy()

# 数值方法计算雅可比矩阵
def numeric_position_jacobian(model, data, frame_id, q, eps=1e-7):
    # FK计算位置向量
    _, p0, _ = forward_kinematics(model, data, frame_id, q)
    # 偏导求解位置雅可比矩阵
    J_num = np.zeros((3, model.nv))

    for i in range(model.nv):
        q_perturb = q.copy()
        q_perturb[i] += eps
        _, p1, _ = forward_kinematics(model, data, frame_id, q_perturb)
        J_num[:, i] = (p1 - p0) / eps

    return J_num

def extract_position_jacobian(J6, J_num):
    J_top = J6[0:3, :]
    J_bottom = J6[3:6, :]

    err_top = np.linalg.norm(J_top - J_num)
    err_bottom = np.linalg.norm(J_bottom - J_num)
    
    return (J_top, "top") if err_top < err_bottom else (J_bottom, "bottom")
