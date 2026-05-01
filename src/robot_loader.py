from pathlib import Path
import numpy as np
import pinocchio as pin

PROJECT_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = PROJECT_ROOT / "mec_arm_model" / "urdf" / "mec_arm.urdf"
EE_FRAME_NAME = "Empty_Link6"

def load_robot():
    model = pin.buildModelFromUrdf(str(URDF_PATH))
    data = model.createData()
    frame_id = model.getFrameId(EE_FRAME_NAME)

    if frame_id == len(model.frames):
        raise ValueError(f"frame not found: {EE_FRAME_NAME}")
    
    return model, data, frame_id

def build_q(model, theta):
    # 转为float数组 并转化为一维向量
    theta = np.asarray(theta, dtype=float).reshape(-1) 
    
    if theta.size != model.nq:
        raise ValueError("theta must have the same number of elements as the model's joint configuration")
    
    q = pin.neutral(model)
    q[:model.nq] = theta
    
    return q
