'''

本文件用于对比 手动计算的FK和线速度雅可比矩阵 与 调用Pinocchio库的结果
通过随机采集各关节限位下的200组数据 进行FK和Jaccobian计算

计算完毕后与Pinocchio库的结果进行对比 

生成对比结果可视化图片

'''

from pathlib import Path
from datetime import datetime
import csv
import json
import sys

import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from robot_loader import load_robot, build_q
from fk_jacobian import forward_kinematics, frame_jacobian
from manual_fk_jacobian import manual_forward_kinematics, manual_position_jacobian


# 本次离线验证固定随机种子，保证每次运行抽样结果可复现。
RNG_SEED = 42

# 本次随机抽取的合法关节角样本组数。
# 这里明确使用 200 组随机数据做批量真值对比。
NUM_SAMPLES = 200


# 从模型的关节上下限中，均匀随机采样一组合法关节角。
def sample_random_theta(model, rng):
    lower = np.asarray(model.lowerPositionLimit[:model.nq], dtype=float)
    upper = np.asarray(model.upperPositionLimit[:model.nq], dtype=float)
    return rng.uniform(lower, upper)


# 为本次批量对比创建独立结果目录，避免覆盖历史结果。
def create_result_dir():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_dir = PROJECT_ROOT / "results" / "batch_compare_manual_vs_pin" / timestamp
    result_dir.mkdir(parents=True, exist_ok=True)
    return result_dir


# 保存逐样本误差日志，便于后续离线分析和复现实验。
def save_sample_errors_csv(result_dir, records):
    csv_path = result_dir / "sample_errors.csv"
    fieldnames = [
        "sample_index",
        "theta1",
        "theta2",
        "theta3",
        "theta4",
        "theta5",
        "theta6",
        "position_error_norm",
        "rotation_error_norm",
        "jacobian_error_norm",
    ]

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)

    return csv_path


# 保存本次批量验证摘要信息，便于后续查看最大/平均误差。
def save_summary_json(
    result_dir,
    num_samples,
    rng_seed,
    pos_errors,
    rot_errors,
    jac_errors,
    worst_pos_theta,
    worst_rot_theta,
    worst_jac_theta,
):
    summary = {
        "num_samples": int(num_samples),
        "rng_seed": int(rng_seed),
        "position_error_mean": float(pos_errors.mean()),
        "position_error_max": float(pos_errors.max()),
        "rotation_error_mean": float(rot_errors.mean()),
        "rotation_error_max": float(rot_errors.max()),
        "jacobian_error_mean": float(jac_errors.mean()),
        "jacobian_error_max": float(jac_errors.max()),
        "worst_position_theta": np.asarray(worst_pos_theta, dtype=float).tolist(),
        "worst_rotation_theta": np.asarray(worst_rot_theta, dtype=float).tolist(),
        "worst_jacobian_theta": np.asarray(worst_jac_theta, dtype=float).tolist(),
    }

    json_path = result_dir / "summary.json"
    with json_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    return json_path


# 生成逐样本误差曲线图。
# 由于误差量级很小，使用对数纵轴更容易观察差异。
def save_error_curves_svg(result_dir, pos_errors, rot_errors, jac_errors):
    sample_indices = np.arange(pos_errors.size)

    fig, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=True)

    curve_specs = [
        (axes[0], pos_errors, "Position Error Norm", "#1D4E89"),
        (axes[1], rot_errors, "Rotation Error Norm", "#A23B72"),
        (axes[2], jac_errors, "Jacobian Error Norm", "#2A9D8F"),
    ]

    for ax, values, title, color in curve_specs:
        ax.plot(sample_indices, values, color=color, linewidth=1.8)
        ax.set_title(title)
        ax.set_ylabel("Error")
        ax.set_yscale("log")
        ax.grid(True, linestyle="--", alpha=0.35)

    axes[-1].set_xlabel("Sample Index")
    fig.suptitle("Manual Kinematics vs Pinocchio: Error Curves", fontsize=13)
    fig.tight_layout()

    svg_path = result_dir / "error_curves.svg"
    fig.savefig(svg_path, format="svg", bbox_inches="tight")
    plt.close(fig)

    return svg_path


# 生成误差分布直方图。
# 这张图用于观察 200 组随机样本误差是否集中在极小量级。
def save_error_histograms_svg(result_dir, pos_errors, rot_errors, jac_errors):
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.2))

    hist_specs = [
        (axes[0], pos_errors, "Position Error Histogram", "#1D4E89"),
        (axes[1], rot_errors, "Rotation Error Histogram", "#A23B72"),
        (axes[2], jac_errors, "Jacobian Error Histogram", "#2A9D8F"),
    ]

    for ax, values, title, color in hist_specs:
        ax.hist(values, bins=20, color=color, alpha=0.85, edgecolor="white")
        ax.set_title(title)
        ax.set_xlabel("Error")
        ax.set_ylabel("Count")
        ax.grid(True, linestyle="--", alpha=0.25)

    fig.suptitle("Manual Kinematics vs Pinocchio: Error Distributions", fontsize=13)
    fig.tight_layout()

    svg_path = result_dir / "error_histograms.svg"
    fig.savefig(svg_path, format="svg", bbox_inches="tight")
    plt.close(fig)

    return svg_path


def main():
    np.set_printoptions(precision=6, suppress=True)

    # 初始化随机数生成器。固定种子后，200 组随机样本可重复生成。
    rng = np.random.default_rng(RNG_SEED)

    # 载入 Pinocchio 模型。它在这里作为 MCU 手写运动学的真值参考。
    model, data, frame_id = load_robot()

    # 为本次离线验证创建独立结果目录，并准备逐样本日志缓存。
    result_dir = create_result_dir()
    records = []

    # 逐样本记录位置误差、姿态误差、Jacobian 误差。
    pos_errors = []
    rot_errors = []
    jac_errors = []

    # 记录三类误差对应的“最坏姿态”，便于后续复查。
    worst_pos_theta = None
    worst_rot_theta = None
    worst_jac_theta = None

    worst_pos_err = -np.inf
    worst_rot_err = -np.inf
    worst_jac_err = -np.inf

    # 这里明确使用 NUM_SAMPLES = 200 组合法随机关节角进行批量对比。
    for sample_index in range(NUM_SAMPLES):
        # 生成一组合法关节角 theta。
        theta = sample_random_theta(model, rng)

        # -------------------- Pinocchio 真值链路 --------------------
        # 1. 把 theta 转成 Pinocchio 需要的 q。
        # 2. 用 Pinocchio 计算末端位置/姿态。
        # 3. 用 Pinocchio 计算 6x6 Jacobian，并提取前 3 行的位置 Jacobian。
        q_pin = build_q(model, theta)
        _, p_pin, R_pin = forward_kinematics(model, data, frame_id, q_pin)
        Jv_pin = frame_jacobian(model, data, frame_id, q_pin)[0:3, :]

        # -------------------- 手写运动学链路 --------------------
        # 1. 用手写齐次变换链计算 FK。
        # 2. 用几何法计算位置 Jacobian。
        _, p_man, R_man = manual_forward_kinematics(theta)
        Jv_man = manual_position_jacobian(theta)

        # 分别计算三类误差的 2 范数。
        pos_err = np.linalg.norm(p_man - p_pin)
        rot_err = np.linalg.norm(R_man - R_pin)
        jac_err = np.linalg.norm(Jv_man - Jv_pin)

        pos_errors.append(pos_err)
        rot_errors.append(rot_err)
        jac_errors.append(jac_err)

        # 保存逐样本日志，后面会导出成 CSV。
        records.append(
            {
                "sample_index": sample_index,
                "theta1": float(theta[0]),
                "theta2": float(theta[1]),
                "theta3": float(theta[2]),
                "theta4": float(theta[3]),
                "theta5": float(theta[4]),
                "theta6": float(theta[5]),
                "position_error_norm": float(pos_err),
                "rotation_error_norm": float(rot_err),
                "jacobian_error_norm": float(jac_err),
            }
        )

        # 用“严格大于”更新最坏样本，避免每轮都重复求 max(list)。
        if pos_err > worst_pos_err:
            worst_pos_err = pos_err
            worst_pos_theta = theta.copy()

        if rot_err > worst_rot_err:
            worst_rot_err = rot_err
            worst_rot_theta = theta.copy()

        if jac_err > worst_jac_err:
            worst_jac_err = jac_err
            worst_jac_theta = theta.copy()

    # 将误差序列转成 NumPy 数组，便于统一统计与绘图。
    pos_errors = np.asarray(pos_errors, dtype=float)
    rot_errors = np.asarray(rot_errors, dtype=float)
    jac_errors = np.asarray(jac_errors, dtype=float)

    # 保存离线日志与可视化结果。
    sample_errors_csv_path = save_sample_errors_csv(result_dir, records)
    summary_json_path = save_summary_json(
        result_dir=result_dir,
        num_samples=NUM_SAMPLES,
        rng_seed=RNG_SEED,
        pos_errors=pos_errors,
        rot_errors=rot_errors,
        jac_errors=jac_errors,
        worst_pos_theta=worst_pos_theta,
        worst_rot_theta=worst_rot_theta,
        worst_jac_theta=worst_jac_theta,
    )
    error_curves_svg_path = save_error_curves_svg(result_dir, pos_errors, rot_errors, jac_errors)
    error_histograms_svg_path = save_error_histograms_svg(result_dir, pos_errors, rot_errors, jac_errors)

    # 终端打印摘要信息，便于快速确认结果。
    print("num_samples =", NUM_SAMPLES)
    print("rng_seed =", RNG_SEED)

    print("position error mean =", pos_errors.mean())
    print("position error max  =", pos_errors.max())
    print("worst position theta =", worst_pos_theta)

    print("rotation error mean =", rot_errors.mean())
    print("rotation error max  =", rot_errors.max())
    print("worst rotation theta =", worst_rot_theta)

    print("jacobian error mean =", jac_errors.mean())
    print("jacobian error max  =", jac_errors.max())
    print("worst jacobian theta =", worst_jac_theta)

    print("result_dir =", result_dir)
    print("sample_errors_csv_path =", sample_errors_csv_path)
    print("summary_json_path =", summary_json_path)
    print("error_curves_svg_path =", error_curves_svg_path)
    print("error_histograms_svg_path =", error_histograms_svg_path)


if __name__ == "__main__":
    main()
