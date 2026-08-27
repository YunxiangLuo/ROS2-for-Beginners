#!/usr/bin/env python3
"""
定位精度评估节点: 对比 EKF 输出 vs CARLA Ground Truth

用法:
  # 在线评估
  python3 localization_eval.py

  # 离线评估 bag
  python3 localization_eval.py --bag ch27_eval/ch27_eval.db3
"""

import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

try:
    from scipy.spatial.transform import Rotation
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

# ============================================================
# 在线评估节点
# ============================================================

class LocalizationEvalNode(Node):
    """实时收集 EKF 和 GT 轨迹并评估"""

    def __init__(self):
        super().__init__("localization_eval_node")

        self.gt_trajectory = []   # [(timestamp, x, y, z, qw, qx, qy, qz)]
        self.est_trajectory = []

        self.gt_sub = self.create_subscription(
            Odometry, "/carla/ground_truth", self.gt_callback, 10
        )
        self.est_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.est_callback, 10
        )

        self.eval_timer = self.create_timer(5.0, self.evaluate)
        self.get_logger().info("定位评估节点已启动, 每5秒输出一次评估结果")

    def _odom_to_tuple(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        return (stamp, p.x, p.y, p.z, q.w, q.x, q.y, q.z)

    def gt_callback(self, msg: Odometry):
        self.gt_trajectory.append(self._odom_to_tuple(msg))

    def est_callback(self, msg: Odometry):
        self.est_trajectory.append(self._odom_to_tuple(msg))

    def evaluate(self):
        if len(self.gt_trajectory) < 10 or len(self.est_trajectory) < 10:
            self.get_logger().info("等待足够数据...")
            return

        gt = np.array(self.gt_trajectory)
        est = np.array(self.est_trajectory)

        ate_rmse, ate_mean, ate_std = compute_ate(gt, est)

        self.get_logger().info(
            f"[评估] ATE RMSE: {ate_rmse:.4f} m, "
            f"Mean: {ate_mean:.4f} m, Std: {ate_std:.4f} m"
        )


# ============================================================
# 核心评估函数
# ============================================================

def align_timestamps(gt, est, max_dt=0.1):
    """时间戳对齐: 对每个 est 找最近的 gt"""
    gt_stamps = gt[:, 0]
    est_stamps = est[:, 0]

    aligned_gt = []
    aligned_est = []

    for est_idx, est_t in enumerate(est_stamps):
        dt = np.abs(gt_stamps - est_t)
        min_idx = np.argmin(dt)
        if dt[min_idx] < max_dt:
            aligned_gt.append(gt[min_idx])
            aligned_est.append(est[est_idx])

    return np.array(aligned_gt), np.array(aligned_est)


def compute_ate(gt, est):
    """计算 ATE (Absolute Trajectory Error)"""
    gt_a, est_a = align_timestamps(gt, est)

    gt_pos = gt_a[:, 1:4]
    est_pos = est_a[:, 1:4]

    # 对齐 (最小二乘相似变换对齐)
    if HAS_SCIPY:
        gt_centered = gt_pos - gt_pos.mean(axis=0)
        est_centered = est_pos - est_pos.mean(axis=0)

        H = est_centered.T @ gt_centered
        U, _, Vt = np.linalg.svd(H)
        R = U @ Vt
        if np.linalg.det(R) < 0:
            U[:, -1] *= -1
            R = U @ Vt

        t = gt_pos.mean(axis=0) - R @ est_pos.mean(axis=0)
        est_aligned = est_pos @ R.T + t
    else:
        est_aligned = est_pos

    errors = np.linalg.norm(gt_pos - est_aligned, axis=1)
    rmse = np.sqrt(np.mean(errors ** 2))
    mean = np.mean(errors)
    std = np.std(errors)
    return rmse, mean, std


def compute_rpe(gt, est, delta=1.0, delta_unit="m"):
    """计算 RPE (Relative Pose Error)"""
    gt_a, est_a = align_timestamps(gt, est)

    trans_errors = []
    rot_errors = []

    for i in range(len(gt_a) - 1):
        gt_dt = gt_a[i + 1, 0] - gt_a[i, 0]
        if delta_unit == "m":
            gt_d = np.linalg.norm(gt_a[i + 1, 1:4] - gt_a[i, 1:4])
            if gt_d < delta:
                continue

        # 相对位姿
        gt_rel = (
            gt_a[i + 1, 1:4] - gt_a[i, 1:4],
            gt_a[i + 1, 4:8] - gt_a[i, 4:8],
        )
        est_rel = (
            est_a[i + 1, 1:4] - est_a[i, 1:4],
            est_a[i + 1, 4:8] - est_a[i, 4:8],
        )

        trans_error = np.linalg.norm(gt_rel[0] - est_rel[0])
        trans_errors.append(trans_error)

        if HAS_SCIPY:
            r_gt = Rotation.from_quat(gt_a[i + 1, 5:8].tolist() +
                                       [gt_a[i + 1, 4]])
            r_est = Rotation.from_quat(est_a[i + 1, 5:8].tolist() +
                                        [est_a[i + 1, 4]])
            r_error = (r_gt * r_est.inv()).magnitude()
            rot_errors.append(r_error)

    rpe_trans = np.mean(trans_errors) if trans_errors else 0.0
    rpe_rot = np.mean(rot_errors) if rot_errors else 0.0
    return rpe_trans, rpe_rot


# ============================================================
# 可视化
# ============================================================

def plot_trajectories(gt, est, output_path="trajectory_comparison.png"):
    """绘制轨迹对比图和误差图"""
    gt_a, est_a = align_timestamps(gt, est)

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    ax = axes[0]
    ax.plot(gt_a[:, 1], gt_a[:, 2], "g-", linewidth=2, label="Ground Truth")
    ax.plot(est_a[:, 1], est_a[:, 2], "r--", linewidth=2, label="EKF Estimate")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("轨迹对比")
    ax.legend()
    ax.grid(True)
    ax.axis("equal")

    ax = axes[1]
    errors = np.linalg.norm(gt_a[:, 1:4] - est_a[:, 1:4], axis=1)
    times = est_a[:, 0] - est_a[0, 0]
    ax.plot(times, errors, "b-", linewidth=1)
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("ATE (m)")
    ax.set_title(f"ATE 时间序列 (RMSE: {np.sqrt(np.mean(errors**2)):.3f} m)")
    ax.grid(True)

    ax = axes[2]
    scatter = ax.scatter(
        est_a[:, 1], est_a[:, 2], c=errors, cmap="jet", s=10, alpha=0.8
    )
    plt.colorbar(scatter, ax=ax, label="ATE (m)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("误差空间分布")
    ax.axis("equal")
    ax.grid(True)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"[可视化] 轨迹图已保存: {output_path}")


# ============================================================
# Bag 离线分析
# ============================================================

def analyze_bag(bag_path, output_dir="."):
    """从 ROS2 bag 提取轨迹并评估"""
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    except ImportError:
        print("错误: 需要 rosbag2_py. 请安装 ros-jazzy-rosbag2-py")
        sys.exit(1)

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    gt_topics = ["/carla/ground_truth"]
    est_topics = ["/odometry/filtered"]

    gt = []
    est = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic in gt_topics:
            gt.append(parse_odom(data))
        elif topic in est_topics:
            est.append(parse_odom(data))

    gt = np.array(gt)
    est = np.array(est)

    if len(gt) < 10 or len(est) < 10:
        print("错误: 数据点不足")
        return

    # 计算 ATE
    ate_rmse, ate_mean, ate_std = compute_ate(gt, est)
    rpe_trans, rpe_rot = compute_rpe(gt, est, delta=1.0)

    # 输出报告
    report_path = os.path.join(output_dir, "evaluation_report.txt")
    with open(report_path, "w", encoding="utf-8") as f:
        f.write("════════════════════════════════════════\n")
        f.write("  定位精度评估报告\n")
        f.write("════════════════════════════════════════\n")
        f.write(f"  数据文件: {bag_path}\n")
        duration = gt[-1, 0] - gt[0, 0]
        distance = np.sum(np.linalg.norm(np.diff(gt[:, 1:4], axis=0), axis=1))
        f.write(f"  时长: {duration:.1f} s\n")
        f.write(f"  里程: {distance:.1f} m\n")
        f.write(f"  真值帧数: {len(gt)}\n")
        f.write(f"  估计帧数: {len(est)}\n")
        f.write(f"  对齐帧数: {len(gt)}\n")
        f.write(f"  ATE RMSE: {ate_rmse:.4f} m\n")
        f.write(f"  ATE mean: {ate_mean:.4f} m\n")
        f.write(f"  ATE std:  {ate_std:.4f} m\n")
        f.write(f"  ATE min:  {np.min(np.linalg.norm(gt[:,1:4]-est[:,1:4], axis=1)):.4f} m\n")
        f.write(f"  ATE max:  {np.max(np.linalg.norm(gt[:,1:4]-est[:,1:4], axis=1)):.4f} m\n")
        f.write(f"  RPE trans: {rpe_trans:.4f} m/m\n")
        f.write(f"  RPE rot:   {rpe_rot:.4f} deg/m\n")
        f.write("════════════════════════════════════════\n")

    print(f"[报告] 已保存: {report_path}")
    print(f"  ATE RMSE: {ate_rmse:.4f} m")
    print(f"  RPE trans: {rpe_trans:.4f} m/m")

    # 绘图
    plot_trajectories(gt, est, os.path.join(output_dir, "trajectory_comparison.png"))

    # 误差分布图
    plot_error_distribution(gt, est, output_dir)


def parse_odom(serialized_msg):
    """从序列化消息解析 Odometry (简化版)"""
    import struct
    data = bytes(serialized_msg)
    stamp_ns = struct.unpack("<I", data[4:8])[0]
    stamp_sec = struct.unpack("<I", data[0:4])[0]
    t = stamp_sec + stamp_ns / 1e9

    x = struct.unpack("<d", data[16:24])[0]
    y = struct.unpack("<d", data[24:32])[0]
    z = struct.unpack("<d", data[32:40])[0]
    qx = struct.unpack("<d", data[40:48])[0]
    qy = struct.unpack("<d", data[48:56])[0]
    qz = struct.unpack("<d", data[56:64])[0]
    qw = struct.unpack("<d", data[64:72])[0]
    return (t, x, y, z, qw, qx, qy, qz)


# ============================================================
# 入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="定位精度评估工具")
    parser.add_argument("--bag", type=str, help="ROS2 bag 路径 (离线模式)")
    parser.add_argument("--output", type=str, default=".",
                        help="输出目录 (默认当前目录)")
    args = parser.parse_args()

    if args.bag:
        analyze_bag(args.bag, args.output)
    else:
        rclpy.init()
        node = LocalizationEvalNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
