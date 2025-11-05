#!/usr/bin/env python3
"""
Evaluate visual odometry trajectory using KITTI-like metrics (ATE, RPE)
Based on TUM RGB-D evaluation tools
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import yaml


def load_trajectory_from_bag(bag_path, topic='/vo/odom'):
    """Load trajectory from ROS 2 bag"""
    try:
        from rosbags.rosbag2 import Reader
        from rosbags.serde import deserialize_cdr
    except ImportError:
        print("Error: rosbags library not found. Install with: pip install rosbags")
        return None
    
    timestamps = []
    positions = []
    orientations = []
    
    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                
                # Extract pose
                pos = msg.pose.pose.position
                ori = msg.pose.pose.orientation
                
                timestamps.append(timestamp * 1e-9)  # Convert to seconds
                positions.append([pos.x, pos.y, pos.z])
                orientations.append([ori.x, ori.y, ori.z, ori.w])
    
    return np.array(timestamps), np.array(positions), np.array(orientations)


def load_trajectory_from_csv(csv_path):
    """Load trajectory from CSV file (timestamp, x, y, z, qx, qy, qz, qw)"""
    data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    timestamps = data[:, 0]
    positions = data[:, 1:4]
    orientations = data[:, 4:8]
    return timestamps, positions, orientations


def align_trajectories(stamps1, poses1, stamps2, poses2, max_diff=0.02):
    """Align two trajectories by timestamp"""
    matches = []
    for i, t1 in enumerate(stamps1):
        diffs = np.abs(stamps2 - t1)
        j = np.argmin(diffs)
        if diffs[j] < max_diff:
            matches.append((i, j))
    
    if len(matches) < 10:
        print(f"Warning: Only {len(matches)} matching timestamps found")
    
    indices1, indices2 = zip(*matches)
    return np.array(indices1), np.array(indices2)


def compute_ate(positions_est, positions_gt):
    """Compute Absolute Trajectory Error (ATE)"""
    errors = np.linalg.norm(positions_est - positions_gt, axis=1)
    return {
        'rmse': np.sqrt(np.mean(errors**2)),
        'mean': np.mean(errors),
        'median': np.median(errors),
        'std': np.std(errors),
        'min': np.min(errors),
        'max': np.max(errors),
        'errors': errors
    }


def compute_rpe(positions_est, positions_gt, delta=1):
    """Compute Relative Pose Error (RPE)"""
    errors = []
    
    for i in range(len(positions_est) - delta):
        # Relative motion in estimated trajectory
        rel_est = positions_est[i + delta] - positions_est[i]
        
        # Relative motion in ground truth
        rel_gt = positions_gt[i + delta] - positions_gt[i]
        
        # Error
        error = np.linalg.norm(rel_est - rel_gt)
        errors.append(error)
    
    errors = np.array(errors)
    return {
        'rmse': np.sqrt(np.mean(errors**2)),
        'mean': np.mean(errors),
        'median': np.median(errors),
        'std': np.std(errors),
        'errors': errors
    }


def compute_drift_rate(positions, distance_traveled):
    """Compute drift as percentage of distance traveled"""
    start_to_end_error = np.linalg.norm(positions[-1] - positions[0])
    drift_percentage = (start_to_end_error / distance_traveled) * 100
    return drift_percentage, start_to_end_error


def compute_distance_traveled(positions):
    """Compute total distance traveled"""
    diffs = np.diff(positions, axis=0)
    distances = np.linalg.norm(diffs, axis=1)
    return np.sum(distances)


def plot_trajectories(positions_est, positions_gt, output_path):
    """Plot 2D trajectories"""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # XY plot
    axes[0].plot(positions_gt[:, 0], positions_gt[:, 1], 'b-', label='Ground Truth', linewidth=2)
    axes[0].plot(positions_est[:, 0], positions_est[:, 1], 'r--', label='Estimated', linewidth=2)
    axes[0].set_xlabel('X (m)')
    axes[0].set_ylabel('Y (m)')
    axes[0].set_title('Trajectory (Top View)')
    axes[0].legend()
    axes[0].grid(True)
    axes[0].axis('equal')
    
    # XZ plot
    axes[1].plot(positions_gt[:, 0], positions_gt[:, 2], 'b-', label='Ground Truth', linewidth=2)
    axes[1].plot(positions_est[:, 0], positions_est[:, 2], 'r--', label='Estimated', linewidth=2)
    axes[1].set_xlabel('X (m)')
    axes[1].set_ylabel('Z (m)')
    axes[1].set_title('Trajectory (Side View)')
    axes[1].legend()
    axes[1].grid(True)
    axes[1].axis('equal')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"Saved trajectory plot to {output_path}")


def plot_errors(ate_errors, rpe_errors, output_path):
    """Plot error distributions"""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # ATE over time
    axes[0, 0].plot(ate_errors, 'b-', linewidth=1)
    axes[0, 0].set_xlabel('Frame')
    axes[0, 0].set_ylabel('Error (m)')
    axes[0, 0].set_title('Absolute Trajectory Error (ATE)')
    axes[0, 0].grid(True)
    
    # ATE histogram
    axes[0, 1].hist(ate_errors, bins=50, color='b', alpha=0.7)
    axes[0, 1].set_xlabel('Error (m)')
    axes[0, 1].set_ylabel('Frequency')
    axes[0, 1].set_title('ATE Distribution')
    axes[0, 1].grid(True)
    
    # RPE over time
    axes[1, 0].plot(rpe_errors, 'r-', linewidth=1)
    axes[1, 0].set_xlabel('Frame')
    axes[1, 0].set_ylabel('Error (m)')
    axes[1, 0].set_title('Relative Pose Error (RPE)')
    axes[1, 0].grid(True)
    
    # RPE histogram
    axes[1, 1].hist(rpe_errors, bins=50, color='r', alpha=0.7)
    axes[1, 1].set_xlabel('Error (m)')
    axes[1, 1].set_ylabel('Frequency')
    axes[1, 1].set_title('RPE Distribution')
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"Saved error plot to {output_path}")


def save_results(results, output_path):
    """Save results to YAML and CSV"""
    # Save YAML summary
    yaml_path = Path(output_path) / 'metrics.yaml'
    with open(yaml_path, 'w') as f:
        yaml.dump(results, f, default_flow_style=False)
    print(f"Saved metrics to {yaml_path}")
    
    # Save CSV
    csv_path = Path(output_path) / 'errors.csv'
    with open(csv_path, 'w') as f:
        f.write("frame,ate_error,rpe_error\n")
        for i, (ate, rpe) in enumerate(zip(results['ate']['errors'], 
                                           results['rpe']['errors'] + [0])):  # Pad RPE
            f.write(f"{i},{ate},{rpe}\n")
    print(f"Saved errors to {csv_path}")


def main():
    parser = argparse.ArgumentParser(description='Evaluate VO trajectory')
    parser.add_argument('--estimated', required=True, help='Estimated trajectory (bag or CSV)')
    parser.add_argument('--ground-truth', required=True, help='Ground truth trajectory (bag or CSV)')
    parser.add_argument('--output', default='./evaluation', help='Output directory')
    parser.add_argument('--max-time-diff', type=float, default=0.02, help='Max timestamp difference (s)')
    parser.add_argument('--rpe-delta', type=int, default=1, help='RPE delta frames')
    parser.add_argument('--topic', default='/vo/odom', help='Topic name for bag files')
    
    args = parser.parse_args()
    
    # Create output directory
    output_path = Path(args.output)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Load trajectories
    print("Loading estimated trajectory...")
    if args.estimated.endswith('.db3') or Path(args.estimated).is_dir():
        stamps_est, pos_est, ori_est = load_trajectory_from_bag(args.estimated, args.topic)
    else:
        stamps_est, pos_est, ori_est = load_trajectory_from_csv(args.estimated)
    
    print("Loading ground truth trajectory...")
    if args.ground_truth.endswith('.db3') or Path(args.ground_truth).is_dir():
        stamps_gt, pos_gt, ori_gt = load_trajectory_from_bag(args.ground_truth, args.topic)
    else:
        stamps_gt, pos_gt, ori_gt = load_trajectory_from_csv(args.ground_truth)
    
    print(f"Estimated: {len(pos_est)} poses")
    print(f"Ground truth: {len(pos_gt)} poses")
    
    # Align trajectories
    print("Aligning trajectories...")
    idx_est, idx_gt = align_trajectories(stamps_est, pos_est, stamps_gt, pos_gt, args.max_time_diff)
    
    pos_est_aligned = pos_est[idx_est]
    pos_gt_aligned = pos_gt[idx_gt]
    
    print(f"Aligned: {len(pos_est_aligned)} poses")
    
    # Compute metrics
    print("\nComputing metrics...")
    ate = compute_ate(pos_est_aligned, pos_gt_aligned)
    rpe = compute_rpe(pos_est_aligned, pos_gt_aligned, args.rpe_delta)
    
    distance = compute_distance_traveled(pos_gt_aligned)
    drift_pct, drift_abs = compute_drift_rate(pos_est_aligned, distance)
    
    # Print results
    print("\n" + "="*60)
    print("EVALUATION RESULTS")
    print("="*60)
    print(f"\nTrajectory:")
    print(f"  Distance traveled: {distance:.2f} m")
    print(f"  Number of poses: {len(pos_est_aligned)}")
    print(f"\nAbsolute Trajectory Error (ATE):")
    print(f"  RMSE:   {ate['rmse']:.4f} m")
    print(f"  Mean:   {ate['mean']:.4f} m")
    print(f"  Median: {ate['median']:.4f} m")
    print(f"  Std:    {ate['std']:.4f} m")
    print(f"  Min:    {ate['min']:.4f} m")
    print(f"  Max:    {ate['max']:.4f} m")
    print(f"\nRelative Pose Error (RPE, delta={args.rpe_delta}):")
    print(f"  RMSE:   {rpe['rmse']:.4f} m")
    print(f"  Mean:   {rpe['mean']:.4f} m")
    print(f"  Median: {rpe['median']:.4f} m")
    print(f"\nDrift:")
    print(f"  Absolute: {drift_abs:.4f} m")
    print(f"  Percentage: {drift_pct:.2f}% of distance traveled")
    print("="*60)
    
    # Save results
    results = {
        'ate': {k: float(v) if not isinstance(v, np.ndarray) else v.tolist() 
                for k, v in ate.items()},
        'rpe': {k: float(v) if not isinstance(v, np.ndarray) else v.tolist() 
                for k, v in rpe.items()},
        'distance_traveled': float(distance),
        'drift_absolute': float(drift_abs),
        'drift_percentage': float(drift_pct),
        'num_poses': int(len(pos_est_aligned))
    }
    
    save_results(results, output_path)
    
    # Plot
    print("\nGenerating plots...")
    plot_trajectories(pos_est_aligned, pos_gt_aligned, output_path / 'trajectory.png')
    plot_errors(ate['errors'], rpe['errors'], output_path / 'errors.png')
    
    print(f"\nEvaluation complete! Results saved to {output_path}")
    
    return 0


if __name__ == '__main__':
    exit(main())
