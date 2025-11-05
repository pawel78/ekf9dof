#!/usr/bin/env python3
"""
Stereo camera calibration using OpenCV
"""

import argparse
import cv2
import numpy as np
import yaml
import glob
from pathlib import Path


def calibrate_stereo(left_images, right_images, pattern_size, square_size, output_dir):
    """
    Calibrate stereo camera pair
    
    Args:
        left_images: List of left camera image paths
        right_images: List of right camera image paths
        pattern_size: Chessboard pattern size (cols, rows)
        square_size: Size of each square in meters
        output_dir: Directory to save calibration results
    """
    # Prepare object points
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world space
    imgpoints_left = []  # 2D points in left image plane
    imgpoints_right = []  # 2D points in right image plane
    
    # Find chessboard corners
    for left_path, right_path in zip(left_images, right_images):
        img_left = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
        img_right = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)
        
        if img_left is None or img_right is None:
            print(f"Failed to load: {left_path} or {right_path}")
            continue
        
        # Find corners
        ret_left, corners_left = cv2.findChessboardCorners(img_left, pattern_size, None)
        ret_right, corners_right = cv2.findChessboardCorners(img_right, pattern_size, None)
        
        if ret_left and ret_right:
            objpoints.append(objp)
            
            # Refine corners
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_left = cv2.cornerSubPix(img_left, corners_left, (11, 11), (-1, -1), criteria)
            corners_right = cv2.cornerSubPix(img_right, corners_right, (11, 11), (-1, -1), criteria)
            
            imgpoints_left.append(corners_left)
            imgpoints_right.append(corners_right)
            
            print(f"✓ Found pattern in: {Path(left_path).name}")
        else:
            print(f"✗ Pattern not found in: {Path(left_path).name}")
    
    if len(objpoints) < 10:
        print(f"Error: Only found {len(objpoints)} valid image pairs. Need at least 10.")
        return False
    
    print(f"\nCalibrating with {len(objpoints)} image pairs...")
    
    # Get image size
    img_size = img_left.shape[::-1]
    
    # Calibrate individual cameras
    ret_left, K_left, D_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
        objpoints, imgpoints_left, img_size, None, None)
    
    ret_right, K_right, D_right, rvecs_right, tvecs_right = cv2.calibrateCamera(
        objpoints, imgpoints_right, img_size, None, None)
    
    print(f"Left camera RMS error: {ret_left:.4f}")
    print(f"Right camera RMS error: {ret_right:.4f}")
    
    # Stereo calibration
    flags = cv2.CALIB_FIX_INTRINSIC
    criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
    
    ret_stereo, K_left, D_left, K_right, D_right, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right,
        K_left, D_left, K_right, D_right,
        img_size, criteria=criteria_stereo, flags=flags)
    
    print(f"Stereo calibration RMS error: {ret_stereo:.4f}")
    print(f"Baseline: {np.linalg.norm(T):.4f} m")
    
    # Save calibration
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Save left camera info
    left_data = {
        'image_width': int(img_size[0]),
        'image_height': int(img_size[1]),
        'camera_name': 'left_camera',
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': K_left.flatten().tolist()
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1,
            'cols': 5,
            'data': D_left.flatten().tolist()
        },
    }
    
    with open(output_dir / 'camera_left.yaml', 'w') as f:
        yaml.dump(left_data, f)
    
    # Save right camera info
    right_data = {
        'image_width': int(img_size[0]),
        'image_height': int(img_size[1]),
        'camera_name': 'right_camera',
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': K_right.flatten().tolist()
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1,
            'cols': 5,
            'data': D_right.flatten().tolist()
        },
    }
    
    with open(output_dir / 'camera_right.yaml', 'w') as f:
        yaml.dump(right_data, f)
    
    # Save stereo extrinsics
    stereo_data = {
        'R': R.tolist(),
        'T': T.flatten().tolist(),
        'E': E.tolist(),
        'F': F.tolist(),
        'baseline': float(np.linalg.norm(T))
    }
    
    with open(output_dir / 'stereo_extrinsics.yaml', 'w') as f:
        yaml.dump(stereo_data, f)
    
    print(f"\nCalibration saved to: {output_dir}")
    return True


def main():
    parser = argparse.ArgumentParser(description='Stereo camera calibration')
    parser.add_argument('--left', required=True, help='Pattern for left images (e.g., "left_*.png")')
    parser.add_argument('--right', required=True, help='Pattern for right images (e.g., "right_*.png")')
    parser.add_argument('--pattern-cols', type=int, default=9, help='Chessboard columns')
    parser.add_argument('--pattern-rows', type=int, default=6, help='Chessboard rows')
    parser.add_argument('--square-size', type=float, default=0.025, help='Square size in meters')
    parser.add_argument('--output', default='./calibration', help='Output directory')
    
    args = parser.parse_args()
    
    # Find matching image pairs
    left_images = sorted(glob.glob(args.left))
    right_images = sorted(glob.glob(args.right))
    
    if len(left_images) != len(right_images):
        print(f"Error: Number of left ({len(left_images)}) and right ({len(right_images)}) images don't match")
        return 1
    
    if len(left_images) == 0:
        print(f"Error: No images found matching patterns")
        return 1
    
    print(f"Found {len(left_images)} image pairs")
    
    success = calibrate_stereo(
        left_images, right_images,
        (args.pattern_cols, args.pattern_rows),
        args.square_size,
        args.output
    )
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())
