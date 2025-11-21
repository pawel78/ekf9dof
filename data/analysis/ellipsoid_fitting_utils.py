import numpy as np
import pandas as pd

def compute_octant_distro(x, y, z) :
    print("="*60)
    print(" DATA VARIATION ANALYSIS")
    print("="*60)

    # 1. Data Range Check
    print("\n1. DATA RANGE CHECK:")
    print(f"   X: [{x.min():.6f}, {x.max():.6f}]")
    print(f"      Range: {x.max()-x.min():.6f}")
    print(f"      Std:   {x.std():.6f}")
    print(f"   Y: [{y.min():.6f}, {y.max():.6f}]")
    print(f"      Range: {y.max()-y.min():.6f}")
    print(f"      Std:   {y.std():.6f}")
    print(f"   Z: [{z.min():.6f}, {z.max():.6f}]")
    print(f"      Range: {z.max()-z.min():.6f}")
    print(f"      Std:   {z.std():.6f}")

    # 2. Magnitude Analysis
    mag = np.sqrt(x**2 + y**2 + z**2)
    print("\n2. MAGNITUDE ANALYSIS:")
    print(f"   Mean:   {mag.mean():.6f}")
    print(f"   Std:    {mag.std():.6f}")
    print(f"   Range:  [{mag.min():.6f}, {mag.max():.6f}]")
    print(f"   CV:     {(mag.std()/mag.mean())*100:.2f}%")

    # 3. Coverage Check (should cover sphere uniformly)
    print("\n3. SPHERICAL COVERAGE CHECK:")
    # Normalize to unit sphere
    x_norm = x / mag
    y_norm = y / mag
    z_norm = z / mag

    # Check octant distribution (should be roughly equal)
    octants = np.zeros(8)
    for i in range(len(x)):
        octant = (0 if x_norm.iloc[i] >= 0 else 4) + \
                 (0 if y_norm.iloc[i] >= 0 else 2) + \
                 (0 if z_norm.iloc[i] >= 0 else 1)
        octants[octant] += 1

    print(f"   Octant distribution (should be balanced):")
    for i in range(8):
        pct = (octants[i] / len(x)) * 100
        bar = '█' * int(pct / 2)
        print(f"      Octant {i}: {int(octants[i]):5d} samples ({pct:5.1f}%) {bar}")

    # 4. Data Quality Assessment
    print("\n4. DATA QUALITY ASSESSMENT:")
    min_range = min(x.max()-x.min(), y.max()-y.min(), z.max()-z.min())
    max_range = max(x.max()-x.min(), y.max()-y.min(), z.max()-z.min())
    range_ratio = min_range / max_range if max_range > 0 else 0

    print(f"   Range balance ratio: {range_ratio:.3f}")
    if range_ratio < 0.5:
        print(f"   ⚠️  WARNING: Data is not well distributed in 3D space")
        print(f"      One or more axes have limited variation")
    else:
        print(f"   ✓ Data appears well distributed")

    min_octant_pct = (octants.min() / len(x)) * 100
    if min_octant_pct < 5:
        print(f"   ⚠️  WARNING: Poor spherical coverage")
        print(f"      Minimum octant coverage: {min_octant_pct:.1f}% (should be >10%)")
    else:
        print(f"   ✓ Good spherical coverage")

    cv_threshold = 5.0  # 5% coefficient of variation
    if (mag.std()/mag.mean())*100 < cv_threshold:
        print(f"   ⚠️  WARNING: Low magnitude variation ({(mag.std()/mag.mean())*100:.2f}%)")
        print(f"      Data may not form a proper ellipsoid")
    else:
        print(f"   ✓ Sufficient magnitude variation")

    # 5. Recommendation
    print("\n5. RECOMMENDATION:")
    if range_ratio >= 0.5 and min_octant_pct >= 5 and (mag.std()/mag.mean())*100 >= cv_threshold:
        print("   ✅ Data quality is GOOD for ellipsoid fitting")
    else:
        print("   ❌ Data quality is POOR - need to re-collect data:")
        print("      • Rotate sensor through ALL orientations")
        print("      • Ensure full 360° rotation on all 3 axes")
        print("      • Move slowly and steadily")
        print("      • Collect data for at least 30-60 seconds")

def extract_octant_consistent_data(x, y, z):
    mag = np.sqrt(x**2 + y**2 + z**2)
    x_norm = x / mag
    y_norm = y / mag
    z_norm = z / mag
    
    octant_labels = np.zeros(len(x), dtype=int)
    for i in range(len(x)):
        octant = (0 if x_norm.iloc[i] >= 0 else 4) + \
                 (0 if y_norm.iloc[i] >= 0 else 2) + \
                 (0 if z_norm.iloc[i] >= 0 else 1)
        octant_labels[i] = octant
    
    df = pd.DataFrame({
        'x': x.values,
        'y': y.values,
        'z': z.values,
        'octant': octant_labels
    })
    
    octant_counts = np.bincount(octant_labels, minlength=8)
    print("Original octant distribution:")
    for i in range(8):
        print(f"  Octant {i}: {octant_counts[i]} samples ({100*octant_counts[i]/len(df):.1f}%)")
    
    non_zero_counts = octant_counts[octant_counts > 0]
    target_count = int(np.min(non_zero_counts))
    print(f"\nTarget count per octant: {target_count} samples")
    
    balanced_indices = []
    for octant in range(8):
        octant_indices = df[df['octant'] == octant].index.tolist()
        if len(octant_indices) > target_count:
            sampled = np.random.choice(octant_indices, size=target_count, replace=False)
            balanced_indices.extend(sampled)
            print(f"  Octant {octant}: Downsampled from {len(octant_indices)} to {target_count}")
        else:
            balanced_indices.extend(octant_indices)
            print(f"  Octant {octant}: Kept all {len(octant_indices)} samples")
    
    df_balanced = df.loc[balanced_indices].copy()
    df_balanced = df_balanced.sort_index()
    
    print(f"\n✓ Balanced dataset created:")
    print(f"  Original: {len(df)} samples")
    print(f"  Balanced: {len(df_balanced)} samples")
    print(f"  Removed:  {len(df) - len(df_balanced)} samples")
    
    balanced_octant_counts = np.bincount(df_balanced['octant'], minlength=8)
    print("\nBalanced octant distribution:")
    for i in range(8):
        pct = 100 * balanced_octant_counts[i] / len(df_balanced)
        bar = '█' * int(pct / 2)
        print(f"  Octant {i}: {balanced_octant_counts[i]:3d} samples ({pct:5.1f}%) {bar}")
    
    return df_balanced

def fit_ellipsoid(x, y, z):
    # Ellipsoid fitting on BALANCED accelerometer data

    print("="*70)
    print("DATA ELLIPSOID FITTING")
    print("="*70)
    print(f"\nFitting ellipsoid to {len(x)} balanced samples\n")

    # Design matrix
    D = np.column_stack([
    x*x,
    y*y,
    z*z,
    2*x*y,
    2*x*z,
    2*y*z,
    2*x,
    2*y,
    2*z,
    np.ones(len(x))
    ])

    # Solve using SVD
    U, s, Vt = np.linalg.svd(D, full_matrices=False   )
    v = Vt[-1, :]

    print("Ellipsoid coefficients:")
    print(f"  A = {v[0]:.6e}")
    print(f"  B = {v[1]:.6e}")
    print(f"  C = {v[2]:.6e}")

    # Build 4x4 matrix
    A_4x4 = np.array([
        [v[0], v[3], v[4], v[6]],
        [v[3], v[1], v[5], v[7]],
        [v[4], v[5], v[2], v[8]],
        [v[6], v[7], v[8], v[9]]
    ])

    # Extract 3x3 quadratic part
    A_3x3 = A_4x4[:3, :3]
    print(f"\n3x3 Matrix rank: {np.linalg.matrix_rank(A_3x3)}")
    print(f"3x3 Determinant: {np.linalg.det(A_3x3):.6e}")

    # Check eigenvalues
    eigvals_check, _ = np.linalg.eig(A_3x3)
    print(f"A_3x3 eigenvalues: {eigvals_check}")

    signs = np.sign(eigvals_check)
    if np.all(signs == signs[0]) and signs[0] != 0:
        print("✓ Valid ellipsoid\n")
    else:
        print("❌ Invalid ellipsoid!\n")
    # Find center (bias/offset)
    if np.linalg.matrix_rank(A_3x3) == 3:
        center = -np.linalg.solve(A_3x3, np.array([v[6], v[7], v[8]]))
    
        print(f"✓ Accelerometer Bias (offset):")
        print(f"  x: {center[0]:.6f} g")
        print(f"  y: {center[1]:.6f} g")
        print(f"  z: {center[2]:.6f} g")
    
        # Translate to center
        T = np.eye(4)
        T[3, :3] = center
        A_centered = T @ A_4x4 @ T.T
    
        A_3x3_c = A_centered[:3, :3]
        k = -A_centered[3, 3]
    
        print(f"\nNormalization constant k: {k:.6e}")
    
        if abs(k) > 1e-10:
            A_norm = A_3x3_c / k
            evals, evecs = np.linalg.eig(A_norm)
        
            print(f"Normalized matrix eigenvalues: {evals}")
        
            if np.all(evals > 0):
                radii = 1.0 / np.sqrt(evals)
            
                print(f"\n✓ Ellipsoid Radii:")
                print(f"  r1: {radii[0]:.6f} g")
                print(f"  r2: {radii[1]:.6f} g")
                print(f"  r3: {radii[2]:.6f} g")
                print(f"  Mean: {radii.mean():.6f} g")
            
                # Soft iron matrix (scale/rotation correction)
                inv_sqrt_evals = 1.0 / np.sqrt(evals)
                W_inv = evecs @ np.diag(inv_sqrt_evals) @ evecs.T
            
                print(f"\n✓ Scale/Rotation Matrix:")
                print(W_inv)
            
                # Apply calibration to balanced accelerometer data
                a_raw_all = np.column_stack([x, y, z])
                a_cal_all = np.array([W_inv @ (a - center) * radii.mean() for a in a_raw_all])
            
                mag_raw = np.linalg.norm(a_raw_all, axis=1)
                mag_cal = np.linalg.norm(a_cal_all, axis=1)
            
                print(f"\n" + "="*70)
                print("CALIBRATION RESULTS")
                print("="*70)
                print(f"  Raw magnitude:")
                print(f"    Mean: {mag_raw.mean():.6f}")
                print(f"    Std:  {mag_raw.std():.6f}")
                print(f"    CV:   {100*mag_raw.std()/mag_raw.mean():.2f}%")
                print(f"    Deviation from 1g: {np.abs(mag_raw.mean() - 1.0):.6f} ({100*np.abs(mag_raw.mean() - 1.0):.2f}%)")

                # Extract final calibration parameters for C++ implementation
                print("="*70)
                print("CALIBRATION PARAMETERS - C++ FORMAT")
                print("="*70)

                print(f"\n// Bias (offset removal)")
                print(f"constexpr float OFFSET_X = {center[0]:.6f}f;")
                print(f"constexpr float OFFSET_Y = {center[1]:.6f}f;")
                print(f"constexpr float OFFSET_Z = {center[2]:.6f}f;")

                print(f"\n// Scale/Rotation Matrix (shape correction)")
                print(f"constexpr float SCALE_MATRIX[3][3] = {{")
                for i in range(3):
                    print(f"    {{{W_inv[i,0]:11.8f}f, {W_inv[i,1]:11.8f}f, {W_inv[i,2]:11.8f}f}},")
                print(f"}};")
                print(f"\n// Expected Magnitude (1g scaling)")
                print(f"constexpr float EXPECTED_MAG = {radii.mean():.6f}f;")
            
                print(f"\n  Calibrated magnitude:")
                print(f"    Mean: {mag_cal.mean():.6f}")
                print(f"    Std:  {mag_cal.std():.6f}")
                print(f"    CV:   {100*mag_cal.std()/mag_cal.mean():.2f}%")
                print(f"    Deviation from 1g: {np.abs(mag_cal.mean() - 1.0):.6f} ({100*np.abs(mag_cal.mean() - 1.0):.2f}%)")
            
                cv_improvement_accel = (mag_raw.std()/mag_raw.mean()) / (mag_cal.std()/mag_cal.mean())
                print(f"\n  CV improvement: {cv_improvement_accel:.2f}x")
            
                if cv_improvement_accel > 1.5:
                    print(f"  ✅ Excellent calibration!")
                elif cv_improvement_accel > 1.1:
                    print(f"  ✅ Good calibration improvement!")
                elif cv_improvement_accel > 0.9:
                    print(f"  ➡️  Neutral calibration")
                else:
                    print(f"  ⚠️  Calibration degraded quality")
                
                print("\n" + "="*70)
            else:
                print("\n❌ ERROR: Some eigenvalues are negative")
        else:
            print("\n❌ ERROR: k is too small")
    else:
        print("\n❌ ERROR: Matrix is singular")        

