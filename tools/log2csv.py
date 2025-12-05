#!/usr/bin/env python3
"""
Convert EKF9DOF binary log files to CSV format for analysis.

Binary format:
- Header: "EKF9DOF_LOG_V1" (15 bytes including null terminator)
- Records: [msg_type:uint8][timestamp_ns:uint64][data:variable]
  - Type 1 (Gyro): [x:float32][y:float32][z:float32]
  - Type 2 (Accel): [x:float32][y:float32][z:float32]
  - Type 3 (Mag): [x:float32][y:float32][z:float32]
  - Type 4 (Temp): [temp:float32]
"""

import struct
import sys
import csv
from pathlib import Path

MSG_TYPE_GYRO = 1
MSG_TYPE_ACCEL = 2
MSG_TYPE_MAG = 3
MSG_TYPE_TEMP = 4

MSG_NAMES = {
    MSG_TYPE_GYRO: "gyro",
    MSG_TYPE_ACCEL: "accel",
    MSG_TYPE_MAG: "mag",
    MSG_TYPE_TEMP: "temp"
}

def read_header(f):
    """Read and validate file header."""
    header = f.read(15)
    expected = b"EKF9DOF_LOG_V1\x00"
    if header != expected:
        raise ValueError(f"Invalid file header: {header}")
    print(f"File header validated: EKF9DOF_LOG_V1")

def read_record(f):
    """Read one record from binary file."""
    # Read message type
    type_bytes = f.read(1)
    if len(type_bytes) == 0:
        return None  # EOF
    
    msg_type = struct.unpack('B', type_bytes)[0]
    
    # Read timestamp
    timestamp_bytes = f.read(8)
    if len(timestamp_bytes) != 8:
        raise ValueError("Truncated timestamp")
    timestamp_ns = struct.unpack('<Q', timestamp_bytes)[0]
    
    # Read data based on message type
    if msg_type in [MSG_TYPE_GYRO, MSG_TYPE_ACCEL, MSG_TYPE_MAG]:
        # 3-axis data
        data_bytes = f.read(12)
        if len(data_bytes) != 12:
            raise ValueError("Truncated 3-axis data")
        x, y, z = struct.unpack('<fff', data_bytes)
        return {
            'type': msg_type,
            'timestamp_ns': timestamp_ns,
            'x': x,
            'y': y,
            'z': z
        }
    elif msg_type == MSG_TYPE_TEMP:
        # Temperature data
        data_bytes = f.read(4)
        if len(data_bytes) != 4:
            raise ValueError("Truncated temperature data")
        temp = struct.unpack('<f', data_bytes)[0]
        return {
            'type': msg_type,
            'timestamp_ns': timestamp_ns,
            'temp': temp
        }
    else:
        raise ValueError(f"Unknown message type: {msg_type}")

def convert_to_csv(input_file, output_prefix=None):
    """Convert binary log to separate CSV files per sensor type."""
    if output_prefix is None:
        # Extract timestamp from filename if present
        # Format: imu_log_YYYYMMDD_HHMMSS_mmm.bin
        stem = Path(input_file).stem
        if stem.startswith('imu_log_'):
            # Use the same timestamp for CSV files
            output_prefix = stem
        else:
            output_prefix = stem
    
    # Open input file
    with open(input_file, 'rb') as f:
        read_header(f)
        
        # Create CSV writers for each sensor type
        csv_files = {}
        csv_writers = {}
        
        try:
            record_count = 0
            type_counts = {t: 0 for t in MSG_NAMES.keys()}
            
            while True:
                record = read_record(f)
                if record is None:
                    break
                
                msg_type = record['type']
                type_counts[msg_type] += 1
                record_count += 1
                
                # Create CSV file if needed
                if msg_type not in csv_files:
                    filename = f"{output_prefix}_{MSG_NAMES[msg_type]}.csv"
                    csv_files[msg_type] = open(filename, 'w', newline='')
                    
                    if msg_type == MSG_TYPE_TEMP:
                        fieldnames = ['timestamp_ns', 'timestamp_s', 'temp_c']
                    else:
                        fieldnames = ['timestamp_ns', 'timestamp_s', 'x', 'y', 'z']
                    
                    csv_writers[msg_type] = csv.DictWriter(
                        csv_files[msg_type], 
                        fieldnames=fieldnames
                    )
                    csv_writers[msg_type].writeheader()
                    print(f"Created: {filename}")
                
                # Write record to CSV
                writer = csv_writers[msg_type]
                timestamp_s = record['timestamp_ns'] / 1e9
                
                if msg_type == MSG_TYPE_TEMP:
                    writer.writerow({
                        'timestamp_ns': record['timestamp_ns'],
                        'timestamp_s': timestamp_s,
                        'temp_c': f"{record['temp']:.2f}"
                    })
                else:
                    writer.writerow({
                        'timestamp_ns': record['timestamp_ns'],
                        'timestamp_s': timestamp_s,
                        'x': f"{record['x']:.6f}",
                        'y': f"{record['y']:.6f}",
                        'z': f"{record['z']:.6f}"
                    })
                
                if record_count % 10000 == 0:
                    print(f"Processed {record_count} records...")
        
        finally:
            # Close all CSV files
            for f in csv_files.values():
                f.close()
    
    print(f"\nConversion complete:")
    print(f"  Total records: {record_count}")
    for msg_type, count in type_counts.items():
        if count > 0:
            print(f"  {MSG_NAMES[msg_type]}: {count} records")

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <input_binary_file> [output_prefix]")
        print(f"\nConverts EKF9DOF binary log to CSV files.")
        print(f"Creates separate CSV files for each sensor type.")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_prefix = sys.argv[2] if len(sys.argv) > 2 else None
    
    if not Path(input_file).exists():
        print(f"Error: File not found: {input_file}")
        sys.exit(1)
    
    try:
        convert_to_csv(input_file, output_prefix)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
