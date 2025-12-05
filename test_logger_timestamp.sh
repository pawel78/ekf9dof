#!/bin/bash
# Test script to demonstrate timestamped filename generation

echo "=== EKF9DOF Data Logger - Filename Test ==="
echo ""
echo "Testing timestamped filename generation..."
echo ""

# Show current directory structure
echo "Directory structure before:"
ls -lh data/logs/ 2>/dev/null || echo "  data/logs/ doesn't exist yet"
echo ""

# Run logger for 2 seconds to create a file
echo "Starting logger for 2 seconds..."
timeout 2 ./build/imu_logger 2>&1 | grep "Output file:" &
LOGGER_PID=$!

# Wait for logger to start and create file
sleep 3

echo ""
echo "Directory structure after:"
ls -lh data/logs/ 2>/dev/null
echo ""

# Show latest log file
LATEST_LOG=$(ls -t data/logs/imu_log_*.bin 2>/dev/null | head -1)
if [ -n "$LATEST_LOG" ]; then
    echo "Latest log file created:"
    echo "  $LATEST_LOG"
    echo "  Size: $(du -h "$LATEST_LOG" | cut -f1)"
    echo ""
    echo "Filename format: imu_log_YYYYMMDD_HHMMSS_mmm.bin"
    echo ""
    
    # Demonstrate CSV conversion with timestamp preservation
    echo "Testing CSV conversion with timestamp preservation..."
    python3 tools/log2csv.py "$LATEST_LOG" 2>&1 | head -10
    echo ""
    
    echo "CSV files created:"
    basename "$LATEST_LOG" .bin | xargs -I {} ls -1 {}_*.csv 2>/dev/null
fi

echo ""
echo "Test complete!"
