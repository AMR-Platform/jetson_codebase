# Windows PowerShell Commands for EKF Testing

## Step 1: Build and Test (PowerShell)

```powershell
# Navigate to your project
cd "c:\Repositories\Jetson\jetson_codebase"

# Create and enter build directory
New-Item -ItemType Directory -Force -Path "build"
cd build

# Configure with CMake
cmake ..

# Build the tests
cmake --build . --config Release

# Verify build
Get-ChildItem -Name "*main_exe*", "*test_ekf*"
```

## Step 2: Run Tests

### Quick Validation
```powershell
# Test basic EKF functionality
.\Release\test_ekf_simple.exe

# Look for PASS/FAIL results in output
```

### Real Robot Tests (when on Jetson)
```powershell
# Main executable with integrated EKF test harness (run on Jetson robot)
# ./main_exe

# This will automatically detect the robot and run the complete test sequence
```

## Step 3: Check Results

```powershell
# List generated files
Get-ChildItem -Name "*.csv"

# View first few lines of log
Get-Content "ekf_test_log.csv" -Head 10

# View last few lines
Get-Content "ekf_test_log.csv" -Tail 10

# Count lines in different test phases
(Select-String -Pattern "stationary" -Path "ekf_test_log.csv").Count
(Select-String -Pattern "forward" -Path "ekf_test_log.csv").Count
```

## Step 4: Transfer to Jetson (if testing on Windows first)

```powershell
# Copy built executables to Jetson (example with SCP)
# You'll need to adapt this to your Jetson's IP and username

# Using WinSCP or command line SCP:
scp -r .\Release\test_ekf_* jetson@192.168.1.100:/home/jetson/ekf_tests/

# Or copy to USB drive:
Copy-Item ".\Release\test_ekf_*" -Destination "E:\jetson_tests\" -Recurse
```

## Step 5: Analysis Commands

```powershell
# If you have Python installed on Windows:
python -c "
import pandas as pd
data = pd.read_csv('ekf_test_log.csv')
print('Test phases:', data['test_phase'].unique())
print('Position range X:', data['x'].min(), 'to', data['x'].max())
print('Position range Y:', data['y'].min(), 'to', data['y'].max())
"

# Or view in Excel:
Start-Process "ekf_test_log.csv"
```

## For Jetson Robot (Linux commands in SSH):

Once you're connected to your Jetson robot:

```bash
# On Jetson - Build and test
cd /home/jetson/jetson_codebase
mkdir -p build && cd build
cmake ..
make -j4

# Run real hardware tests
./test_ekf_real_robot

# Monitor in real-time
tail -f ekf_test_log.csv
```

## Expected Output from Tests:

### test_ekf_simple:
```
╔══════════════════════════════════════════════╗
║           EKF VALIDATION TESTS               ║
║     Testing the issues you encountered      ║
╚══════════════════════════════════════════════╝

=== TEST 1: FREE SPINNING WHEELS ===
RESULT: ✓ PASS

=== TEST 2: NORMAL MOVEMENT ===  
RESULT: ✓ PASS

ALL TESTS COMPLETED
```

### test_ekf_real_robot (on actual robot):
```
╔════════════════════════════════════════════════════════════╗
║                    ROBOT EKF TESTING                      ║
║              Real Hardware Validation                     ║
╚════════════════════════════════════════════════════════════╝

Current Test Phase: stationary
EKF State: x=0.002 y=-0.001 θ=0.05° vx=0.01 vy=0.00 ω=0.00
Encoders: L=0.00 R=0.00 m/s | IMU: θ=0.1° | Accel: (0.02, 0.01) m/s²
```

## Troubleshooting:

### Build Errors:
```powershell
# Clean build
Remove-Item -Recurse -Force build
New-Item -ItemType Directory -Path build
cd build
cmake .. -G "Visual Studio 16 2019"
cmake --build . --config Release --verbose
```

### Missing Dependencies:
```powershell
# Install vcpkg packages if needed
vcpkg install eigen3:x64-windows
```

Remember: The real robot testing (`test_ekf_real_robot`) must be run on the actual Jetson robot with hardware connected, not on your Windows development machine.
