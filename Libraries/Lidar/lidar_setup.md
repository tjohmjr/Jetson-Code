# Lidar (RPLIDAR A1M8) with ROS2: setup and first run

## 0. Check your ROS2 distro
```bash
lsb_release -a        # Ubuntu 22.04 (JetPack 6) -> ROS2 Humble
```
If it says 20.04 (JetPack 5), tell the team: Humble isn't natively supported there (use Foxy or a Humble Docker container).

## 1. Install ROS2 Humble (skip if already installed)
Follow https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html, then:
```bash
sudo apt install ros-humble-ros-base python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

## 2. Build the Slamtec driver
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/ros2_ws
colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

## 3. USB permissions
```bash
ls /dev/ttyUSB*                       # should show /dev/ttyUSB0 with lidar plugged in
sudo usermod -aG dialout $USER        # log out/in afterwards
# or: cd ~/ros2_ws/src/sllidar_ros2 && sudo bash scripts/create_udev_rules.sh
```

## 4. Launch the driver (terminal 1)
```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0
```
Check (terminal 2):
```bash
ros2 topic list                # /scan should exist
ros2 topic hz /scan            # A1M8 is ~5-10 Hz
ros2 topic echo /scan --once   # sanity check ranges
```
If the motor spins but there's no data, check the launch file's baud rate (A1 = 115200).

## 5. Run the logger (terminal 2)
Install dependencies once. Use a venv that can see ROS's Python packages:
```bash
python3 -m venv --system-site-packages ~/sme_venv && source ~/sme_venv/bin/activate
pip install pyarrow pandas
python3 lidar_logger.py --ros-args -p output_dir:=~/sme_data -p notes:="first test"
```
Ctrl+C to stop; it flushes remaining data on exit.

## 6. Verify the data
```python
import pandas as pd, glob, sqlite3, os
files = glob.glob(os.path.expanduser("~/sme_data/lidar/session=*/part-*.parquet"))
df = pd.concat(pd.read_parquet(f) for f in files)
print(len(df), df[["scan_idx", "t_wall_ns"]].head())
print(sqlite3.connect(os.path.expanduser("~/sme_data/sme.db")).execute("select * from sessions").fetchall())
```
Expect roughly 360+ points per scan (`len(df.ranges.iloc[0])`) and monotonically increasing `scan_idx`.

## Next steps
1. Confirm scan rate and dropped scans (gaps in `t_mono_ns`).
2. Add a "min distance in front" metric (Milestone 2.2) computed from the ranges.
3. Copy the pattern for the IMU (ROS2 `bno08x_driver` or the Adafruit library).
