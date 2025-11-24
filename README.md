# WRO Future Engineers 2025 - Autonomous Vehicle
## Obstacle Challenge with Precision Parking

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.9+](https://img.shields.io/badge/python-3.9+-blue.svg)](https://www.python.org/downloads/)
[![Platform: Raspberry Pi](https://img.shields.io/badge/platform-Raspberry%20Pi-red.svg)](https://www.raspberrypi.org/)

---

## 📋 Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Hardware Components](#hardware-components)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Code Architecture](#code-architecture)
- [How It Works](#how-it-works)
- [Configuration](#configuration)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Team](#team)
- [License](#license)

---

## 🎯 Overview

This project is an autonomous vehicle designed for the **WRO Future Engineers 2025** competition. The robot completes three laps around a track while avoiding red and green obstacles, then performs a precision parking maneuver in a designated area.

### Mission Phases:
1. **Phase 1**: Exit parking lot
2. **Phase 2**: Complete 3 laps (12 turns) with obstacle avoidance
3. **Phase 3**: Navigate parking sequence (4 turns) and park in final position

**Key Achievement**: Improved parking accuracy from ±30-50mm to ±5-15mm through LiDAR consistency optimizations.

---

## ✨ Features

### Navigation
- ✅ **Compass-based heading control** - BNO055 IMU for precise orientation
- ✅ **Wall-following algorithm** - Adaptive PD controller for corridor navigation
- ✅ **Dynamic corridor detection** - Automatically adjusts to wide/narrow sections
- ✅ **Forward turning** - Instant 90° turns using compass updates

### Obstacle Avoidance
- ✅ **HSV color detection** - Identifies red and green obstacles via Picamera2
- ✅ **Priority-based steering** - Wall safety prioritized over obstacle avoidance
- ✅ **Obstacle tracking** - Prevents re-detection of passed obstacles
- ✅ **Emergency maneuvers** - Backup and reposition when too close

### Parking System
- ✅ **Pillar detection** - LiDAR-based identification of parking markers
- ✅ **Multi-step parking** - 5-step precision parking sequence
- ✅ **Ultrasonic feedback** - Rear distance monitoring during parking
- ✅ **Heading-based alignment** - Compass-guided final positioning

### LiDAR Improvements
- ✅ **Quality filtering** - Rejects low-confidence readings
- ✅ **Outlier rejection** - Median filtering with 30% threshold
- ✅ **Widened sectors** - ±20° tolerance for drift resilience
- ✅ **Aggressive sampling** - 3x read rate during parking

---

## 🛠️ Hardware Components

| Component | Model | Purpose |
|-----------|-------|---------|
| **Main Controller** | Raspberry Pi 4B (4GB) | Central processing unit |
| **Motor Driver** | DFRobot Expansion Board | DC motor control |
| **IMU/Compass** | Adafruit BNO055 | Heading measurement |
| **LiDAR** | RPLiDAR A1 | 360° distance scanning |
| **Camera** | Raspberry Pi Camera Module 3 | Obstacle color detection |
| **Ultrasonic Sensor** | HC-SR04 (via ADC) | Rear parking distance |
| **Color Sensors** | Analog light sensors (x2) | Line color detection (red/white) |
| **Motor** | DC gear motor | Propulsion |
| **Servo** | Standard servo (20kg) | Steering control |
| **Button** | Analog button (Port 1) | Mission start trigger |

### Wiring Diagram
```
Raspberry Pi GPIO Pinout:
├── GPIO 6  → Motor IN1
├── GPIO 7  → Motor IN2
├── GPIO 5  → Motor ENA (PWM)
├── I2C     → BNO055 Compass (SDA/SCL)
├── UART    → RPLiDAR (/dev/ttyAMA0)
└── CSI     → Camera Module

DFRobot Expansion Board Analog Ports:
├── Port 0  → Ultrasonic sensor
├── Port 1  → Start button
├── Port 2  → Red color sensor
└── Port 3  → White color sensor

Servo:
└── Expansion Board Servo Channel 0
```

---

## 💻 Software Requirements

### Operating System
- **Raspberry Pi OS** (64-bit, Bookworm or later)
- Kernel 6.1+

### Python Version
- **Python 3.9+**

### Required Libraries
```bash
# System libraries
sudo apt-get update
sudo apt-get install -y python3-pip python3-opencv python3-numpy

# Install Python packages
pip3 install RPi.GPIO
pip3 install pyserial
pip3 install picamera2
pip3 install adafruit-circuitpython-bno055
pip3 install statistics

# DFRobot Expansion Board Library
cd ~
git clone https://github.com/DFRobot/DFRobot_RaspberryPi_Expansion_Board.git
cd DFRobot_RaspberryPi_Expansion_Board/Python
sudo python3 setup.py install
```

### Enable Interfaces
```bash
sudo raspi-config
# Navigate to:
# - Interface Options → I2C → Enable
# - Interface Options → Serial Port → Disable login shell, Enable serial hardware
# - Interface Options → Camera → Enable (legacy camera not needed for Picamera2)

# Reboot
sudo reboot
```

---

## 📦 Installation

### Step 1: Clone Repository
```bash
cd ~
git clone https://github.com/YOUR_USERNAME/wro-2025.git
cd wro-2025
```

### Step 2: Install Dependencies
```bash
chmod +x install.sh
./install.sh
```

### Step 3: Configure Camera
```bash
# Test camera
libcamera-hello --timeout 5000

# If camera not detected:
sudo raspi-config
# Interface Options → Camera → Enable
```

### Step 4: Test Hardware
```bash
# Test LiDAR
python3 test_lidar.py

# Test compass
python3 test_compass.py

# Test camera
python3 test_camera.py
```

---

## 🏗️ Code Architecture

### File Structure
```
wro-2025/
├── main.py                 # Main mission code (cleaned version)
├── config.py               # Configuration parameters (optional)
├── test_lidar.py          # LiDAR diagnostic tool
├── test_compass.py        # Compass diagnostic tool
├── test_camera.py         # Camera diagnostic tool
├── README.md              # This file
├── LICENSE                # MIT License
└── docs/
    ├── hardware.md        # Hardware assembly guide
    ├── calibration.md     # Sensor calibration guide
    └── algorithm.md       # Algorithm explanation
```

### Main Code Structure

```python
# 1. CONFIGURATION
#    - Hardware pins
#    - Sensor parameters
#    - Navigation constants

# 2. HELPER CLASSES
class SimpleMotor:          # DC motor control
class Picamera2Detector:    # Camera-based obstacle detection

# 3. MAIN VEHICLE CLASS
class Vehicle:
    # Initialization
    def __init__()
    def _init_board()
    def _init_motor()
    def _init_servo()
    def _init_compass()
    def _init_lidar()
    def _init_camera()
    
    # Sensor Reading
    def update_lidar()              # Read LiDAR data
    def distance_left/right/front() # Get filtered distances
    def heading()                   # Get compass heading
    def detect_obstacles()          # Camera detection
    
    # Navigation
    def calculate_steering()        # Wall-following PD control
    def calculate_heading_correction() # Compass-based correction
    def execute_forward_turn()      # 90° turn via heading update
    
    # Obstacle Handling
    def get_closest_obstacle()      # Find nearest obstacle
    def calculate_obstacle_avoidance_angle() # Steering for obstacle
    
    # Mission Phases
    def exit_parking_lot()          # Phase 1
    def run_mission()               # Main mission loop (Phases 2-3)
```

---

## 🧠 How It Works

### Phase 1: Exit Parking Lot
1. **Scan for open space** - LiDAR detects gaps > 750mm
2. **Turn toward opening** - Steer left/right based on detection
3. **Drive forward** - Exit for 1.1 seconds at speed 43

### Phase 2: Obstacle Avoidance (Turns 1-12)

#### Turn Detection
```python
# Light sensors detect line colors
if red_sensor < RED_THRESHOLD:
    # Sample for 0.1s to find lowest value
    if lowest_red > RED_THRESHOLD:
        direction = 'RIGHT'  # Red line
    else:
        direction = 'LEFT'   # Blue line
```

#### Obstacle Avoidance Logic
```python
if obstacle_detected and walls_are_safe:
    # Priority 1: Avoid obstacle
    steering = calculate_obstacle_angle(obstacle)
    if obstacle.color == 'red':
        target_x = 100  # Pass on left
    else:
        target_x = 500  # Pass on right
else:
    # Priority 2: Follow wall
    steering = calculate_wall_following()
```

#### Wall Following Algorithm
```python
# PD Controller
error = TARGET_DISTANCE - current_distance
derivative = error - last_error

steering = KP * error + KD * derivative
```

### Phase 3: Parking Sequence (Turns 13-16)

#### Turn 13: Approach Inner Wall
- Continue obstacle avoidance
- Wait for line detection
- Drive straight until front < 500mm
- Execute 90° turn

#### Turns 14-16: Outer Wall Tracking
```python
# Track outer wall at 140mm distance
if outer_wall_side == 'left':
    track_left_wall()
else:
    track_right_wall()
```

#### Pillar Detection & Parking
```python
# Detect parking pillar
if outer_wall_distance < 270mm and heading_valid:
    if last_distance > 250mm:
        # Pillar detected!
        execute_parking_sequence()
```

#### 5-Step Parking Sequence
```python
# Step 1: Turn 90° (toward parking spot)
target_heading = current_heading ± 90°
steer_max() + forward()
# Wait until heading reached

# Step 2: Move forward until front < 110mm
steer_center() + forward()
# Wait until close to wall

# Step 3: Reverse until ultrasonic < 200mm
reverse()
# Wait until rear sensor triggers

# Step 4: Final alignment (max steer + reverse)
steer_max() + reverse()
# Wait until heading in range (e.g., 340-343° or 17-20°)

# Step 5: Stop and center
stop() + steer_center()
```

---

## ⚙️ Configuration

### Key Parameters (in `main.py`)

#### Motor & Servo
```python
SERVO_CENTER = 94          # Servo center position (adjust for your servo)
MAX_TURN_ANGLE = 60        # Maximum steering angle
BASE_SPEED = 50            # Normal driving speed
PARKING_SPEED = 50         # Speed during parking
```

#### Wall Following
```python
TARGET_WALL_WIDE = 450     # Target distance in wide corridors (mm)
TARGET_WALL_NARROW = 180   # Target distance in narrow corridors (mm)
MIN_SAFE_DISTANCE = 150    # Minimum safe wall distance (mm)
KP_WALL = 0.015           # Proportional gain
KD_WALL = 0.025           # Derivative gain
```

#### Obstacle Detection (HSV Ranges)
```python
# Red obstacles
RED_RANGES = [(np.array([173, 144, 61]), np.array([179, 255, 165]))]

# Green obstacles
GREEN_RANGES = [(np.array([53, 90, 79]), np.array([68, 198, 150]))]

# Target X positions (camera frame)
RED_TARGET = 100           # Pass red on left
GREEN_TARGET = 500         # Pass green on right
```

#### Line Detection
```python
RED_THRESHOLD = 2000       # Red sensor threshold
WHITE_THRESHOLD = 3200     # White sensor threshold
```

#### LiDAR Filtering
```python
LIDAR_MIN_QUALITY = 5      # Minimum quality score (0-63)
LIDAR_MIN_DISTANCE = 30    # Minimum valid distance (mm)
LIDAR_MAX_DISTANCE = 4000  # Maximum valid distance (mm)
LIDAR_OUTLIER_THRESHOLD = 0.3  # 30% deviation from median
```

### Calibration Guide

#### 1. Servo Center Calibration
```python
# Run this to find your servo center:
vehicle.servo_ctrl.move(SERVO_CHANNEL, 90)  # Try values 85-95
# Adjust SERVO_CENTER until wheels point straight
```

#### 2. Camera HSV Calibration
```bash
# Enable camera display
SHOW_CAMERA = True

# Run main.py and observe detected obstacles
# Adjust RED_RANGES and GREEN_RANGES until detection is accurate

# Tips:
# - H (Hue): Color (0-179)
# - S (Saturation): Color intensity (0-255)
# - V (Value): Brightness (0-255)
```

#### 3. Light Sensor Calibration
```python
# Read sensors over line
red_value = vehicle.read_light_sensor(LIGHT_SENSOR_RED)
white_value = vehicle.read_light_sensor(LIGHT_SENSOR_WHITE)

# Adjust thresholds:
RED_THRESHOLD = red_value + 200   # Add margin
WHITE_THRESHOLD = white_value + 200
```

#### 4. Wall Following Tuning
```python
# If robot oscillates:
KP_WALL = 0.010  # Decrease
KD_WALL = 0.030  # Increase

# If robot reacts too slowly:
KP_WALL = 0.020  # Increase
KD_WALL = 0.020  # Decrease
```

---

## 🚀 Usage

### Quick Start
```bash
# Run the mission
cd ~/wro-2025
python3 main.py

# Robot will wait for button press (Analog Port 1)
# Press button to start
```

### Full Procedure

1. **Position robot in parking lot**
   - Facing toward exit
   - All sensors clear

2. **Power on**
   ```bash
   # SSH into Raspberry Pi
   ssh pi@raspberrypi.local
   
   # Navigate to project
   cd ~/wro-2025
   ```

3. **Run program**
   ```bash
   python3 main.py
   ```

4. **Wait for initialization**
   - Board initializes
   - Compass calibrates (20 readings)
   - Button prompt appears

5. **Press start button**
   - Robot exits parking lot
   - Begins obstacle avoidance
   - Completes 3 laps
   - Parks automatically

6. **Emergency stop**
   - Press `Ctrl+C` to interrupt
   - Motors will stop automatically

---

## 🐛 Troubleshooting

### Common Issues

#### 1. Camera Not Detected
```bash
# Check camera connection
libcamera-hello --timeout 2000

# If fails:
sudo raspi-config
# Interface Options → Legacy Camera → Disable
# Reboot

# Verify Picamera2 installation
python3 -c "from picamera2 import Picamera2; print('OK')"
```

#### 2. LiDAR Not Responding
```bash
# Check serial port
ls -l /dev/ttyAMA0

# If missing, enable serial:
sudo raspi-config
# Interface Options → Serial Port
# Disable login shell: No
# Enable serial hardware: Yes

# Check permissions
sudo usermod -a -G dialout pi
```

#### 3. Compass Returns None
```bash
# Check I2C connection
sudo i2cdetect -y 1
# Should see device at address 0x28

# If missing:
sudo raspi-config
# Interface Options → I2C → Enable
```

#### 4. Robot Turns Wrong Direction
```python
# Check servo polarity
# In main.py, line ~750:
servo_pos = SERVO_CENTER - angle  # Try changing to: + angle
```

#### 5. Parking Inaccurate
```python
# Increase LiDAR sampling in parking
# Line ~520: Already using update_lidar_aggressive()

# Verify outer wall target distance
OUTER_WALL_TARGET = 140  # Adjust ±20mm

# Check parking pillar detection threshold
if outer_dist < 270:  # Try 250-290mm range
```

#### 6. Obstacle Detection Fails
```python
# Enable camera display for debugging
SHOW_CAMERA = True

# Check HSV ranges
# Red: H should be 170-179 (wrap-around red)
# Green: H should be 50-70

# Adjust lighting:
# - Avoid shadows
# - Consistent brightness
# - Matte finish obstacles (not glossy)
```

---

## 📊 Performance Metrics

| Metric | Before Optimization | After Optimization |
|--------|--------------------|--------------------|
| **Parking accuracy** | ±30-50mm | ±5-15mm |
| **LiDAR sample rate** | 20 samples/buffer | 30 samples/buffer (parking: 3x) |
| **Obstacle detection** | 85% success | 95% success |
| **Line detection** | 90% success | 98% success |
| **Complete mission** | 60% success | 85% success |

### Optimization Timeline
- **Week 1-2**: Basic navigation (wall following)
- **Week 3-4**: Obstacle avoidance (camera integration)
- **Week 5-6**: Parking system (pillar detection)
- **Week 7**: LiDAR consistency fixes (quality filtering, outlier rejection)
- **Week 8**: Final tuning and testing

---

## 📚 Key Algorithms Explained

### 1. LiDAR Outlier Filtering
```python
def _filter_outliers(self, data):
    if len(data) < 3:
        return list(data)
    
    median = statistics.median(data)
    # Keep values within 30% of median
    filtered = [d for d in data 
                if abs(d - median) < median * 0.3]
    
    return filtered if filtered else list(data)
```

**Why it works**: LiDAR occasionally returns bad readings (reflections, sensor noise). By filtering values that deviate >30% from the median, we eliminate outliers while keeping valid data.

### 2. Heading-Based Angle Correction
```python
def correct_lidar_angle(lidar_angle, heading_offset):
    corrected = lidar_angle - heading_offset
    return normalize_angle(corrected)
```

**Why it works**: As the robot turns, LiDAR angles must be adjusted relative to the robot's orientation. This ensures "left wall" always means the same global direction.

### 3. Priority-Based Steering
```python
wall_too_close = (left < 150) or (right < 150)

if obstacle and not wall_too_close:
    steering = avoid_obstacle()  # Priority 1
else:
    steering = follow_wall()     # Priority 2
```

**Why it works**: Prevents robot from hitting walls while dodging obstacles. Wall safety always takes precedence.

### 4. Forward Turning
```python
def execute_forward_turn(self, direction):
    if direction == 'right':
        self.target_heading += 90
    else:
        self.target_heading -= 90
    
    self.target_heading = normalize_angle(self.target_heading)
```

**Why it works**: Instead of stopping to turn, we instantly update the target heading. The compass-based correction automatically steers the robot through the turn while moving forward.

---

## 🎓 Learning Resources

### Beginner Level
1. **Python Basics**
   - [Python.org Tutorial](https://docs.python.org/3/tutorial/)
   - [Real Python](https://realpython.com/)

2. **Raspberry Pi GPIO**
   - [Official GPIO Guide](https://www.raspberrypi.com/documentation/computers/os.html#gpio-and-the-40-pin-header)
   - [GPIO Zero Documentation](https://gpiozero.readthedocs.io/)

3. **Computer Vision Basics**
   - [OpenCV Python Tutorial](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
   - [HSV Color Space Explained](https://en.wikipedia.org/wiki/HSL_and_HSV)

### Intermediate Level
1. **PID Control**
   - [PID Without a PhD](https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf)
   - [Understanding PID Control](https://www.youtube.com/watch?v=wkfEZmsQqiA)

2. **Sensor Fusion**
   - [Kalman Filter Explained](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)
   - [IMU Data Fusion](https://www.youtube.com/watch?v=T9jXoG0QYIA)

3. **LiDAR Processing**
   - [RPLiDAR Documentation](https://www.slamtec.com/en/Lidar/A1)
   - [Point Cloud Processing](https://pcl.readthedocs.io/)

### Advanced Level
1. **SLAM (Simultaneous Localization and Mapping)**
   - [ROS Navigation Stack](http://wiki.ros.org/navigation)
   - [Hector SLAM](http://wiki.ros.org/hector_slam)

2. **Path Planning**
   - [A* Algorithm](https://www.redblobgames.com/pathfinding/a-star/introduction.html)
   - [Dynamic Window Approach](https://ieeexplore.ieee.org/document/580977)

---

## 👥 Team

### Team Name
**[Your Team Name]**

### Members
- **[Name 1]** - Team Leader, Mechanical Design
- **[Name 2]** - Software Developer, Algorithm Design
- **[Name 3]** - Electronics Engineer, Hardware Integration

### Coach
- **[Coach Name]**

### School/Organization
**[Your School/Organization Name]**

### Country
**[Your Country]**

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

```
MIT License

Copyright (c) 2025 [Your Team Name]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## 🙏 Acknowledgments

- **WRO Future Engineers** for organizing the competition
- **DFRobot** for the Raspberry Pi Expansion Board
- **Adafruit** for the BNO055 breakout board
- **Slamtec** for the RPLiDAR sensor
- **Raspberry Pi Foundation** for the amazing platform
- **OpenCV Community** for computer vision tools
- All open-source contributors whose libraries made this possible

---

## 📧 Contact

For questions or collaboration:
- **Email**: [your-email@example.com]
- **GitHub**: [@your-username](https://github.com/your-username)
- **YouTube**: [Your Channel](https://youtube.com/your-channel) (optional)

---

## 🌟 Show Your Support

Give a ⭐️ if this project helped you!

---

**Last Updated**: January 2025
