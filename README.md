# WRO Robot Code Explanation - Complete Guide

I'll explain this Python code step by step, focusing on how it works and what each part does.

---

## Table of Contents
1. [Overview](#overview)
2. [Library Imports and Setup](#imports)
3. [GPIO Initialization](#gpio)
4. [Configuration Constants](#constants)
5. [Helper Functions](#helpers)
6. [SimpleMotor Class](#motor)
7. [Vehicle Class - Initialization](#vehicle-init)
8. [Compass System](#compass)
9. [LiDAR System](#lidar)
10. [Light Sensor](#light-sensor)
11. [Turn Decision Strategy](#turn-decision)
12. [Motor Control and Steering](#motor-control)
13. [Navigation System](#navigation)
14. [90-Degree Turn Execution](#turning)
15. [Main Mission Loop](#mission)
16. [Complete Mission Flow](#flow)

---

<a name="overview"></a>
## 1. Overview

This is a WRO (World Robot Olympiad) Future Engineers autonomous robot that:
- Navigates a track with walls on both sides
- Detects corner lines using a light sensor
- Makes turn decisions based on LiDAR side clearance measurements
- Completes 12 turns (3 laps × 4 corners)
- Stops when it returns to the starting position

**Key Strategy:**
- When it detects the first corner line, it checks which side has more than 2000mm of clearance
- It turns toward that clear side
- For all remaining 11 turns, it uses the same direction

---

<a name="imports"></a>
## 2. Library Imports and Setup

```python
import sys
sys.path.append("/home/Book_Pi/DFRobot_RaspberryPi_Expansion_Board")
from DFRobot_RaspberryPi_Expansion_Board import DFRobot_Expansion_Board_IIC as Board
from DFRobot_RaspberryPi_Expansion_Board import DFRobot_Expansion_Board_Servo as Servo
```

**Purpose:** Imports the DFRobot expansion board libraries for:
- Reading analog sensors (light sensors)
- Controlling servo motors (steering)

```python
import serial
import time
from collections import deque
```

**Purpose:**
- `serial`: Communicate with LiDAR sensor via UART
- `time`: Timing control and delays
- `deque`: Efficient buffer for storing LiDAR data

```python
import RPi.GPIO as GPIO
```

**Purpose:** Control Raspberry Pi GPIO pins for motor control

```python
import board as board_pins
import busio
import adafruit_bno055
COMPASS_AVAILABLE = True
```

**Purpose:** 
- `board_pins`: Pin definitions for I2C communication
- `busio`: I2C communication interface
- `adafruit_bno055`: BNO055 compass sensor library
- Sets compass availability flag

---

<a name="gpio"></a>
## 3. GPIO Initialization

```python
print("Initializing GPIO...")
try:
    GPIO.cleanup()
except:
    pass

time.sleep(0.2)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
```

**What this does:**
1. `GPIO.cleanup()`: Clears any previous GPIO configurations (prevents conflicts)
2. `time.sleep(0.2)`: Waits 200ms for cleanup to complete
3. `GPIO.setmode(GPIO.BCM)`: Uses Broadcom pin numbering (GPIO numbers, not physical pin numbers)
4. `GPIO.setwarnings(False)`: Suppresses warning messages

**Why cleanup first?**
If the program crashed previously, GPIO pins might still be configured. Cleanup ensures a fresh start.

```python
# Motor pins
MOTOR_IN1 = 6
MOTOR_IN2 = 7
MOTOR_ENA = 5

# Setup motor pins
print("Setting up motor pins...")
try:
    GPIO.setup([MOTOR_IN1, MOTOR_IN2, MOTOR_ENA], GPIO.OUT, initial=GPIO.LOW)
    print("GPIO pins configured successfully")
except Exception as e:
    print(f"GPIO setup warning: {e}")
```

**Motor pin configuration:**
- `MOTOR_IN1` (GPIO 6): Motor direction control (forward/backward)
- `MOTOR_IN2` (GPIO 7): Motor direction control (forward/backward)
- `MOTOR_ENA` (GPIO 5): Motor enable pin with PWM speed control

**Pin setup:**
- `GPIO.OUT`: Configures pins as outputs
- `initial=GPIO.LOW`: Sets all pins to LOW (0V) initially to prevent motor from running

---

<a name="constants"></a>
## 4. Configuration Constants

### Servo Configuration

```python
SERVO_CHANNEL = 0
SERVO_CENTER = 92
MAX_TURN_ANGLE = 45
```

**Explanation:**
- `SERVO_CHANNEL = 0`: Uses channel 0 on the expansion board
- `SERVO_CENTER = 92`: The servo angle value for driving straight (calibrated value)
- `MAX_TURN_ANGLE = 45`: Maximum steering angle (±45 degrees from center)

### LiDAR Configuration

```python
LIDAR_PORT = '/dev/ttyAMA0'
LIDAR_BAUD = 460800
```

**Explanation:**
- `LIDAR_PORT`: Serial port where LiDAR is connected (Raspberry Pi's primary UART)
- `LIDAR_BAUD`: Communication speed (460,800 bits per second)

### Light Sensor Configuration

```python
LIGHT_SENSOR_A3 = 3
LINE_THRESHOLD = 2600
```

**Explanation:**
- `LIGHT_SENSOR_A3 = 3`: Light sensor connected to analog input A3
- `LINE_THRESHOLD = 2600`: Values below this indicate a dark line
  - Light surface: ~3000-4000
  - Dark line: <2600

### LiDAR Angle Zones

```python
FRONT_MIN = 355
FRONT_MAX = 5
LEFT_MIN = 265
LEFT_MAX = 275
RIGHT_MIN = 85
RIGHT_MAX = 95
```

**How LiDAR angles work:**
LiDAR measures 360° around the robot:
```
        0° (Front)
           |
   270° ---|--- 90°
           |
       180° (Back)
```

**Zone definitions:**
- **Front zone**: 355°-360° and 0°-5° (wraps around 0°)
- **Left zone**: 265°-275° (10° window pointing left)
- **Right zone**: 85°-95° (10° window pointing right)

**Why narrow zones (10° instead of 40°)?**
- More precise wall distance measurements
- Reduces interference from diagonal walls
- Better for tight corridor navigation

### Turn Decision Parameter

```python
LIDAR_SIDE_CLEARANCE_THRESHOLD = 2000
```

**Critical parameter:** Robot turns toward whichever side has more than 2000mm (2 meters) of clearance.

**Why 2000mm?**
- Typical WRO obstacle sections: pillars are placed ~1500mm from the wall
- Open corner areas: usually >2000mm clearance
- This threshold reliably distinguishes between "obstacle side" vs "open side"

### Compass Configuration

```python
HEADING_TOLERANCE = 10
HEADING_KP = 0.5
```

**Explanation:**
- `HEADING_TOLERANCE = 10`: Turn is complete when within ±10° of target heading
- `HEADING_KP = 0.5`: Proportional gain for compass-based steering correction

### Speed Configuration

```python
BASE_SPEED = 55
SLOW_SPEED = 50
TURN_SPEED = 50
```

**Speed values (0-100 scale):**
- `BASE_SPEED = 55`: Normal driving speed
- `SLOW_SPEED = 50`: Used when close to walls or turning
- `TURN_SPEED = 50`: Speed during 90-degree turns

### Wall Following Parameters

```python
TARGET_WALL_WIDE = 450
TARGET_WALL_NARROW = 250
MIN_SAFE_DISTANCE = 180
KP_WALL = 0.04
KD_WALL = 0.025
```

**Wall following behavior:**
- `TARGET_WALL_WIDE = 450`: Tries to maintain 450mm from wall in wide corridors
- `TARGET_WALL_NARROW = 250`: Tries to maintain 250mm from wall in narrow corridors
- `MIN_SAFE_DISTANCE = 180`: Emergency distance - slows down if closer than 180mm
- `KP_WALL = 0.04`: Proportional gain for wall-following steering
- `KD_WALL = 0.025`: Derivative gain for wall-following steering (damping)

**PD Control explanation:**
- **P (Proportional)**: Steers harder when farther from target distance
- **D (Derivative)**: Reduces steering when approaching target (prevents oscillation)

### Turn Execution Parameters

```python
FRONT_TURN_THRESHOLD = 700
PRE_TURN_STEER_ANGLE = 30
REVERSE_SPEED = 50
```

**Explanation:**
- `FRONT_TURN_THRESHOLD = 700`: Considers front obstacle when within 700mm
- `PRE_TURN_STEER_ANGLE = 30`: Steering angle applied during turns (±30°)
- `REVERSE_SPEED = 50`: Speed for backing up (not used in current code, but available)

### Board Initialization

```python
board = Board(1, 0x10)
```

**Purpose:** Creates global expansion board object
- `1`: I2C bus number
- `0x10`: I2C address of the expansion board

---

<a name="helpers"></a>
## 5. Helper Functions

### Normalize Angle

```python
def normalize_angle(angle):
    while angle < 0:
        angle += 360
    while angle >= 360:
        angle -= 360
    return angle
```

**Purpose:** Keeps angles within 0-360° range

**Examples:**
- Input: 450° → Output: 90° (450 - 360 = 90)
- Input: -45° → Output: 315° (-45 + 360 = 315)
- Input: 180° → Output: 180° (already in range)

**Why needed?**
Compass readings and turn calculations can produce angles outside 0-360° range. This function wraps them back.

### Angle Difference

```python
def angle_difference(target, current):
    diff = target - current
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360
    return diff
```

**Purpose:** Calculates the shortest angular distance between two angles

**Examples:**
- Target: 10°, Current: 350° → Difference: +20° (not -340°)
- Target: 350°, Current: 10° → Difference: -20° (not +340°)
- Target: 180°, Current: 90° → Difference: +90°

**Why needed?**
Without this, the robot might turn 340° counterclockwise instead of 20° clockwise.

**How it works:**
```
If difference > 180°: subtract 360° (go the other way)
If difference < -180°: add 360° (go the other way)
```

### Correct LiDAR Angle

```python
def correct_lidar_angle(lidar_angle, heading_offset):
    corrected = lidar_angle - heading_offset
    return normalize_angle(corrected)
```

**Purpose:** Adjusts LiDAR readings based on robot's current heading

**Why needed?**
LiDAR measures angles relative to its own orientation. If the robot is facing 45° instead of 0°, the LiDAR readings need to be corrected.

**Example:**
```
Robot is facing 45° (heading_offset = 45°)
LiDAR detects wall at 90° (its own reference)
Corrected angle = 90° - 45° = 45° (actual wall direction relative to starting orientation)
```

**Process:**
1. Subtracts heading offset from LiDAR angle
2. Normalizes result to 0-360° range

---

<a name="motor"></a>
## 6. SimpleMotor Class

```python
class SimpleMotor:
    def __init__(self, in1, in2, ena):
        self.in1, self.in2, self.ena = in1, in2, ena
        
        print("Configuring motor pins...")
        
        # Motor control pins
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.ena, GPIO.OUT)
        
        # PWM
        self.pwm = GPIO.PWM(self.ena, 1000)
        self.pwm.start(0)
        print("Motor configured successfully")
```

**Initialization:**
1. Stores pin numbers (IN1, IN2, ENA)
2. Configures GPIO pins as outputs
3. Creates PWM object on ENA pin at 1000 Hz frequency
4. Starts PWM at 0% duty cycle (motor off)

**Motor control logic:**
- **IN1 HIGH, IN2 LOW**: Motor spins forward
- **IN1 LOW, IN2 HIGH**: Motor spins backward
- **ENA PWM duty cycle**: Controls speed (0-100%)

### Set Speed Method

```python
def set_speed(self, speed):
    speed = max(-100, min(100, speed))
    try:
        if speed > 0:
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
            self.pwm.ChangeDutyCycle(abs(speed))
        elif speed < 0:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
            self.pwm.ChangeDutyCycle(abs(speed))
        else:
            self.stop()
    except Exception as e:
        print(f"Motor speed error: {e}")
```

**How it works:**

1. **Speed clamping:** `speed = max(-100, min(100, speed))`
   - Ensures speed is between -100 and +100
   - Example: set_speed(150) → actually uses 100

2. **Forward (speed > 0):**
   ```
   IN1 = HIGH, IN2 = LOW
   PWM = speed value (0-100%)
   ```

3. **Backward (speed < 0):**
   ```
   IN1 = LOW, IN2 = HIGH
   PWM = absolute value of speed
   ```

4. **Stop (speed = 0):**
   - Calls the stop() method

**Example:**
```python
motor.set_speed(75)   # Forward at 75% power
motor.set_speed(-50)  # Backward at 50% power
motor.set_speed(0)    # Stop
```

### Stop Method

```python
def stop(self):
    try:
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
    except:
        pass
```

**Purpose:** Completely stops the motor
- Sets both IN1 and IN2 to LOW
- Sets PWM to 0%
- Uses `try/except` to handle potential errors gracefully

### Cleanup Method

```python
def cleanup(self):
    self.stop()
    try:
        self.pwm.stop()
    except:
        pass
```

**Purpose:** Proper shutdown procedure
1. Stops the motor
2. Stops the PWM signal
3. Should be called before program exits

---

<a name="vehicle-init"></a>
## 7. Vehicle Class - Initialization

```python
class Vehicle:
    def __init__(self):
        print("\n" + "="*70)
        print(" WRO FUTURE ENGINEERS 2025 - LIDAR TURN DECISION")
        print("="*70)
        
        self._init_board()
        self._init_motor()
        self._init_servo()
        self._init_compass()
        self._init_lidar()
```

**Initialization sequence:**
The `__init__` method calls five initialization functions in order. Each must succeed before moving to the next.

### State Variables

```python
        # Compass navigation
        self.initial_heading = None
        self.target_heading = None
        self.heading_offset = 0
```

**Compass variables:**
- `initial_heading`: The compass direction when the robot starts (e.g., 0°)
- `target_heading`: Where the robot should be pointing now (updates after each turn)
- `heading_offset`: Error between where robot should point vs. where it actually points

**Example:**
```
initial_heading = 0° (start facing north)
After 1st turn: target_heading = 90° (should face east)
If compass reads 85°: heading_offset = +5° (pointing 5° too far left)
```

```python
        # Mission state
        self.turn_count = 0
        self.turn_direction = None  # Will be set by first LiDAR decision
```

**Mission tracking:**
- `turn_count`: How many turns completed (0-12)
- `turn_direction`: Memorized direction ('left' or 'right', starts as None)

**The key strategy:**
- First corner: robot decides 'left' or 'right' based on LiDAR
- Remaining 11 corners: robot uses the same memorized direction

```python
        # Navigation state
        self.last_left_error = 0
        self.last_right_error = 0
        self.current_speed = BASE_SPEED
        self.navigation_mode = "wall_follow"
```

**Navigation variables:**
- `last_left_error`: Previous error from left wall (for derivative calculation)
- `last_right_error`: Previous error from right wall (for derivative calculation)
- `current_speed`: Current motor speed value
- `navigation_mode`: Current behavior ("wall_follow" or "turning")

```python
        # Turn sequence state
        self.waiting_for_clearance = False
        self.line_detected_time = 0
```

**Turn state machine:**
- `waiting_for_clearance`: True when robot detected a line and is waiting to turn
- `line_detected_time`: Timestamp when line was detected (for timeout protection)

**Two-phase turn sequence:**
```
Phase 1: Detect line → set waiting_for_clearance = True
Phase 2: Check side clearance → turn → set waiting_for_clearance = False
```

```python
        # Corridor width
        self.corridor_width_history = deque(maxlen=20)
```

**Corridor tracking:**
- Stores the last 20 measurements of total corridor width (left + right distances)
- Used to determine if in "wide" or "narrow" corridor
- Automatically removes old data when full (FIFO - First In First Out)

```python
        # Steering
        self.steering_history = deque(maxlen=3)
```

**Smooth steering:**
- Stores the last 3 steering commands
- Averages them for smoother steering transitions
- Prevents jerky movements

```python
        # LiDAR
        self.lidar_buffer = bytearray()
        self.front_data = deque(maxlen=10)
        self.left_data = deque(maxlen=10)
        self.right_data = deque(maxlen=10)
```

**LiDAR data storage:**
- `lidar_buffer`: Raw bytes received from LiDAR (unparsed)
- `front_data`: Last 10 front distance readings
- `left_data`: Last 10 left distance readings  
- `right_data`: Last 10 right distance readings

**Why deque with maxlen=10?**
- Automatically removes oldest reading when adding 11th reading
- Provides averaging over recent data (noise reduction)
- Fixed memory usage (doesn't grow indefinitely)

### Board Initialization

```python
def _init_board(self):
    print("[1/5] Expansion Board...")
    self.board = Board(1, 0x10)
    while self.board.begin() != self.board.STA_OK:
        time.sleep(1)
    self.board.set_adc_enable()
    print("  OK")
```

**Process:**
1. Creates Board object (I2C bus 1, address 0x10)
2. Calls `board.begin()` repeatedly until it returns `STA_OK` (board ready)
3. Enables ADC (Analog-to-Digital Converter) for light sensors
4. Prints confirmation

**Why the while loop?**
The expansion board needs time to initialize. If it's not ready, the program waits 1 second and tries again.

### Motor Initialization

```python
def _init_motor(self):
    print("[2/5] Simple Motor...")
    self.motor = SimpleMotor(MOTOR_IN1, MOTOR_IN2, MOTOR_ENA)
    print("  OK")
```

**Process:**
- Creates SimpleMotor object with pin numbers
- SimpleMotor.__init__() handles the actual GPIO setup

### Servo Initialization

```python
def _init_servo(self):
    print("[3/5] Servo...")
    self.servo_ctrl = Servo(self.board)
    self.servo_ctrl.begin()
    self.servo_ctrl.move(SERVO_CHANNEL, SERVO_CENTER)
    print("  OK")
```

**Process:**
1. Creates Servo controller object (uses the expansion board)
2. Calls `begin()` to initialize servo system
3. Moves servo to center position (92°) for straight driving
4. Prints confirmation

### Compass Initialization

```python
def _init_compass(self):
    print("[4/5] BNO055 Compass...")
    try:
        i2c = busio.I2C(board_pins.SCL, board_pins.SDA)
        self.compass = adafruit_bno055.BNO055_I2C(i2c)
        time.sleep(1)
        print("  OK")
    except:
        print("  FAILED (continuing without compass)")
        self.compass = None
```

**Process:**
1. Creates I2C bus object using SCL and SDA pins
2. Creates BNO055 compass object on that I2C bus
3. Waits 1 second for compass to stabilize
4. If any error occurs, sets `self.compass = None` and continues

**Why try/except?**
Compass failure shouldn't stop the entire program. The robot can still navigate using dead reckoning.

### LiDAR Initialization

```python
def _init_lidar(self):
    print("[5/5] LiDAR...")
    self.lidar = serial.Serial(port=LIDAR_PORT, baudrate=LIDAR_BAUD, timeout=1)
    
    print("  Sending scan command...")
    self.lidar.write(bytes([0xA5, 0x20]))
    time.sleep(0.5)
    response = self.lidar.read(7)
    print(f"  Scan response: {response.hex()}")
    print("  OK")
```

**Process:**
1. Opens serial connection to LiDAR at 460800 baud
2. Sends scan start command: `0xA5 0x20`
3. Waits 500ms for LiDAR to respond
4. Reads 7-byte response and displays it in hexadecimal
5. Prints confirmation

**LiDAR command breakdown:**
- `0xA5`: Command header byte
- `0x20`: "Start scan" command code

**Expected response format:**
```
Byte 0-1: Response header (0xA5 0x5A)
Byte 2: Response length
Byte 3: Command type
Byte 4-6: Additional data
```

---

<a name="compass"></a>
## 8. Compass System

### Get Current Heading

```python
def heading(self):
    if self.compass:
        try:
            euler = self.compass.euler
            if euler and euler[0] is not None:
                return round(euler[0], 1)
        except:
            pass
    return None
```

**Purpose:** Returns robot's current compass heading

**Process:**
1. Checks if compass exists
2. Reads Euler angles from compass
3. Returns first Euler angle (heading) rounded to 0.1°
4. Returns `None` if compass unavailable or reading fails

**Euler angles explained:**
- `euler[0]`: Heading (0-360°, where 0° = north)
- `euler[1]`: Pitch (tilt forward/backward)
- `euler[2]`: Roll (tilt left/right)

**Example output:**
```python
heading() → 45.2    # Robot facing northeast
heading() → 180.0   # Robot facing south
heading() → None    # Compass error
```

### Set Initial Heading

```python
def set_initial_heading(self):
    heading = self.heading()
    if heading is not None:
        self.initial_heading = heading
        self.target_heading = heading
        print(f"Initial heading set: {self.initial_heading}°")
    else:
        print("WARNING: Could not set initial heading!")
```

**Purpose:** Records the robot's starting direction

**When called:** Once at mission start, after compass stabilizes

**What it does:**
1. Reads current heading
2. Sets both `initial_heading` and `target_heading` to this value
3. Prints confirmation or warning

**Example:**
```
Robot starts facing: 0°
initial_heading = 0°
target_heading = 0°

After 1st right turn (90°):
initial_heading = 0° (never changes)
target_heading = 90° (updated after turn)
```

### Update Heading Offset

```python
def update_heading_offset(self):
    current = self.heading()
    if current is not None and self.target_heading is not None:
        self.heading_offset = angle_difference(self.target_heading, current)
    else:
        self.heading_offset = 0
```

**Purpose:** Calculates how far off-course the robot is

**Process:**
1. Reads current heading from compass
2. Calculates difference between target and current heading
3. Stores result in `heading_offset`

**Example:**
```
target_heading = 90° (should be facing east)
current heading = 85° (actually facing slightly north of east)
heading_offset = 90° - 85° = +5° (need to turn right by 5°)
```

**Sign convention:**
- **Positive offset**: Robot is left of target, should turn right
- **Negative offset**: Robot is right of target, should turn left

### Calculate Heading Correction

```python
def calculate_heading_correction(self):
    if self.heading_offset == 0:
        return 0
    correction = -self.heading_offset * HEADING_KP
    return max(-30, min(30, correction))
```

**Purpose:** Converts heading error into steering adjustment

**Formula:** `correction = -heading_offset × HEADING_KP`

**Why negative?**
- If heading_offset = +5° (too far left), we want negative steering (turn right)
- If heading_offset = -5° (too far right), we want positive steering (turn left)

**Example calculations:**
```
HEADING_KP = 0.5

heading_offset = +10° → correction = -10 × 0.5 = -5° (steer right)
heading_offset = -8°  → correction = -(-8) × 0.5 = +4° (steer left)
heading_offset = +100° → correction = -50°, clamped to -30° (max right)
```

**Clamping:** `max(-30, min(30, correction))`
- Limits correction to ±30° maximum
- Prevents excessive steering commands

---

<a name="lidar"></a>
## 9. LiDAR System

### Update LiDAR

```python
def update_lidar(self):
    try:
        if self.lidar.in_waiting > 0:
            self.lidar_buffer.extend(self.lidar.read(self.lidar.in_waiting))
        
        self.update_heading_offset()
        
        while len(self.lidar_buffer) >= 5:
            point_data = self.lidar_buffer[:5]
            self.lidar_buffer = self.lidar_buffer[5:]
            
            point = self._parse_scan_point(point_data)
            if point and point['distance'] > 0:
                raw_angle = point['angle']
                corrected_angle = correct_lidar_angle(raw_angle, self.heading_offset)
                dist = point['distance']
                
                if self._is_front_angle(corrected_angle):
                    self.front_data.append(dist)
                elif self._is_left(corrected_angle):
                    self.left_data.append(dist)
                elif self._is_right(corrected_angle):
                    self.right_data.append(dist)
    except Exception as e:
        print(f"LiDAR update error: {e}")
```

**Purpose:** Main LiDAR processing function, called continuously in the main loop

**Step-by-step process:**

**Step 1: Read new data**
```python
if self.lidar.in_waiting > 0:
    self.lidar_buffer.extend(self.lidar.read(self.lidar.in_waiting))
```
- Checks if serial port has data waiting
- Reads all available bytes and adds to buffer
- `in_waiting`: Number of bytes in serial receive buffer

**Step 2: Update compass**
```python
self.update_heading_offset()
```
- Updates the robot's heading error
- Needed for angle correction

**Step 3: Parse data points**
```python
while len(self.lidar_buffer) >= 5:
    point_data = self.lidar_buffer[:5]
    self.lidar_buffer = self.lidar_buffer[5:]
```
- Each LiDAR point is 5 bytes
- Extracts first 5 bytes for processing
- Removes those 5 bytes from buffer

**Step 4: Process each point**
```python
point = self._parse_scan_point(point_data)
if point and point['distance'] > 0:
```
- Parses the 5-byte packet into angle and distance
- Checks if valid (distance > 0 means actual detection)

**Step 5: Correct angle and store**
```python
raw_angle = point['angle']
corrected_angle = correct_lidar_angle(raw_angle, self.heading_offset)
dist = point['distance']
```
- Gets raw angle from LiDAR
- Corrects it based on robot's current heading
- Extracts distance value

**Step 6: Sort into zones**
```python
if self._is_front_angle(corrected_angle):
    self.front_data.append(dist)
elif self._is_left(corrected_angle):
    self.left_data.append(dist)
elif self._is_right(corrected_angle):
    self.right_data.append(dist)
```
- Determines which zone the angle belongs to
- Adds distance to appropriate deque buffer

### Parse Scan Point

```python
def _parse_scan_point(self, data):
    if len(data) < 5:
        return None
    
    quality = (data[0] >> 2) & 0x3F
    angle_q6 = (data[1] >> 1) | (data[2] << 7)
    angle = (angle_q6 / 64.0) % 360.0
    distance_q2 = data[3] | (data[4] << 8)
    distance = distance_q2 / 4.0
    
    return {
        'angle': angle,
        'distance': distance,
        'quality': quality
    }
```

**Purpose:** Converts 5-byte LiDAR packet into useful data

**LiDAR packet format (5 bytes):**
```
Byte 0: [Quality(6 bits)][Start flag(1 bit)][Not used(1 bit)]
Byte 1-2: Angle data (15 bits)
Byte 3-4: Distance data (16 bits)
```

**Parsing breakdown:**

**1. Quality extraction:**
```python
quality = (data[0] >> 2) & 0x3F
```
- Right-shifts
byte 0 by 2 bits
- Masks with 0x3F (0011 1111) to get 6 bits
- Quality range: 0-63 (higher = better)

**2. Angle extraction:**
```python
angle_q6 = (data[1] >> 1) | (data[2] << 7)
angle = (angle_q6 / 64.0) % 360.0
```
- Combines bytes 1 and 2 into 15-bit number
- Divides by 64 (Q6 fixed-point format)
- Applies modulo 360 to get 0-360° range

**Example:**
```
data[1] = 0b10101010, data[2] = 0b00000001
Step 1: data[1] >> 1 = 0b01010101 (85)
Step 2: data[2] << 7 = 0b10000000 (128)
Step 3: angle_q6 = 85 | 128 = 213
Step 4: angle = 213 / 64.0 = 3.328°
```

**3. Distance extraction:**
```python
distance_q2 = data[3] | (data[4] << 8)
distance = distance_q2 / 4.0
```
- Combines bytes 3 and 4 into 16-bit number
- Divides by 4 (Q2 fixed-point format)
- Result in millimeters

**Example:**
```
data[3] = 0xE8 (232), data[4] = 0x03 (3)
Step 1: data[4] << 8 = 0x0300 (768)
Step 2: distance_q2 = 232 | 768 = 1000
Step 3: distance = 1000 / 4.0 = 250.0 mm
```

**Return format:**
```python
{
    'angle': 45.5,      # degrees (0-360)
    'distance': 1250.0, # millimeters
    'quality': 47       # signal quality (0-63)
}
```

### Angle Detection Methods

```python
def _is_front_angle(self, angle):
    return angle >= FRONT_MIN or angle <= FRONT_MAX
```

**Front detection:** 355° - 360° OR 0° - 5°

**Why "or" instead of "and"?**
The front zone wraps around 0°:
```
        0°/360°
    355° | 5°
        robot
```

**Examples:**
```python
_is_front_angle(358) → True  (358 >= 355)
_is_front_angle(2)   → True  (2 <= 5)
_is_front_angle(180) → False (not in range)
```

```python
def _is_left(self, angle):
    return abs(angle - LEFT_MIN) <= 5
```

**Left detection:** Within 5° of 265°

**Formula:** `|angle - 265°| ≤ 5°`

**Examples:**
```python
_is_left(265) → True  (|265-265| = 0 ≤ 5)
_is_left(270) → True  (|270-265| = 5 ≤ 5)
_is_left(262) → True  (|262-265| = 3 ≤ 5)
_is_left(280) → False (|280-265| = 15 > 5)
```

**Effective range:** 260° - 270°

```python
def _is_right(self, angle):
    return abs(angle - RIGHT_MIN) <= 5
```

**Right detection:** Within 5° of 85°

**Effective range:** 80° - 90°

**Visual representation:**
```
           0° (Front: 355-5°)
            |
    265° ---|--- 85°
    (Left:  |  (Right:
   260-270°)|  80-90°)
            |
          180°
```

### Distance Methods

```python
def distance_front(self):
    if self.front_data:
        return round(sum(self.front_data) / len(self.front_data))
    return None
```

**Purpose:** Returns average front distance from recent readings

**How it works:**
1. Checks if `front_data` deque has any readings
2. Sums all values
3. Divides by count to get average
4. Rounds to nearest integer (millimeter)
5. Returns `None` if no data available

**Example:**
```python
front_data = [1200, 1180, 1220, 1190, 1210]

sum = 6000
len = 5
average = 6000 / 5 = 1200 mm
```

**Why averaging?**
- Reduces noise from individual readings
- Provides more stable distance values
- With `maxlen=10`, averages over ~0.2 seconds of data

```python
def distance_left(self):
    if self.left_data:
        return round(sum(self.left_data) / len(self.left_data))
    return None

def distance_right(self):
    if self.right_data:
        return round(sum(self.right_data) / len(self.right_data))
    return None
```

**Same logic for left and right distances**

**Return values:**
- Integer (mm) if data available
- `None` if no recent readings

---

<a name="light-sensor"></a>
## 10. Light Sensor

### Read Light Sensor

```python
def read_light_sensor_a3(self):
    """Read only A3 light sensor"""
    try:
        value = self.board.get_adc_value(LIGHT_SENSOR_A3)
        return value
    except:
        return None
```

**Purpose:** Reads the analog light sensor value

**How it works:**
1. Calls `get_adc_value()` with sensor channel (3)
2. Returns raw ADC value (typically 0-4095)
3. Returns `None` if read fails

**Light sensor behavior:**
- **Bright surface (white mat):** High values (~3000-4000)
- **Dark surface (orange/blue line):** Low values (~1000-2500)
- **Very dark:** Very low values (<1000)

### Detect Line

```python
def detect_line(self):
    """
    Detect line using ONLY A3 sensor
    Returns: True if line detected, False otherwise
    """
    a3_value = self.read_light_sensor_a3()
    
    if a3_value is None:
        return False
    
    # Line detected when sensor reads below threshold (dark line)
    if a3_value < LINE_THRESHOLD:
        print(f">>> LINE DETECTED! A3 sensor: {a3_value}")
        return True
    
    return False
```

**Purpose:** Determines if robot is over a corner line

**Detection logic:**
```
if sensor_value < 2600:
    → Line detected (dark surface)
else:
    → No line (light surface)
```

**Example scenarios:**
```python
# On white mat
a3_value = 3200
3200 < 2600? No → False (no line)

# On orange corner line
a3_value = 2100
2100 < 2600? Yes → True (line detected!)

# Sensor error
a3_value = None
→ False (safe default)
```

**Why print when detected?**
- Provides visual feedback during operation
- Helps with debugging and monitoring
- Shows actual sensor value for calibration

**Threshold calibration:**
The `LINE_THRESHOLD = 2600` value should be adjusted based on:
- Ambient lighting conditions
- Mat color and condition
- Line color (orange or blue)
- Sensor positioning and height

---

<a name="turn-decision"></a>
## 11. Turn Decision Strategy

```python
def decide_turn_direction_lidar(self):
    """
    NEW: Decide turn direction based on which side has > 1500mm clearance
    Returns: 'left' if left side is clear, 'right' if right side is clear
    """
    left_dist = self.distance_left()
    right_dist = self.distance_right()
    
    print(f"\n  LiDAR Turn Decision:")
    print(f"    Left side: {left_dist}mm")
    print(f"    Right side: {right_dist}mm")
    print(f"    Threshold: {LIDAR_SIDE_CLEARANCE_THRESHOLD}mm")
```

**Purpose:** This is the core strategy - determines which direction to turn based on LiDAR measurements

**Strategy overview:**
1. Measure distances on both sides
2. Check which side has > 2000mm clearance
3. Turn toward the clear side
4. Remember this direction for all future turns

### Handle Missing Data

```python
    if left_dist is None:
        left_dist = 0
    if right_dist is None:
        right_dist = 0
```

**Safety handling:** If LiDAR can't read a side (no data), treats it as 0mm

**Why this matters:**
- Prevents program crashes from `None` values
- Ensures that side won't be chosen (0mm is never > 2000mm)
- Allows decision logic to continue even with partial data

### Decision Logic

```python
    # Check which side has > 1500mm clearance
    left_clear = left_dist > LIDAR_SIDE_CLEARANCE_THRESHOLD
    right_clear = right_dist > LIDAR_SIDE_CLEARANCE_THRESHOLD
```

**Clearance check:**
- `left_clear = True` if left distance > 2000mm
- `right_clear = True` if right distance > 2000mm

**What 2000mm clearance means:**
- WRO obstacle sections: Pillars typically ~1200-1500mm from wall
- Open corners: Usually >2000mm clearance
- This threshold reliably distinguishes "obstacle side" vs "open side"

### Case 1: Only Left Clear

```python
    if left_clear and not right_clear:
        print(f"  → LEFT side is CLEAR ({left_dist}mm > {LIDAR_SIDE_CLEARANCE_THRESHOLD}mm)")
        return 'left'
```

**Scenario:**
```
Left: 2300mm (clear!) 
Right: 1400mm (blocked by obstacle)
Decision: Turn LEFT
```

**Why?**
The right side has an obstacle (pillar), so turning right would hit it. Left side is open, so turn that way.

### Case 2: Only Right Clear

```python
    elif right_clear and not left_clear:
        print(f"  → RIGHT side is CLEAR ({right_dist}mm > {LIDAR_SIDE_CLEARANCE_THRESHOLD}mm)")
        return 'right'
```

**Scenario:**
```
Left: 1300mm (blocked by obstacle)
Right: 2500mm (clear!)
Decision: Turn RIGHT
```

### Case 3: Both Sides Clear

```python
    elif left_clear and right_clear:
        # Both clear, pick the clearer one
        if left_dist > right_dist:
            print(f"  → BOTH clear, LEFT is clearer ({left_dist}mm > {right_dist}mm)")
            return 'left'
        else:
            print(f"  → BOTH clear, RIGHT is clearer ({right_dist}mm >= {left_dist}mm)")
            return 'right'
```

**Scenario:**
```
Left: 2400mm (clear)
Right: 2100mm (clear)
Both > 2000mm threshold
Decision: Turn LEFT (2400 > 2100)
```

**Why choose the clearer side?**
- More room to maneuver
- Less risk of hitting walls during turn
- Better for obstacle avoidance courses

### Case 4: Neither Side Clear

```python
    else:
        # Neither clear, pick the larger one
        if left_dist > right_dist:
            print(f"  → NEITHER > {LIDAR_SIDE_CLEARANCE_THRESHOLD}mm, choosing LEFT ({left_dist}mm > {right_dist}mm)")
            return 'left'
        else:
            print(f"  → NEITHER > {LIDAR_SIDE_CLEARANCE_THRESHOLD}mm, choosing RIGHT ({right_dist}mm >= {left_dist}mm)")
            return 'right'
```

**Scenario:**
```
Left: 1800mm (not clear, but larger)
Right: 1500mm (not clear, smaller)
Neither > 2000mm threshold
Decision: Turn LEFT (1800 > 1500 - best available)
```

**Why this case exists:**
- Narrow tracks where neither side reaches 2000mm
- Early detection before reaching full clearance
- Still makes a reasonable decision based on available space

**Example decision flow:**
```
Corner approach:
Time 0s: L:1200mm R:1100mm → neither clear, keep driving
Time 1s: L:1500mm R:1300mm → neither clear, keep driving
Time 2s: L:1900mm R:1600mm → neither clear, keep driving
Time 3s: L:2100mm R:1700mm → LEFT CLEAR! Decide "left"
Turn left, remember "left" for all future turns
```

---

<a name="motor-control"></a>
## 12. Motor Control and Steering

### Forward Movement

```python
def forward(self, speed=BASE_SPEED):
    self.motor.set_speed(speed)
    self.current_speed = speed
```

**Purpose:** Makes robot move forward at specified speed

**Parameters:**
- `speed`: Motor speed (0-100), defaults to `BASE_SPEED` (55)

**What it does:**
1. Sends speed command to motor
2. Records current speed in state variable

**Usage examples:**
```python
self.forward()          # Move at BASE_SPEED (55)
self.forward(50)        # Move at SLOW_SPEED
self.forward(BASE_SPEED)  # Move at normal speed
```

### Stop

```python
def stop(self):
    self.motor.stop()
    self.current_speed = 0
```

**Purpose:** Stops the robot completely

**What it does:**
1. Calls motor's stop method (sets both direction pins LOW, PWM to 0)
2. Records speed as 0

### Steering

```python
def steer(self, angle):
    angle = max(-30, min(30, angle))
    servo_pos = SERVO_CENTER - angle
    self.servo_ctrl.move(SERVO_CHANNEL, int(servo_pos))
```

**Purpose:** Points wheels in specified direction

**How it works:**

**Step 1: Clamp angle**
```python
angle = max(-30, min(30, angle))
```
Limits angle to ±30° range:
- `angle = 50` → clamped to `30`
- `angle = -50` → clamped to `-30`
- `angle = 15` → unchanged

**Step 2: Convert to servo position**
```python
servo_pos = SERVO_CENTER - angle
```

**Formula explained:**
```
SERVO_CENTER = 92° (straight ahead)

Turn left (+15°):  servo_pos = 92 - 15 = 77°
Straight (0°):     servo_pos = 92 - 0 = 92°
Turn right (-15°): servo_pos = 92 - (-15) = 107°
```

**Why subtract?**
Servo mechanics: 
- Lower servo angles → wheels turn left
- Higher servo angles → wheels turn right
- This is opposite of intuitive steering angles

**Step 3: Move servo**
```python
self.servo_ctrl.move(SERVO_CHANNEL, int(servo_pos))
```
- Converts to integer (servos need whole numbers)
- Commands servo to move to calculated position

**Example scenarios:**
```python
self.steer(30)   # Max left: servo → 62°
self.steer(0)    # Straight: servo → 92°
self.steer(-30)  # Max right: servo → 122°
```

### Smooth Steering

```python
def steer_smooth(self, target_angle):
    target_angle = max(-30, min(30, target_angle))
    self.steering_history.append(target_angle)
    
    if len(self.steering_history) > 0:
        smooth_angle = sum(self.steering_history) / len(self.steering_history)
        self.steer(smooth_angle)
```

**Purpose:** Applies steering with smoothing to prevent jerky movements

**How smoothing works:**

**Step 1: Clamp and store**
```python
target_angle = max(-30, min(30, target_angle))
self.steering_history.append(target_angle)
```
- Clamps angle to valid range
- Adds to history deque (maxlen=3)

**Step 2: Calculate average**
```python
smooth_angle = sum(self.steering_history) / len(self.steering_history)
```

**Example:**
```python
# Initial state
steering_history = []

# First call: steer_smooth(20)
steering_history = [20]
smooth_angle = 20 / 1 = 20° → steer(20)

# Second call: steer_smooth(10)
steering_history = [20, 10]
smooth_angle = 30 / 2 = 15° → steer(15)

# Third call: steer_smooth(5)
steering_history = [20, 10, 5]
smooth_angle = 35 / 3 = 11.67° → steer(11.67)

# Fourth call: steer_smooth(-5)
steering_history = [10, 5, -5]  # Oldest (20) removed automatically
smooth_angle = 10 / 3 = 3.33° → steer(3.33)
```

**Why smooth steering?**
- Prevents sudden wheel movements
- Reduces mechanical stress on servo
- Makes navigation more stable
- Improves wall-following consistency

**Trade-off:**
- Slight delay in response (uses last 3 commands)
- More predictable behavior overall

### Center Steering

```python
def steer_center(self):
    self.servo_ctrl.move(SERVO_CHANNEL, SERVO_CENTER)
    self.steering_history.clear()
```

**Purpose:** Returns steering to center position for straight driving

**What it does:**
1. Moves servo directly to 92° (center)
2. Clears steering history (fresh start for smoothing)

**When used:**
- Before/after turns
- When stopping
- When resetting navigation state

---

<a name="navigation"></a>
## 13. Navigation System

### Estimate Corridor Width

```python
def estimate_corridor_width(self):
    left = self.distance_left()
    right = self.distance_right()
    
    if left and right:
        total_width = left + right
        self.corridor_width_history.append(total_width)
        
        if len(self.corridor_width_history) >= 10:
            avg_width = sum(self.corridor_width_history) / len(self.corridor_width_history)
            return "wide" if avg_width > 750 else "narrow"
    
    return "unknown"
```

**Purpose:** Determines if robot is in a wide or narrow corridor section

**Process:**

**Step 1: Get wall distances**
```python
left = self.distance_left()
right = self.distance_right()
```

**Step 2: Calculate total width**
```python
if left and right:
    total_width = left + right
    self.corridor_width_history.append(total_width)
```

**Example:**
```
Left wall: 400mm
Right wall: 450mm
Total width: 400 + 450 = 850mm
```

**Step 3: Average recent measurements**
```python
if len(self.corridor_width_history) >= 10:
    avg_width = sum(self.corridor_width_history) / len(self.corridor_width_history)
```

Waits for at least 10 measurements before making decision

**Step 4: Classify corridor**
```python
return "wide" if avg_width > 750 else "narrow"
```

**Classification:**
- **Wide corridor:** Average width > 750mm
  - Uses `TARGET_WALL_WIDE = 450mm` for wall following
- **Narrow corridor:** Average width ≤ 750mm
  - Uses `TARGET_WALL_NARROW = 250mm` for wall following

**Why different targets?**
- Wide corridors: Robot can maintain more distance from walls
- Narrow corridors: Robot must stay closer to walls to avoid hitting opposite side

**Return values:**
- `"wide"`: Corridor > 750mm
- `"narrow"`: Corridor ≤ 750mm
- `"unknown"`: Not enough data yet, or walls not detected

### Calculate Steering

This is the main navigation function with a priority system:

```python
def calculate_steering(self):
    """Improved wall following + Compass heading correction"""
    left_dist = self.distance_left()
    right_dist = self.distance_right()
    front_dist = self.distance_front()
    
    corridor_type = self.estimate_corridor_width()
    
    # Adjust target based on corridor width
    if corridor_type == "narrow":
        target_distance = TARGET_WALL_NARROW
    else:
        target_distance = TARGET_WALL_WIDE
```

**Setup:**
1. Gets current sensor readings (left, right, front)
2. Determines corridor type
3. Sets appropriate target distance:
   - Narrow: 250mm
   - Wide/Unknown: 450mm

#### Priority 1: Front Obstacle Avoidance

```python
    steering = 0
    heading_correction = self.calculate_heading_correction()
    
    # PRIORITY 1: Front obstacle avoidance
    if front_dist and front_dist < FRONT_TURN_THRESHOLD:
        # Front obstacle detected - prepare to turn
        if left_dist and right_dist:
            # Steer away from closer wall
            if right_dist < left_dist:
                steering = 25  # Steer left away from right wall
                print(f"  Front obstacle! Steering LEFT (R:{right_dist} < L:{left_dist})")
            else:
                steering = -25  # Steer right away from left wall
                print(f"  Front obstacle! Steering RIGHT (L:{left_dist} < R:{right_dist})")
        self.forward(SLOW_SPEED)
        return max(-30, min(30, steering))
```

**When triggered:** Front distance < 700mm

**Logic:**
1. Detects wall ahead
2. Checks which side wall is closer
3. Steers AWAY from the closer wall
4. Slows down to SLOW_SPEED (50)
5. Returns immediately (overrides all other navigation)

**Example:**
```
Front: 600mm (obstacle!)
Left: 300mm
Right: 450mm

Right < Left? No
→ Steer RIGHT (-25°) away from left wall
```

**Why this works:**
- Approaching a corner from left → right wall is farther → steer right
- Approaching a corner from right → left wall is farther → steer left
- Naturally positions robot for the turn

#### Priority 2: Right Wall Following

```python
    # PRIORITY 2: Right wall following (preferred)
    if right_dist and right_dist < 1000:  # Right wall detected within 1000mm
        error = target_distance - right_dist
        derivative = error - self.last_right_error
        self.last_right_error = error
        
        # Calculate steering to maintain target distance
        steering = error * KP_WALL + derivative * KD_WALL
```

**When triggered:** Right wall within 1000mm

**PD Control explanation:**

**Proportional term (P):**
```python
error = target_distance - right_dist
steering += error * KP_WALL
```

**Example (wide corridor, target = 450mm):**
```
Right distance: 300mm
Error: 450 - 300 = +150mm (too close)
P contribution: 150 * 0.04 = +6° (steer left to get farther)

Right distance: 600mm
Error: 450 - 600 = -150mm (too far)
P contribution: -150 * 0.04 = -6° (steer right to get closer)
```

**Derivative term (D):**
```python
derivative = error - self.last_right_error
steering += derivative * KD_WALL
```

**Purpose:** Dampens oscillations

**Example:**
```
Last error: +100mm
Current error: +150mm
Derivative: +150 - 100 = +50 (error increasing - moving toward wall fast)
D contribution: 50 * 0.025 = +1.25° (adds extra left steering to slow approach)

Last error: +150mm
Current error: +100mm
Derivative: +100 - 150 = -50 (error decreasing - moving away well)
D contribution: -50 * 0.025 = -1.25° (reduces left steering, don't overshoot)
```

**Speed adjustment based on proximity:**
```python
        # Speed adjustment based on proximity
        if right_dist < MIN_SAFE_DISTANCE:
            self.forward(SLOW_SPEED)
            steering *= 1.5  # More aggressive steering when too close
        elif right_dist < MIN_SAFE_DISTANCE * 1.5:
            self.forward(int(BASE_SPEED * 0.75))
        else:
            self.forward(BASE_SPEED)
```

**Speed zones:**
```
< 180mm:  SLOW_SPEED (50), steering × 1.5 (aggressive)
180-270mm: 75% speed (41), normal steering
> 270mm:  BASE_SPEED (55), normal steering
```

**Example:**
```
Right distance: 150mm (< 180mm)
→ Speed: 50
→ Base steering: +5°
→ Applied steering: +5 * 1.5 = +7.5° (more aggressive correction)
```

#### Priority 3: Left Wall Following

```python
    # PRIORITY 3: Left wall following (if no right wall)
    elif left_dist and left_dist < 1000:  # Left wall detected within 1000mm
        error = target_distance - left_dist
        derivative = error - self.last_left_error
        self.last_left_error = error
        
        # Calculate steering (negative because we want to steer right when too close to left)
        steering = -(error * KP_WALL + derivative * KD_WALL)
```

**When triggered:** No right wall, but left wall within 1000mm

**Why negative steering?**

```
Target: 450mm
Left distance: 300mm (too close)
Error: +150mm
Base correction: +150 * 0.04 = +6°
Applied steering: -(+6°) = -6° (steer RIGHT away from left wall)
```

**Sign conversion:**
- Right wall following: Positive error → steer left (positive)
- Left wall following: Positive error → steer right (negative)

**Same speed adjustment logic:**
```python
        if left_dist < MIN_SAFE_DISTANCE:
            self.forward(SLOW_SPEED)
            steering *= 1.5
        elif left_dist < MIN_SAFE_DISTANCE * 1.5:
            self.forward(int(BASE_SPEED * 0.75))
        else:
            self.forward(BASE_SPEED)
```

#### Priority 4: No Walls Detected

```python
    # PRIORITY 4: No walls detected - go straight
    else:
        steering = 0
        self.forward(BASE_SPEED)
        self.last_left_error = 0
        self.last_right_error = 0
```

**When triggered:** No walls within 1000mm on either side

**Action:**
- Steering: 0° (straight)
- Speed: BASE_SPEED (55)
- Resets error history (fresh start when walls reappear)

#### Compass Correction Overlay

```python
    # Add compass correction (smaller weight so it doesn't override wall avoidance)
    steering += heading_correction * 0.5
    
    return max(-30, min(30, steering))
```

**Final step:**
1. Adds compass-based heading correction
2. Multiplied by 0.5 (reduced weight)
3. Clamps final result to ±30°

**Example:**
```
Wall-following steering: +10°
Heading correction: -4°
Combined: 10 + (-4 * 0.5) = 10 - 2 = +8°
Final steering: +8°
```

**Why 0.5 weight?**
- Wall avoidance is higher priority
- Compass provides gentle corrections
- Prevents compass from overriding safety behaviors

---

<a name="turning"></a>
## 14. 90-Degree Turn Execution

```python
def execute_90_degree_turn(self, direction):
    """Execute a compass-guided 90-degree turn"""
    print(f"\n  Executing 90° turn {direction.upper()}")
    self.navigation_mode = "turning"
    
    pre_turn_target = self.target_heading
```

**Setup:**
- Prints turn direction
- Changes mode to "turning" (disables wall-following)
- Saves current target heading

### Compass Failure Fallback

```python
    # Safety check: if no compass heading, use dead reckoning
    if pre_turn_target is None:
        print("  WARNING: No compass heading! Using dead reckoning...")
        self.steer_center()
        self.forward(SLOW_SPEED)
        time.sleep(0.3)  # Move forward
        
        # Execute turn by time
        self.forward(TURN_SPEED)
        if direction == 'right':
            self.steer(-PRE_TURN_STEER_ANGLE)
        else:
            self.steer(PRE_TURN_STEER_ANGLE)
        
        time.sleep(0.8)  # Turn for fixed time
        
        self.steer_center()
        self.forward(BASE_SPEED)
        time.sleep(0.3)
        self.navigation_mode = "wall_follow"
        return
```

**Dead reckoning mode** (if compass unavailable):

1. **Move forward:** 0.3 seconds straight
2. **Turn:** Steer ±30° for 0.8 seconds
3. **Continue:** Center steering and drive forward 0.3 seconds
4. **Resume:** Return to wall-following mode

**Why needed?**
If compass fails, robot can still complete turns (though less accurately).

### Calculate New Target Heading

```python
    if direction == 'right':
        new_target = normalize_angle(pre_turn_target + 90)
    else:
        new_target = normalize_angle(pre_turn_target - 90)
    
    print(f"  Current target: {pre_turn_target:.0f}° → New target: {new_target:.0f}°")
```

**Heading calculation:**

**Right turn:**
```python
new_target = normalize_angle(pre_turn_target + 90)
```

**Examples:**
```
Current: 0° → Right turn → 90°
Current: 90° → Right turn → 180°
Current: 270° → Right turn → 360° → normalizes to 0°
```

**Left turn:**
```python
new_target = normalize_angle(pre_turn_target - 90)
```

**Examples:**
```
Current: 90° → Left turn → 0°
Current: 0° → Left turn → -90° → normalizes to 270°
```

### Three-Phase Turn Sequence

#### Phase 1: Move Forward

```python
    # Move forward to clear obstacle
    print("  Moving forward...")
    self.steer_center()
    self.forward(SLOW_SPEED)
    time.sleep(0.1)
```

**Purpose:** Moves slightly forward to clear any front obstacles

**Duration:** 0.1 seconds at SLOW_SPEED (50)

**Distance traveled:** ~2-3cm (depending on actual speed calibration)

#### Phase 2: Execute Turn

```python
    # Execute turn
    print(f"  Turning to {new_target:.0f}°...")
    self.forward(TURN_SPEED)
    
    if direction == 'right':
        self.steer(-PRE_TURN_STEER_ANGLE)
    else:
        self.steer(PRE_TURN_STEER_ANGLE)
```

**Steering setup:**
- Right turn: Steer -30° (turn wheels right)
- Left turn: Steer +30° (turn wheels left)
- Speed: TURN_SPEED (50)

#### Phase 3: Compass-Guided Completion

```python
    turn_start_time = time.time()
    max_turn_time = 0.8
    
    while time.time() - turn_start_time < max_turn_time:
        self.update_lidar()
        current = self.heading()
        
        if current is not None:
            error = abs(angle_difference(new_target, current))
            
            if error < HEADING_TOLERANCE:
                print(f"  Turn complete! Heading: {current:.0f}° (error: {error:.1f}°)")
                break
            
            if error < 15:
                self.forward(SLOW_SPEED)
                if direction == 'right':
                    self.steer(-20)
                else:
                    self.steer(20)
        
        time.sleep(0.02)
```

**Turn monitoring loop:**

1. **Timeout protection:** Max 0.8 seconds
2. **Continuous LiDAR updates:** Maintains sensor data
3. **Check heading:** Compare current vs target
4. **Exit condition:** Error < 10° (HEADING_TOLERANCE)
5. **Fine adjustment:** When error < 15°, reduce steering to ±20°

**Example:**
```
Target: 90°
Current: 45° → Error: 45° → Keep turning at ±30°
Current: 75° → Error: 15° → Still turning at ±30°
Current: 82° → Error: 8° → COMPLETE! Exit loop
```

#### Finalization

```python
    self.target_heading = new_target
    
    self.steer_center()
    self.forward(BASE_SPEED)
    time.sleep(0.5)
    
    self.navigation_mode = "wall_follow"
    print(f"  Now facing {self.target_heading:.0f}° - continuing")
```

**Cleanup:**
1. Updates target_heading to new value
2. Centers steering
3. Resumes normal speed for 0.5 seconds
4. Returns to wall-following mode
5. Prints confirmation

---

<a name="mission"></a>
## 15. Main Mission Loop

### Mission Initialization

```python
def run_mission(self):
    print("\n" + "="*70)
    print(" WRO MISSION: NEW TURNING STRATEGY")
    print(" 1. Detect line with A3 sensor")
    print(" 2. Move forward until side > 1500mm")
    print(" 3. Turn to that side and remember direction")
    print(" 4. Repeat for all subsequent lines")
    print("="*70 + "\n")
    
    print("Calibrating compass...")
    for _ in range(20):
        self.update_lidar()
        time.sleep(0.05)
    self.set_initial_heading()
```

**Startup sequence:**
1. Prints mission strategy
2. Calibrates compass (20 readings over 1 second)
3. Sets initial heading
4. Prepares to start

### Main Loop Structure

```python
    try:
        print("Starting mission...")
        self.forward(BASE_SPEED)
        last_print = time.time()
        
        while True:
            self.update_lidar()
```

**Loop basics:**
- Starts moving forward immediately
- Updates LiDAR every iteration (~50Hz)
- Continues until stop conditions met

### Stop Conditions (After 12 Turns)

```python
            # ========================================================
            # CHECK STOP CONDITIONS (after 12 turns completed)
            # ========================================================
            if self.turn_count >= 12 and not self.waiting_for_clearance:
                current_heading = self.heading()
                front_dist = self.distance_front()
                
                if current_heading is not None and front_dist is not None:
                    heading_error = abs(angle_difference(self.target_heading, current_heading))
                    
                    if heading_error <= HEADING_TOLERANCE and front_dist < 1800:
                        print(f"\n{'='*70}")
                        print(" ✓ STOP CONDITIONS MET!")
                        print(f" Heading error: {heading_error:.1f}° (≤10°) ✓")
                        print(f" Front distance: {front_dist}mm (<1800mm) ✓")
                        print(f"{'='*70}")
                        break
```

**Two conditions must be met:**
1. **Heading error ≤ 10°:** Robot facing starting direction
2. **Front distance < 1800mm:** Robot near starting wall

**Why both?**
- Heading alone: Might stop mid-lap
- Distance alone: Might stop at wrong corner
- Combined: Confirms return to start position

### Phase 1: Line Detection

```python
            if not self.waiting_for_clearance and self.turn_count < 12:
                current_time = time.time()
                if not hasattr(self, 'last_line_detect_time_ignore'):
                    self.last_line_detect_time_ignore = 0
                
                LINE_IGNORE_TIME = 3.0
                
                if current_time - self.last_line_detect_time_ignore > LINE_IGNORE_TIME:
                    if self.detect_line():
                        self.last_line_detect_time_ignore = current_time
                        self.turn_count += 1
                        lap = (self.turn_count - 1) // 4 + 1
                        section = (self.turn_count - 1) % 4 + 1
                        
                        print(f"\n{'='*70}")
                        print(f" TURN #{self.turn_count} - LINE DETECTED")
                        print(f" LAP: {lap}/3 | SECTION: {section}/4")
                        print(f"{'='*70}")
```

**Line detection with debouncing:**

1. **Ignore timer:** Prevents detecting same line multiple times
2. **3-second cooldown:** After detecting a line, ignores lines for 3 seconds
3. **Turn counter:** Increments on each detection
4. **Progress calculation:**
   - Lap = (turn_count - 1) ÷ 4 + 1
   - Section = (turn_count - 1) mod 4 + 1

**Examples:**
```
Turn 1: Lap 1, Section 1
Turn 4: Lap 1, Section 4
Turn 5: Lap 2, Section 1
Turn 12: Lap 3, Section 4
```

### First Turn Decision

```python
                        if self.turn_direction is None:
                            print(" FIRST TURN - Need to decide direction")
                            print(" Moving forward to check side clearance...")
                            self.waiting_for_clearance = True
                            self.line_detected_time = time.time()
                        else:
                            print(f" Using memorized direction: {self.turn_direction.upper()}")
                            self.waiting_for_clearance = True
                            self.line_detected_time = time.time()
```

**First turn (turn_direction = None):**
- Enters "waiting for clearance" state
- Will check LiDAR to decide direction

**Subsequent turns (turn_direction = 'left' or 'right'):**
- Uses memorized direction
- Enters "waiting for clearance" state
- Will turn when memorized side becomes clear

### Phase 2: Clearance Detection & Turning

```python
            if self.waiting_for_clearance:
                steering = self.calculate_steering()
                self.steer_smooth(steering)
                
                if self.turn_direction is None:
                    left_dist = self.distance_left()
                    right_dist = self.distance_right()
                    
                    left_clear = left_dist and left_dist > LIDAR_SIDE_CLEARANCE_THRESHOLD
                    right_clear = right_dist and right_dist > LIDAR_SIDE_CLEARANCE_THRESHOLD
                    
                    if left_clear or right_clear:
                        self.turn_direction = self.decide_turn_direction_lidar()
                        print(f"\n>>> TURN DIRECTION SET: {self.turn_direction.upper()} <<<")
                        print(f">>> This direction will be used for all 12 turns <<<\n")
                        
                        self.execute_90_degree_turn(self.turn_direction)
                        self.waiting_for_clearance = False
```

**First turn - direction decision:**
1. Continues driving forward with wall-following
2. Checks both sides continuously
3. When either side > 2000mm: Decides direction
4. Executes turn immediately
5. Memorizes direction for future use

**Subsequent turns - wait for memorized side:**

```python
                else:
                    if self.turn_direction == 'left':
                        check_dist = self.distance_left()
                    else:
                        check_dist = self.distance_right()
                    
                    if check_dist and check_dist > LIDAR_SIDE_CLEARANCE_THRESHOLD:
                        print(f"\n>>> {self.turn_direction.upper()} side CLEAR ({check_dist}mm) - TURNING <<<\n")
                        self.execute_90_degree_turn(self.turn_direction)
                        self.waiting_for_clearance = False
```

**Logic:**
1. Only checks the memorized side
2. When that side > 2000mm: Turns
3. Consistent turning pattern throughout mission

**Safety timeout:**

```python
                if time.time() - self.line_detected_time > 5.0:
                    print("TIMEOUT - forcing turn")
                    if self.turn_direction is None:
                        self.turn_direction = self.decide_turn_direction_lidar()
                    self.execute_90_degree_turn(self.turn_direction)
                    self.waiting_for_clearance = False
```

**Purpose:** If waiting > 5 seconds, forces turn anyway

**Prevents:** Getting stuck if LiDAR readings are incorrect

### Normal Navigation

```python
            if self.navigation_mode == "wall_follow" and not self.waiting_for_clearance:
                steering = self.calculate_steering()
                self.steer_smooth(steering)
```

**When active:** Between turns, when not in turn sequence

**Does:** Normal wall-following with PD control and compass correction

### Status Updates

```python
            if time.time() - last_print >= 0.5 and not self.waiting_for_clearance:
                last_print = time.time()
                
                left = self.distance_left() or 0
                right = self.distance_right() or 0
                front = self.distance_front() or 0
                corridor = self.estimate_corridor_width()
                current_heading = self.heading() or 0
```

**Prints every 0.5 seconds:**

**After 12 turns:**
```python
                if self.turn_count >= 12:
                    heading_error = abs(angle_difference(self.target_heading, current_heading))
                    print(f"[12/12 COMPLETE] Checking stop: H_err={heading_error:.1f}° | Front={front}mm | "
                          f"L:{left:4.0f} R:{right:4.0f}")
```

**During mission:**
```python
                else:
                    print(f"[{self.turn_count}/12] {corridor.upper()} | "
                          f"H:{current_heading:.0f}° → {target_heading_str}° (Δ{heading_offset_str}°) | "
                          f"L:{left:4.0f} R:{right:4.0f} F:{front:4.0f}mm | "
                          f"Dir:{self.turn_direction or 'TBD'}")
```

**Example output:**
```
[3/12] WIDE | H:180° → 180° (Δ+0.5°) | L:420 R:480 F:2400mm | Dir:right
```

### Mission Completion

```python
        # After breaking from loop (stop conditions met)
        self.stop()
        self.steer_center()
        
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        self.cleanup()
```

**Normal completion:**
1. Stops motor
2. Centers steering

**Keyboard interrupt:** Catches Ctrl+C

**Finally block:** Always calls cleanup

### Cleanup Function

```python
def cleanup(self):
    print("\nShutting down...")
    try:
        self.stop()
        self.steer_center()
        time.sleep(0.1)
        self.lidar.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        self.lidar.close()
    except:
        pass
    try:
        self.motor.cleanup()
    except:
        pass
    try:
        GPIO.cleanup()
    except:
        pass
    print("Done\n")
```

**Shutdown sequence:**
1. Stops motor
2. Centers steering
3. Sends LiDAR stop command (0xA5 0x25)
4. Closes LiDAR serial connection
5. Cleans up motor PWM
6. Releases all GPIO pins

---

<a name="flow"></a>
## 16. Complete Mission Flow

**Mission Summary:**

```
1. START
   ├─ Initialize all systems
   ├─ Calibrate compass
   └─ Begin driving forward

2. DETECT CORNER (Line sensor < 2600)
   ├─ Increment turn counter
   └─ Enter "waiting for clearance" state

3. FIRST TURN ONLY
   ├─ Drive forward while checking LiDAR
   ├─ When side > 2000mm: Decide direction
   ├─ Memorize direction (left or right)
   └─ Execute 90° turn

4. TURNS 2-12
   ├─ Drive forward while checking memorized side
   ├─ When memorized side > 2000mm: Turn
   └─ Execute 90° turn in memorized direction

5. BETWEEN TURNS
   ├─ Wall-following navigation (PD control)
   ├─ Compass heading correction
   └─ Speed adjustment based on wall proximity

6. AFTER 12 TURNS
   ├─ Check heading error (≤10°?)
   ├─ Check front distance (<1800mm?)
   └─ If both true: STOP

7. CLEANUP
   └─ Shutdown all systems safely
```

**Key Features:**

1. **Consistent turning:** Always turns same direction (determined by first corner)
2. **Reliable detection:** 2000mm threshold distinguishes obstacle vs open sides
3. **Safe navigation:** Multiple priority levels prevent collisions
4. **Precise stopping:** Uses both heading and position to verify return to start
5. **Robust design:** Handles sensor failures and edge cases gracefully

---

**End of Explanation**

This code represents a complete autonomous navigation system for WRO Future Engineers, using LiDAR-based decision making, compass-guided turning, and adaptive wall-following to complete 3 laps (12 turns) of an obstacle course.
