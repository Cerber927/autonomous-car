## 🏎️ROS2-Based Model Car Control System
### 📌Overview
- An autonomous model car with ROS2-based system.
- The system consists of three nodes running on a Raspberry Pi, with commands sent remotely from a laptop via SSH. 
- Incoroperate with IMU sensor BNNO055 to receive real-time orientation data.
- Actuation command sends to Arduino-based control system using PySerial. For more details about motor control, please check [autonomous-car](https://github.com/vb-ee/autonomous-car)
---
## 📃System Architecture
### 🔹Hardware Components 
- Raspberry Pi 4:  Runs the ROS2 nodes for sensor reading, and command processing.
- Arduino Uno: Executes movement commands received from Raspberry Pi.
- Adafruit 9-DOF Absolute Orientation IMU [(BNO055)](https://www.adafruit.com/product/2472): Measures real-time orientation data.
### 🔹ROS2 Nodes
| **Node Name**     | **Function** |
|-------------------|-------------|
| **IMU Node** | Reads real-time orientation data from the IMU sensor and publishes it to `/imu_euler`. |
| **Command Publisher Node** | Allows the user to send movement commands to `/car`. |
| **Subscriber Node** | Subscribes to both `/imu_euler` and `/car`, compares real-time data with the command, and sends corrections to the Arduino via PySerial. |
### 🔹Command Format
```
speed:<value> distance:<value> direction:<value> angle:<value>
```
- speed (`-1.0 - 1.0 m/s`) → Desired speed of the car (> 0: forward, < 0: backward).
- distance (`meters`) → Distance to move.
- direction (`-1.0 - 1.0`) → Steering input (-1.0: full left, 1.0: full right, 0.0: straight).
- angle (`0 - 360`) → Target angle, compared with IMU data.
---
## 💼Workflow
### ① Launch the nodes
- After `source` the workspace, run the Launch file `car_launch.py`. Three nodes are run simultaneously.
``` markdown
ros2 launch python_parameters car_launch.py
```
- If you launch the file successfully, you will see the IMU data is published orientation data twice a second.
### ② Publish the command
- Publish actuation command
```markdown
ros2 topic pub -1 /car std_msgs/msg/String "{data: speed:20 distance:1 direction:-1 angle: 0}"
```
- Or publish command through the `.sh` file
```markdown
./command.sh
```
### ③ Subscriber Node Compares and Sends Command to Arduino:
- The subscriber node reads the current angle from /imu_euler node and the specified angle from /car node.
- If the real-time angle deviates from the desired angle, it sends correction signals to the Arduino via PySerial.
```
put the publishing message?
```
---

## 🔎Control Logic and Use Scenarios
### 🔹Case 1
- Move straight.

**Example Command**
  ```
  speed:0.3 distance:8 direction:0 angle:275
  ```
**Correction Mechanism**

-
- 

### 🔹Case 2
- Turn at a certain angle.

**Example Command**
```
speed:0.3 distance:8 direction:0.8 angle:-1
```
**Correction Mechanism**

-

## 🔧Configuration

### 🔹Authority
- Change the authority if needed.
```
chmod a+x file_name
```
- In our case we have `.sh` file to change authority.
```
sudo ./add_r_w_4_devices.sh

```
### 🔹IMU BNO055
- i2c package is needed to run the IMU node.
- Check if the library is installed on your system.
```
dpkg -l | grep libi2c-dev

```
- Install i2c llibrary
```
sudo apt-get install libi2c-dev
```

### 🔹PySerial
- Add dependency in `package.xml` file in `python_parameter`to configure the Pyserial library.
- [Using Python Packages with ROS2](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html)
```
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>python3-serial</depend> 
```





