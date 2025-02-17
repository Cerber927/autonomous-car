# Autonomous Car Project

## Overview

This project is an Arduino-based autonomous car system that receives movement commands via serial communication from a Raspberry Pi. It processes speed, distance, and direction commands and controls a motor and steering servo accordingly. The system utilizes sensors and a PID controller for smooth and accurate movement.

## Hardware Components

- **Arduino Board** (Compatible with PlatformIO)
- **AS5047P Rotary Encoder** (Measures motor rotation and speed)
- **BTS7960 Motor Driver** (Controls motor movement and speed)
- **Servo Motor** (Steering control)
- **Raspberry Pi** (Sends serial commands via PySerial)

## Software Requirements

- **PlatformIO** (VSCode extension for building and uploading firmware)
- **Arduino Framework**
- **Libraries Used:**
  - `Arduino.h`
  - `AS5047P.h`
  - `BTS7960.h`
  - `Servo.h`
  - `FastPID.h`

## Function Descriptions

### `setup()`

- Initializes serial communication and hardware components.
- Checks connection to the AS5047P sensor.
- Sets initial steering position and enables the motor controller.
- Reads the initial angle from the rotary encoder.

### `loop()`

- Reads and parses serial commands if available.
- Calculates motor speed based on encoder data.
- Implements PID control for speed regulation.
- Controls motor and steering based on parsed commands.
- Monitors distance traveled and stops the car when necessary.

### `parseCommand(String input)`

- Parses serial input for speed, distance, and direction commands.
- Updates the command structure.
- Determines the driving mode (FORWARD, BACKWARD, STOP) based on speed.

### `handleRollover(float deltaAngle)`

- Ensures angle measurements remain within the range (-180, 180) degrees.

### `calculateCurrentSpeed(float deltaAngle, unsigned long deltaTime)`

- Computes current motor speed using encoder readings and time delta.

### `stop()`

- Stops the motor and clears the PID controller.

### `runMotor(int mode, int signal)`

- Drives the motor in the specified direction with a given speed signal.

### `steer(float direction)`

- Maps command direction to servo angles and adjusts steering accordingly.

### `passDistance(float currentAngle, float prevAngle)`

- Tracks distance traveled based on encoder angle changes.
- Stops the car when the commanded distance is reached.

## Serial Command Format

Commands are received via serial communication in the following format:

```
speed:<value>,distance:<value>,direction:<value>
```

- **speed**: Motor speed range in m/s (-1.0 to 1.0), mapped to a range of (-64 to 64). (For safety reasons the car speed is limited to max of 1 m/s)
- **distance**: Distance to travel in meters.
- **direction**: Steering direction (-1.0 to 1.0), where negative values turn left, positive values turn right, 0 goes straight.

### Example Commands

- `speed:0.5,distance:2.0,direction:0.2`
- `speed:0,distance:0,direction:0` (Stops the car)

## Building and Uploading

1. Install **PlatformIO** in **VSCode**.
2. Clone the project repository.
3. Open the project folder in VSCode.
4. Connect the Arduino board via USB.
5. Run the following command to build and upload the firmware:
   ```sh
   platformio run --target upload
   ```

## Raspberry Pi Repository

The Raspberry Pi code that communicates with this Arduino system is available in a separate repository. You can find it here:
[Autonomous Car Raspberry Pi Repository](https://github.com/Cerber927/ros2_ws_car.git)

## Notes

- The PID controller regulates speed every 40ms.
- Ensure the AS5047P sensor is properly connected before running.
- The motor will stop automatically if the specified distance is reached.

## Future Improvements

- Implement timer-based loop execution for consistent timing.
- Improve PID tuning for smoother speed control.
- Add obstacle detection using ultrasonic sensors.

---

This documentation provides a comprehensive overview of the current system's functionality. Modify and expand as needed as the project develops.
