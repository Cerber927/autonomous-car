#include <Arduino.h>
#include <AS5047P.h>
#include <BTS7960.h>
#include <Servo.h>
#include <FastPID.h>

#define FORWARD 1
#define BACKWARD -1
#define STOP 0
#define LEFT -1
#define RIGHT 1
#define STRAIGHT 0

#define CENTER_SERVO_POSITION 109
#define MIN_SERVO_POSITION 60
#define MAX_SERVO_POSITION 150
#define MAX_SPEED 60
#define MIN_SPEED -60
#define PID_SAMPLING_FREQUENCY 25  // delay time = 40ms

#define DEGREES_PER_REVOLUTION 360
#define DISTANCE_PER_REVOLUTION 0.02353   // For each rotation of the motor how far the car moves

class CarController {

public:
    CarController(uint8_t lEn = 3, uint8_t rEn = 4, uint8_t lPwm = 6, uint8_t rPwm = 5, uint8_t servoPin = 9, unsigned long baudRate = 115200, float kp = 0.1, float ki = 0.5, float kd = 0)
        : motorDriver(lEn, rEn, lPwm, rPwm), servoPin(servoPin), steeringServo(),
          pid(kp, ki, kd, PID_SAMPLING_FREQUENCY, 8, true), 
          baudRate(baudRate), prevTime(0), prevAngle(0), prevMode(STOP), currentSpeed(0), targetSpeed(0), lastSampleTime(0) {}

    void setup() {
        Serial.begin(baudRate);
        steeringServo.attach(servoPin);
        encoder.initSPI();
        motorDriver.Enable();
        prevAngle = encoder.readAngleDegree();
        prevTime = micros();
    }

    void readCommand() {
      if (Serial.available() > 0)
      {
          String input = Serial.readStringUntil('\n');
          if (input) // Prevent unintended reset to 0
          {
              parseCommand(input);
          }
      }
    }

private:
    BTS7960 motorDriver;
    uint8_t servoPin;
    Servo steeringServo;
    FastPID pid;
    AS5047P encoder;  // Encoder object handles angle reading

    unsigned long baudRate;
    unsigned long prevTime;
    float prevAngle;
    int prevMode;
    int currentSpeed;
    int targetSpeed;
    unsigned long lastSampleTime;
    
    void applyMotorSignal(int mode, int signal) {
        switch (mode) {
            case FORWARD:
                motorDriver.TurnRight(signal);
                break;
            case BACKWARD:
                motorDriver.TurnLeft(signal);
                break;
            case STOP:
                motorDriver.Stop();
                break;
        }
    }

    void parseCommand(String input) {
        // Example command format: "speed:20,distance:2,direction:0"
        // If the speed > 0 then mode = FORWARD, if the speed < 0 then mode = BACKWARD, if speed = 0 then mode = STOP
        int speed = 0, distance = 0, direction = STRAIGHT, mode = STOP;

        int speedIndex = input.indexOf("speed:");
        int distanceIndex = input.indexOf("distance:");
        int directionIndex = input.indexOf("direction:");

        if (speedIndex != -1) {
            speed = constrain(input.substring(speedIndex + 6).toInt(), -60, 60);
        }
        if (distanceIndex != -1) {
            distance = input.substring(distanceIndex + 9).toInt();
        }
        if (directionIndex != -1) {
            direction = input.substring(directionIndex + 10).toInt();
        }
        if (speed > 0) {
            mode = FORWARD;
        }
        else if (speed < 0) {
            mode = BACKWARD;
        }

        steer(direction);
        setTargetSpeed(mode, speed, distance);
    }

    void setTargetSpeed(int mode, int target, float distance) {
        targetSpeed = target;
        runMotorWithPID(mode, prevAngle, distance);
    }

    void runMotorWithPID(int mode, float prevAngle, float distance) { 
        if (mode == STOP || prevMode != mode) {
            stop();
        }
        else {
            float currentAngle = encoder.readAngleDegree();
            unsigned long currentTime = micros();

            float deltaAngle = handleRollover(currentAngle - prevAngle);
            unsigned long deltaTime = currentTime - prevTime;
            prevAngle = currentAngle;  // Update for the next calculation
            
            float currentSpeed = calculateCurrentSpeed(deltaAngle, deltaTime);
            
            if (distance > 0){
                passDistance(distance, deltaAngle);
            }

            if ((currentTime - lastSampleTime) >= (1000000 / PID_SAMPLING_FREQUENCY)) {  // Check if it's time to sample
                lastSampleTime = currentTime;  // Update the last sample time

                // Measure the change in angle and calculate the current speed
                float currentAngle = encoder.readAngleDegree();
                float deltaAngle = handleRollover(currentAngle - prevAngle);
                currentSpeed = calculateCurrentSpeed(deltaAngle, deltaTime);

                // Calculate PID output
                int pidOutput = abs(constrain(pid.step(targetSpeed, currentSpeed), -60, 60));

                // Apply calculated PID output as PWM signal to the motor
                applyMotorSignal(mode, constrain(pidOutput, MIN_SPEED, MAX_SPEED));                
            }
        }
        prevMode = mode;
    }

    void stop() {
        motorDriver.Stop();
        pid.clear();
        delay(500);  // Wait briefly to ensure a full stop
    }

    void steer(int direction) {
        int position = CENTER_SERVO_POSITION;
        switch (direction) {
            case LEFT:
                position = MIN_SERVO_POSITION;
                break;
            case RIGHT:
                position = MAX_SERVO_POSITION;
                break;
            case STRAIGHT:
                position = CENTER_SERVO_POSITION;
                break;
        }
        steeringServo.write(position);
    }

    void passDistance(float targetDistance, float deltaAngle) {
        if (targetDistance > 0) {
            targetDistance -= deltaAngle / DEGREES_PER_REVOLUTION * DISTANCE_PER_REVOLUTION;
        }
        else {
            stop();
            targetSpeed = 0;
        }
    }
    
    float calculateCurrentSpeed(float deltaAngle, unsigned long deltaTime) {
        float rpm = (deltaAngle / DEGREES_PER_REVOLUTION) / (deltaTime / 60000000.0);
        return 0.25 * rpm;
    }

    float handleRollover(float deltaAngle) {
        if (deltaAngle > DEGREES_PER_REVOLUTION / 2) {
            deltaAngle -= DEGREES_PER_REVOLUTION;
        } 
        else if (deltaAngle < -DEGREES_PER_REVOLUTION / 2) {
            deltaAngle += DEGREES_PER_REVOLUTION;
        }
        return deltaAngle;
    }

};
