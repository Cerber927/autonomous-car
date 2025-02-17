#include <Arduino.h>
#include <AS5047P.h>
#include <BTS7960.h>
#include <Servo.h>
#include <FastPID.h>

#define AS5047P_CHIP_SELECT_PORT 10
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000
#define DEGREES_PER_REVOLUTION 360
#define SAMPLING_FREQUENCY 250
#define PID_SAMPLING_FREQUENCY 25   // speed pid controller works per 40ms

#define STOP 0
#define FORWARD 1
#define BACKWARD 2

#define MAX_SERVO_POSITION 150
#define MIN_SERVO_POSITION 60
#define CENTER_SERVO_POSITION 111

#define DISTANCE_PER_REVOLUTION 0.02353   // 0.02353m per rotation of the motor

const uint8_t L_EN = 3;
const uint8_t R_EN = 4;
const uint8_t L_PWM = 6;
const uint8_t R_PWM = 5;
const uint8_t servo_signal = 9;

const float pidMin = -64; // Minimum PID output
const float pidMax = 64;  // Maximum PID output
int outputSpeed = 0;

const float kp = 0.1; // Proportional gain
const float ki = 0.5; // Integral gain
const float kd = 0;   // Derivative gain

const int output_bits = 8;
const bool output_signed = true;

float prevAngle = 0;
unsigned long prevTime = 0;
unsigned long pidSamplingTime = 0;
int prevMode = STOP;

FastPID pid_motor(kp, ki, kd, PID_SAMPLING_FREQUENCY, output_bits, output_signed);
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);
BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);
Servo steering;

struct Command // The structure of the command read from the serial monitor
{
  int mode;         // forward, backward, stop
  int speed;        // speed of the car, from 0.3 to 1m/s
  float distance;
  float direction;
};
Command command;

float handleRollover(float deltaAngle);
float calculateCurrentSpeed(float deltaAngle, unsigned long deltaTime);

void runMotor(int mode, int signal);
void stop();
void steer(float direction);

void passDistance(float currentAngle, float prevAngle);
void parseCommand(String input);

void setup()
{
  steering.attach(servo_signal);

  Serial.begin(115200);

  while (!as5047p.initSPI())
  {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(5000);
  }

  command.mode = STOP;
  command.speed = 0;
  command.distance = 0;
  command.direction = 0;

  steering.write(CENTER_SERVO_POSITION);
  motorController.Enable();

  prevAngle = as5047p.readAngleDegree();
  prevTime = micros();
}

// is the loop frequency same each time, if no then it's problem. maybe use timer interrupts
void loop()
{
  if (Serial.available() > 0)
  {
    String input = Serial.readStringUntil('\n');
    if (input) // Prevent unintended reset to 0
    {
      parseCommand(input);
    }
  }

  float currentAngle = as5047p.readAngleDegree();
  unsigned long currentTime = micros();

  float deltaAngle = handleRollover(currentAngle - prevAngle);  // calculate the delta angle of 2 loops
  long deltaTime = currentTime - prevTime;

  float currentSpeed = calculateCurrentSpeed(deltaAngle, deltaTime);  // calculate the current setpoint of the motor

// speed pid controller works per 40ms
  if (currentTime - pidSamplingTime >= 1000000 / PID_SAMPLING_FREQUENCY)
  {
    outputSpeed = abs(constrain(pid_motor.step(command.speed, currentSpeed), pidMin, pidMax));
    pidSamplingTime = currentTime;
  }

  if (command.mode == STOP)
  {
    stop();
  }
  else
  {
    // if the mode changes and current mode is not stop, stop first
    if (prevMode != command.mode)
    {
      stop();
    }

    runMotor(command.mode, outputSpeed);

    // if command.distance>0, command.distance decreases one revolution when the motor angle surpasses 360 or 0 degree
    if (command.distance > 0)
    {
      passDistance(currentAngle, prevAngle);
    }

    steer(command.direction);

  }
  prevAngle = currentAngle;
  prevTime = currentTime;
}

// convert delta angle to the range of (-180, 180)
float handleRollover(float deltaAngle)
{
  if (deltaAngle < -DEGREES_PER_REVOLUTION / 2)
  {
    deltaAngle += DEGREES_PER_REVOLUTION;
  }
  else if (deltaAngle > DEGREES_PER_REVOLUTION / 2)
  {
    deltaAngle -= DEGREES_PER_REVOLUTION;
  }

  return deltaAngle;
}

// calculate the current setpoint of the motor by delta angle and delta time
float calculateCurrentSpeed(float deltaAngle, unsigned long deltaTime)
{
  float rpm = (deltaAngle / deltaTime / 6) * 1000000;
  float current = 0.025 * rpm; // 0.025 * rpm = setpoint of the motor
  return current;
}

// stop the motor
void stop()
{
  motorController.Stop();
  pid_motor.clear();
  prevMode = command.mode;
}

// run the motor with the given mode and signal
void runMotor(int mode, int signal)
{
  if (mode == FORWARD)
  {
    motorController.TurnRight(signal);
  }
  else if (mode == BACKWARD)
  {
    motorController.TurnLeft(signal);
  }
}

// map the command direction to the servo signal(from 60 to 150 degree) and steer the car
void steer(float direction)
{
  float steerSignal;
  if (direction < 0)
  {
    steerSignal = CENTER_SERVO_POSITION + (CENTER_SERVO_POSITION - MIN_SERVO_POSITION) * direction;
  }
  else
  {
    steerSignal = CENTER_SERVO_POSITION + (MAX_SERVO_POSITION - CENTER_SERVO_POSITION) * direction;
  }
  steering.write((int)steerSignal);
}

// calculate the distance of the car based on the angle of the motor, if the distance is less than 0, stop the car
void passDistance(float currentAngle, float prevAngle)
{
  // If the angle of the motor surpasses 360 or 0 degree, then distance decreases one revolution
  if ((prevAngle > 300 && currentAngle < 60) || (prevAngle < 60 && currentAngle > 300))
  {
    command.distance -= DISTANCE_PER_REVOLUTION; // For one rotation of the motor, the car moves 0.02353m
  }

  if (command.distance <= 0)
  {
    command.speed = 0;
    command.mode = STOP;
  }
}

// parse the command read from the serial monitor, and set the command structure
// if speed>0, set command.mode=FORWARD, if speed<0, set command.mode=BACKWARD, if speed=0, set command.mode=STOP
void parseCommand(String input)
{
  // Split string based on commas
  // Serial.println(input);
  int speedIndex = input.indexOf("speed:");
  int distanceIndex = input.indexOf("distance:");
  int directionIndex = input.indexOf("direction:");

  if (speedIndex != -1)
  {
    int endIndex = input.indexOf(',', speedIndex);
    command.speed = (int)(input.substring(speedIndex + 6, endIndex).toFloat() * 63.7484);
    if (command.speed > 0)
    {
      command.mode = FORWARD;
    }
    else if (command.speed < 0)
    {
      command.mode = BACKWARD;
    }
    else
    {
      command.mode = STOP;
    }
    command.speed = constrain(command.speed, pidMin, pidMax); // the setpoint of motor should be in the range of (-64, 64), 64 is 1m/s
  }

  if (distanceIndex != -1)
  {
    int endIndex = input.length();
    command.distance = input.substring(distanceIndex + 9, endIndex).toFloat();
  }

  if (directionIndex != -1)
  {
    int endIndex = input.length();
    command.direction = constrain(input.substring(directionIndex + 10, endIndex).toFloat(), -1, 1);
  }
}
