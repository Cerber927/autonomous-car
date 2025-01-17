#include <Arduino.h>
#include <AS5047P.h>
#include <BTS7960.h>
#include <Servo.h>
#include <FastPID.h>

#define AS5047P_CHIP_SELECT_PORT 10
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000
#define DEGREES_PER_REVOLUTION 360
#define PRESCALER 64
#define SAMPLING_FREQUENCY 250
#define PID_SAMPLING_FREQUENCY 25

#define STOP 0
#define FORWARD 1
#define BACKWARD 2

#define STRAIGHT 0
#define LEFT 1
#define RIGHT 2

#define MAX_SERVO_POSITION 150
#define MIN_SERVO_POSITION 60
#define CENTER_SERVO_POSITION 111

#define DISTANCE_PER_REVOLUTION 0.02353

const uint8_t L_EN = 3;
const uint8_t R_EN = 4;
const uint8_t L_PWM = 6;
const uint8_t R_PWM = 5;
const uint8_t servo_signal = 9;

float pidI = 0;
float prevError = 0;
const float pidMin = -60; // Minimum PID output
const float pidMax = 60;  // Maximum PID output
int outputSpeed = 0;

const float kp = 0.1; // Proportional gain
const float ki = 0.5; // Integral gain (why integral gain higher than proportional gain? not very common)
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
  int mode;
  int speed;
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
void setupTimer();
void parseCommand(String input);

void setup()
{
  setupTimer();

  steering.attach(servo_signal);

  sei(); // Enable global interrupts

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

  float deltaAngle = handleRollover(currentAngle - prevAngle);
  long deltaTime = currentTime - prevTime;

  float currentSpeed = calculateCurrentSpeed(deltaAngle, deltaTime);

  if (currentTime - pidSamplingTime >= 1000000 / PID_SAMPLING_FREQUENCY)
  {
    // the output is integer why to cast the output again to integer?
    outputSpeed = abs(constrain(pid_motor.step(command.speed, currentSpeed), -60, 60));
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

    if (command.distance > 0)
    {
      passDistance(currentAngle, prevAngle);
    }

    steer(command.direction);

  }
  prevAngle = currentAngle;
  prevTime = currentTime;
}

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

float calculateCurrentSpeed(float deltaAngle, unsigned long deltaTime)
{
  float rpm = (deltaAngle / deltaTime / 6) * 1000000;
  float current = 0.025 * rpm; // test
  return current;
}

void stop()
{
  motorController.Stop();
  pid_motor.clear();
  prevMode = command.mode;
}

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

// why to use 2 arguments, isn't it possible to implement it with delta angle only
void passDistance(float currentAngle, float prevAngle)
{
  // If the angle of the motor surpasses 360 or 0 degree, then distance decreases
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

void setupTimer()
{
  TCCR1A = 0; // Set entire TCCR1A register to 0
  TCCR1B = 0; // Same for TCCR1B
  TCNT1 = 0;  // Initialize counter value to 0

  OCR1A = SAMPLING_FREQUENCY; // Set the compare match register

  // Set prescaler and mode
  TCCR1B |= (1 << WGM12);              // CTC mode (Clear Timer on Compare)
  TCCR1B |= (1 << CS11) | (1 << CS10); // Set prescaler to 64

  // Enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

void parseCommand(String input)
{
  // Split string based on commas
  Serial.println(input);
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
    command.speed = constrain(command.speed, -64, 64); // the speed of motor should be in the range of (-64, 64)
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
