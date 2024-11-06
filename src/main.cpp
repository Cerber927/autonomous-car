#include <Arduino.h>
#include <AS5047P.h>
#include <BTS7960.h>
#include <Servo.h>

#define AS5047P_CHIP_SELECT_PORT 10
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000
#define DEGREES_PER_REVOLUTION 360
#define PRESCALER 64
#define SAMPLING_FREQUENCY 250

#define STOP 0
#define FORWARD 1
#define BACKWARD 2

#define STRAIGHT 0
#define LEFT 1
#define RIGHT 2

#define DISTANCE_PER_REVOLUTION 0.02353

const uint8_t L_EN = 3;
const uint8_t R_EN = 4;
const uint8_t L_PWM = 6;
const uint8_t R_PWM = 5;

float pidI = 0;
float prevError = 0;
const float kp = 0.2;      // Proportional gain
const float ki = 0.02;     // Integral gain
const float kd = 0;        // Derivative gain
const float pidMin = -120; // Minimum PID output
const float pidMax = 120;  // Maximum PID output
int outputSpeed = 0;

struct Command // The structure of the command read from the serial monitor
{
  int mode;
  int speed;
  float distance;
  int direction;
};
Command command;
Servo myServo;

AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);
BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);

float pid(int setpoint, float current);
float handleRollover(float deltaAngle);
float calculateCurrentSpeed(float deltaAngle, unsigned long deltaTime);

void runForward(int signal);
void runBackward(int signal);
void stop();

void turnLeft();
void goStraight();
void turnRight();

void passDistance(float currentAngle, float prevAngle);
void setup_timer();
void parseCommand(String input);

float prevAngle = 0;
unsigned long prevTime = 0;
int preMode = STOP;

void setup()
{
  setup_timer();

  myServo.attach(9);

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

  motorController.Enable();

  prevAngle = as5047p.readAngleDegree();
  prevTime = micros();
}

void loop()
{
  if (Serial.available() > 0)
  {
    String input = Serial.readStringUntil('\n');
    if (input) // Prevent unintended reset to 0
    {
      Serial.println(input);
      parseCommand(input);
    }
  }

  float currentAngle = as5047p.readAngleDegree();
  unsigned long currentTime = micros();

  float deltaAngle = handleRollover(currentAngle - prevAngle);
  long deltaTime = currentTime - prevTime;

  float currentSpeed = calculateCurrentSpeed(deltaAngle, deltaTime);
  float pidOutput = pid(command.speed, currentSpeed);
  outputSpeed = (int)constrain(currentSpeed + pidOutput, 20, 120);

  // mode has priority, if the mode is STOP, then the car stops no matter what other parameters are
  // by default, no input for distance then distance = 0, it runs infinitely.
  if (command.mode == STOP)
  {
    stop();
  }
  else
  {
    // if the mode changes and current mode is not stop, stop first
    if (command.mode != preMode)
    {
      stop();
    }
    else if (command.mode == FORWARD)
    {
      runForward(outputSpeed);
    }
    else if (command.mode == BACKWARD) // move backward
    {
      runBackward(outputSpeed);
    }
    if (command.distance > 0)
    {
      passDistance(currentAngle, prevAngle);
    }
    if (command.direction == LEFT)
    {
      turnLeft();
    }
    else if (command.direction == RIGHT)
    {
      turnRight();
    }
    else
    {
      goStraight();
    }
  }
  prevAngle = currentAngle;
  prevTime = currentTime;
  preMode = command.mode;
}

float pid(int setpoint, float current)
{
  float error = (float)setpoint - current;
  pidI += ki * error;
  pidI = constrain(pidI, -1, 1); // the limiting between -1 and 1 basically means that integral part has no effect on the output
  float deltaError = error - prevError;
  float pidOutput = kp * error + pidI + kd * deltaError;
  pidOutput = constrain(pidOutput, pidMin, pidMax);
  prevError = error;
  return pidOutput;
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
  float rpm = abs((deltaAngle / deltaTime / 6) * 1000000);
  // float current = 0.02309 * rpm + 3.577;    // the mapping in the air
  float current = 0.02358 * rpm + 5.685;      // the mapping on the ground
  return current;
}

void runForward(int signal)
{
  motorController.TurnRight(signal);
}

void runBackward(int signal)
{
  motorController.TurnLeft(signal);
}

void stop()
{
  motorController.Stop();
}

void turnLeft()
{
  myServo.write(60);
}

void goStraight()
{
  myServo.write(105);
}

void turnRight()
{
  myServo.write(150);
}

void passDistance(float currentAngle, float prevAngle)
{
  // If the angle of the motor surpasses 360 or 0 degree, than distance decreases
  if ((prevAngle > 300 && currentAngle < 60) || (prevAngle < 60 && currentAngle > 300))
  {
    command.distance -= DISTANCE_PER_REVOLUTION; // For one rotation of the motor, the car moves 0.02353m
  }

  if (command.distance <= 0)
  {
    command.mode = STOP;
  }
}

void setup_timer()
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
  int modeIndex = input.indexOf("mode:");
  int speedIndex = input.indexOf("speed:");
  int distanceIndex = input.indexOf("distance:");
  int directionIndex = input.indexOf("direction:");

  if (modeIndex != -1)
  {
    int endIndex = input.indexOf(',', modeIndex);
    command.mode = input.substring(modeIndex + 5, endIndex).toInt();
  }

  if (speedIndex != -1)
  {
    int endIndex = input.indexOf(',', speedIndex);
    command.speed = input.substring(speedIndex + 6, endIndex).toInt();
    command.speed = constrain(command.speed, 0, 120); // the speed of motor should be in the range of (20, 120)
  }

  if (distanceIndex != -1)
  {
    int endIndex = input.length();
    command.distance = input.substring(distanceIndex + 9, endIndex).toFloat();
  }

  if (directionIndex != -1)
  {
    int endIndex = input.length();
    command.direction = input.substring(directionIndex + 9, endIndex).toInt();
  }
}

// interrupt subroutine, for now it's not used

// ISR(TIMER1_COMPA_vect)
// {
//   float currentSpeed = calculateCurrentSpeed(deltaAngle, deltaTime);
//   pidOutput = pid(setpoint, currentSpeed);
//   outputSpeed = (int)constrain(currentSpeed + pidOutput, 20, 120);
// }