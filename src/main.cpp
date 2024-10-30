#include <Arduino.h>
#include <AS5047P.h>
#include <BTS7960.h>

#define AS5047P_CHIP_SELECT_PORT 10
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000
#define DEGREES_PER_REVOLUTION 360

const uint8_t L_EN = 3;
const uint8_t R_EN = 4;
const uint8_t L_PWM = 6;
const uint8_t R_PWM = 5;

float pidI = 0;
float prevError = 0;
const float kp = 0.1;      // Proportional gain
const float ki = 0.01;     // Integral gain
const float kd = 0;        // Derivative gain
const float pidMin = -100; // Minimum PID output
const float pidMax = 100;  // Maximum PID output

AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);
BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);

float pid(int setpoint, float current);
float handleRollover(float deltaAngle);
float calculateCurrentSpeed(float deltaAngle, unsigned long deltaTime);
float noiseSmooth(float rpm);
void runForward(int signal);
void runBackward(int signal);
void stop();

int setpoint = 0;
float prevAngle = 0;
unsigned long prevTime = 0;

const int windowSize = 100;
float readings[windowSize];      // Array to store sensor readings
int currentIndex = 0;            // Current index in the readings array
double sum = 711.7 * windowSize; // Sum of the readings in the window

void setup()
{
  Serial.begin(115200);

  while (!as5047p.initSPI())
  {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(5000);
  }

  motorController.Enable();

  prevAngle = as5047p.readAngleDegree();
  prevTime = micros();
}

void loop()
{
  if (Serial.available() > 0)
  {
    int newSetpoint = Serial.parseInt();
    if (newSetpoint != 0) // Prevent unintended reset to 0
    {
      setpoint = newSetpoint;
    }
  }

  float currentAngle = as5047p.readAngleDegree();
  unsigned long currentTime = micros();

  float deltaAngle = handleRollover(currentAngle - prevAngle);

  unsigned long deltaTime = currentTime - prevTime;

  float currentSpeed = calculateCurrentSpeed(deltaAngle, deltaTime);

  float pidOutput = pid(setpoint, currentSpeed);
  Serial.print("pidOutput : ");
  Serial.println(pidOutput);

  int targetSetpoint = (int)constrain(currentSpeed + pidOutput, 20, 120);
  Serial.print("setpoint : ");
  Serial.println(setpoint);

  motorController.TurnLeft(targetSetpoint);

  prevAngle = currentAngle;
  prevTime = currentTime;
}

float pid(int setpoint, float current)
{
  float error = (float)setpoint - current;
  // TODO: is there more better way to handle the noise
  if (abs(error) < 1)
  {
    pidI += ki * error;
    pidI = constrain(pidI, -1, 1);
    Serial.print("pid i : ");
    Serial.println(pidI);
  }
  else
  {
    pidI = 0;
  }

  float deltaError = error - prevError;
  Serial.print("error: ");
  Serial.println(error);
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
  float current = 0.02309 * rpm + 3.577;
  return current;
}

float noiseSmooth(float rpm)
{
  sum = sum - readings[currentIndex] + rpm;
  readings[currentIndex] = rpm;
  // Increment the index and wrap around if necessary
  currentIndex = (currentIndex + 1) % windowSize;
  // Calculate the average value over the window
  float averageValue = sum / (float)windowSize;
  return averageValue;
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