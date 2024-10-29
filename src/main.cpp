#include <Arduino.h>
#include <AS5047P.h>
#include <BTS7960.h>

#define AS5047P_CHIP_SELECT_PORT 10
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

const uint8_t L_EN = 3;
const uint8_t R_EN = 4;
const uint8_t L_PWM = 6;
const uint8_t R_PWM = 5;

float pidI = 0;
float prevError = 0;
const float kp = 1.0;   // Proportional gain
const float ki = 0;     // Integral gain
const float kd = 0;     // Derivative gain
const float pidMax = 0; // Maximum PID output
const float pidMin = 0; // Minimum PID output

AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);
BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);

float pid(float setpoint, float current);
void motorControl(float pidOutput);

int setpoint = 0;
float prevAngle = 0;
unsigned long prevTime = 0;
float deltaAngle = 0;

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
    setpoint = Serial.parseInt();
    Serial.println(setpoint);
  }
  Serial.println("tttt");

  // float currentAngle = as5047p.readAngleDegree();
  // unsigned long currentTime = micros();

  // if (currentAngle < prevAngle)
  // {
  //   deltaAngle = currentAngle - prevAngle;
  // }
  // else
  // {
  //   deltaAngle = currentAngle - prevAngle - 360;
  // }

  // unsigned long deltaTime = currentTime - prevTime;
  // float rpm = (deltaAngle / deltaTime / 6) * 1000000;
  // Serial.println(rpm);
  // motorController.TurnLeft(setpoint);

  // prevAngle = currentAngle;
  // prevTime = currentTime;
}

float pid(float setpoint, float current)
{
  float error = setpoint - current;
  pidI += ki * error;
  pidI = constrain(pidI, pidMin, pidMax);
  float deltaError = error - prevError;
  float pidOutput = kp * error + pidI + kd * deltaError;
  pidOutput = constrain(pidOutput, pidMin, pidMax);
  prevError = error;

  return pidOutput;
}

void motorControl(float pidOutput)
{
  // implement motor control here
}