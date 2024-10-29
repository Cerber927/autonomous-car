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
const float kp = 0.01;   // Proportional gain
const float ki = 0;     // Integral gain
const float kd = 0;     // Derivative gain
const float pidMin = -100; // Minimum PID output
const float pidMax = 100; // Maximum PID output

AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);
BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);

int pid(int setpoint, int current);
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
    // Serial.println(setpoint);
    int newSetpoint = Serial.parseInt();
    if (newSetpoint != 0) // Prevent unintended reset to 0
    {
      setpoint = newSetpoint;
    }
  }

  float currentAngle = as5047p.readAngleDegree();
  unsigned long currentTime = micros();

  if (currentAngle < prevAngle)
  {
    deltaAngle = currentAngle - prevAngle;
  }
  else
  {
    deltaAngle = currentAngle - prevAngle - 360;
  }

  unsigned long deltaTime = currentTime - prevTime;
  float rpm = abs((deltaAngle / deltaTime / 6) * 1000000);
  // todo
  // if the car is not moving, rpm can be very big, current setpoint can be bigger than 255. But actually it does not move
  int currentValue = (int) (0.02309 * rpm + 3.577);
  if (currentValue > 255) {
    currentValue = 0;
  }
  
  int pidOutput = pid(setpoint, currentValue);
  Serial.print("pidOutput : ");
  Serial.println(pidOutput);

  setpoint = constrain(setpoint + pidOutput, 20, 120);
  Serial.print("setpoint : ");
  Serial.println(setpoint);

  motorController.TurnLeft(setpoint);

  prevAngle = currentAngle;
  prevTime = currentTime;
}

int pid(int setpoint, int current)
{
  int error = setpoint - current;
  pidI += ki * error;
  pidI = constrain(pidI, pidMin, pidMax);
  int deltaError = error - prevError;
  Serial.print("error: ");
  Serial.println(error);
  int pidOutput = kp * error + pidI + kd * deltaError;
  pidOutput = constrain(pidOutput, pidMin, pidMax);
  prevError = error;
  return pidOutput;
}

void motorControl(float pidOutput)
{
  // implement motor control here
}