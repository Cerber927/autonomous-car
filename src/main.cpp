#include <Arduino.h>
#include <AS5047P.h>
#include <BTS7960.h>

#define AS5047P_CHIP_SELECT_PORT 10
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

const uint8_t L_EN = 3;
const uint8_t R_EN = 4;
const uint8_t L_PWM = 6;
const uint8_t R_PWM = 5;

AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);
BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);

float prevAngle = 0;
unsigned long prevTime = 0;

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
  float currentAngle = as5047p.readAngleDegree();
  unsigned long currentTime = micros();

  float deltaAngle = currentAngle - prevAngle;
  unsigned long deltaTime = currentAngle - prevTime;

  float rpm = (deltaAngle / (deltaTime / 6)) * 1000000;

  prevAngle = currentAngle;
  prevTime = currentTime;
}

float pid(float setpoint, float current)
{
  float error = setpoint - current;
  // implement PID control here
  float output = 0;

  return output;
}

void motorControl(float pidOutput)
{
  // implement motor control here
}