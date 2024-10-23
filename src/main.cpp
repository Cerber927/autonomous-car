#include <Arduino.h>
#include <AS5047P.h>
#include <BTS7960.h>

// define the chip select port.
#define AS5047P_CHIP_SELECT_PORT 10

// define the spi bus speed
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

// initialize a new AS5047P sensor object.
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);

const uint8_t L_EN = 3;
const uint8_t R_EN = 4;
const uint8_t L_PWM = 6;
const uint8_t R_PWM = 5;

void setup()
{
  // initialize the serial bus for the communication with your pc.
  Serial.begin(115200);

  // initialize the AS5047P sensor and hold if sensor can't be initialized.
  while (!as5047p.initSPI())
  {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(5000);
  }

  motorController.Enable();
}

void loop()
{
  // put your main code here, to run repeatedly:
}