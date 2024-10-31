#include <Arduino.h>
#include <AS5047P.h>
#include <BTS7960.h>

#define AS5047P_CHIP_SELECT_PORT 10
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000
#define DEGREES_PER_REVOLUTION 360
#define PRESCALER 64
#define SAMPLING_FREQUENCY 250

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
int outputSpeed = 20;

AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);
BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);

float pid(int setpoint, float current);
float handleRollover(float deltaAngle);
float calculateCurrentSpeed(float deltaAngle, unsigned long deltaTime);
void runForward(int signal);
void runBackward(int signal);
void stop();
void detectDistance(double distance, float currentAngle, float prevAngle);
void setup_timer();

int setpoint = 20;
float prevAngle = 0;
unsigned long prevTime = 0;

// How far should the car move, later we can read from the serial monitor
double distance = 2; // After 2 meters the motor stops

void setup()
{
  setup_timer();

  sei(); // Enable global interrupts

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

  // detectDistance(distance, currentAngle, prevAngle);

  float deltaAngle = handleRollover(currentAngle - prevAngle);
  long deltaTime = currentTime - prevTime;

  float currentSpeed = calculateCurrentSpeed(deltaAngle, deltaTime);
  float pidOutput = pid(setpoint, currentSpeed);
  outputSpeed = (int)constrain(currentSpeed + pidOutput, 20, 120);

  Serial.print("setpoint : ");
  Serial.println(setpoint);

  motorController.TurnLeft(outputSpeed);

  prevAngle = currentAngle;
  prevTime = currentTime;
}

float pid(int setpoint, float current)
{
  float error = (float)setpoint - current;
  // TODO: is there more better way to handle this
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

void detectDistance(double distance, float currentAngle, float prevAngle)
{
  if (distance <= 0)
  {
    stop();
  }

  // If the angle of the motor surpasses 360 or 0 degree, than distance decreases
  if ((prevAngle > 300 && currentAngle < 60) || (prevAngle < 60 && currentAngle > 300))
  {
    distance -= 0.0117647058823529; // For one rotation of the motor, the car moves 0.0117647m
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

// interrupt subroutine, for now it's not used

// ISR(TIMER1_COMPA_vect)
// {
//   float currentSpeed = calculateCurrentSpeed(deltaAngle, deltaTime);
//   pidOutput = pid(setpoint, currentSpeed);
//   outputSpeed = (int)constrain(currentSpeed + pidOutput, 20, 120);
// }