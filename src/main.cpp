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

struct Command  // The structure of the command read from the serial monitor
{
  int mode;
  int speed;
  float distance;
};
Command command;

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
void parseCommand(String input);

float prevAngle = 0;
unsigned long prevTime = 0;

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
    String input = Serial.readStringUntil('\n');  // Read the input string until newline, might have some read issue like before
    parseCommand(input);                          // Parse and store the command
  }

  float currentAngle = as5047p.readAngleDegree();
  unsigned long currentTime = micros();

  // mode = 0 forward, mode = 1 backward, other value stop. by default, no input for mode then mode = 0, moves forward
  // by default, no input for distance then distance = 0, it runs infinitely.
  if ((command.mode != 0 && command.mode != 1) || command.distance < 0){
    stop();
  }
  else
  {
    float deltaAngle = handleRollover(currentAngle - prevAngle);
    long deltaTime = currentTime - prevTime;

    float currentSpeed = calculateCurrentSpeed(deltaAngle, deltaTime);
    float pidOutput = pid(command.speed, currentSpeed);
    outputSpeed = (int)constrain(currentSpeed + pidOutput, 20, 120);

    Serial.print("speed : ");
    Serial.println(command.speed);

    prevAngle = currentAngle;
    prevTime = currentTime;

    if (command.distance > 0){
      detectDistance(command.distance, currentAngle, prevAngle);
    }

    if (command.mode = 0){        // move forward
    runForward(outputSpeed);
    }
    else if (command.mode = 1)    // move backward
    {
      runBackward(outputSpeed);
    }

  }

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

void parseCommand(String input) {
    // Split string based on commas
    int modeIndex = input.indexOf("mode:");
    int speedIndex = input.indexOf("speed:");
    int distanceIndex = input.indexOf("distance:");
    
    if (modeIndex != -1) {
        int endIndex = input.indexOf(',', modeIndex);
        command.mode = input.substring(modeIndex + 5, endIndex).toInt();
    }

    if (speedIndex != -1) {
        int endIndex = input.indexOf(',', speedIndex);
        command.speed = input.substring(speedIndex + 6, endIndex).toInt();
        command.speed = constrain(command.speed, 20, 120);    // the speed of motor should be in the range of (20, 120)
    }

    if (distanceIndex != -1) {
        int endIndex = input.length();
        command.distance = input.substring(distanceIndex + 9, endIndex).toFloat();
    }
}