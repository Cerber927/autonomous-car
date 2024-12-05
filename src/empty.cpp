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
#define CENTER_SERVO_POSITION 109

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
AS5047P encoder(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);
BTS7960 motorDriver(L_EN, R_EN, L_PWM, R_PWM);
Servo steering;

struct Command // The structure of the command read from the serial monitor
{
    int mode;
    int speed;
    float distance;
    int direction;
};
Command command;

float handleRollover(float deltaAngle);
float calculateCurrentSpeed(float deltaAngle, unsigned long deltaTime);

void runMotor(int mode, int signal);
void stop();
void steer(int direction);

void passDistance(float currentAngle, float prevAngle);
void setupTimer();
void parseCommand(String input);

void setup() {
    // put your setup code here, to run once:
    steering.attach(servo_signal);
    Serial.begin(115200);

    while (!encoder.initSPI())
    {
        Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
        delay(5000);
    }

    steering.write(CENTER_SERVO_POSITION);
    motorDriver.Enable();

    prevAngle = encoder.readAngleDegree();
    prevTime = micros();

}

void loop() {
    // put your main code here, to run repeatedly:
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        if (input) // Prevent unintended reset to 0
        {
        parseCommand(input);
        }
    }

    float currentAngle = encoder.readAngleDegree();
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
  // float current = 0.02309 * rpm + 3.577;    // the mapping in the air
  // float current = 0.02358 * rpm + 5.685; // the mapping on the ground
  float current = 0.025 * rpm; // test
  return current;
}

void stop()
{
  motorDriver.Stop();
  pid_motor.clear();
  prevMode = command.mode;
}

void runMotor(int mode, int signal)
{
  if (mode == FORWARD)
  {
    motorDriver.TurnRight(signal);
  }
  else if (mode == BACKWARD)
  {
    motorDriver.TurnLeft(signal);
  }
}

void steer(int direction)
{
  if (direction == LEFT)
  {
    steering.write(MIN_SERVO_POSITION);
  }
  else if (direction == RIGHT)
  {
    steering.write(MAX_SERVO_POSITION);
  }
  else
  {
    steering.write(CENTER_SERVO_POSITION);
  }
}

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

void parseCommand(String input)
{
    

}