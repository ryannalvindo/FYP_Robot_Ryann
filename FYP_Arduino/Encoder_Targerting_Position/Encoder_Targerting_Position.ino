// Debugger
#define BAILEY_MOTOR_D false
#define RYANN_MOTOR_D true
#define ENCODER_DEBUG false

#include <Encoder.h>
#include "PID_routine.h"

// Define pins for the encoder
#define encoderPinA 2
#define encoderPinB 4

// Define pins for motor control
#if BAILEY_MOTOR_D
#define motorPin 5
#define enablePin 9
#elif RYANN_MOTOR_D
#define motorPin1 8
#define motorPin2 9
#define enablePin 7
#endif

// Target encoder value
// #define TARGET_ENCODER_VALUE 100
long TARGET_ENCODER_VALUE = 1000;

// Create an Encoder object
Encoder myEncoder(encoderPinA, encoderPinB);

// Variables for PID control
float error, lastError, integral, derivative, output;

// Variables for motor control
int motorSpeed;

void setup()
{
  // Initialize Serial communication
  Serial.begin(115200);

// Set motor control pins as outputs
#if BAILEY_MOTOR_D
Serial.println("BAILEY SETTING IS USED");
  pinMode(motorPin, OUTPUT);
#elif RYANN_MOTOR_D
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
#endif
  pinMode(enablePin, OUTPUT);

  // Set enablePin to HIGH to enable the motor
  digitalWrite(enablePin, HIGH);
}

void loop()
{
  // Read the encoder value
  long encoderValue = myEncoder.read();

#if BAILEY_MOTOR_D
  goToTargetEncoderValue(TARGET_ENCODER_VALUE, motorPin, 0, enablePin);
#elif RYANN_MOTOR_D
  goToTargetPos(myEncoder, TARGET_ENCODER_VALUE, motorPin1, motorPin2, enablePin);
#endif
}