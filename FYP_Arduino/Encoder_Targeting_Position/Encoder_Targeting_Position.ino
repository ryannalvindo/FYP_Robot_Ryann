// Debugger
#define BAILEY_MOTOR_D true
#define RYANN_MOTOR_D false
#define ENCODER_DEBUG false

#include <Encoder.h>
#include "PID_routine.h"

// Define pins for the encoders (use interrupts pin in one of the signals)
#define encoderPinA_Motor1 2
#define encoderPinB_Motor1 4
#define encoderPinA_Motor2 3
#define encoderPinB_Motor2 5
// Define pins for motor control
#if BAILEY_MOTOR_D
#define motor1Pin 5
#define speed1Pin 9
#define motor2Pin 5
#define speed2Pin 9
#elif RYANN_MOTOR_D
#define motor1Pin1 8
#define motor1Pin2 9
#define enablePin1 7
#define motor2Pin1 10
#define motor2Pin2 11
#define enablePin2 12
#endif

// Target encoder value
// #define TARGET_ENCODER_VALUE 100
long targetMotor1 = 1000;
long targetMotor2 = 1000;

// Create Encoder objects for both motors
Encoder encoderMotor1(encoderPinA_Motor1, encoderPinB_Motor1);
Encoder encoderMotor2(encoderPinA_Motor2, encoderPinB_Motor2);

void setup()
{
  // Initialize Serial communication
  Serial.begin(115200);

// Set motor control pins as outputs
#if BAILEY_MOTOR_D
  Serial.println("BAILEY SETTING IS USED");
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(speed1Pin, OUTPUT);
  pinMode(speed2Pin, OUTPUT);
#elif RYANN_MOTOR_D
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);
#endif
  

#if BAILEY_MOTOR_D
  goToTargetPos(encoderMotor1, targetMotor1, encoderMotor2, targetMotor2, motor1Pin, 0, speed1Pin, motor2Pin, 0, speed2Pin);
#elif RYANN_MOTOR_D
  goToTargetPos(encoderMotor1, targetMotor1, encoderMotor2, targetMotor2, motor1Pin1, motor1Pin2, enablePin1, motor2Pin1, motor2Pin2, enablePin2);
#endif


}

void loop()
{

}