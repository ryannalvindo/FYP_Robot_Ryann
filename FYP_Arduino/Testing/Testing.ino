#include <Encoder.h>
#include "PID.h"

// Define interrupt pin
#define homeInterrupt 2

// Define pins for the encoders
#define encoderPinA_Motor1 2
#define encoderPinB_Motor1 3
#define encoderPinA_Motor2 6
#define encoderPinB_Motor2 7

// Define pins for motor control (L298N)
#define motor1Pin1 8
#define motor1Pin2 9
#define enablePin_Motor1 10
#define motor2Pin1 11
#define motor2Pin2 12
#define enablePin_Motor2 113

// Constants for PID control
#define KP 1.0 // Proportional gain
#define KI 0.0 // Integral gain
#define KD 0.0 // Derivative gain

// Target encoder values for both motors
#define TARGET_ENCODER_VALUE_Motor1 100
#define TARGET_ENCODER_VALUE_Motor2 -100

// Create Encoder objects for both motors
Encoder encoderMotor1(encoderPinA_Motor1, encoderPinB_Motor1);
Encoder encoderMotor2(encoderPinA_Motor2, encoderPinB_Motor2);

// Create PID controllers for both motors
PID pidMotor1(KP, KI, KD);
PID pidMotor2(KP, KI, KD);

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);

  // Set motor control pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enablePin_Motor1, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enablePin_Motor2, OUTPUT);

  // Set enable pins to HIGH to enable the motors
  digitalWrite(enablePin_Motor1, HIGH);
  digitalWrite(enablePin_Motor2, HIGH);

  // Attach interrupt handlers for encoder pins
  attachInterrupt(digitalPinToInterrupt(homeInterrupt), motor_ISR, CHANGE);
}

void loop() {
  // Read the encoder values (handled by interrupts)
  
  // Read the encoder value
  long encoderValue = encoderMotor1.read();

  // Print the encoder value to the Serial monitor
  Serial.println(encoderValue);

  delay(10);


}

// Interrupt service routine for Motor encoder
void motor_ISR() {
  Serial.println("INTERRUPTED \n\n\n\n ");
  while(true){
    // Read the encoder value
    long encoderValueMotor1 = encoderMotor1.read();
    long encoderValueMotor2 = encoderMotor2.read();
  
    // Compute PID output for Motor 1
    float pidOutputMotor1 = pidMotor1.compute(TARGET_ENCODER_VALUE_Motor1, encoderValueMotor1);
    float pidOutputMotor2 = pidMotor2.compute(TARGET_ENCODER_VALUE_Motor2, encoderValueMotor2);
  
    // Set motor speed based on PID output for Motor 1
    int motorSpeed1 = constrain(pidOutputMotor1, -255, 255);
    int motorSpeed2 = constrain(pidOutputMotor2, -255, 255);
  
    // Set motor direction based on the sign of the motor speed for Motor
    if (motorSpeed1 > 0) {
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
    } else {
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      break;
    }
    if (motorSpeed2 > 0) {
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
    } else {
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
    }
  
    // Set motor speed for Motor
    analogWrite(enablePin_Motor1, abs(motorSpeed1));
    analogWrite(enablePin_Motor2, abs(motorSpeed2));
    delay(10);
  }
}
