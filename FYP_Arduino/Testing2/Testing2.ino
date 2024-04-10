#include <Encoder.h>

// Define pins for the encoders
#define encoderPinA_Motor1 18
#define encoderPinB_Motor1 19
#define encoderPinA_Motor2 6
#define encoderPinB_Motor2 7

// Define pins for motor control (L298N)
#define motor1Pin1 8
#define motor1Pin2 9
#define enablePin_Motor1 10
#define motor2Pin1 11
#define motor2Pin2 12
#define enablePin_Motor2 13

// PID constants for motor 1 (CW)
#define KP_Motor1 1.0
#define KI_Motor1 0.0
#define KD_Motor1 0.0

// PID constants for motor 2 (CCW)
#define KP_Motor2 1.0
#define KI_Motor2 0.0
#define KD_Motor2 0.0

// Target encoder values for both motors
#define TARGET_ENCODER_VALUE_Motor1 100
#define TARGET_ENCODER_VALUE_Motor2 -100

// Create Encoder objects for both motors
Encoder encoderMotor1(encoderPinA_Motor1, encoderPinB_Motor1);
Encoder encoderMotor2(encoderPinA_Motor2, encoderPinB_Motor2);



// Variables for PID control
float errorMotor1, lastErrorMotor1, integralMotor1, derivativeMotor1, outputMotor1;
float errorMotor2, lastErrorMotor2, integralMotor2, derivativeMotor2, outputMotor2;

// Variables for motor control
int motorSpeed1, motorSpeed2;

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
//  pinMode(encoderPinA_Motor1, INPUT); 
//  pinMode(encoderPinB_Motor1, INPUT); 
//  pinMode(encoderPinA_Motor2, INPUT); 
//  pinMode(encoderPinB_Motor2, INPUT); 
//  
//  digitalWrite(encoderPinA_Motor1, LOW); // Set pin to LOW (pulled down)
//  digitalWrite(encoderPinB_Motor1, LOW); // Set pin to LOW (pulled down)
//  digitalWrite(encoderPinA_Motor2, LOW); // Set pin to LOW (pulled down)
//  digitalWrite(encoderPinB_Motor2, LOW); // Set pin to LOW (pulled down)


  

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
}



void loop() {
  // Read the encoder values for both motors
  long encoderValueMotor1 = encoderMotor1.read();
  long encoderValueMotor2 = encoderMotor2.read();

  // Print the encoder values to the Serial monitor
  Serial.print("Motor 1: ");
  Serial.print(encoderValueMotor1);
  Serial.print("\t Motor 2: ");
  Serial.println(encoderValueMotor2);

  // Calculate the errors between target and current encoder values for both motors
  errorMotor1 = TARGET_ENCODER_VALUE_Motor1 - encoderValueMotor1;
  errorMotor2 = TARGET_ENCODER_VALUE_Motor2 - encoderValueMotor2;

  // Calculate integral and derivative terms for both motors
  integralMotor1 += errorMotor1;
  integralMotor2 += errorMotor2;
  derivativeMotor1 = errorMotor1 - lastErrorMotor1;
  derivativeMotor2 = errorMotor2 - lastErrorMotor2;

  // Calculate the output using PID control for both motors
  outputMotor1 = KP_Motor1 * errorMotor1 + KI_Motor1 * integralMotor1 + KD_Motor1 * derivativeMotor1;
  outputMotor2 = KP_Motor2 * errorMotor2 + KI_Motor2 * integralMotor2 + KD_Motor2 * derivativeMotor2;

  // Update last errors for both motors
  lastErrorMotor1 = errorMotor1;
  lastErrorMotor2 = errorMotor2;

  // Set motor speeds based on PID outputs for both motors
  motorSpeed1 = constrain(outputMotor1, -255, 255);
  motorSpeed2 = constrain(outputMotor2, -255, 255);

  // Set motor directions based on the sign of the motor speeds for both motors
  if (motorSpeed1 > 0) {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  } else {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  }

  if (motorSpeed2 > 0) {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }

  // Set motor speeds for both motors
  analogWrite(enablePin_Motor1, abs(motorSpeed1));
  analogWrite(enablePin_Motor2, abs(motorSpeed2));

  // Check if both motors have reached target encoder values
  if (abs(errorMotor1) < 5 && abs(errorMotor2) < 5) {
    // Stop both motors
    analogWrite(enablePin_Motor1, 0);
    analogWrite(enablePin_Motor2, 0);
  }

  // Add a delay to avoid excessive readings
  delay(100);
}
