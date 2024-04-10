#include <Encoder.h>

// Define pins for the encoder
#define encoderPinA 2
#define encoderPinB 3

// Define pins for motor control
#define motorPin1 8
#define motorPin2 9
#define enablePin 17  //A3

// PID constants
#define KP 1000 // Proportional gain
#define KI 0.005 // Integral gain
#define KD 0.0001 // Derivative gain

// Target encoder value
//#define TARGET_ENCODER_VALUE 100
long TARGET_ENCODER_VALUE = 0;
unsigned long lastTime;

// Create an Encoder object
Encoder myEncoder(encoderPinA, encoderPinB);

// Variables for PID control
float error, lastError, integral, derivative, output;

// Variables for motor control
int motorSpeed;

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
 

  // Set motor control pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // Set enablePin to HIGH to enable the motor
  digitalWrite(enablePin, HIGH);
}

void loop() {
  // Read the encoder value
  long encoderValue = myEncoder.read();

  // Print the encoder value to the Serial monitor
  Serial.print("Encoder: ");
  Serial.print(encoderValue);
  Serial.print("\t");

  
  while (Serial.available() > 0) {
  
    // Read the next character
    TARGET_ENCODER_VALUE = Serial.parseInt(SKIP_ALL);
    Serial.println(TARGET_ENCODER_VALUE);
    Serial.flush();
  }
  Serial.print("Target: ");
  Serial.print(TARGET_ENCODER_VALUE);
  Serial.print("\t");

  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);
  // Calculate the error between target and current encoder value
  error = TARGET_ENCODER_VALUE - encoderValue;

  // Calculate integral and derivative terms
  integral += error * timeChange;
  derivative = (error - lastError) / timeChange;

  // Calculate the output using PID control
  output = KP * error + KI * integral + KD * derivative;

  // Update last error
  lastError = error;
  lastTime = now;

  // Set motor speed based on PID output
  
  motorSpeed = constrain(output, -255, 255) * 0.55;
  if (abs(error) <= 500){
    motorSpeed = motorSpeed * 0.55;
  }
 

  // Set motor direction based on the sign of the motor speed
  if (motorSpeed > 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else if (motorSpeed < 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  } else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  }

  

  // Set motor speed
  analogWrite(enablePin, abs(motorSpeed));
  Serial.print("output:"+String(abs(output)) + "\t");
  Serial.println("motorSpeed:"+String(abs(motorSpeed)));

  // Check if motor has reached target encoder value
  if (abs(error) < 5) {
    // Stop the motor
    analogWrite(enablePin, 0);
  }

  // Add a delay to avoid excessive readings
  delay(100);
}
