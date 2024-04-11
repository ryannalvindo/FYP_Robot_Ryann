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
  Serial.begin(115200); 

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
  while (Serial.available() == 0){
    // Do nothing
  }
  
  while (Serial.available() > 0) {
  
    // Read the next character
    TARGET_ENCODER_VALUE = Serial.parseInt(SKIP_ALL);
    Serial.println(TARGET_ENCODER_VALUE);
    Serial.flush();
  }
  
  Serial.print("Target: ");
  Serial.println(TARGET_ENCODER_VALUE);
//  Serial.print("\t");
  
  goToTargetEncoderValue(TARGET_ENCODER_VALUE, motorPin1, motorPin2, enablePin );


}

void goToTargetEncoderValue(long targetEncoderValue, uint8_t motor_pin_1, uint8_t motor_pin_2, uint8_t enable_pin ){

  //declare variable
  static float error, lastError, integral, derivative, output;
  static unsigned long lastTime;
  static int motorSpeed;

  while(true){

    // Read the encoder value
    long encoderValue = myEncoder.read();
    
    unsigned long now = millis();
    double timeChange = (double)(now - lastTime);
    // Calculate the error between target and current encoder value
    error = targetEncoderValue - encoderValue;
    
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
//    if (abs(error) <= 500){
//      motorSpeed = motorSpeed * 0.55;
//      break;
//    }
   
  
    // Set motor direction based on the sign of the motor speed
    if (motorSpeed > 0) {
      digitalWrite(motor_pin_1, HIGH);
      digitalWrite(motor_pin_2, LOW);
    } else if (motorSpeed < 0) {
      digitalWrite(motor_pin_1, LOW);
      digitalWrite(motor_pin_2, HIGH);
    } else {
      digitalWrite(motor_pin_1, LOW);
      digitalWrite(motor_pin_2, LOW);
    }
  
      
    // Set motor speed
    analogWrite(enable_pin, abs(motorSpeed));

    // Debug monitoring
    Serial.print("encoder:"+String(encoderValue) + "\t");
    Serial.print("target:"+String(targetEncoderValue) + "\t");
    Serial.print("output:"+String(abs(output)) + "\t");
    Serial.println("motorSpeed:"+String(abs(motorSpeed)));
  
    // Check if motor has reached target encoder value
    if (abs(error) < 100) {
      // Stop the motor
      analogWrite(enable_pin, 0);
      break;
    }
  
    // Add a delay to avoid excessive readings
    delay(100);
  }  
  
}
