#include <Encoder.h>
#include "PID_routine.h"

// Define pins for the encoder
#define encoderPinA 2
#define encoderPinB 4

double KP = 2.0;
double KI = 0.0;
double KD = 0.0;

// Create an Encoder object
// Encoder encoder1(encoderPinA, encoderPinB);

float pidController(long target, long encoderCount, float *ePrevious, float *eIntegral, unsigned long *previousTime){
    
    // Measure the time
    unsigned long currentTime = micros();
    float deltaT = ((float)(currentTime - *previousTime))/1.0e6;

    // Compute the error, eDerivative, and eIntegral
    int e = target - encoderCount;
    float eDerivative = (e - *ePrevious) / deltaT;
    *eIntegral = *eIntegral + (e * deltaT);
    
    // Compute the PID control signal
    float output = (KP * e) + (KD * eDerivative) + (KI * *eIntegral);

    // Update variables for the next iteration
    *previousTime = currentTime;
    *ePrevious = e;

    return output;
}

void goToTargetPos (Encoder &encoder1, long targetPos, uint8_t motor_pin_1, uint8_t motor_pin_2, uint8_t enable_pin){
  // declare variable
  float error, lastError, eIntegral, output;
  unsigned long lastTime = micros();
  int motorSpeed;  
  
  while(true){
    // Read the encoder value
    
    long encoderValue = encoder1.read();

    output = pidController(targetPos, encoderValue, &lastError, &eIntegral, &lastTime);
    
    motorSpeed = constrain(output, -255, 255);

    if (motorSpeed > 0){
      digitalWrite(motor_pin_1, HIGH);
      digitalWrite(motor_pin_2, LOW);
      analogWrite(enable_pin, abs(motorSpeed));
    }
    else if (motorSpeed < 0){
      digitalWrite(motor_pin_1, LOW);
      digitalWrite(motor_pin_2, HIGH);
      analogWrite(enable_pin, abs(motorSpeed));
    }
    else{
      digitalWrite(motor_pin_1, LOW);
      digitalWrite(motor_pin_2, LOW);
      // Stop the motor
      analogWrite(enable_pin, 0);
      break;
    }

  Serial.print(targetPos);
  Serial.print(", ");
  Serial.println(encoder1.read()); 

    // Add a delay to avoid excessive readings
    delay(100);
  }
}




