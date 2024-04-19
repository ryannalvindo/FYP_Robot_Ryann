#include <Encoder.h>
#include "PID_routine.h"

double KP = 2.0;
double KI = 0.0;
double KD = 0.0;


float pidController(long target, Encoder &encoder, float *ePrevious, float *eIntegral, unsigned long *previousTime)
{

  // Read the encoder value
  long encoderCount = encoder.read();

  // Measure the time
  unsigned long currentTime = micros();
  float deltaT = ((float)(currentTime - *previousTime)) / 1.0e6;
  
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


// Running the motor
bool motorRun(int motorSpeed, uint8_t motorPin1, uint8_t motorPin2, uint8_t enablePin)
{
  // margin of error is 10
  if (motorSpeed >= 100)  {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin, abs(motorSpeed));
  }
  else if (motorSpeed <= -100)  {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(enablePin, abs(motorSpeed));
  }
  else  {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    // Stop the motor
    analogWrite(enablePin, 0);
    return 0;
  }

  return 1;
}

void goToTargetPos(Encoder &encoder1, long targetPos1, Encoder &encoder2, long targetPos2, uint8_t motor1_pin_1, uint8_t motor1_pin_2, uint8_t enable_pin1, uint8_t motor2_pin_1, uint8_t motor2_pin_2, uint8_t enable_pin2)
{
  // declare variable
  float lastError_1, eIntegral_1, output_1, lastError_2, eIntegral_2, output_2;
  unsigned long lastTime_1 = micros();
  unsigned long lastTime_2 = micros();
  int motorSpeed_1, motorSpeed_2;
  bool motor1_status = 1;
  bool motor2_status = 1;

  do
  {
    output_1 = pidController(targetPos1, encoder1, &lastError_1, &eIntegral_1, &lastTime_1);
    output_2 = pidController(targetPos2, encoder2, &lastError_2, &eIntegral_2, &lastTime_2);

    motorSpeed_1 = constrain(output_1, -255, 255);
    motorSpeed_2 = constrain(output_2, -255, 255);

    motor1_status = motorRun(motorSpeed_1, motor1_pin_1, motor1_pin_2, enable_pin1);
    motor2_status = motorRun(motorSpeed_2, motor2_pin_1, motor2_pin_2, enable_pin2);

    Serial.print(targetPos1);
    Serial.print(", ");
    Serial.println(encoder1.read());

    Serial.print(targetPos2);
    Serial.print(", ");
    Serial.println(encoder2.read());

    // Add a delay to avoid excessive readings
    delay(100);
  }while (motor1_status + motor2_status);
}


