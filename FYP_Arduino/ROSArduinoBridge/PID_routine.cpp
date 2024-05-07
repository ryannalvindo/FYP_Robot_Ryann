#include <Encoder.h>
#include "PID_routine.h"

double KP = 1.005;
double KI = 0.000;
double KD = 0 ; //0.25712;


float pidController(long target, long encoderCount, float *ePrevious, float *eIntegral, unsigned long *previousTime)
{

  // Read the encoder value
  // long encoderCount = encoder.read();

  // Measure the time
  unsigned long currentTime = micros();
  float deltaT = ((float)(currentTime - *previousTime)) / 1.0e6;
  
  // Compute the error, eDerivative, and eIntegral
  long e = target - encoderCount;
  float eDerivative = (e - *ePrevious) / deltaT;
  *eIntegral = *eIntegral + (e * deltaT);

  // Compute the PID control signal
  float output = (KP * e) + (KD * eDerivative) + (KI * *eIntegral);

  // Update variables for the next iteration
  *previousTime = currentTime;
  *ePrevious = e;

  //Serial.println(output);

  return output;
}


// Running the motor
bool motorRun(int motorSpeed, uint8_t motorPin, uint8_t speedPin)
{

  // margin of error is 10
  // Forward
  if (motorSpeed >= 100)  {
    digitalWrite(motorPin, LOW);         
    analogWrite(speedPin, abs(motorSpeed));
  }
  // Backward
  else if (motorSpeed <= -100)  {
    digitalWrite(motorPin, HIGH);        
    analogWrite(speedPin, abs(motorSpeed));
  }
  else  {
    digitalWrite(motorPin, LOW);
    // Stop the motor
    analogWrite(speedPin, 0);
    return 0;
  }

  return 1;
}

void goToTargetPos(Encoder &encoder_left, long target_pos_left, Encoder &encoder_right, long target_pos_right, uint8_t motorLeft, uint8_t speedLeft, uint8_t motorRight, uint8_t speedRight)
{
  // declare variable
  float last_error_left, eIntegral_left, output_left, last_error_right, eIntegral_right, output_right;
  unsigned long last_time_left = micros();
  unsigned long last_time_right = micros();
  int motor_speed_left, motor_speed_right;
  bool motor1_status = 1;
  bool motor2_status = 1;

  do
  {
    output_left = pidController(target_pos_left, encoder_left.read(), &last_error_left, &eIntegral_left, &last_time_left);
    // output right moves in different rotation and the encoder measures in negative way
    output_right = pidController(target_pos_right, (encoder_right.read() * -1 ), &last_error_right, &eIntegral_right, &last_time_right);

    motor_speed_left = constrain(output_left, -255, 255);
    motor_speed_right = constrain(output_right, -255, 255);

    motor1_status = motorRun(motor_speed_left, motorLeft, speedLeft);
    motor2_status = motorRun(motor_speed_right, motorRight, speedRight);

    Serial.print(target_pos_left);
    Serial.print(", ");
    Serial.print(encoder_left.read());
    Serial.print("\t\t\t");
    Serial.print(target_pos_right);
    Serial.print(", ");
    Serial.println((encoder_right.read()*-1));    // adjustment for right wheel

    // Add a delay to avoid excessive readings
    delay(100);
  }while (motor1_status + motor2_status);
}
