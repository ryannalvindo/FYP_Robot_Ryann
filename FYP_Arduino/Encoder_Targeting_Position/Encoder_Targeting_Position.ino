// Debugger
#define BAILEY_MOTOR_D true
#define RYANN_MOTOR_D false
#define ENCODER_DEBUG false

#include <Encoder.h>
#include "PID_routine.h"

// Define pins for the encoders (use interrupts pin in one of the signals)
#define encoder_A_Left 2
#define encoder_B_Left 4
#define encoder_A_Right 3
#define encoder_B_Right 5
// Define pins for motor control
#if BAILEY_MOTOR_D
#define motorLeft 7
#define speedLeft 6
#define motorRight 10
#define speedRight 9
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
long targetmotorLeft = 13500;
long targetmotorRight = 0;

// Create Encoder objects for both motors
Encoder encoderLeft(encoder_A_Left, encoder_B_Left);
Encoder encoderRight(encoder_A_Right, encoder_B_Right);

void setup()
{
  // Initialize Serial communication
  Serial.begin(115200);

// Set motor control pins as outputs
#if BAILEY_MOTOR_D
  Serial.println("BAILEY SETTING IS USED");
  pinMode(motorLeft, OUTPUT);
  pinMode(speedLeft, OUTPUT);
  pinMode(motorRight, OUTPUT);
  pinMode(speedRight, OUTPUT);
#elif RYANN_MOTOR_D
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);
#endif
  

#if BAILEY_MOTOR_D
  goToTargetPos(encoderLeft, targetmotorLeft, encoderRight, targetmotorRight, motorLeft, speedLeft, motorRight, speedRight);
#elif RYANN_MOTOR_D
  goToTargetPos(encoderLeft, targetMotor1, encoderRight, targetMotor2, motor1Pin1, motor1Pin2, enablePin1, motor2Pin1, motor2Pin2, enablePin2);
#endif


}

void loop()
{

  //ask user to type in the serial monitor (limited to vowel and a-g)
  Serial.println("Type targeted position for right wheel");
  Serial.flush();
  while(Serial.available() == 0){
    //do nothing
  }
  while(Serial.available() > 0){
    targetmotorRight = Serial.parseInt(SKIP_ALL);  //read string until newline
  }
  goToTargetPos(encoderLeft, targetmotorLeft, encoderRight, targetmotorRight, motorLeft, speedLeft, motorRight, speedRight);

  delay(1000);

}
