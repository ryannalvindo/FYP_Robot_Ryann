#include <Encoder.h>

// Define pins for the encoders (use interrupts pin in one of the signals)
#define encoderPinA_Motor1 2 
#define encoderPinB_Motor1 4
#define encoderPinA_Motor2 3
#define encoderPinB_Motor2 5

// Create Encoder objects for both motors
Encoder encoderMotor1(encoderPinA_Motor1, encoderPinB_Motor1);
Encoder encoderMotor2(encoderPinA_Motor2, encoderPinB_Motor2);

void setup()
{
  // Initialize Serial communication
  Serial.begin(115200);
}

void loop()
{

  // Read the encoder value
  long encoderValue1 = encoderMotor1.read();
  long encoderValue2 = encoderMotor2.read();

  Serial.print("encoderPinA_Motor1:");
  Serial.print(encoderValue1);
  Serial.print("\t");

  Serial.print("encoderPinB_Motor1:");
  Serial.println(encoderValue2);

  delay(10);
}
