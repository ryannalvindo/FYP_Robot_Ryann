// Debugger
#define BAILEY_MOTOR_D false
#define RYANN_MOTOR_D true
#define ENCODER_DEBUG true


#include <Encoder.h>

// Define pins for the encoder
#define encoderPinA 2
#define encoderPinB 4

// Define pins for motor control
#if BAILEY_MOTOR_D
#define motorPin 5
#define enablePin 9
#elif RYANN_MOTOR_D
#define motorPin1 8
#define motorPin2 9
#define enablePin 7
#endif

// PID constants
#define KP 100   // Proportional gain
#define KI 0.05  // Integral gain
#define KD 0.05 // Derivative gain

// Target encoder value
// #define TARGET_ENCODER_VALUE 100
long TARGET_ENCODER_VALUE = 0;
unsigned long lastTime;

// Create an Encoder object
Encoder myEncoder(encoderPinA, encoderPinB);

// Variables for PID control
float error, lastError, integral, derivative, output;

// Variables for motor control
int motorSpeed;

void setup()
{
  // Initialize Serial communication
  Serial.begin(115200);

// Set motor control pins as outputs
#if BAILEY_MOTOR_D
Serial.println("BAILEY SETTING IS USED");
  pinMode(motorPin, OUTPUT);
#elif RYANN_MOTOR_D
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
#endif
  pinMode(enablePin, OUTPUT);

  // Set enablePin to HIGH to enable the motor
  digitalWrite(enablePin, HIGH);
}

void loop()
{
  // Read the encoder value
  long encoderValue = myEncoder.read();

  // Print the encoder value to the Serial monitor
  Serial.print("Encoder: ");
  Serial.print(encoderValue);
  Serial.print("\t");
  while (Serial.available() == 0)
  {
    // Do nothing
  }

  while (Serial.available() > 0)
  {

    // Read the next character
    TARGET_ENCODER_VALUE = Serial.parseInt(SKIP_ALL);
    Serial.println(TARGET_ENCODER_VALUE);
    Serial.flush();
  }

  Serial.print("Target: ");
  Serial.println(TARGET_ENCODER_VALUE);
//  Serial.print("\t");
#if BAILEY_MOTOR_D
  goToTargetEncoderValue(TARGET_ENCODER_VALUE, motorPin, 0, enablePin);
#elif RYANN_MOTOR_D
  goToTargetEncoderValue(TARGET_ENCODER_VALUE, motorPin1, motorPin2, enablePin);
#endif
}

void goToTargetEncoderValue(long targetEncoderValue, uint8_t motor_pin_1, uint8_t motor_pin_2, uint8_t enable_pin)
{

  // declare variable
  static float error, lastError, integral, derivative, output;
  static unsigned long lastTime;
  static int motorSpeed;

  while (true)
  {

    // Read the encoder value
    long encoderValue = myEncoder.read();

    unsigned long now = micros();
    double timeChange = (double)(now - lastTime)/1.0e6;
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
    motorSpeed = constrain(output/100, -255, 255);  //the speed is the ratio of the output


    // Set motor direction based on the sign of the motor speed
    if (motorSpeed > 0)
    {
      digitalWrite(motor_pin_1, HIGH);
#if RYANN_MOTOR_D
      digitalWrite(motor_pin_2, LOW);
#endif
    }
    else if (motorSpeed < 0)
    {
      digitalWrite(motor_pin_1, LOW);
#if RYANN_MOTOR_D
      digitalWrite(motor_pin_2, HIGH);
#endif
    }
    else
    {
#if RYANN_MOTOR_D
      digitalWrite(motor_pin_1, LOW);
      digitalWrite(motor_pin_2, LOW);
#endif
    }

    // Set motor speed
    analogWrite(enable_pin, abs(motorSpeed));

#if ENCODER_DEBUG
    // Debug monitoring
    Serial.print("encoder:" + String(encoderValue) + "\t");
    Serial.print("target:" + String(targetEncoderValue) + "\t");
    Serial.print("output:" + String(abs(output)) + "\t");
    Serial.println("motorSpeed:" + String((motorSpeed)));
#endif

    // Check if motor has reached target encoder value
    if (abs(error) < 10)
    {
      // Stop the motor
      analogWrite(enable_pin, 0);
      break;
    }
    

    // Add a delay to avoid excessive readings
    delay(100);
  }
}
