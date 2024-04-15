// These define's must be placed at the beginning before #include "TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG 0 // 2
#define _TIMERINTERRUPT_LOGLEVEL_ 0

#define USE_TIMER_1 true

#if (defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) ||                   \
     defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MINI) || defined(ARDUINO_AVR_ETHERNET) ||                        \
     defined(ARDUINO_AVR_FIO) || defined(ARDUINO_AVR_BT) || defined(ARDUINO_AVR_LILYPAD) || defined(ARDUINO_AVR_PRO) ||                            \
     defined(ARDUINO_AVR_NG) || defined(ARDUINO_AVR_UNO_WIFI_DEV_ED) || defined(ARDUINO_AVR_DUEMILANOVE) || defined(ARDUINO_AVR_FEATHER328P) ||    \
     defined(ARDUINO_AVR_METRO) || defined(ARDUINO_AVR_PROTRINKET5) || defined(ARDUINO_AVR_PROTRINKET3) || defined(ARDUINO_AVR_PROTRINKET5FTDI) || \
     defined(ARDUINO_AVR_PROTRINKET3FTDI))
#define USE_TIMER_2 true
#warning Using Timer1, Timer2
#else
#define USE_TIMER_3 true
#warning Using Timer1, Timer3
#endif

// Define libraries
#include <TimerInterrupt.h>
#include <Encoder.h>
#include "eeprom_routine.h"

// DEBUG
#define ENCODER_DEBUG true
#define EEPROM_DEBUG true
#define LOOP_DEBUG false
#define SERIAL_INPUT_DEBUG true
#define BAILEY_MOTOR_D false
#define RYANN_MOTOR_D true

// Define pins for the encoders (use interrupts pin in one of the signals)
#define encoderPinA_Motor1 2
#define encoderPinB_Motor1 4
#define encoderPinA_Motor2 3
#define encoderPinB_Motor2 5
// Define pins for motor control
#if BAILEY_MOTOR_D
#define motorPin 5
#define enablePin 9
#elif RYANN_MOTOR_D
#define motorPin1 8
#define motorPin2 9
#define enablePin 10
#endif

#define TIMER1_INTERVAL_MS 1000 // Timer 1 is more accurate compared to timer 2
#define TIMER1_DURATION_MS 0

// PID constants
#define KP 100  // Proportional gain
#define KI 0.05 // Integral gain
#define KD 0.05 // Derivative gain

// Create Encoder objects for both motors
Encoder encoderMotor1(encoderPinA_Motor1, encoderPinB_Motor1);
Encoder encoderMotor2(encoderPinA_Motor2, encoderPinB_Motor2);

// Define global variables
long address1 = 0;   // EEPROM address length is 1024
long address2 = 512; // address 2 will start in the middle
long encoderValue1;  // encoder value to be accessible throughout the whole file
long encoderValue2;  // encoder value to be accessible throughout the whole file
// Variables for PID control
float error, lastError, integral, derivative, output;

void setup()
{

  Serial.begin(115200);

  clearEeprom();

  // Set motor control pins as outputs
#if BAILEY_MOTOR_D
  Serial.println("BAILEY SETTING IS USED");
  pinMode(motorPin, OUTPUT);
#elif RYANN_MOTOR_D
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
#endif
  pinMode(enablePin, OUTPUT);

  // First reading of the encoder value
  encoderValue1 = encoderMotor1.read();
  encoderValue2 = encoderMotor2.read();

  // Initialize timer interrupt for reading
  ITimer1.init();

  // Using ATmega328 used in UNO => 16MHz CPU clock ,

  if (ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, TimerHandler1, TIMER1_DURATION_MS))
  {
    Serial.print(F("Starting  ITimer1 OK, millis() = "));
    Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));

#if SERIAL_INPUT_DEBUG
  Serial.println("Type \"H\" to start on Homing routine");
#endif
}

void loop()
{
  static bool homingStart = false;

#if SERIAL_INPUT_DEBUG
  static char userInput;
  while (Serial.available() > 0)
  {
    // Read the next character
    userInput = Serial.read();
    Serial.println("You typed: " + String(userInput));
    Serial.flush();
  }

  if (userInput == 'H')
  {
    homingStart = true;
    userInput = 0;
  }

#endif

  if (homingStart)
  {
    ITimer1.stopTimer();
    Serial.println("Stopping the encoder reading now");
    // TODO: applying goToTargetPos for 2 wheels
    do
    {
      long targetMotor1 = readingEeprom(address1);
      long targetMotor2 = readingEeprom(address2);

      goToTargetEncoderValue(targetMotor1, motorPin1, motorPin2, enablePin);
      address1 = address1 - 4;
      address2 = address2 - 4;
    } while (address1 != 0 && address2 != 512);

    // Final step going to origin
    goToTargetEncoderValue(0, motorPin1, motorPin2, enablePin);
    homingStart = false;
    userInput = 0;
  }
  else
  {
    // Keep reading on the encoder value
    encoderValue1 = encoderMotor1.read();
    encoderValue2 = encoderMotor2.read();
  }

#if LOOP_DEBUG
  Serial.print("encoderMotor1:");
  Serial.print(readingEeprom(address1 - 4));
  Serial.print(" is stored in ");
  Serial.print(address1 - 4);
  Serial.print("\t\t encoderMotor2:");
  Serial.print(readingEeprom(address2 - 4));
  Serial.print(" is stored in ");
  Serial.println(address2 - 4);
#endif

  delay(100);
}

void TimerHandler1()
{
  static long oldEncoderValue1 = 0;
  static long oldEncoderValue2 = 0;

#if TIMER_INTERRUPT_DEBUG
  Serial.print("ITimer1 called, millis() = ");
  Serial.println(millis());
#endif

#if ENCODER_DEBUG
  Serial.print("encoderPinA_Motor1:");
  Serial.print(encoderValue1);
  Serial.print("\t");

  Serial.print("encoderPinB_Motor1:");
  Serial.println(encoderValue2);
#endif
  // Do quit the timer when the old encoder values are the same as the new read values
  if (encoderValue1 == oldEncoderValue1 && encoderValue2 == oldEncoderValue2)
  {
    return;
  }

  // increment the new addresses by 4 byte
  address1 = address1 + 4;
  address2 = address2 + 4;

  // Writting encoder data from global variable into EEPROM
  writingEeprom(encoderValue1, address1);
  writingEeprom(encoderValue2, address2);

#if EEPROM_DEBUG
  Serial.print("encoder_Motor1:");
  Serial.print(readingEeprom(address1));
  Serial.print(" is stored in ");
  Serial.print(address1);
  Serial.print("\t\t encoder_Motor2:");
  Serial.print(readingEeprom(address2));
  Serial.print(" is stored in ");
  Serial.println(address2);
#endif

  // update the old encoder value
  oldEncoderValue1 = encoderValue1;
  oldEncoderValue2 = encoderValue2;
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
    long encoderValue = encoderMotor1.read();

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
    motorSpeed = constrain(output / 100, -255, 255); // the speed is the ratio of the output

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
