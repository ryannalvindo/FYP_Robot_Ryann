// These define's must be placed at the beginning before #include "TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG 2
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
#include <EEPROM.h>

// DEBUG
#define ENCODER_DEBUG true
#define EEPROM_DEBUG true

// Define pins for the encoders (use interrupts pin in one of the signals)
#define encoderPinA_Motor1 2
#define encoderPinB_Motor1 4
#define encoderPinA_Motor2 3
#define encoderPinB_Motor2 5

#define TIMER1_INTERVAL_MS 1000 // Timer 1 is more accurate compared to timer 2
#define TIMER1_DURATION_MS 0

// Create Encoder objects for both motors
Encoder encoderMotor1(encoderPinA_Motor1, encoderPinB_Motor1);
Encoder encoderMotor2(encoderPinA_Motor2, encoderPinB_Motor2);

// Define global variables
long address1 = 0;   // EEPROM address length is 1024
long address2 = 512; // address 2 will start in the middle

void setup()
{

  Serial.begin(115200);

  Serial.println(EEPROM.length());

  Serial.println(EEPROM.read(1022));
  Serial.println(EEPROM.read(1));
  return 0;

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
}

void loop()
{
}

void TimerHandler1()
{
#if TIMER_INTERRUPT_DEBUG
  Serial.print("ITimer1 called, millis() = ");
  Serial.println(millis());
#endif

  // Read the encoder value
  long encoderValue1 = encoderMotor1.read();
  long encoderValue2 = encoderMotor2.read();

#if ENCODER_DEBUG
  Serial.print("encoderPinA_Motor1:");
  Serial.print(encoderValue1);
  Serial.print("\t");

  Serial.print("encoderPinB_Motor1:");
  Serial.println(encoderValue2);
#endif

  // Writting ecoder data into EEPROM
  writingEeprom(encoderValue1, address1);
  writingEeprom(encoderValue2, address2);

#if EEPROM_DEBUG
  Serial.print("encoderPinA_Motor1:");
  Serial.print(readingEeprom(address1));
  Serial.print(" is stored in ");
  Serial.print(address1);
  Serial.print("\t\t encoderPinA_Motor2:");
  Serial.print(readingEeprom(address2));
  Serial.print(" is stored in ");
  Serial.println(address2);
#endif

  // increment the address by 4 byte
  address1 = address1 + 4;
  address2 = address2 + 4;
}

void writingEeprom(long inputNum, long eepromAddress)
{
  // Writing the encoder data into EEPROM
  EEPROM.write(eepromAddress, inputNum & 0xFF);             // Stores 0x38 in address 0
  EEPROM.write(eepromAddress + 1, (inputNum >> 8) & 0xFF);  // Stores 0x30 in address 1
  EEPROM.write(eepromAddress + 2, (inputNum >> 16) & 0xFF); // Stores 0x01 in address 2
  EEPROM.write(eepromAddress + 3, (inputNum >> 24) & 0xFF); // Stores 0x00 in address 3
}

long readingEeprom(long eepromAddress)
{
  // displaying the notification by reading from EEPROM
  long val = (long)EEPROM.read(eepromAddress) |
             ((long)EEPROM.read(eepromAddress + 1) << 8) |
             ((long)EEPROM.read(eepromAddress + 2) << 16) |
             ((long)EEPROM.read(eepromAddress + 3) << 24);

  return val;
}