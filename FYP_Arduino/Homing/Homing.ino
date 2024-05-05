
// DEBUG
#define ENCODER_DEBUG true
#define EEPROM_DEBUG true
#define LOOP_DEBUG false
#define SERIAL_INPUT_DEBUG false
#define BUTTON_INPUT_DEBUG true
#define BAILEY_MOTOR_D false
#define RYANN_MOTOR_D true

// Define pins for the encoders (use interrupts pin in one of the signals)
#define encoderPinA_Motor1 2
#define encoderPinB_Motor1 4
#define encoderPinA_Motor2 3
#define encoderPinB_Motor2 5
// Define pins for motor control
#if BAILEY_MOTOR_D
#define motor1Pin 5
#define enablePin1 9
#elif RYANN_MOTOR_D
#define motor1Pin1 8
#define motor1Pin2 9
#define enablePin1 7
#define motor2Pin1 10
#define motor2Pin2 11
#define enablePin2 12
#endif

// DEBUG Pin
#if BUTTON_INPUT_DEBUG
#define buttonPin 13
#endif



void setup()
{

  Serial.begin(115200);
  
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
#elif BUTTON_INPUT_DEBUG
  if(digitalRead(buttonPin) == HIGH){
    homingStart = true;
  }
#endif

  if (homingStart)
  {
    
  }
  else
  {

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
