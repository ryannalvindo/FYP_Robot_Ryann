// declare library for EEPROM
#include <EEPROM.h>

void setup()
{
    // setup the serial
    Serial.begin(115200);
}

void loop()
{

    long inputNum;

    // ask user to type in the serial monitor
    Serial.println("Type an integer number ");
    Serial.flush();
    while (Serial.available() == 0)
    {
        // do nothing
    }
    while (Serial.available() > 0)
    {
        inputNum = Serial.parseInt(SKIP_ALL); // read number until newline
    }

    // saving the input into EEPROM
    // variable for address needs to be static
    static long address = 0;
    // saving to EEPROM
    // EEPROM.put(address, inputNum);
    EEPROM.write(address, inputNum & 0xFF);             // Stores 0x38 in address 0
    EEPROM.write(address + 1, (inputNum >> 8) & 0xFF);  // Stores 0x30 in address 1
    EEPROM.write(address + 2, (inputNum >> 16) & 0xFF); // Stores 0x01 in address 2
    EEPROM.write(address + 3, (inputNum >> 24) & 0xFF); // Stores 0x00 in address 3
    // displaying the notification by reading from EEPROM
    long val = (long)EEPROM.read(address) |
               ((long)EEPROM.read(address + 1) << 8) |
               ((long)EEPROM.read(address + 2) << 16) |
               ((long)EEPROM.read(address + 3) << 24);
    Serial.print(val);
    Serial.print(" is stored in ");
    Serial.println(address);
    // increment the address by 4 byte
    address = address + 4;
}