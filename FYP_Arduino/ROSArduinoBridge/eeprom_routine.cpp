#include <EEPROM.h>
#include "eeprom_routine.h"

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

void clearEeprom()
{
    // Clear EEPROM
    for (int i = 0; i < EEPROM.length(); i++)
    {
        EEPROM.write(i, 0);
    }
}