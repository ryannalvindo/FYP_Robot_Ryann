#ifndef EEPROM_ROUTINE_H
#define EEPROM_ROUTINE_H

void writingEeprom(long inputNum, long eepromAddress);
long readingEeprom(long eepromAddress);
void clearEeprom();

#endif