/*
 * eeprom_emulation.h
 *
 *  Created on: Apr 11, 2025
 *      Author: andrasnagy
 */
#ifndef EEPROM_EMULATION_H
#define EEPROM_EMULATION_H

#include "main.h"

void EEPROM_Init(void);
uint8_t EEPROM_Read(uint16_t offset);
void EEPROM_Write(uint16_t offset, uint8_t value);

#endif
