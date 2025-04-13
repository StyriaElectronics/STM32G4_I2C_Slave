/*
 * eeprom_emulation.c
 *
 *  Created on: Apr 11, 2025
 *      Author: andrasnagy
 */
#include "eeprom_emulation.h"

#define EEPROM_BASE_ADDR 0x08007C00  // letzte 1K des Flash
#define EEPROM_SIZE      1024        // 1K reserviert

void EEPROM_Init(void)
{
    // optional: Bereich löschen, wenn nötig
}

// eeprom_emulation.c
void EEPROM_Write(uint16_t offset, uint8_t value)
{
    HAL_FLASH_Unlock();

    uint64_t val64 = (uint64_t)value;  // untere 8 Bit nutzen
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, EEPROM_BASE_ADDR + (offset * 8), val64);

    HAL_FLASH_Lock();
}

uint8_t EEPROM_Read(uint16_t offset)
{
    return *(volatile uint8_t*)(EEPROM_BASE_ADDR + (offset * 8));
}
