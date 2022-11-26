/*!
 *  @file   SPI_FRAM.h
 *  @brief  This is a redistribution of the SPI_FRAM chip driver implemented on STM32L412xx MCU using HAL Driver.
 *  Phuoc K. Ly (SODAQ)
 ******************************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Portions copyright (c) 2013, Adafruit Industries, All rights reserved.
 *  Portioins copyright (c) 2022, phuocly2304, All rights reserved
 *  
 *  This software is a free software and there is NO WARRANTY.
 *  No restriction on use. You can use, modify and redistribute it for
 *  personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
 *  Redistributions of source code must retain the above copyright notice.
 */

/*  
    This code was ported by Phuoc K. Ly (SODAQ) from a copywrited (C) library written by K.Townsend (Adafruit Industries)
    available at https://github.com/adafruit/Adafruit_FRAM_SPI
*/

#ifndef _SPI_FRAM_H_
#define _SPI_FRAM_H_

#include <stdbool.h>
#include <string.h>

#define FRAM_OK 0
/** Operation Codes **/
typedef enum opcodes_e {
  OPCODE_WREN = 0b0110,         /* Write Enable Latch D6*/
  OPCODE_WRDI = 0b0100,         /* Reset Write Enable Latch D4*/
  OPCODE_RDSR = 0b0101,         /* Read Status Register D5*/
  OPCODE_WRSR = 0b0001,         /* Write Status Register D1*/
  OPCODE_READ = 0b0011,         /* Read Memory D3*/
  OPCODE_WRITE = 0b0010,        /* Write Memory D2*/
  OPCODE_RDID = 0b10011111,     /* Read Device ID D159*/
  OPCODE_SLEEP = 0b10111001     /*Sleep Mode 185 apply to MB85RS2MTA chip*/
} opcodes_t;

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          FRAM SPI
 */

typedef struct SPI_FRAM{
    uint8_t _nAddressSizeBytes;
}SPI_FRAM;

bool FRAM_begin(uint8_t nAddressSizeBytes);
void FRAM_writeEnable(bool enable);
void FRAM_write8(uint32_t addr, uint8_t value);
uint8_t FRAM_write(uint32_t *addr, char value[]);
//void write(uint32_t addr, const uint8_t *values, uint16_t count);
uint8_t FRAM_read8(uint32_t addr);
const char* FRAM_read(uint32_t *addr);
void FRAM_getID(uint8_t *manufacturerID, uint16_t *productID);
uint32_t check_supported_device(uint8_t *manufID, uint16_t *prodID); 
uint8_t getStatusRegister(void);
void setStatusRegister(uint8_t value);
void setAddressSize(uint8_t nAddressSize);
void FRAM_sleepEnable(bool enable);

#endif

