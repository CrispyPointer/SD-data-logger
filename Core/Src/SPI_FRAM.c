/*!
 *  @file   SPI_FRAM.c
 *  @brief  This is a redistribution of the SPI_FRAM chip driver implemented on STM32L412xx MCU using HAL Driver.
 *  @author K. Ly (SODAQ)
 *  @version 0.1
 ******************************************************************************
 * @attention 
 * Software License Agreement (BSD License)
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
    This code was derived by Phuoc K. Ly (SODAQ) from a copywrited (C) library written by K.Townsend (Adafruit Industries)
    available at https://github.com/adafruit/Adafruit_FRAM_SPI
    This file provides the FRAM driver functions and SPI code required to manage
    an SPI-connected to the MB85RS2MTA FRAM 
*/

/*-----------------------------------------------------------------------------/
/ Additional user header to be used
/-----------------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/*SPI_FRAM header file*/
#include "SPI_FRAM.h"

extern SPI_HandleTypeDef SPI_FRAM_HANDLE;
SPI_FRAM spi_fram;
/* Function prototype*/

#define FCLK_SLOW() { MODIFY_REG(FRAM_SPI_HANDLE.Instance->CR1, SPI_BAUDRATEPRESCALER_256, SPI_BAUDRATEPRESCALER_8); }	/* Set SCLK = slow, approx 280 KBits/s*/
#define FCLK_FAST() { MODIFY_REG(FRAM_SPI_HANDLE.Instance->CR1, SPI_BAUDRATEPRESCALER_256, SPI_BAUDRATEPRESCALER_4); }	/* Set SCLK = fast, approx 4.5 MBits/s */

#define CS_HIGH()	{HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);}
#define CS_LOW()	{HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);}

/// Supported flash devices
const struct {
  uint8_t manufID; ///< Manufacture ID
  uint16_t prodID; ///< Product ID
  uint32_t size;   ///< Size in bytes
} _supported_devices[] = {
    // Sorted in numerical order
    // Fujitsu
    {0x04, 0x0101, 2 * 1024UL},   // MB85RS16
    {0x04, 0x0302, 8 * 1024UL},   // MB85RS64V
    {0x04, 0x2303, 8 * 1024UL},   // MB85RS64T
    {0x04, 0x2503, 32 * 1024UL},  // MB85RS256TY
    {0x04, 0x4803, 256 * 1024UL}, // MB85RS2MTA
    {0x04, 0x4903, 512 * 1024UL}, // MB85RS4MT

    // Cypress
    {0x7F, 0x7F7f, 32 * 1024UL}, // FM25V02
                                 // (manu = 7F7F7F7F7F7FC2, device = 0x2200)

    // Lapis
    {0xAE, 0x8305, 8 * 1024UL} // MR45V064B
};

/*!
 *  @brief  Check if the flash device is supported
 *  @param  manufID
 *          ManufactureID to be checked
 *  @param  prodID
 *          ProductID to be checked
 *  @return size of device, 0 if not supported
 */
  uint32_t check_supported_device(uint8_t *manufID, uint16_t *prodID) {
    for (uint8_t i = 0;
        i < sizeof(_supported_devices) / sizeof(_supported_devices[0]); i++) {
      if (*manufID == _supported_devices[i].manufID &&
          *prodID == _supported_devices[i].prodID)
        return _supported_devices[i].size;
    }
    return 0;
  }

//   Serial.print(F("Unexpected Device: Manufacturer ID = 0x"));
//   Serial.print(manufID, HEX);
//   Serial.print(F(", Product ID = 0x"));
//   Serial.println(prodID, HEX);

//   return 0;
// }

/*!
 *  @brief  Initializes SPI and configures the chip (call this function before
 *          doing anything else)
 *  @param  nAddressSizeBytes
 *          address size in bytes (2MByte - 3)
 *  @return true if successful
 */
bool FRAM_begin(uint8_t nAddressSizeBytes) {
  /* Check SPI state*/
  if (SPI_FRAM_HANDLE.State != HAL_SPI_STATE_READY) {
    return false;
  }
  setAddressSize(3);
  return true;
}

/*!
    @brief  Enables or disables writing to the SPI dram
    @param enable
            True enables writes, false disables writes
*/
void FRAM_writeEnable(bool enable){
  uint8_t cmd;
  if (enable){
    cmd = OPCODE_WREN;
  }
  else cmd = OPCODE_WRDI;
  CS_LOW();
  HAL_SPI_Transmit(&SPI_FRAM_HANDLE, &cmd, sizeof(cmd), 10);
  CS_HIGH();
}

/*!
 * @brief Enables or disables sleep mode of the FRAM  
 * @param enable
 *              True enables sleep, flash disables sleep
 */
void FRAM_sleepEnable(bool enable){
  uint8_t cmd;
  if(enable){
    cmd = OPCODE_SLEEP;
    CS_LOW();
    HAL_SPI_Transmit(&SPI_FRAM_HANDLE, &cmd, sizeof(cmd), 10);
    CS_HIGH();
  }
  else{
    CS_LOW();
    HAL_Delay(1);
    CS_HIGH();
  }
}

/*!
 *  @brief  Writes a byte at the specific FRAM address
 *  @param addr
 *         The 32-bit address to write to in FRAM memory
 *  @param value
 *         The 8-bit value to write at framAddr
 */
void FRAM_write8(uint32_t addr, uint8_t value) {
  uint8_t buffer[10];
  uint8_t i = 0;

  buffer[i++] = OPCODE_WRITE;
  if (spi_fram._nAddressSizeBytes > 3)
    buffer[i++] = (uint8_t)(addr >> 24);
  if (spi_fram._nAddressSizeBytes > 2)
    buffer[i++] = (uint8_t)(addr >> 16);
  buffer[i++] = (uint8_t)(addr >> 8);
  buffer[i++] = (uint8_t)(addr & 0xFF);
  buffer[i++] = value;

  CS_LOW();
  HAL_SPI_Transmit(&SPI_FRAM_HANDLE, (uint8_t*)buffer, i, 10);
  CS_HIGH();
  //spi_dev->write(buffer, i);
}

/*!
 *   @brief  Writes string starting at the specific FRAM address
 *   @param addr
 *           The 32-bit address to write to in FRAM memory
 *   @param values
 *           The pointer to an array of 8-bit values to write starting at addr
 */
uint8_t FRAM_write(uint32_t *addr, char value[]){
  FRAM_writeEnable(true);
  int iterator = 0; //Point to FRAM address
  while(iterator < strlen(value)){
    FRAM_write8(*addr + iterator, value[iterator]);
    iterator++;
  }
  *addr += strlen(value);
  FRAM_writeEnable(false);
  return FRAM_OK;
}

/*!
 *   @brief  Reads an 8 bit value from the specified FRAM address
 *   @param  addr
 *           The 32-bit address to read from in FRAM memory
 *   @return The 8-bit value retrieved at framAddr
 */
uint8_t FRAM_read8(uint32_t addr) {
  uint8_t buffer[10], val;
  uint8_t i = 0;

  buffer[i++] = OPCODE_READ;
  if (spi_fram._nAddressSizeBytes > 3)
    buffer[i++] = (uint8_t)(addr >> 24);
  if (spi_fram._nAddressSizeBytes > 2)
    buffer[i++] = (uint8_t)(addr >> 16);
  buffer[i++] = (uint8_t)(addr >> 8);
  buffer[i++] = (uint8_t)(addr & 0xFF);

  CS_LOW();
  HAL_SPI_Transmit(&SPI_FRAM_HANDLE, (uint8_t*)buffer, i, 10);
  HAL_SPI_Receive(&SPI_FRAM_HANDLE, &val,sizeof(val), 10);
  CS_HIGH();
  //spi_dev->write_then_read(buffer, i, &val, 1);

  return val;
}

/*!
 *   @brief  Read 'string' starting at the specific FRAM address
 *   @param  addr
 *           The 32-bit address to write to in FRAM memory
 *   @param  values
 *           The pointer to an array of 8-bit values to read starting at addr
 */
const char* FRAM_read(uint32_t *addr){
  int index = 0;
  char value[4096] = {'\0'};
  while(1){
    value[index] = FRAM_read8(*addr);
    ++*addr;
    if(value[index]=='\n'){
      char* buffer = value;
      return buffer;
    }
    index++;
  }
}

void FRAM_getID(uint8_t *manufacturerID, uint16_t *productID){
  uint8_t cmd = OPCODE_RDID;
  uint8_t a[4] = {0, 0, 0, 0};
  CS_LOW();
  HAL_SPI_Transmit(&SPI_FRAM_HANDLE, &cmd, 1, 10);
  HAL_SPI_Receive(&SPI_FRAM_HANDLE, (uint8_t *)a, sizeof(a), 10);
  CS_HIGH();
  if(a[1] == 0x7f){
    // Device with continuation code (0x7F) in their second byte
    // Manu ( 1 byte)  - 0x7F - Product (2 bytes)
    *manufacturerID = (a[0]);
    *productID = (a[2] << 8) + a[3];
  } else {
    // Device without continuation code
    // Manu ( 1 byte)  - Product (2 bytes)
    *manufacturerID = (a[0]);
    *productID = (a[1] << 8) + a[2];
  }
}

/*!
 *   @brief  Sets adress size to provided value
 *   @param  nAddressSize
 *           address size in bytes
 */
void setAddressSize(uint8_t nAddressSize) {
  spi_fram._nAddressSizeBytes = nAddressSize;
}