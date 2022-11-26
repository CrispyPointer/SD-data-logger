/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
#include "SPI_FRAM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DEBUG 1 //Define debug mode

//External FRAM locations for user settings
#define LOCATION_FILE_NUMBER_LSB 0x03
#define LOCATION_FILE_NUMBER_MSB 0x04

#define LOCATION_BUFFER_START 10
#define LOCATION_BUFFER_END 0x3ff70
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
UART_WakeUpTypeDef wakeup;        //Wake up handler
char Rx_data[1024];               // Creat a buffer of 1 kbytes
uint8_t RX_Flag = 0;              //RX_flag when interrupt
uint8_t SD_Flag = 0;
uint8_t SD_Init_Flag = 0;
volatile int LOCATION_BUFFER_ITERATOR = LOCATION_BUFFER_START;
const uint32_t MAX_IDLE_TIME_MSEC  = 2000;
static const int BUFFER_MAX = 20;
//some variables for FatFs
FATFS FatFs; 	//Fatfs handle
FIL newFile, workingFile, fil; 		//File handle
FRESULT fres; //Result after operations
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void turn_off_spi_SD();
static void turn_off_spi_FRAM();
void myprintf(const char *fmt, ...);
void appendBUFFER();
void Stop_Mode_Entry();
char* newLog(void);
uint8_t appendFile(char* fileName);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void turn_off_spi_FRAM(){
  HAL_SPI_DeInit(&hspi1);
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4| GPIO_PIN_5 |GPIO_PIN_6| FRAM_CS_Pin| LED_Pin, RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_4| GPIO_PIN_5 |GPIO_PIN_6| FRAM_CS_Pin| LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void turn_off_spi_SD(){
  HAL_SPI_DeInit(&hspi2);
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /*Configure GPIO pin Output Level */ 
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10| GPIO_PIN_14 |GPIO_PIN_15| SD_CS_Pin, RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_10| GPIO_PIN_14 |GPIO_PIN_15 |SD_CS_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
}

void myprintf(const char *fmt, ...){
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);
}

/*Receive data until idle time (1 frame reception) */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size){
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, Rx_data, sizeof(Rx_data));
  myprintf("\nReceived data: ");
  HAL_UART_Transmit(&huart2, Rx_data, strlen(Rx_data), 10);
  //memset(Rx_data, 0, sizeof(Rx_data));
  RX_Flag = 1;
  //myprintf(": End receive\n");
}

void appendBUFFER(){
  FRAM_sleepEnable(false);
  if(!FRAM_begin(3)){
    myprintf("SPI FRAM begin error.. check your connection !!\n");
    while(1);
  }
  else myprintf("FRAM begin success!!\n");
  #ifdef DEBUG
  uint8_t manufID;
  uint16_t prodID;
  FRAM_getID(&manufID, &prodID);
  myprintf("Manufacture ID: 0x%X\n", manufID);
  myprintf("product ID: 0x%X\n", prodID);
  #endif
  
  #ifdef DEBUG
  myprintf("<");
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
  #endif

  unsigned long lastSyncTime = HAL_GetTick(); //Keeps track of the last time the file was synced
  //Start recording incoming characters
  while(1){
    FRAM_sleepEnable(false); //make sure FRAM is woken up properly
    if(RX_Flag != 0){
      FRAM_sleepEnable(false);
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      char readBuf[1024] = {'0'};
      strncpy((char*)readBuf, Rx_data, strlen(Rx_data));
      memset(Rx_data, 0, strlen(Rx_data));
      #ifdef DEBUG
      myprintf("Writing to FRAM...\n");
      myprintf("Current iterator: %d\n", LOCATION_BUFFER_ITERATOR);
      #endif
      if(FRAM_write(&LOCATION_BUFFER_ITERATOR, readBuf) != FRAM_OK) Error_Handler();
      #ifdef DEBUG
      myprintf("Done writing to FRAM...\n");
      myprintf("Current iterator: %d\n", LOCATION_BUFFER_ITERATOR);
      #endif
      /*Buffer until ~255kByte*/
      if(LOCATION_BUFFER_ITERATOR >= BUFFER_MAX+LOCATION_BUFFER_START){
        SD_Flag = 1;
        break;
      }   
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      RX_Flag = 0;
    }
    else if ((unsigned long)(HAL_GetTick()-lastSyncTime) > MAX_IDLE_TIME_MSEC){
      lastSyncTime = HAL_GetTick();
      #ifdef DEBUG
        myprintf("******I am done waiting!!! Now I go sleep ******\n");
      #endif
      FRAM_sleepEnable(true);
      Stop_Mode_Entry();
    }
  }
}

void Stop_Mode_Entry(){

  /*Kindly turn off FRAM and other peripheral :) */
  turn_off_spi_FRAM();
  /* make sure no LPUART transfer is on-going*/
  while(__HAL_UART_GET_FLAG(&huart2, USART_ISR_BUSY) == SET);

  /* make sure that LPUART is ready to receive
  * (test carried out again later in HAL_UARTEx_StopModeWakeUpSourceConfig) */
  while(__HAL_UART_GET_FLAG(&huart2, USART_ISR_REACK) == RESET);
  
  /*set up wake-up event*/
  wakeup.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
  if (HAL_UARTEx_StopModeWakeUpSourceConfig(&huart2, wakeup)!= HAL_OK)
  {
    Error_Handler(); 
  }
  /* Enable the USART2 Wake UP from STOP mode Interrupt */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_WUF);

  /* enable MCU wake-up by USART1 */
  HAL_UARTEx_EnableStopMode(&huart2);      
  #ifdef DEBUG
  myprintf("I go to sleep now!");
  #endif

  /*Deinit peripherals*/
  HAL_DBGMCU_DisableDBGSleepMode( );
  HAL_DBGMCU_DisableDBGStopMode( );
  HAL_DBGMCU_DisableDBGStandbyMode( );
  HAL_SuspendTick();
  /*Enter stop mode*/
  HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);

  /*Escape from stop mode*/
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  HAL_ResumeTick();
  /* Wake up based on RXNE flag successful*/
  HAL_UARTEx_DisableStopMode(&huart2);
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, Rx_data, sizeof(Rx_data));
  HAL_GPIO_WritePin(SD_EN_GPIO_Port,SD_EN_Pin,SET);
}

char* newlog(void){
  uint8_t msb, lsb;
  uint8_t addr_size = 3;
  unsigned int newFileNumber;
  //FRAM_begin(addr_size);
  #ifdef DEBUG
  if(!FRAM_begin(addr_size))
  {
    myprintf("SPI FRAM begin error.. check your connection !!\n");
    while(1);
  }
  else myprintf("SPI begin success!!\n");
  #endif

  //Combine two 8-bit FRAM spots into one 16-bit number
  lsb = FRAM_read8(LOCATION_FILE_NUMBER_LSB);
  msb = FRAM_read8(LOCATION_FILE_NUMBER_MSB);
  #ifdef DEBUG
  myprintf("Location lsb: %d\n", lsb);
  myprintf("Location msb: %d\n", msb);
  //HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, SET);
  #endif
  newFileNumber = msb;
  newFileNumber = newFileNumber << 8;
  newFileNumber |= lsb;

  //If both FRAM spots are 255 (0xFF), that means they are un-initialized (first time Logger has been turned on)
  //Let's init them both to 0
  if((lsb==0xff) && (msb == 0xff)){
    newFileNumber = 0;
    FRAM_writeEnable(true);
    FRAM_write8(LOCATION_FILE_NUMBER_LSB, 0x00);
    FRAM_write8(LOCATION_FILE_NUMBER_MSB, 0x00);
    FRAM_writeEnable(false);
    #ifdef DEBUG
    myprintf("Location lsb updated (255): %d\n", lsb);
    myprintf("Location msb updated (255): %d\n", msb);
    //HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, SET);
    #endif
  }
  //Let's quit if we reach 65534 files
  if(newFileNumber == 65534){
    return 0;
  }

  // Create new file name based on the available log spot
  static char newFileName[13];
  while(1){
    sprintf(newFileName, "LOG%05d.TXT", newFileNumber);

    //If we are able to create this file, then it didn't exist, we're good, break
    fres = f_open(&newFile, newFileName, FA_WRITE | FA_CREATE_NEW);
    if(fres == FR_OK){
      f_close(&newFile);
      break;
    }
    
    //If file exists, see if empty. If so, use it.
    if(fres == FR_EXIST){
      fres = f_open(&newFile, newFileName, FA_READ | FA_OPEN_EXISTING);
      if(f_size(&newFile) == 0){
        f_close(&newFile);  // Close this existing file we just opened.
        return newFileName; // Use existing empty file.
      }
      f_close(&newFile);
    }

    //try the next number
    newFileNumber++;
    if(newFileNumber > 65533) return 0;
  }

  newFileNumber++; //Increment so the next power up uses the next file #
  lsb = (uint8_t)(newFileNumber & 0x00ff);
  msb = (uint8_t)((newFileNumber & 0xff00)>>8);

  FRAM_writeEnable(true);
  FRAM_write8(LOCATION_FILE_NUMBER_LSB, lsb); //LSB

  if(FRAM_read8(LOCATION_FILE_NUMBER_MSB) != msb)
  FRAM_write8(LOCATION_FILE_NUMBER_MSB, msb); //MSB
  FRAM_writeEnable(false);
  #ifdef DEBUG
    myprintf("Location lsb updated: %d\n", lsb);
    myprintf("Location msb updated: %d\n", msb);
    //HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, SET);
  #endif
  return newFileName;
}

uint8_t appendFile(char* fileName){
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, SET);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, SET);
  //HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15| SD_CS_Pin);
  MX_GPIO_Init();
  MX_SPI2_Init(); 
  MX_FATFS_Init();
  UINT bytesWrote = 0;
  /*Init and Mount SD Card*/
  #ifdef DEBUG
  myprintf("\r\n~ SD card initializing... ~\r\n\r\n");
  HAL_Delay(100);
  #endif
  //Open the file system
  fres = f_mount(&FatFs, "", 1); //1=mount now
  while (fres != FR_OK) {
	  myprintf("f_mount error (%i)\r\n", fres);
    fres = f_mount(&FatFs, "", 1); //1=mount now
    HAL_Delay(1000);
	//Error_Handler();
  }
  #ifdef DEBUG
  BYTE readBuf[300];
  BYTE writeBuf[300];
  myprintf("\r\n~ SD card mounted successfully ~\r\n\r\n");
  #endif
  fres = f_open(&workingFile, fileName, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
  if(fres != FR_OK) Error_Handler();

  if(f_size(&workingFile) == 0){
    f_rewind(&workingFile);
    //f_sync(&workingFile);
  }
  FRAM_sleepEnable(false);
  uint32_t index = LOCATION_BUFFER_START;
  while(index < LOCATION_BUFFER_ITERATOR){
    char data = FRAM_read8(index);
    #ifdef DEBUG
    myprintf("Buffer data at address %d: %c\n",index, data);
    strncpy((char*)writeBuf,FRAM_read(&index),300);
    //char* test_Buf = FRAM_read(&index);
    myprintf("Writebuf: %s\n", writeBuf);
    myprintf("Current Index: %d\n", index);
    #endif
    fres = f_write(&workingFile, writeBuf, strlen(writeBuf), &bytesWrote);
    myprintf("Wrote %i bytes to 'logger file'!\r\n", bytesWrote);
    if(fres != FR_OK){
      myprintf("f_write error (%i)\r\n");
    }
  }
  #ifdef DEBUG 
  f_close(&workingFile);
  //Now let's try to open file "test.txt"
  fres = f_open(&fil, fileName, FA_READ);
  if (fres != FR_OK) {
	  myprintf("f_open error (%i)\r\n");
	  while(1);
  }
  myprintf("I was able to open 'test.txt' for reading!\r\n");
  TCHAR* rres = f_gets((TCHAR*)readBuf, sizeof(readBuf), &fil);
  
  while(1){
    myprintf("Read string from 'test.txt' contents: %s\r\n", readBuf);
    rres = f_gets((TCHAR*)readBuf, sizeof(readBuf), &fil);
    if(rres == 0) {
	    myprintf("f_gets error (%i) | end of file\r\n", fres);
      break;
    }
  }

  if(fres == FR_OK) {
    RX_Flag = 0;
    memset(Rx_data,0,sizeof(Rx_data));
    f_sync(&workingFile);
  } 
  else {
    f_sync(&workingFile);
    myprintf("Error (%i)\r\n");
  }
  #endif
  SD_Flag = 0;
  /*We're done with SD, let's un-mount the drive*/
  if (FATFS_UnLinkDriver(1)){
    myprintf("FATFS Unlinked fail!!");
    Error_Handler();
  }
  myprintf("FATFS Unlink success\n");
  f_mount(NULL, " ", 0);
  
  /*We're also done with FRAM, let's reset it iterator*/
  LOCATION_BUFFER_ITERATOR = LOCATION_BUFFER_START;
  
  /*Don't forget to turn off the power after using :) */
  turn_off_spi_SD();
  HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, RESET);
  FRAM_sleepEnable(true);
  return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, Rx_data, sizeof(Rx_data));
  if(!FRAM_begin(3)){
    myprintf("SPI FRAM begin error.. check your connection !!\n");
    while(1);
  }
  else myprintf("SPI begin success!!\n");
  uint8_t manufID;
  uint16_t prodID;
  FRAM_getID(&manufID, &prodID);
  myprintf("Manufacture ID: 0x%X\n", manufID);
  myprintf("product ID: 0x%X\n", prodID);
  turn_off_spi_SD();
  HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(SD_Flag) appendFile("Write12.TXT");
    // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    // HAL_Delay(1000);
    appendBUFFER();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FRAM_HOLD_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FRAM_CS_Pin|SD_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA9 PA10 PA11
                           PA12 PA13 PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FRAM_HOLD_Pin FRAM_CS_Pin SD_EN_Pin LED_Pin */
  GPIO_InitStruct.Pin = FRAM_HOLD_Pin|FRAM_CS_Pin|SD_EN_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12
                           PB3 PB4 PB5 PB6
                           PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(500);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
