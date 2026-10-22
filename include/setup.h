#ifndef SETUP_H
#define SETUP_H
#include "stm32l4xx_hal.h"
void Global_Init(void);
void SystemClock_Config(void);
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
#endif
