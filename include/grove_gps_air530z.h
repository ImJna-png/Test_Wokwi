#ifndef GROVE_GPS_AIR530Z_H
#define GROVE_GPS_AIR530Z_H
#include <stdbool.h>
#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "cesi_types.h"

void GroveGPS_Init(void);
bool GroveGPS_IsReady(void);
/* A appeler souvent depuis main : aucune attente de trame ou de fix. */
bool GroveGPS_Process(GPS_Data *gps);
uint32_t GroveGPS_LastFrameTick(void);
uint32_t GroveGPS_ReceivedBytes(void);
void GroveGPS_RxComplete(UART_HandleTypeDef *uart);
void GroveGPS_RxError(UART_HandleTypeDef *uart);
/* Compatibilite pour les anciens tests. Ne pas melanger ces lectures
 * avec Process : elles consomment le meme flux. */
bool GroveGPS_ReadLine(char *buffer, unsigned int max_len, unsigned int timeout_ms);
bool GroveGPS_ReadGGA(GPS_Data *gps, unsigned int timeout_ms);
#endif
