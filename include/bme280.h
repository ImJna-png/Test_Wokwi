#ifndef BME280_H
#define BME280_H

#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define BME280_CHIP_ID 0x60u

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5;
    int16_t dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4, dig_H5;
    int8_t dig_H6;
} BME280_CalibData;

typedef struct {
    float temperature_c;
    float humidity_percent;
    float pressure_hpa;
} BME280_Data;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    BME280_CalibData calib;
    int32_t t_fine;
    uint16_t address_hal;
    bool initialized;
    const char *error;
} BME280_Handle;

/* Le bus doit deja etre initialise par Global_Init().
 * Init cherche automatiquement 0x76 puis 0x77 et verifie l'ID 0x60.
 */
bool BME280_Init(BME280_Handle *dev, I2C_HandleTypeDef *hi2c);
bool BME280_Read(BME280_Handle *dev, BME280_Data *data);
uint8_t BME280_Address(const BME280_Handle *dev);
const char *BME280_Error(const BME280_Handle *dev);

#endif
