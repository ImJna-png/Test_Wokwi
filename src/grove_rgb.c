#include "grove_rgb.h"
#include "board_pins.h"

static void GroveRGB_ClockPulse(void)
{
    HAL_GPIO_WritePin(RGB_CLK_PORT, RGB_CLK_PIN, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 50; i++);
    HAL_GPIO_WritePin(RGB_CLK_PORT, RGB_CLK_PIN, GPIO_PIN_SET);
    for (volatile int i = 0; i < 50; i++);
}

static void GroveRGB_SendByte(uint8_t value)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(RGB_DATA_PORT, RGB_DATA_PIN, (value & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        GroveRGB_ClockPulse();
        value <<= 1;
    }
}

static void GroveRGB_SendColor(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t prefix = 0xC0;
    if ((b & 0x80) == 0) prefix |= 0x20;
    if ((b & 0x40) == 0) prefix |= 0x10;
    if ((g & 0x80) == 0) prefix |= 0x08;
    if ((g & 0x40) == 0) prefix |= 0x04;
    if ((r & 0x80) == 0) prefix |= 0x02;
    if ((r & 0x40) == 0) prefix |= 0x01;

    GroveRGB_SendByte(prefix);
    GroveRGB_SendByte(b);
    GroveRGB_SendByte(g);
    GroveRGB_SendByte(r);
}

void GroveRGB_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = RGB_DATA_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RGB_DATA_PORT, &gpio);

    gpio.Pin = RGB_CLK_PIN;
    HAL_GPIO_Init(RGB_CLK_PORT, &gpio);

    GroveRGB_Off();
}

void GroveRGB_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < 4; i++) GroveRGB_SendByte(0x00);
    GroveRGB_SendColor(r, g, b);
    for (int i = 0; i < 4; i++) GroveRGB_SendByte(0x00);
}

void GroveRGB_Off(void)
{
    GroveRGB_SetColor(0, 0, 0);
}

void GroveRGB_UpdateStatus(SystemMode mode, SystemError error)
{
    if (error != SYS_OK)
    {
        // En cas d'erreur, alternance à 1Hz gérée dans le scheduler/main
        switch (error)
        {
            case ERR_RTC:
                GroveRGB_SetColor(255, 0, 0); // Rouge / Bleu
                break;
            case ERR_GPS:
                GroveRGB_SetColor(255, 255, 0); // Rouge / Jaune
                break;
            case ERR_SENSOR:
            case ERR_INCOHERENT_DATA:
                GroveRGB_SetColor(255, 0, 0); // Rouge / Vert
                break;
            case ERR_SD_FULL:
            case ERR_SD_ACCESS:
                GroveRGB_SetColor(255, 0, 0); // Rouge / Blanc
                break;
            default:
                break;
        }
    }
    else
    {
        // Signaux continus selon le mode[cite: 4]
        switch (mode)
        {
            case MODE_STANDARD:
                GroveRGB_SetColor(0, 255, 0);   // Vert continu[cite: 4]
                break;
            case MODE_CONFIG:
                GroveRGB_SetColor(255, 255, 0); // Jaune continu[cite: 4]
                break;
            case MODE_ECO:
                GroveRGB_SetColor(0, 0, 255);   // Bleu continu[cite: 4]
                break;
            case MODE_MAINTENANCE:
                GroveRGB_SetColor(255, 128, 0); // Orange continu[cite: 4]
                break;
        }
    }
}