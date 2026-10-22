#include "setup.h"

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;

int _write(int file, char *ptr, int len)
{
    (void)file;
    if (len<=0) return 0;
    return HAL_UART_Transmit(&huart2,(uint8_t *)ptr,(uint16_t)len,HAL_MAX_DELAY)==HAL_OK ? len : -1;
}

void SysTick_Handler(void) { HAL_IncTick(); }

static void MX_I2C1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CONFIG(RCC_I2C1CLKSOURCE_SYSCLK);
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    GPIO_InitTypeDef gpio={0};
    gpio.Pin=GPIO_PIN_8|GPIO_PIN_9;
    gpio.Mode=GPIO_MODE_AF_OD;
    gpio.Pull=GPIO_PULLUP;
    gpio.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate=GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB,&gpio);
    hi2c1.Instance=I2C1;
    hi2c1.Init.Timing=0x00707CBB;
    hi2c1.Init.OwnAddress1=0;
    hi2c1.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2=0;
    hi2c1.Init.OwnAddress2Masks=I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc={0};
    RCC_ClkInitTypeDef clk={0};
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
    osc.OscillatorType=RCC_OSCILLATORTYPE_MSI;
    osc.MSIState=RCC_MSI_ON;
    osc.MSIClockRange=RCC_MSIRANGE_6;
    osc.MSICalibrationValue=RCC_MSICALIBRATION_DEFAULT;
    HAL_RCC_OscConfig(&osc);
    clk.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource=RCC_SYSCLKSOURCE_MSI;
    clk.AHBCLKDivider=RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider=RCC_HCLK_DIV1;
    clk.APB2CLKDivider=RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk,FLASH_LATENCY_0);
}

void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
    GPIO_InitTypeDef gpio={0};
    gpio.Mode=GPIO_MODE_AF_PP;
    gpio.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
    if (uart->Instance==USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpio.Pin=GPIO_PIN_2|GPIO_PIN_3;
        gpio.Pull=GPIO_NOPULL;
        gpio.Alternate=GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA,&gpio);
    } else if (uart->Instance==UART4) {
        __HAL_RCC_UART4_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpio.Pin=GPIO_PIN_0|GPIO_PIN_1;
        gpio.Pull=GPIO_PULLUP;
        gpio.Alternate=GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOA,&gpio);
    }
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance=USART2;
    huart2.Init.BaudRate=115200;
    huart2.Init.WordLength=UART_WORDLENGTH_8B;
    huart2.Init.StopBits=UART_STOPBITS_1;
    huart2.Init.Parity=UART_PARITY_NONE;
    huart2.Init.Mode=UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    huart2.Init.OverSampling=UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling=UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit=UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2)!=HAL_OK) { while (1) {} }
}

void Global_Init(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_I2C1_Init();
    /* SPI non initialise : meme perimetre que ton test actuel, sans SD. */
    MX_USART2_UART_Init();
}
