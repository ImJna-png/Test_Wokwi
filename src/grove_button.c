#include "grove_button.h"
#include "board_pins.h"

void GroveButton_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio={0};
    gpio.Mode=GPIO_MODE_INPUT; gpio.Pull=GPIO_PULLUP;
    gpio.Pin=BUTTON1_PIN; HAL_GPIO_Init(BUTTON1_PORT,&gpio);
    gpio.Pin=BUTTON2_PIN; HAL_GPIO_Init(BUTTON2_PORT,&gpio);
}
bool GroveButton1_IsPressed(void)
{ return HAL_GPIO_ReadPin(BUTTON1_PORT,BUTTON1_PIN)==GPIO_PIN_RESET; }
bool GroveButton2_IsPressed(void)
{ return HAL_GPIO_ReadPin(BUTTON2_PORT,BUTTON2_PIN)==GPIO_PIN_RESET; }

/* API longue declaree dans le header du groupe. Le scheduler board.c
 * possede son propre anti-rebond pour reconnaitre aussi l'appui rouge court. */
typedef struct { bool down, fired; uint32_t since; } LongPress;
static LongPress long_state[2];
static bool pressed_long(unsigned index,bool down,uint32_t duration)
{
    LongPress *s=&long_state[index]; uint32_t now=HAL_GetTick();
    if (!down) { s->down=false; s->fired=false; return false; }
    if (!s->down) { s->down=true; s->since=now; }
    if (!s->fired && (uint32_t)(now-s->since)>=duration) { s->fired=true; return true; }
    return false;
}
bool GroveButton1_IsPressedLong(uint32_t duration_ms)
{ return pressed_long(0,GroveButton1_IsPressed(),duration_ms); }
bool GroveButton2_IsPressedLong(uint32_t duration_ms)
{ return pressed_long(1,GroveButton2_IsPressed(),duration_ms); }
