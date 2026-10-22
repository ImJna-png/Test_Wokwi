/* Transitions et couleurs reprises du main d'Ibrahim, temporisations
 * rendues non bloquantes pour conserver la reception UART du GPS et de SET. */
#include "board.h"
#include "grove_button.h"
#include "grove_rgb.h"
#include "stm32l4xx_hal.h"
#define DEBOUNCE_MS 30U
#define LONG_MS 5000U
#define CONFIG_SHORT_MS 800U
#define CONFIG_IDLE_MS (30U*60U*1000U)

typedef struct {
    bool raw, stable, fired;
    uint32_t raw_since, pressed_since;
} Button;
typedef struct { bool long_press, released; uint32_t duration; } Event;
static Button green, red;
static uint32_t last_activity, last_blink;
static SystemMode shown_mode;
static SystemError shown_error;
static bool red_phase;

const char *Board_ModeName(SystemMode mode)
{
    switch (mode) {
        case MODE_STANDARD: return "STANDARD";
        case MODE_CONFIG: return "CONFIGURATION";
        case MODE_MAINTENANCE: return "MAINTENANCE";
        case MODE_ECO: return "ECO";
        default: return "INCONNU";
    }
}
void Board_NotifyActivity(void) { last_activity=HAL_GetTick(); }

static void error_colour(SystemError error)
{
    switch (error) {
        case ERR_RTC: GroveRGB_SetColor(0,0,255); break;
        case ERR_GPS: GroveRGB_SetColor(255,255,0); break;
        case ERR_SENSOR:
        case ERR_INCOHERENT_DATA: GroveRGB_SetColor(0,255,0); break;
        case ERR_SD_FULL:
        case ERR_SD_ACCESS: GroveRGB_SetColor(255,255,255); break;
        default: break;
    }
}
static void update_led(uint32_t now)
{
    if (g_current_error==SYS_OK) {
        if (shown_error!=SYS_OK || shown_mode!=g_current_mode)
            GroveRGB_UpdateStatus(g_current_mode,SYS_OK);
    } else if (shown_error!=g_current_error) {
        red_phase=true; last_blink=now; GroveRGB_SetColor(255,0,0);
    } else if ((uint32_t)(now-last_blink)>=500U) {
        last_blink=now; red_phase=!red_phase;
        if (red_phase) GroveRGB_SetColor(255,0,0);
        else error_colour(g_current_error);
    }
    shown_error=g_current_error; shown_mode=g_current_mode;
}

void Board_Init(void)
{
    GroveButton_Init(); GroveRGB_Init(); HAL_Delay(100);
    uint32_t now=HAL_GetTick();
    bool r=GroveButton2_IsPressed(),g=GroveButton1_IsPressed();
    red=(Button){r,r,r,now,now}; green=(Button){g,g,g,now,now};
    g_current_mode=r ? MODE_CONFIG : MODE_STANDARD;
    last_activity=now; shown_mode=(SystemMode)-1; shown_error=SYS_OK;
    red_phase=false; last_blink=now; update_led(now);
}

static Event poll(Button *b,bool sample,uint32_t now)
{
    Event event={0};
    if (sample!=b->raw) { b->raw=sample; b->raw_since=now; }
    if (b->stable!=b->raw && (uint32_t)(now-b->raw_since)>=DEBOUNCE_MS) {
        b->stable=b->raw;
        if (b->stable) { b->pressed_since=now; b->fired=false; }
        else { event.released=!b->fired; event.duration=now-b->pressed_since; b->fired=false; }
    }
    if (b->stable && b->raw && !b->fired && (uint32_t)(now-b->pressed_since)>=LONG_MS) {
        b->fired=true; event.long_press=true;
    }
    return event;
}
void Board_Update(void)
{
    uint32_t now=HAL_GetTick();
    Event r=poll(&red,GroveButton2_IsPressed(),now);
    Event g=poll(&green,GroveButton1_IsPressed(),now);
    SystemMode before=g_current_mode;
    if ((red.raw && green.raw) || (red.stable && green.stable)) {
        red.fired=green.fired=true;
    } else if (r.long_press) {
        if (g_current_mode==MODE_STANDARD) g_current_mode=MODE_MAINTENANCE;
        else if (g_current_mode==MODE_ECO || g_current_mode==MODE_MAINTENANCE)
            g_current_mode=MODE_STANDARD;
    } else if (r.released && g_current_mode==MODE_STANDARD &&
               r.duration>=CONFIG_SHORT_MS && r.duration<LONG_MS) {
        g_current_mode=MODE_CONFIG;
    } else if (g.long_press && g_current_mode==MODE_STANDARD) {
        g_current_mode=MODE_ECO;
    }
    if (before!=g_current_mode) {
        red.fired=red.stable; green.fired=green.stable; Board_NotifyActivity();
    }
    if (g_current_mode==MODE_CONFIG && (uint32_t)(now-last_activity)>=CONFIG_IDLE_MS)
        g_current_mode=MODE_STANDARD;
    update_led(now);
}
