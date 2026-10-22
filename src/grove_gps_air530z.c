#include "grove_gps_air530z.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

extern UART_HandleTypeDef huart4; /* Definition unique dans setup.c. */
#ifndef GROVE_GPS_SWAP
#define GROVE_GPS_SWAP 1 /* Meme inversion TX/RX que le test des amis. */
#endif
#define GPS_RX_SIZE 1024U
static volatile uint8_t rx[GPS_RX_SIZE];
static volatile unsigned head, tail;
static volatile bool fault, restart_rx;
static volatile uint32_t received;
static uint8_t byte;
static bool ready;
static uint32_t last_frame;
static char assembly[128];
static unsigned used;
static bool collecting;

void UART4_IRQHandler(void) { HAL_UART_IRQHandler(&huart4); }

void GroveGPS_Init(void)
{
    ready=false; head=tail=0; fault=restart_rx=false;
    received=last_frame=0; used=0; collecting=false;
    huart4.Instance=UART4;
    huart4.Init.BaudRate=9600;
    huart4.Init.WordLength=UART_WORDLENGTH_8B;
    huart4.Init.StopBits=UART_STOPBITS_1;
    huart4.Init.Parity=UART_PARITY_NONE;
    huart4.Init.Mode=UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    huart4.Init.OverSampling=UART_OVERSAMPLING_16;
    huart4.Init.OneBitSampling=UART_ONE_BIT_SAMPLE_DISABLE;
    huart4.AdvancedInit.AdvFeatureInit=UART_ADVFEATURE_SWAP_INIT;
    huart4.AdvancedInit.Swap=GROVE_GPS_SWAP ? UART_ADVFEATURE_SWAP_ENABLE : UART_ADVFEATURE_SWAP_DISABLE;
    if (HAL_UART_Init(&huart4)!=HAL_OK) return;
    HAL_NVIC_SetPriority(UART4_IRQn,1,0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
    ready=HAL_UART_Receive_IT(&huart4,&byte,1)==HAL_OK;
}

bool GroveGPS_IsReady(void) { return ready; }
uint32_t GroveGPS_LastFrameTick(void) { return last_frame; }
uint32_t GroveGPS_ReceivedBytes(void) { return received; }

void GroveGPS_RxComplete(UART_HandleTypeDef *uart)
{
    if (uart!=&huart4) return;
    ++received;
    unsigned next=(head+1U)%GPS_RX_SIZE;
    if (next==tail) fault=true;
    else { rx[head]=byte; head=next; }
    if (HAL_UART_Receive_IT(&huart4,&byte,1)!=HAL_OK) restart_rx=true;
}

void GroveGPS_RxError(UART_HandleTypeDef *uart)
{
    if (uart==&huart4) { fault=true; restart_rx=true; }
}

static bool service_rx(void)
{
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    bool lost=fault;
    if (lost) { tail=head; fault=false; }
    if (restart_rx) {
        HAL_UART_AbortReceive(&huart4);
        restart_rx=HAL_UART_Receive_IT(&huart4,&byte,1)!=HAL_OK;
    }
    __set_PRIMASK(mask);
    if (lost) { collecting=false; used=0; }
    return lost;
}

/* Conserve une trame partielle entre deux passages et se resynchronise sur $. */
static bool next_line(char *out, unsigned capacity)
{
    unsigned budget=GPS_RX_SIZE;
    while (tail!=head && budget--) {
        uint8_t c=rx[tail]; tail=(tail+1U)%GPS_RX_SIZE;
        if (c=='$') { collecting=true; used=0; assembly[used++]=(char)c; }
        else if (collecting && c=='\n') {
            collecting=false; assembly[used]='\0';
            if (used+1U>capacity) { used=0; continue; }
            memcpy(out,assembly,used+1U); used=0; return true;
        } else if (collecting && c!='\r') {
            if (c<32 || c>126 || used>=sizeof assembly-1U) {
                collecting=false; used=0;
            } else assembly[used++]=(char)c;
        }
    }
    return false;
}

static int hex_digit(char c)
{
    if (c>='0' && c<='9') return c-'0';
    if (c>='A' && c<='F') return c-'A'+10;
    if (c>='a' && c<='f') return c-'a'+10;
    return -1;
}

static bool small_integer(const char *text, unsigned max, unsigned *out)
{
    if (!text || !*text) return false;
    unsigned value=0;
    for (const char *p=text; *p; ++p) {
        if (*p<'0' || *p>'9') return false;
        value=value*10U+(unsigned)(*p-'0');
        if (value>max) return false;
    }
    *out=value; return true;
}

static bool coordinate(const char *text, const char *hemisphere, bool latitude, double *out)
{
    if (!text || !hemisphere || !*hemisphere || hemisphere[1]) return false;
    char hem=hemisphere[0];
    if (latitude ? (hem!='N' && hem!='S') : (hem!='E' && hem!='W')) return false;
    unsigned digits=0; bool dot=false;
    for (const char *p=text; *p; ++p) {
        if (*p=='.' && !dot) { dot=true; continue; }
        if (*p<'0' || *p>'9') return false;
        if (!dot) ++digits;
    }
    if (digits!=(latitude ? 4U : 5U)) return false;
    char *end;
    double raw=strtod(text,&end);
    if (*end || !isfinite(raw)) return false;
    unsigned deg=(unsigned)(raw/100.0);
    double min=raw-deg*100.0;
    unsigned limit=latitude ? 90U : 180U;
    if (min<0 || min>=60 || deg>limit || (deg==limit && min!=0)) return false;
    *out=(deg+min/60.0)*((hem=='S' || hem=='W') ? -1.0 : 1.0);
    return true;
}

static bool parse_gga(const char *line, GPS_Data *gps)
{
    size_t len=strlen(line);
    if (len<10 || line[0]!='$' || line[1]<'A' || line[1]>'Z' ||
        line[2]<'A' || line[2]>'Z' || strncmp(line+3,"GGA,",4)!=0) return false;
    const char *star=strchr(line,'*');
    if (!star || strlen(star)!=3) return false;
    int a=hex_digit(star[1]), b=hex_digit(star[2]);
    if (a<0 || b<0) return false;
    uint8_t checksum=0;
    for (const char *p=line+1; p<star; ++p) checksum^=(uint8_t)*p;
    if (checksum!=(uint8_t)(a*16+b)) return false;
    char copy[128];
    size_t n=(size_t)(star-line);
    if (n>=sizeof copy) return false;
    memcpy(copy,line,n); copy[n]='\0';
    char *fields[16]={copy}; unsigned count=1;
    for (char *p=copy; *p; ++p) {
        if (*p==',') {
            *p='\0';
            if (count>=16) return false;
            fields[count++]=p+1;
        }
    }
    unsigned quality, satellites;
    if (count<8 || !small_integer(fields[6],8,&quality) ||
        !small_integer(fields[7],99,&satellites)) return false;
    GPS_Data result={0};
    result.fix_quality=(uint8_t)quality; result.satellites=(uint8_t)satellites;
    if (quality>0) {
        if (!coordinate(fields[2],fields[3],true,&result.latitude) ||
            !coordinate(fields[4],fields[5],false,&result.longitude)) return false;
        result.valid=true;
    }
    size_t raw_len=len<sizeof result.raw_sentence-1U ? len : sizeof result.raw_sentence-1U;
    memcpy(result.raw_sentence,line,raw_len); result.raw_sentence[raw_len]='\0';
    *gps=result; return true;
}

bool GroveGPS_Process(GPS_Data *gps)
{
    if (!ready || !gps) return false;
    if (service_rx()) gps->valid=false;
    char line[128]; bool updated=false;
    for (unsigned count=0; count<16 && next_line(line,sizeof line); ++count) {
        if (parse_gga(line,gps)) { last_frame=HAL_GetTick(); updated=true; }
    }
    return updated;
}

bool GroveGPS_ReadLine(char *buffer, unsigned int max_len, unsigned int timeout_ms)
{
    if (!ready || !buffer || max_len==0) return false;
    buffer[0]='\0'; service_rx(); uint32_t start=HAL_GetTick();
    do {
        if (next_line(buffer,max_len)) return true;
        if ((uint32_t)(HAL_GetTick()-start)>=timeout_ms) break;
        HAL_Delay(1); service_rx();
    } while (true);
    return false;
}

bool GroveGPS_ReadGGA(GPS_Data *gps, unsigned int timeout_ms)
{
    if (!ready || !gps) return false;
    uint32_t start=HAL_GetTick();
    do {
        if (GroveGPS_Process(gps)) return true;
        if ((uint32_t)(HAL_GetTick()-start)>=timeout_ms) break;
        HAL_Delay(1);
    } while (true);
    return false;
}
