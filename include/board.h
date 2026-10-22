#ifndef BOARD_H
#define BOARD_H
#include "cesi_types.h"
extern SystemMode g_current_mode;
extern SystemError g_current_error;
extern SystemConfig g_config;
void Board_Init(void);
void Board_Update(void);
void Board_NotifyActivity(void);
const char *Board_ModeName(SystemMode mode);
#endif
