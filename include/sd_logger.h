#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
	SDLOGGER_CARD_UNKNOWN = 0,
	SDLOGGER_CARD_SDSC,
	SDLOGGER_CARD_SDHC
} SDLoggerCardType;

typedef enum
{
	SDLOGGER_INIT_OK = 0,
	SDLOGGER_INIT_ERR_CMD0,
	SDLOGGER_INIT_ERR_CMD8_PATTERN,
	SDLOGGER_INIT_ERR_ACMD41_TIMEOUT,
	SDLOGGER_INIT_ERR_CMD58,
	SDLOGGER_INIT_ERR_CMD1_TIMEOUT
} SDLoggerInitStatus;

bool SDLogger_Init(void);
SDLoggerCardType SDLogger_GetCardType(void);
SDLoggerInitStatus SDLogger_GetLastInitStatus(void);
const char *SDLogger_GetActiveCsLabel(void);
bool SDLogger_BitBangCMD0Test(void);
/* Ajoute la ligne a la fin du fichier (cree s'il n'existe pas). */
bool SDLogger_WriteLine(const char *filename, const char *line);

/* Acces par blocs de 512 octets, utilises par sd_diskio.c (FatFs). */
bool SDLogger_IsReady(void);
bool SDLogger_ReadBlock(uint32_t sector, uint8_t *buffer);
bool SDLogger_WriteBlock(uint32_t sector, const uint8_t *buffer);
bool SDLogger_Sync(void);

#endif

