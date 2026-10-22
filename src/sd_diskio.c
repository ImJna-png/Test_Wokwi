/* Pont entre FatFs (lib/FatFs) et le pilote SD bit-bang (sd_logger.c).
 * FatFs ne sait pas parler a la carte : il appelle ces 5 fonctions
 * pour lire et ecrire des blocs de 512 octets. */
#include "ff.h"
#include "diskio.h"
#include "sd_logger.h"
#include "grove_rtc_ds1307.h"

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != 0) return STA_NOINIT;
    return SDLogger_IsReady() ? 0 : STA_NOINIT;
}

DSTATUS disk_initialize(BYTE pdrv)
{
    /* La carte est deja initialisee par SDLogger_Init() au demarrage. */
    return disk_status(pdrv);
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != 0 || count == 0) return RES_PARERR;
    if (!SDLogger_IsReady()) return RES_NOTRDY;
    for (UINT i = 0; i < count; ++i) {
        if (!SDLogger_ReadBlock(sector + i, buff + 512U * i)) return RES_ERROR;
    }
    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != 0 || count == 0) return RES_PARERR;
    if (!SDLogger_IsReady()) return RES_NOTRDY;
    for (UINT i = 0; i < count; ++i) {
        if (!SDLogger_WriteBlock(sector + i, buff + 512U * i)) return RES_ERROR;
    }
    return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    (void)buff;
    if (pdrv != 0) return RES_PARERR;
    if (cmd == CTRL_SYNC) return SDLogger_Sync() ? RES_OK : RES_ERROR;
    return RES_PARERR; /* le reste ne sert qu'a f_mkfs, desactive */
}

/* Date des fichiers : lue sur la RTC, 01/01/2026 si la RTC ne repond pas. */
DWORD get_fattime(void)
{
    RTC_DateTime dt;
    if (!GroveRTC_GetDateTime(&dt) || dt.year < 1980) {
        dt.year = 2026; dt.month = 1; dt.day = 1;
        dt.hours = 0; dt.minutes = 0; dt.seconds = 0;
    }
    return ((DWORD)(dt.year - 1980) << 25) | ((DWORD)dt.month << 21) |
           ((DWORD)dt.day << 16) | ((DWORD)dt.hours << 11) |
           ((DWORD)dt.minutes << 5) | ((DWORD)(dt.seconds / 2));
}
