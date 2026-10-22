#ifndef GROVE_RGB_H
#define GROVE_RGB_H

#include <stdint.h>
#include "cesi_types.h"

void GroveRGB_Init(void);
void GroveRGB_SetColor(uint8_t r, uint8_t g, uint8_t b);
void GroveRGB_Off(void);

// Mise à jour de la LED en fonction du mode ou de l'erreur courante
void GroveRGB_UpdateStatus(SystemMode mode, SystemError error);

#endif // GROVE_RGB_H