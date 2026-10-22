#ifndef GROVE_LIGHT_H
#define GROVE_LIGHT_H
#include <stdbool.h>
#include <stdint.h>
void GroveLight_Init(void);
bool GroveLight_IsReady(void);
/* Valeur brute 12 bits ; 0xFFFF indique une erreur, pas une mesure. */
uint16_t GroveLight_ReadRaw(void);
#endif
