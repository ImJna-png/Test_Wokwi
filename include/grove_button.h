#ifndef GROVE_BUTTON_H
#define GROVE_BUTTON_H

#include <stdbool.h>
#include <stdint.h>

void GroveButton_Init(void);
bool GroveButton1_IsPressed(void); // Bouton 1 (Vert)
bool GroveButton2_IsPressed(void); // Bouton 2 (Rouge)

bool GroveButton1_IsPressedLong(uint32_t duration_ms);
bool GroveButton2_IsPressedLong(uint32_t duration_ms);

#endif // GROVE_BUTTON_H