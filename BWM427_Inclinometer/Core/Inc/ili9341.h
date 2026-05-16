#ifndef ILI9341_H_
#define ILI9341_H_

#include "main.h"
#include "fonts.h"

// --- Базовые цвета для дисплея ---
#define ILI9341_BLACK       0x0000
#define ILI9341_WHITE       0xFFFF
#define ILI9341_RED         0xF800
#define ILI9341_GREEN       0x07E0
#define ILI9341_BLUE        0x001F
#define ILI9341_YELLOW      0xFFE0
#define ILI9341_CYAN        0x07FF

// --- Прототипы функций ---
void ILI9341_Init(void);
void ILI9341_SetRotation(uint8_t rotation);
void ILI9341_FillScreen(uint16_t color);
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ILI9341_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor);
void ILI9341_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor);

#endif /* ILI9341_H_ */
