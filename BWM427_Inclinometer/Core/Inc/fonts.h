#ifndef FONTS_H_
#define FONTS_H_

#include <stdint.h>

// Структура, описывающая один шрифт
typedef struct {
    const uint8_t width;
    uint8_t height;
    const uint16_t *data;
} FontDef;

// Прототипы доступных шрифтов. Сами массивы будут лежать в fonts.c
extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

#endif /* FONTS_H_ */
