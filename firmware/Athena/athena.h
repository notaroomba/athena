
/* VERSION 1*/
#ifndef __ATHENA_H
#define __ATHENA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


typedef enum {
    LED_OFF = 0,
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_YELLOW,
    LED_CYAN,
    LED_MAGENTA,
    LED_WHITE
} LED_ColorTypeDef;



typedef struct  {
    void* port_r;
    uint16_t pin_r;
    void* port_g;
    uint16_t pin_g;
    void* port_b;
    uint16_t pin_b;
} Athena_LED_PinConfig;



void Athena_Init(Athena_LED_PinConfig* config);
void Set_LED_Color(LED_ColorTypeDef color);
void LED_Test_Sequence(void);
void print(const char* format, ...) __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif

#endif /* __ATHENA_H */