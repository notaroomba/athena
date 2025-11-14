/* VERSION 1*/

#include "athena.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* Forward Declaration Start */
extern void HAL_GPIO_WritePin(void* GPIOx, uint16_t GPIO_Pin, int PinState);
extern void CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
extern void HAL_Delay(uint32_t Delay);
/* Forward Declaration End */

Athena_LED_PinConfig led_config;

void Athena_Init(Athena_LED_PinConfig* config) {
    led_config = *config;
}

void Set_LED_Color(LED_ColorTypeDef color) {
    switch (color) {
        case LED_OFF:
            HAL_GPIO_WritePin(led_config.port_r, led_config.pin_r, 1);
            HAL_GPIO_WritePin(led_config.port_g, led_config.pin_g, 1);
            HAL_GPIO_WritePin(led_config.port_b, led_config.pin_b, 1);
            break;
        case LED_RED:
            HAL_GPIO_WritePin(led_config.port_r, led_config.pin_r, 0);
            HAL_GPIO_WritePin(led_config.port_g, led_config.pin_g, 1);
            HAL_GPIO_WritePin(led_config.port_b, led_config.pin_b, 1);
            break;
        case LED_GREEN:
            HAL_GPIO_WritePin(led_config.port_r, led_config.pin_r, 1);
            HAL_GPIO_WritePin(led_config.port_g, led_config.pin_g, 0);
            HAL_GPIO_WritePin(led_config.port_b, led_config.pin_b, 1);
            break;
        case LED_BLUE:
            HAL_GPIO_WritePin(led_config.port_r, led_config.pin_r, 1);
            HAL_GPIO_WritePin(led_config.port_g, led_config.pin_g, 1);
            HAL_GPIO_WritePin(led_config.port_b, led_config.pin_b, 0);
            break;
        case LED_YELLOW:
            HAL_GPIO_WritePin(led_config.port_r, led_config.pin_r, 0);
            HAL_GPIO_WritePin(led_config.port_g, led_config.pin_g, 0);
            HAL_GPIO_WritePin(led_config.port_b, led_config.pin_b, 1);
            break;
        case LED_CYAN:
            HAL_GPIO_WritePin(led_config.port_r, led_config.pin_r, 1);
            HAL_GPIO_WritePin(led_config.port_g, led_config.pin_g, 0);
            HAL_GPIO_WritePin(led_config.port_b, led_config.pin_b, 0);
            break;
        case LED_MAGENTA:
            HAL_GPIO_WritePin(led_config.port_r, led_config.pin_r, 0);
            HAL_GPIO_WritePin(led_config.port_g, led_config.pin_g, 1);
            HAL_GPIO_WritePin(led_config.port_b, led_config.pin_b, 0);
            break;
        case LED_WHITE:
            HAL_GPIO_WritePin(led_config.port_r, led_config.pin_r, 0);
            HAL_GPIO_WritePin(led_config.port_g, led_config.pin_g, 0);
            HAL_GPIO_WritePin(led_config.port_b, led_config.pin_b, 0);
            break;
        default:
            break;
    }
}

void LED_Test_Sequence() {
    Set_LED_Color(LED_RED);
    HAL_Delay(500);
    Set_LED_Color(LED_GREEN);
    HAL_Delay(500);
    Set_LED_Color(LED_BLUE);
    HAL_Delay(500);
    Set_LED_Color(LED_YELLOW);
    HAL_Delay(500);
    Set_LED_Color(LED_CYAN);
    HAL_Delay(500);
    Set_LED_Color(LED_MAGENTA);
    HAL_Delay(500);
    Set_LED_Color(LED_WHITE);
    HAL_Delay(500);
    Set_LED_Color(LED_OFF);
}

void print(const char* format, ...) {
    char buffer[256];
    va_list args;
    
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len > 0 && len < (int)sizeof(buffer)) {
        CDC_Transmit_FS((uint8_t *)buffer, len);
    }
}