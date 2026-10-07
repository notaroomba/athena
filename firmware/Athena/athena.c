/* VERSION 1*/

#include "athena.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* Forward Declaration Start */
extern void HAL_GPIO_WritePin(void* GPIOx, uint16_t GPIO_Pin, int PinState);
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);   /* USBD_OK = 0, USBD_BUSY = 1 */
extern void HAL_Delay(uint32_t Delay);
extern uint32_t HAL_GetTick(void);

// Timer handle type
typedef struct {
    void* Instance;
} TIM_HandleTypeDef;

#define __HAL_TIM_GET_COUNTER(__HANDLE__) (((TIM_HandleTypeDef*)(__HANDLE__))->Instance ? \
    (*(volatile uint32_t*)(((uint8_t*)((TIM_HandleTypeDef*)(__HANDLE__))->Instance) + 0x24)) : 0)
/* Forward Declaration End */

Athena_LED_PinConfig led_config;
Athena_TimerConfig timer_config = {0};

void Athena_Init(Athena_LED_PinConfig* led_cfg, Athena_TimerConfig* timer_cfg) {
    if (led_cfg != NULL) {
        led_config = *led_cfg;
    }
    if (timer_cfg != NULL) {
        timer_config = *timer_cfg;
    }
}

uint32_t GetTimestamp(void) {
    if (timer_config.htim == NULL) {
        return 0;
    }
    return __HAL_TIM_GET_COUNTER(timer_config.htim);
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
        /* The CDC endpoint takes one buffer at a time; back-to-back lines were silently
         * dropped. Wait a few ms for the previous one to drain. With no host attached the
         * endpoint stays busy, so this is bounded rather than blocking. */
        uint32_t t0 = HAL_GetTick();
        while (CDC_Transmit_FS((uint8_t *)buffer, (uint16_t)len) == 1 /* USBD_BUSY */ && (HAL_GetTick() - t0) < 3) {}
    }
}