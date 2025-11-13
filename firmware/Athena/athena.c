/* VERSION 1*/

#include "athena.h"

Athena_LED_PinConfig led_config;

static void Athena_Init(Athena_LED_PinConfig* config) {
    led_config = *config;
}

static void Set_LED_Color(LED_ColorTypeDef color) {
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