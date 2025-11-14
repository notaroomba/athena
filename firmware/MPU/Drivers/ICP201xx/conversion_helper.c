/*
 *
 * Copyright (c) [2024] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */
#include <math.h>
#include "conversion_helper.h"

#define TO_KELVIN(temp_C) (273.15 + temp_C)
#define HEIGHT_TO_PRESSURE_COEFF 0.03424 // M*g/R = (0,0289644 * 9,80665 / 8,31432)

#define PRESSURE_TO_HEIGHT_COEFF 29.27127 // R / (M*g) = 8,31432 / (0,0289644 * 9,80665)
#define LOG_ATMOSPHERICAL_PRESSURE 4.61833 // ln(101.325)

float convertToHeight(float pressure_kp, float temperature_C)
{
    return PRESSURE_TO_HEIGHT_COEFF * TO_KELVIN(temperature_C) * (LOG_ATMOSPHERICAL_PRESSURE - log(pressure_kp));
}

float convertToPressure(float height_m, float temperature_C)
{
    return ATMOSPHERICAL_PRESSURE_KPA * exp(-HEIGHT_TO_PRESSURE_COEFF*height_m/TO_KELVIN(temperature_C));
}
