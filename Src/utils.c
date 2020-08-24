/*
 * utils.c
 *
 *  Created on: Jun 15, 2020
 *      Author: joaquin
 */

#include <stdint.h>

#include "utils.h"

void utils_delay()
{
    for (uint32_t i = 0u; i < 500000u; i++)
    {
    }
}
bool utils_is_bit_set_u16(uint16_t reg, uint16_t mask);