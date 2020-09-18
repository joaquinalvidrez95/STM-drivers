/*
 * utils.h
 *
 *  Created on: Jun 15, 2020
 *      Author: joaquin
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    UTILS_MECHANISM_POLLING,
    UTILS_MECHANISM_INTERRUPT,
} utils_mechanism_t;

void utils_delay();

inline bool utils_is_bit_set_u16(uint16_t reg, uint16_t mask)
{
    return (reg & mask) == mask;
}

inline void utils_set_bits_u16(volatile uint16_t *reg, uint16_t mask, bool b_set)
{
    if (b_set)
    {
        *reg |= mask;
    }
    else
    {
        *reg &= ~mask;
    }
}

inline void utils_set_bit_u32(volatile uint32_t *reg, uint8_t bit_position, bool b_set)
{
    if (b_set)
    {
        *reg |= (uint32_t)(1u << bit_position);
    }
    else
    {
        *reg &= (uint32_t)(~(1u << bit_position));
    }
}

#endif /* UTILS_H_ */
