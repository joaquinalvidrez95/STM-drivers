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

void utils_delay();

inline bool utils_is_bit_set_u16(uint16_t reg, uint16_t mask)
{
    return (reg & mask) == mask;
}

#endif /* UTILS_H_ */
