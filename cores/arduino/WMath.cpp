/*
  Copyright (c) 2025 Arduino. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "stdlib.h"
#include "stdint.h"
#include "Arduino.h"

/**
 * @brief Initializes the random number generator with a seed value.
 *
 * The seed value is used to generate a sequence of pseudo-random numbers. If
 * the seed is set to 0, the random number generator will not be initialized,
 * and the first call to random() will generate a random seed value.
 *
 * @param seed The seed value to initialize the random number generator.
 */
extern void randomSeed(unsigned long seed) {
    if (seed != 0U) {
        srand(seed);
    }
}

/**
 * @brief Generates a pseudo-random number.
 *
 * This function generates a pseudo-random number in the range [0, size).
 * If size is 0, the function returns 0.
 *
 * @param size The range of the pseudo-random number.
 * @return A pseudo-random number in the range [0, size).
 */
extern long random(long size) {
    if (size == 0) {
        return 0;
    }
    return rand() % size;
}

/**
 * @brief Generates a pseudo-random number within a range.
 *
 * This function generates a pseudo-random number in the range [min, max).
 * If min is equal to max, the function returns min.
 *
 * @param min The minimum value of the range.
 * @param max The maximum value of the range.
 * @return A pseudo-random number in the range [min, max).
 */
extern long random(long min, long max) {
    if (min >= max) {
        return min;
    }
    long diff = max - min;
    return random(diff) + min;
}
