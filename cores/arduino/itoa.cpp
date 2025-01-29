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

#include <string.h>

extern "C" {

    constexpr const int MAX_BUFFER_SIZE = 33;

    /**
     * @brief Converts a signed long integer to a string in a specified radix.
     *
     * Converts a signed long integer to a string in a specified radix. If the
     * radix is 10, the function prepends a '-' character to the string for
     * negative values. The function does not check for overflow, and the
     * resulting string is stored in the provided character array. The
     * function returns a pointer to the beginning of the string.
     *
     * @param value The signed long integer to convert.
     * @param string The character array to store the resulting string in.
     * @param radix The radix to use for the conversion (2 to 36).
     * @return A pointer to the beginning of the resulting string.
     */
    char* ltoa(long value, char* string, int radix) {
        if (string == nullptr || radix > 36 || radix <= 1) {
            return nullptr;
        }

        char tmp[MAX_BUFFER_SIZE];
        char* tp = tmp;
        unsigned long v;
        bool sign = (radix == 10 && value < 0);

        if (sign) {
            v = -value;
        } else {
            v = (unsigned long)value;
        }

        do {
            long i = v % radix;
            v = v / radix;
            *tp++ = (i < 10) ? i + '0' : i + 'a' - 10;
        } while (v || tp == tmp);

        char* sp = string;
        if (sign) {
            *sp++ = '-';
        }
        while (tp > tmp) {
            *sp++ = *--tp;
        }
        *sp = '\0';
        return string;
    }

    /**
     * @brief Converts a signed integer to a string in a specified radix.
     *
     * This function is equivalent to ltoa, but is provided for compatibility
     * with the Arduino API.
     *
     * @param value The signed integer to convert.
     * @param string The character array to store the resulting string in.
     * @param radix The radix to use for the conversion (2 to 36).
     * @return A pointer to the beginning of the resulting string.
     */
    char* itoa(int value, char* string, int radix) {
        return ltoa(value, string, radix);
    }

    /**
     * @brief Converts an unsigned long integer to a string in a specified radix.
     *
     * This function converts an unsigned long integer to a string in the specified
     * radix. The resulting string is stored in the provided character array.
     * The conversion supports radices from 2 to 36, and the function does not 
     * perform any checks for overflow. A pointer to the beginning of the resulting
     * string is returned.
     *
     * @param value The unsigned long integer to convert.
     * @param string The character array to store the resulting string in.
     * @param radix The radix to use for the conversion (2 to 36).
     * @return A pointer to the beginning of the resulting string.
     */

    char* ultoa(unsigned long value, char* string, int radix) {
        if (string == nullptr || radix > 36 || radix <= 1) {
            return nullptr;
        }

        char tmp[MAX_BUFFER_SIZE];
        char* tp = tmp;
        do {
            long i = value % radix;
            value = value / radix;
            *tp++ = (i < 10) ? i + '0' : i + 'a' - 10;
        } while (value || tp == tmp);

        char* sp = string;
        while (tp > tmp) {
            *sp++ = *--tp;
        }
        *sp = '\0';
        return string;
    }

    /**
     * @brief Converts an unsigned integer to a string in a specified radix.
     *
     * This function converts an unsigned integer to a string in the specified
     * radix. The resulting string is stored in the provided character array.
     * The conversion supports radices from 2 to 36, and the function does not 
     * perform any checks for overflow. A pointer to the beginning of the resulting
     * string is returned.
     *
     * @param value The unsigned integer to convert.
     * @param string The character array to store the resulting string in.
     * @param radix The radix to use for the conversion (2 to 36).
     * @return A pointer to the beginning of the resulting string.
     */
    char* utoa(unsigned int value, char* string, int radix) {
        return ultoa(value, string, radix) ;
    }

} // extern "C"
