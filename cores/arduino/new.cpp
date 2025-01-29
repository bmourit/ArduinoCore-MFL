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

#include <stdlib.h>

/**
 * @brief Replacement for global operator new.
 *
 * This implementation simply calls the ::malloc C standard library function.
 * This is required because the C++ standard library (libstdc++), which is
 * linked against by the Arduino Core, expects the new and delete operators
 * to be defined. If these operators are not defined, the Arduino Core will
 * fail to link.
 *
 * @param size The number of bytes to allocate.
 * @return A pointer to the start of the allocated memory block.
 */
void* operator new(size_t size) {
    return malloc(size);
}

/**
 * @brief Replacement for global operator delete.
 *
 * This implementation simply calls the ::free C standard library function.
 * This is required because the C++ standard library (libstdc++), which is
 * linked against by the Arduino Core, expects the new and delete operators
 * to be defined. If these operators are not defined, the Arduino Core will
 * fail to link.
 *
 * @param ptr The pointer to the memory block to free.
 */
void operator delete(void* ptr) {
    free(ptr);
}

/**
 * @brief Replacement for global operator delete.
 *
 * This implementation simply calls the ::free C standard library function.
 * This is required because the C++ standard library (libstdc++), which is
 * linked against by the Arduino Core, expects the new and delete operators
 * to be defined. If these operators are not defined, the Arduino Core will
 * fail to link.
 *
 * @param ptr The pointer to the memory block to free.
 * @param size The number of bytes to free. This parameter is ignored.
 */
void operator delete(void* ptr, size_t) {
    free(ptr);
}

/**
 * @brief Replacement for global operator new[].
 *
 * This implementation simply calls the ::malloc C standard library function.
 * This is required because the C++ standard library (libstdc++), which is
 * linked against by the Arduino Core, expects the new and delete operators
 * to be defined. If these operators are not defined, the Arduino Core will
 * fail to link.
 *
 * @param size The number of bytes to allocate.
 * @return A pointer to the start of the allocated memory block.
 */
void* operator new[](size_t size) {
    return malloc(size);
}

/**
 * @brief Replacement for global operator delete[].
 *
 * This implementation simply calls the ::free C standard library function.
 * This is required because the C++ standard library (libstdc++), which is
 * linked against by the Arduino Core, expects the new and delete operators
 * to be defined. If these operators are not defined, the Arduino Core will
 * fail to link.
 *
 * @param ptr The pointer to the memory block to free.
 */
void operator delete[](void* ptr) {
    free(ptr);
}

/**
 * @brief Replacement for global operator delete[].
 *
 * This implementation simply calls the ::free C standard library function.
 * This is required because the C++ standard library (libstdc++), which is
 * linked against by the Arduino Core, expects the new and delete operators
 * to be defined. If these operators are not defined, the Arduino Core will
 * fail to link.
 *
 * @param ptr The pointer to the memory block to free.
 * @param size The number of bytes to free. This parameter is ignored.
 */
void operator delete[](void* ptr, size_t) {
    free(ptr);
}
