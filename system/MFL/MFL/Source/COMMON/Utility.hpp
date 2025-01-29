//
// MFL Simple C++ standard library replacements
//
// Copyright (C) 2025 B. Mouritsen <bnmguy@gmail.com>. All rights reserved.
//
// This file is part of the Microcontroller Firmware Library (MFL).
//
// MFL is free software: you can redistribute it and/or modify it under the terms of the
// GNU Lesser General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// MFL is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License along with MFL.
// If not, see <https://www.gnu.org/licenses/>.
//

#pragma once

#include <stdlib.h>

inline uint32_t custom_min(uint32_t a, uint32_t b) {
    return (a < b) ? a : b;
}

inline uint32_t custom_max(uint32_t a, uint32_t b) {
    return (a > b) ? a : b;
}

inline uint32_t custom_clamp(uint32_t value, uint32_t min_val, uint32_t max_val) {
    return custom_max(min_val, custom_min(value, max_val));
}
