//
// MFL gd32 MCU definitions
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

namespace mcu {

    // Series
    struct F103R {};
    struct F303R {};

    // Variants
    struct F103RE {};
    struct F103RC {};
    struct F303RE {};
    struct F303RC {};

    using ChipSeries =
    #if defined(GD32F103RET6) || defined(GD32F103RCT6)
        F103R;
    #elif defined(GD32F303RET6) || defined(GD32F303RCT6)
        F303R;
    #else
        #error "Unsupported mcu series!"
    #endif

    using ChipVariant =
    #if defined(GD32F103RET6)
        F103RE;
    #elif defined(GD32F103RCT6)
        F103RC;
    #elif defined(GD32F303RET6)
        F303RE;
    #elif defined(GD32F303RCT6)
        F303RC;
    #else
        #error "Unsupported mcu variant!"
    #endif

} // namespace mcu
