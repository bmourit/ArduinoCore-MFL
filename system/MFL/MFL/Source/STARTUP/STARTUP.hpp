//
// MFL gd32f30x STARTUP and clock initialization in C++
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

#include <stdint.h>

namespace startup {

class RCU;
class PMU;
class CEE;

class STARTUP {
public:
    static STARTUP& get_instance();

    void startup_init();

private:
    STARTUP();

    // Prevent copying or assigning
    STARTUP(const STARTUP&) = delete;
    STARTUP& operator=(const STARTUP&) = delete;
};


} // namespace startup
