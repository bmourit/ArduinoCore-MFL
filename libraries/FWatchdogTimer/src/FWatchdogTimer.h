/*
 * Free Watchdog Timer library for the MFL Ardunio Core
 * Copyright (c) 2025 Arduino LLC. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#pragma once

#include "Arduino.h"

class FWatchdogTimer {
public:
    static FWatchdogTimer& get_instance();

    bool begin(uint32_t timeout);
    bool set(uint32_t timeout);
    void get(uint32_t* timeout);
    void reload();
    bool isEnabled() { return enabled_; }
    bool isReset(bool clear = false);
    void clearReset();

private:
    FWatchdogTimer();

    fwdgt::FWDGT& fwdgt_;
    bool enabled_;

    bool isTimeout(uint32_t timeout);
};
