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

#pragma once

#include <stdbool.h>

#include "Arduino.h"
#include "variant.h"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"

inline bool isPinNumberValid(pin_size_t pin) { return ((pin >= 0U) && (pin <= DIGITAL_PIN_COUNT)); }
inline pin_size_t digitalPinToInterrupt(pin_size_t pin) { return pin; }
inline pin_size_t analogInputToDigitalPin(pin_size_t pin) { return pin; }
inline gpio::GPIO_Base digitalPinToPort(pin_size_t pin) { return getPortFromPin(pin); }

uint32_t portOutputRegister(gpio::GPIO_Base port) { return reinterpret_cast<uint32_t>(portToInstance(port).reg_address(gpio::GPIO_Regs::OCTL)); }
uint32_t portInputRegister(gpio::GPIO_Base port) { return reinterpret_cast<uint32_t>(portToInstance(port).reg_address(gpio::GPIO_Regs::ISTAT)); }
uint32_t portSetRegister(gpio::GPIO_Base port) { return reinterpret_cast<uint32_t>(portToInstance(port).reg_address(gpio::GPIO_Regs::BOP)); }
uint32_t portClearRegister(gpio::GPIO_Base port) { return reinterpret_cast<uint32_t>(portToInstance(port).reg_address(gpio::GPIO_Regs::BC)); }
