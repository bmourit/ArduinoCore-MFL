//
// MFL Template error handling mechanism
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

#include <stdio.h>
#include <type_traits>

template<typename T, typename E>
class Result {
public:
    Result(T* value, E error, const char* file, int line)
        : value_(value), error_(error), file_(file), line_(line) {}

    T& value() const {
        return *value_;
    }
    E error() const {
        return error_;
    }
    const char* file() const {
        return file_;
    }
    int line() const {
        return line_;
    }

private:
    T* value_;
    E error_;
    const char* file_;
    int line_;
};

// Macro update to return nullptr when an error occurs
#define RETURN_RESULT(type, code) Result<type, decltype(code)>{ nullptr, code, __FILE__, __LINE__ }

template<typename EnumClass, typename InstanceType, typename ErrorCode>
Result<InstanceType, ErrorCode> get_enum_instance(EnumClass Base, EnumClass valid_base, InstanceType& instance) {
    if (Base == valid_base) {
        return { &instance, ErrorCode::OK, nullptr, 0 };
    } else {
        return RETURN_RESULT(InstanceType, ErrorCode::INVALID_SELECTION);
    }
}
