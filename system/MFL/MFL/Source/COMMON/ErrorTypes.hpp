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

#include <cstdio>
#include <type_traits>

template<typename T, typename E>
class Result {
public:
    Result(T* value, E error, const char* file, int line)
        : value_(value), error_(error), file_(file), line_(line) {}

    auto value() -> T& {
        return *value_;
    }
    auto error() const -> E {
        return error_;
    }
    [[nodiscard]] auto file() const -> const char* {
        return file_;
    }
    [[nodiscard]] auto line() const -> int {
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

/**
 * \brief Construct an instance of a class based on a given enum.
 *
 * \details
 * This function is used to construct an instance of a class based on a given
 * enum value. If the enum value is valid, an instance of the class is
 * constructed and returned. If the enum value is not valid, an error code
 * is returned.
 *
 * \tparam EnumClass The type of the enum class.
 * \tparam InstanceType The type of the instance to be constructed.
 * \tparam ErrorCode The type of the error code to be returned.
 *
 * \param Base The base enum value.
 * \param valid_base The valid enum value.
 * \param instance The instance to be constructed.
 *
 * \returns A result object containing a pointer to the instance and an
 * error code. If the error code is not OK, the pointer is null.
 */
template<typename EnumClass, typename InstanceType, typename ErrorCode>
auto get_enum_instance(EnumClass Base, EnumClass valid_base, InstanceType& instance) -> Result<InstanceType, ErrorCode> {
    if (Base == valid_base) {
        return { &instance, ErrorCode::OK, nullptr, 0 };
    } else {
        return RETURN_RESULT(InstanceType, ErrorCode::INVALID_SELECTION);
    }
}
