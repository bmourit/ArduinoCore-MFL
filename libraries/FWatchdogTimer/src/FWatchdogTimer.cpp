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

#include "FWatchdogTimer.h"

constexpr uint32_t minPrescaler = 4U;
constexpr uint32_t maxPrescaler = 256U;
constexpr uint32_t minTimeout = ((minPrescaler * 1'000'000U) / rcu::IRC40K_VALUE);       // microseconds
constexpr uint32_t maxTimeoutBase = ((maxPrescaler * 1'000'000U) / rcu::IRC40K_VALUE);   // microseconds

FWatchdogTimer& FWatchdogTimer::get_instance() {
    static FWatchdogTimer instance;
    return instance;
}

FWatchdogTimer::FWatchdogTimer() :
        fwdgt_(fwdgt::FWDGT::get_instance()),
        enabled_(false) {}

/**
 * @brief Initializes the FWDGT hardware block.
 * @param[in] timeout The time in microseconds until a CPU reset is generated.
 * @return true if the FWDGT could be initialized with the given timeout, false otherwise.
 *
 * The timeout value is in microseconds and sets the timer reset timeout.
 * When the timer reaches zero the hardware block will generate a reset signal
 * for the CPU. When specifying timeout value, plan to refresh the timer at
 * least twice as often. The `reload()` operation is not expensive.
 *
 * The downside of selecting a very large timeout value is that your system
 * may be left in an unresponsive state before the reset is generated.
 *
 * Valid timeout values depends of the IRC40K clock value. Typically, its 32kH value is
 * suitable for most applications. However, the range of valid values is:
 * - minimum timeout: 125us (minPrescaler * 1'000'000 / rcu::IRC40K_VALUE)
 * - maximum timeout: 32.768s (maxPrescaler * 1'000'000 / rcu::IRC40K_VALUE)
 *
 * @note This function will only return false if the given timeout value is invalid.
 */
bool FWatchdogTimer::begin(uint32_t timeout) {
    if (!isTimeout(timeout)) {
        core_debug("Invalid timeout value");
        return false;
    }
    fwdgt_.enable();
    enabled_ = true;

    return set(timeout);
}

/**
 * @brief Updates the timeout value for the FWDGT hardware block.
 * @param[in] timeout The time in microseconds until a CPU reset is generated.
 * @return true if the FWDGT was updated with the given timeout, false otherwise.
 *
 * The timeout value is in microseconds and sets the timer reset timeout.
 * When the timer reaches zero the hardware block will generate a reset signal
 * for the CPU. When specifying timeout value, plan to refresh the timer at
 * least twice as often. The `reload()` operation is not expensive.
 *
 * The downside of selecting a very large timeout value is that your system
 * may be left in an unresponsive state before the reset is generated.
 *
 * Valid timeout values depends of the IRC40K clock value. Typically, its 32kH value is
 * suitable for most applications. However, the range of valid values is:
 * - minimum timeout: 125us (minPrescaler * 1'000'000 / rcu::IRC40K_VALUE)
 * - maximum timeout: 32.768s (maxPrescaler * 1'000'000 / rcu::IRC40K_VALUE)
 *
 * @note This function will only return false if the given timeout value is invalid or if the
 * FWDGT hardware block is not enabled.
 */
bool FWatchdogTimer::set(uint32_t timeout) {
    if (!isEnabled() || !isTimeout(timeout)) {
        return false;
    }

    // Pre-calculate timeout in seconds to avoid repeated division
    float timeout_sec = static_cast<float>(timeout) / 1'000'000U * rcu::IRC40K_VALUE;
    uint8_t prescaler = 0U;

    // Find optimal prescaler using bit shifting
    uint16_t divider = 0U;
    do {
        divider = 4U << prescaler;
        prescaler++;
    } while ((timeout_sec / divider) > fwdgt_.get_reload());

    if (--prescaler > static_cast<uint8_t>(fwdgt::Prescaler_Value::DIV256)) {
        prescaler = static_cast<uint8_t>(fwdgt::Prescaler_Value::DIV256);
    }

    // Calculate reload value
    uint32_t reload = static_cast<uint32_t>(timeout_sec / divider) - 1U;

    // Update prescaler and reload values
    if (fwdgt_.set_prescaler(static_cast<fwdgt::Prescaler_Value>(prescaler)) || fwdgt_.set_reload(reload)) {
        return false;
    }

    // Refresh the watchdog counter
    fwdgt_.reload_counter();

    return true;
}

/**
 * @brief Retrieves the current timeout value set for the FWDGT hardware block.
 * @param[out] timeout The current timeout value in microseconds.
 *
 * This function retrieves the current timeout value set for the FWDGT hardware block.
 * The value is in microseconds. If the passed pointer is null, the function does
 * not do anything.
 *
 * @note This function will not return false, as it is a getter.
 */
void FWatchdogTimer::get(uint32_t* timeout) {
    if (!timeout) return;

    float base = (1'000'000.0f / rcu::IRC40K_VALUE);

    // Wait for reload value update with timeout protection
    while (fwdgt_.get_flag(fwdgt::Status_Flags::FLAG_RUD)) {
    }
    uint32_t reload = fwdgt_.get_reload();
    // Wait for prescaler update with timeout protection
    while (fwdgt_.get_flag(fwdgt::Status_Flags::FLAG_PUD)) {
    }
    uint32_t prescaler = fwdgt_.get_prescaler();
    *timeout = static_cast<uint32_t>((4U << prescaler) * (reload + 1U) * base);
}

/**
 * @brief Reloads the FWDGT counter.
 *
 * This function reloads the FWDGT counter. The counter is automatically
 * reloaded when the `begin()` function is called.
 *
 * @note The FWDGT hardware block needs to be initialized before calling this
 *       function.
 */
void FWatchdogTimer::reload() {
    if (isEnabled()) {
        fwdgt_.reload_counter();
    }
}

/**
 * @brief Checks if the system has resumed from FWDGT reset.
 * @param[in] clear If true, the FWDGT reset flag is cleared.
 * @return true if the system has resumed from FWDGT reset, false otherwise.
 *
 * The FWDGT reset flag is cleared when the `clearReset()` method is called or
 * when `isReset(true)` is called.
 *
 * @note This function will return false if the FWDGT reset flag is cleared.
 */
bool FWatchdogTimer::isReset(bool clear) {
    bool status = RCU_I.get_flag(rcu::Status_Flags::FLAG_FWDGTRST);
    if (status && clear) clearReset();
    return status;
}

/**
 * @brief Clears the FWDGT reset flag.
 *
 * This function clears the FWDGT reset flag and all other reset flags.
 *
 * @note This function will not return anything.
 */
void FWatchdogTimer::clearReset() {
    RCU_I.clear_all_reset_flags();
}

/**
 * @brief Checks if the given timeout value is valid.
 * @param[in] timeout The timeout value in microseconds.
 * @return true if the timeout value is valid, false otherwise.
 *
 * The minimum valid timeout value is 125us and the maximum valid timeout value
 * is 32,768s (~32.8 seconds). The precision depends of the IRC40K clock value.
 */
inline bool FWatchdogTimer::isTimeout(uint32_t timeout) {
    return (((timeout) >= minTimeout) && ((timeout) <= (maxTimeoutBase * fwdgt_.get_reload())));
}
