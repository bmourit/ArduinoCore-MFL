
#pragma once

#include "Arduino.h"

/**
 * @brief Resets the backup domain.
 *
 * This function retrieves the singleton instance of the BKP class and
 * calls its reset method to reset the backup domain. It is used to
 * clear all backup registers and reset backup domain peripherals.
 */
inline void backup_domain_kill() {
    bkp::BKP::get_instance().reset();
}

/**
 * @brief Resets the backup domain by resetting the BKP peripheral and
 *        enabling writing to the backup domain.
 *
 * This function enables writing to the backup domain and then resets the BKP
 * peripheral by calling its reset method. The reset method clears all backup
 * registers and resets backup domain peripherals. Note that it is necessary to
 * call PMU_I.set_backup_write_enable twice to flush the APB-AHB bridge.
 */
inline void backup_domain_reset() {
    PMU_I.set_backup_write_enable(true);
    // Call twice to flush the APB-AHB bridge
    PMU_I.set_backup_write_enable(true);
    bkp::BKP::get_instance().reset();
}

/**
 * @brief Enables write access to the backup domain.
 *
 * This function enables write access to the backup domain by
 * setting the appropriate bit in the PMU control register.
 */
inline void backup_domain_enable() {
    PMU_I.set_backup_write_enable(true);
}

/**
 * @brief Disables write access to the backup domain.
 *
 * This function disables write access to the backup domain by
 * clearing the appropriate bit in the PMU control register.
 */
inline void backup_domain_disable() {
    PMU_I.set_backup_write_enable(false);
}

/**
 * @brief Sets the value of a backup register.
 *
 * This function sets the value of a backup register with the
 * specified address and value.
 *
 * @param data The address of the backup register to set.
 * @param value The value to set the backup register to.
 */
inline void backup_register_set(bkp::Backup_Data data, uint16_t value) {
    bkp::BKP::get_instance().set_data(data, value);
}

inline uint16_t backup_rergister_get(bkp::Backup_Data data) {
    return bkp::BKP::get_instance().get_data(data);
}
