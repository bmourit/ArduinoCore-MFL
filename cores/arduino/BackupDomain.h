
#pragma once

#include "Arduino.h"

/**
 * Reset the Backup Domain registers to their reset values and disable write access.
 *
 * This function is intended to be called from within the reset handler or startup
 * code to reset the Backup Domain to its power-on-reset state.
 *
 * @note This function does not reset the contents of the Backup registers.
 *
 * @see backup_domain_reset()
 */
inline void backup_domain_kill() {
    bkp::BKP::get_instance().reset();
}

/**
 * Resets the Backup Domain to its power-on-reset state.
 *
 * This function resets the Backup Domain's registers to their power-on-reset
 * values and re-enables write access to the domain.
 *
 * @note This function does not reset the contents of the Backup registers.
 *
 * @see backup_domain_kill()
 */
inline void backup_domain_reset() {
    PMU_I.set_backup_write_enable(true);
    // Call twice to flush the APB-AHB bridge
    PMU_I.set_backup_write_enable(true);
    bkp::BKP::get_instance().reset();
}

/**
 * Enables write access to the Backup Domain.
 *
 * This function can be used to re-enable write access to the Backup Domain
 * after it has been disabled.
 *
 * @see backup_domain_disable()
 */
inline void backup_domain_enable() {
    PMU_I.set_backup_write_enable(true);
}

/**
 * Disables write access to the Backup Domain.
 *
 * This function is intended to be used to protect the Backup Domain's registers
 * and contents from accidental modification.
 *
 * @note This function does not clear the Backup Domain's registers or contents.
 *
 * @see backup_domain_enable()
 */
inline void backup_domain_disable() {
    PMU_I.set_backup_write_enable(false);
}

/**
 * Sets the value of a Backup register.
 *
 * This function sets the value of one of the Backup Domain's data registers
 * to the provided 16-bit value.
 *
 * @param data The Backup register to set, specified as a Backup_Data enumeration value.
 * @param value The 16-bit value to set the register to.
 */
inline void backup_register_set(bkp::Backup_Data data, uint16_t value) {
    bkp::BKP::get_instance().set_data(data, value);
}

/**
 * Retrieves the value of a Backup register.
 *
 * This function retrieves the value of one of the Backup Domain's data registers
 * and returns it as a 16-bit value.
 *
 * @param data The Backup register to retrieve, specified as a Backup_Data enumeration value.
 * @return The 16-bit value of the specified Backup register.
 */
inline uint16_t backup_rergister_get(bkp::Backup_Data data) {
    return bkp::BKP::get_instance().get_data(data);
}
