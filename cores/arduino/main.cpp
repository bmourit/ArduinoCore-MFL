
#define ARDUINO_MAIN
#include "Arduino.h"
#include "TickTime.h"
#include "UsartSerial.hpp"

#ifdef ENABLE_DWT
    #include "CortexDwt.h"
#endif

/**
 * @brief Performs the necessary initialization before static objects are initialized.
 *
 * This function is called before any static objects are initialized. It sets the
 * NVIC priority group to use 4 bits of preemption priority and 0 bits of subpriority,
 * and then calls the startup_init function to finish any additional startup
 * initialization. Finally, it sets the SysTick_IRQn priority.
 *
 * @note This function is marked as WEAK so that it can be overridden by the user.
 */
WEAK void init() {
    cortex::CORTEX::get_instance().set_nvic_priority_group(cortex::Priority_Group::PRIO_GROUP_PRE4SUB0);
    // Finish startup
    startup::STARTUP::get_instance().startup_init();
    // Set SysTick_IRQn priority
    tickInit(0x00U);
}

/**
 * @brief A constructor function called before main that calls init().
 *
 * This function is marked with the constructor attribute and a priority of 101.
 * This ensures that the function is called after the crt0 init functions and
 * before any static object constructors. It is used to call the init() function,
 * which sets up the NVIC priority group and calls the startup_init function.
 */
__attribute__((constructor)) void premain() {
    init();
}

/**
 * @brief Main entry point of Arduino application.
 *
 * This function is the main entry point of the Arduino application. It first
 * initializes the DWT (if enabled) and then calls the user-provided setup()
 * function. It then enters an infinite loop, where it calls the user-provided
 * loop() function and the serialEventRun() function. The serialEventRun()
 * function is a placeholder for the user-provided serialEvent() function.
 *
 * @note The serialEventRun() function is used to run the user-provided
 * serialEvent() function. The serialEvent() function is a placeholder for a
 * user-provided function that is called when there is serial data available.
 */
int main(void) {
#ifdef ENABLE_DWT
    armDWT::get_instance().init();
#endif
    setup();

    for (;;) {
        loop();
        serialEventRun();
    }

    return 0;
}
