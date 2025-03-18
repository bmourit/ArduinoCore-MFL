// CMSIS compatability

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// IRQs
typedef enum IRQn {
    // Cortex-M4
    NonMaskableInt_IRQn          = -14,
    HardFault_IRQn               = -13,
    MemoryManagement_IRQn        = -12,
    BusFault_IRQn                = -11,
    UsageFault_IRQn              = -10,
    SVCall_IRQn                  = -5,
    DebugMonitor_IRQn            = -4,
    PendSV_IRQn                  = -2,
    SysTick_IRQn                 = -1,
    // F303RE
    WWDGT_IRQn                   = 0,
    LVD_IRQn                     = 1,
    TAMPER_IRQn                  = 2,
    RTC_IRQn                     = 3,
    FMC_IRQn                     = 4,
    RCU_CTC_IRQn                 = 5,
    EXTI0_IRQn                   = 6,
    EXTI1_IRQn                   = 7,
    EXTI2_IRQn                   = 8,
    EXTI3_IRQn                   = 9,
    EXTI4_IRQn                   = 10,
    DMA0_Channel0_IRQn           = 11,
    DMA0_Channel1_IRQn           = 12,
    DMA0_Channel2_IRQn           = 13,
    DMA0_Channel3_IRQn           = 14,
    DMA0_Channel4_IRQn           = 15,
    DMA0_Channel5_IRQn           = 16,
    DMA0_Channel6_IRQn           = 17,
    ADC0_1_IRQn                  = 18,
    USBD_HP_CAN0_TX_IRQn         = 19,
    USBD_LP_CAN0_RX0_IRQn        = 20,
    CAN0_RX1_IRQn                = 21,
    CAN0_EWMC_IRQn               = 22,
    EXTI5_9_IRQn                 = 23,
    TIMER0_BRK_IRQn              = 24,
    TIMER0_UP_IRQn               = 25,
    TIMER0_TRG_CMT_IRQn          = 26,
    TIMER0_Channel_IRQn          = 27,
    TIMER1_IRQn                  = 28,
    TIMER2_IRQn                  = 29,
    TIMER3_IRQn                  = 30,
    I2C0_EV_IRQn                 = 31,
    I2C0_ER_IRQn                 = 32,
    I2C1_EV_IRQn                 = 33,
    I2C1_ER_IRQn                 = 34,
    SPI0_IRQn                    = 35,
    SPI1_IRQn                    = 36,
    USART0_IRQn                  = 37,
    USART1_IRQn                  = 38,
    USART2_IRQn                  = 39,
    EXTI10_15_IRQn               = 40,
    RTC_Alarm_IRQn               = 41,
    USBD_WKUP_IRQn               = 42,
    TIMER7_BRK_IRQn              = 43,
    TIMER7_UP_IRQn               = 44,
    TIMER7_TRG_CMT_IRQn          = 45,
    TIMER7_Channel_IRQn          = 46,
    ADC2_IRQn                    = 47,
    EXMC_IRQn                    = 48,
    SDIO_IRQn                    = 49,
    TIMER4_IRQn                  = 50,
    SPI2_IRQn                    = 51,
    UART3_IRQn                   = 52,
    UART4_IRQn                   = 53,
    TIMER5_IRQn                  = 54,
    TIMER6_IRQn                  = 55,
    DMA1_Channel0_IRQn           = 56,
    DMA1_Channel1_IRQn           = 57,
    DMA1_Channel2_IRQn           = 58,
    DMA1_Channel3_4_IRQn         = 59
} IRQn_Type;

#if defined(__CC_ARM)
    #pragma push
    #pragma anon_unions
#elif defined(__ICCARM__)
    #pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wc11-extensions"
    #pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined (__GNUC__)
    // Anonymous unions are enabled by default
#elif defined(__TMS470__)
    // Anonymous unions are enabled by default
#elif defined(__TASKING__)
    #pragma warning 586
#elif defined(__CSMC__)
    // Anonymous unions are enabled by default
#else
    #warning Not supported compiler type
#endif

//////////////////////////////////////// NOTICE ////////////////////////////////////////
//  Some early batches of this chip shipped without an FPU/MPU.
//  If you experience issues, you may need to disable these features.
////////////////////////////////////////////////////////////////////////////////////////

// Cortex-M4 processor and peripherals configuration
#define __CM4_REV                 0x0001U    // Core revision r0p1
#define __Vendor_SysTickConfig    0U         // set to 1 if different SysTick config is used
#define __NVIC_PRIO_BITS          4U         // GD32F303RE uses 4 bits for priority levels
#define __VTOR_PRESENT            1U         // Set to 1 if VTOR is present
#define __MPU_PRESENT             1U         // GD32F303RE provide MPU
#define __FPU_PRESENT             1U         // Set to 1 if FPU is present
#define __FPU_DP                  0U         // Set to 1 if FPU is double precision FPU (default is single precision FPU)

#include <core_cm4.h>

#if defined(__CC_ARM)
    #pragma pop
#elif defined(__ICCARM__)
    // Leave anonymous unions enabled
#elif (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
    #pragma clang diagnostic pop
#elif defined(__GNUC__)
    // Anonymous unions are enabled by default
#elif defined(__TMS470__)
    // Anonymous unions are enabled by default
#elif defined(__TASKING__)
    #pragma warning restore
#elif defined(__CSMC__)
    // Anonymous unions are enabled by default
#else
    #warning Not supported compiler type
#endif

#ifdef __cplusplus
}
#endif
