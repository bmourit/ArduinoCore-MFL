#pragma once


///////////////////////////////// STARTUP /////////////////////////////////

#include "F303RE.hpp"
#include "CONFIG.hpp"
#include "Interrupt_Handlers.hpp"
#include "STARTUP.hpp"


///////////////////////////////// COMMON /////////////////////////////////

#include "BitRW.hpp"
#include "ErrorTypes.hpp"
#include "RegRW.hpp"
#include "RingBuffer.hpp"
#include "Utility.hpp"


///////////////////////////////// PERIPHERALS /////////////////////////////////

#include "ADC.hpp"
#include "adc_config.hpp"
#include "AFIO.hpp"
#include "BKP.hpp"
#include "bkp_config.hpp"
#include "CEE.hpp"
#include "cee_config.hpp"
#include "CORTEX.hpp"
#include "cortex_config.hpp"
#include "CRC.hpp"
#include "crc_config.hpp"
#include "CTC.hpp"
#include "ctc_config.hpp"
#include "DAC.hpp"
#include "dac_config.hpp"
#include "DBG.hpp"
#include "dbg_config.hpp"
#include "DMA.hpp"
#include "dma_config.hpp"
#include "EXMC.hpp"
#include "exmc_config.hpp"
#include "EXTI.hpp"
#include "exti_config.hpp"
#include "FMC.hpp"
#include "fmc_config.hpp"
#include "FWDGT.hpp"
#include "fwdgt_config.hpp"
#include "GPIO.hpp"
#include "gpio_config.hpp"
#include "I2C.hpp"
#include "i2c_config.hpp"
#include "OB.hpp"
#include "PMU.hpp"
#include "pmu_config.hpp"
#include "RCU.hpp"
#include "rcu_config.hpp"
#include "RTC.hpp"
#include "rtc_config.hpp"
#include "SDIO.hpp"
#include "sdio_config.hpp"
#include "SPI.hpp"
#include "spi_config.hpp"
#include "TIMER.hpp"
#include "timer_config.hpp"
#include "USART.hpp"
#include "usart_config.hpp"
#include "WWDGT.hpp"
#include "wwdgt_config.hpp"


#ifndef F_CPU
    #define F_CPU   120'000'000U
#endif

template<typename T>
inline void UNUSED(T&&) noexcept {
    // Do nothing with the parameter
}
