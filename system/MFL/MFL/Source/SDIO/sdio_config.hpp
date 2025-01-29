//
// MFL gd32f30x SDIO peripheral register access in C++
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

#include <stdlib.h>
#include <stdint.h>
#include <array>

#include "CONFIG.hpp"
#include "DMA.hpp"

namespace sdio {


///////////////////////////// BASE ADDRESS /////////////////////////////

enum class SDIO_Base : uint32_t {
    SDIO_BASE,
    INVALID
};


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class SDIO_Regs : uint32_t {
    PWRCTL = 0x00U,
    CLKCTL = 0x04U,
    CMDAGMT = 0x08U,
    CMDCTL	 = 0x0CU,
    RSPCMDIDX = 0x10U,
    RESP0 = 0x14U,
    RESP1 = 0x18U,
    RESP2 = 0x1CU,
    RESP3 = 0x20U,
    DATATO = 0x24U,
    DATALEN = 0x28U,
    DATACTL = 0x2CU,
    DATACNT = 0x30U,
    STAT = 0x34U,
    INTC = 0x38U,
    INTEN = 0x3CU,
    FIFOCNT = 0x48U,
    FIFO = 0x80U
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class PWRCTL_Bits : uint32_t {
    PWRCTL = REG_BIT_DEF(0, 1)
};

enum class CLKCTL_Bits : uint32_t {
    DIV = REG_BIT_DEF(0, 7),
    CLKEN = 8,
    CLKPWRSAV = 9,
    CLKBYP = 10,
    BUSMODE = REG_BIT_DEF(11, 12),
    CLKEDGE = 13,
    HWCLKEN = 14,
    DIV8 = 31
};

inline constexpr uint8_t DIV_Pos = 0U;
inline constexpr uint8_t DIV_Mask = 0xFFU;
inline constexpr uint8_t BUSMODE_Pos = 11U;
inline constexpr uint8_t BUSMODE_Mask = 0x3U;

enum class CMDAGMT_Bits : uint32_t {
    CMDAGMT = REG_BIT_DEF(0, 31)
};

// Needs sync
enum class CMDCTL_Bits : uint32_t {
    CMDIDX = REG_BIT_DEF(0, 5),
    CMDRESP = REG_BIT_DEF(6, 7),
    INTWAIT = 8,
    WAITDEND = 9,
    WAITTYPE = REG_BIT_DEF(8, 9),   // Convenience
    CSMEN = 10,
    SUSPEND = 11,
    ENCMDC = 12,
    NINTEN = 13,
    ATAEN = 14
};

inline constexpr uint8_t CMDIDX_Pos = 0U;
inline constexpr uint8_t CMDIDX_Mask = 0x3FU;
inline constexpr uint8_t CMDRESP_Pos = 6U;
inline constexpr uint8_t CMDRESP_Mask = 0x3U;
inline constexpr uint8_t WAITTYPE_Pos = 8U;
inline constexpr uint8_t WAITTYPE_Mask = 0x3U;

enum class RSPCMDIDX_Bits : uint32_t {
    RSPCMDIDX = REG_BIT_DEF(0, 5)
};

enum class DATALEN_Bits : uint32_t {
    DATALEN = REG_BIT_DEF(0, 24)
};

enum class DATACTL_Bits : uint32_t {
    DATAEN = 0,
    DATADIR = 1,
    TRANSMOD = 2,
    DMAEN = 3,
    BLKSZ = REG_BIT_DEF(4, 7),
    RWEN = 8,
    RWSTOP = 9,
    RWTYPE = 10,
    IOEN = 11
};

inline constexpr uint8_t BLKSZ_Pos = 4U;
inline constexpr uint8_t BLKSZ_Mask = 0x7U;

enum class DATACNT_Bits : uint32_t {
    DATACNT = REG_BIT_DEF(0, 24)
};

enum class FIFOCNT_Bits : uint32_t {
    FIFOCNT = REG_BIT_DEF(0, 23)
};

enum class FIFO_Bits : uint32_t {
    FIFODT = REG_BIT_DEF(0, 31)
};

enum class Status_Flags : uint8_t {
    FLAG_CCRCERR,
    FLAG_DTCRCERR,
    FLAG_CMDTMOUT,
    FLAG_DTTMOUT,
    FLAG_TXURE,
    FLAG_RXORE,
    FLAG_CMDRECV,
    FLAG_CMDSEND,
    FLAG_DTEND,
    FLAG_STBITE,
    FLAG_DTBLKEND,
    FLAG_CMDRUN,
    FLAG_TXRUN,
    FLAG_RXRUN,
    FLAG_TFH,
    FLAG_RFH,
    FLAG_TFF,
    FLAG_RFF,
    FLAG_TFE,
    FLAG_RFE,
    FLAG_TXDTVAL,
    FLAG_RXDTVAL,
    FLAG_SDIOINT,
    FLAG_ATAEND
};

inline constexpr uint32_t CCRCERR_Mask = (1U << 0U);
inline constexpr uint32_t CMDTMOUT_Mask = (1U << 2U);
inline constexpr uint32_t CMDRECV_Mask = (1U << 6U);
inline constexpr uint32_t CMDRUN_Mask = (1U << 11U);

enum class Clear_Flags : uint8_t {
    FLAG_CCRCERRC,
    FLAG_DTCRCERRC,
    FLAG_CMDTMOUTC,
    FLAG_DTTMOUTC,
    FLAG_TXUREC,
    FLAG_RXOREC,
    FLAG_CMDRECVC,
    FLAG_CMDSENDC,
    FLAG_DTENDC,
    FLAG_STBITEC,
    FLAG_DTBLKENDC,
    FLAG_SDIOINTC = 22,
    FLAG_ATAENDC = 23
};

inline const uint32_t clear_command_flags = (
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_CCRCERRC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_CMDTMOUTC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_CMDRECVC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_CMDSENDC))
);

inline const uint32_t clear_data_flags = (
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_DTCRCERRC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_DTTMOUTC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_TXUREC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_RXOREC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_DTENDC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_DTBLKENDC))
);

inline const uint32_t clear_common_flags = (
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_CCRCERRC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_DTCRCERRC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_CMDTMOUTC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_DTTMOUTC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_TXUREC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_RXOREC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_CMDRECVC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_CMDSENDC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_DTENDC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_DTBLKENDC)) |
    (1U << static_cast<uint32_t>(Clear_Flags::FLAG_SDIOINTC))
);

enum class Interrupt_Flags : uint8_t {
    FLAG_INTR_CCRCERR,
    FLAG_INTR_DTCRCERR,
    FLAG_INTR_CMDTMOUT,
    FLAG_INTR_DTTMOUT,
    FLAG_INTR_TXURE,
    FLAG_INTR_RXORE,
    FLAG_INTR_CMDRECV,
    FLAG_INTR_CMDSEND,
    FLAG_INTR_DTEND,
    FLAG_INTR_STBITE,
    FLAG_INTR_DTBLKEND,
    FLAG_INTR_CMDRUN,
    FLAG_INTR_TXRUN,
    FLAG_INTR_RXRUN,
    FLAG_INTR_TFH,
    FLAG_INTR_RFH,
    FLAG_INTR_TFF,
    FLAG_INTR_RFF,
    FLAG_INTR_TFE,
    FLAG_INTR_RFE,
    FLAG_INTR_TXDTVAL,
    FLAG_INTR_RXDTVAL,
    FLAG_INTR_SDIOINT,
    FLAG_INTR_ATAEND
};

// Alias
enum class Interrupt_Type : uint8_t {
    CCRCERRIE,
    DTCRCERRIE,
    CMDTMOUTIE,
    DTTMOUTIE,
    TXUREIE,
    RXOREIE,
    CMDRECVIE,
    CMDSENDIE,
    DTENDIE,
    STBITEIE,
    DTBLKENDIE,
    CMDRUNIE,
    TXRUNIE,
    RXRUNIE,
    TFHIE,
    RFHIE,
    TFFIE,
    RFFIE,
    TFEIE,
    RFEIE,
    TXDTVALIE,
    RXDTVALIE,
    SDIOINTIE,
    ATAENDIE
};


///////////////////////////// ENUMS /////////////////////////////

enum class Power_Control : uint8_t {
    POWER_OFF,
    POWER_ON = 3U
};

enum class Bus_Width : uint8_t {
    WIDTH_1BIT,
    WIDTH_4BIT,
    WIDTH_8BIT
};

enum class Clock_Edge : uint8_t {
    RISING_EDGE,
    FALLING_EDGE
};

enum class Command_Response : uint8_t {
    RSP_NONE = 0U,
    RSP_SHORT = 1U,
    RSP_LONG = 3U
};

enum class Wait_Type : uint8_t {
    WT_NONE,
    WT_INTERRUPT,
    WT_COMPLETE
};

enum class Response_Type : uint8_t {
    RESPONSE0,
    RESPONSE1,
    RESPONSE2,
    RESPONSE3
};

enum class Block_Size : uint8_t {
    BYTES_1,
    BYTES_2,
    BYTES_4,
    BYTES_8,
    BYTES_16,
    BYTES_32,
    BYTES_64,
    BYTES_128,
    BYTES_256,
    BYTES_512,
    BYTES_1024,
    BYTES_2048,
    BYTES_4096,
    BYTES_8192,
    BYTES_16384
};

enum Transfer_Mode : uint8_t {
    BLOCK,
    STREAM
};

enum class Transfer_Direction : uint8_t {
    SDIO_TO_CARD,
    CARD_TO_SDIO
};

enum class Read_Wait : uint8_t {
    DAT2,
    CLK
};

enum class Command_Index : uint8_t {
    CMD0 = 0U,          // GO_IDLE_STATE
    CMD1 = 1U,          // SEND_OP_COND (MMC)
    CMD2 = 2U,          // ALL_SEND_CID
    CMD3 = 3U,          // SEND_RELATIVE_ADDR
    CMD4 = 4U,          // SET_DSR
    CMD5 = 5U,          // SEND_OP_COND (SD-IO)
    CMD6 = 6U,          // SWITCH_FUNC
    ACMD6 = 6U,         // ACMD
    CMD7 = 7U,          // SELECT_DESELECT_CARD
    CMD8 = 8U,          // SEND_IF_COND
    CMD9 = 9U,          // SEND_CSD
    CMD10 = 10U,        // SEND_CID
    CMD11 = 11U,        // VOL_SWITCH
    CMD12 = 12U,        // STOP_TRANSMISSION
    CMD13 = 13U,        // SEND_STATUS
    ACMD13 = 13U,       // ACMD
    CMD14 = 14U,        // CHECK_BUS_TEST (MMC)
    CMD15 = 15U,        // GO_INACTIVE_STATE
    CMD16 = 16U,        // SET_BLOCKLEN
    CMD17 = 17U,        // READ_SINGLE_BLOCK
    CMD18 = 18U,        // READ_MULTIPLE_BLOCK
    CMD19 = 19U,        // SEND_TUNING_BLOCK
    CMD20 = 20U,        // SEND_TUNING_BLOCK (MMC)
    ACMD22 = 22U,       // ACMD
    CMD23 = 23U,        // SET_BLOCK_COUNT
    ACMD23 = 23U,       // ACMD
    CMD24 = 24U,        // WRITE_BLOCK
    CMD25 = 25U,        // WRITE_MULTIPLE_BLOCK
    CMD27 = 27U,        // PROGRAM_CSD
    CMD28 = 28U,        // SET_WRITE_PROT
    CMD29 = 29U,        // CLR_WRITE_PROT
    CMD30 = 30U,        // SEND_WRITE_PROT
    CMD32 = 32U,        // ERASE_WR_BLK_START
    CMD33 = 33U,        // ERASE_WR_BLK_END
    CMD38 = 38U,        // ERASE
    ACMD41 = 41U,       // ACMD
    CMD42 = 42U,        // LOCK_UNLOCK
    ACMD42 = 42U,       // ACMD
    ACMD51 = 51U,       // ACMD
    CMD55 = 55U,        // APP_CMD
    CMD56 = 56U,        // GEN_CMD
    INVALID = 60U
};

enum class Application_Command_Index : uint8_t {
    ACMD6 = 6U,         // SET_BUS_WIDTH
    ACMD13 = 13U,       // SD_STATUS
    ACMD22 = 22U,       // SEND_NUM_WR_BLOCKS
    ACMD23 = 23U,       // SET_WR_BLK_ERASE_COUNT
    ACMD41 = 41U,       // SD_APP_OP_COND
    ACMD42 = 42U,       // SET_CLR_CARD_DETECT
    ACMD51 = 51U        // SEND_SCR
};

enum class Card_Command_Class : uint8_t {
    BASIC = 0U,
    BLOCK_READ = 2U,
    BLOCK_WRITE = 4U,
    ERASE = 5U,
    WRITE_PROTECTION = 6U,
    LOCK_CARD = 7U,
    APPICATION_SPECIFIC = 8U,
    IO_MODE = 9U,
    SWITCH = 10U
};

enum class Lock_State : uint8_t {
    UNLOCK = 2U,
    LOCK = 5U
};

enum class Interface_Version : uint8_t {
    INTERFACE_V1_1,
    INTERFACE_V2_0,
    UNKNOWN
};

enum class Card_Type : uint8_t {
    SDCARD_STANDARD_CAPACITY,
    SDCARD_HIGH_CAPACITY,
    SD_IO_CARD,
    SD_IO_COMBO_CARD,
    SD_MMC,
    SD_MMC_HIGH_CAPACITY,
    SD_MMC_HIGH_SPEED,
    UNKNOWN
};

enum class Transfer_State : uint8_t {
    IDLE,	// No transfer
    BUSY	// Transfer in prgress
};

enum class Transfer_Method : uint8_t {
    METHOD_POLLING,
    METHOD_DMA
};

enum class Card_State : uint8_t {
    IDLE,
    READY,
    IDENTIFICATION,
    STANDBY,
    TRANSFER,
    SEND_DATA,
    RECEIVE_DATA,
    PROGRAMMING,
    DISCONNECTED,
    LOCKED
};

enum class Voltage_Validation : uint32_t {
    STANDARD_CAPACITY = 0U,
    SWITCH_1_8V_CAPACITY = (1U << 24U),
    HIGH_CAPACITY = (1U << 30U)
};

enum class R1_Status : uint32_t {
    R1_OK = 0U,
    AKE_SEQ_ERROR = (1U << 3U),
    APP_CMD = (1U << 5U),
    SWITCH_ERROR = (1U << 7U),
    READY_FOR_DATA = (1U << 8U),
    CURRENT_STATE = (0xFU << 9U),
    ERASE_RESET = (1U << 13U),
    CARD_ECC_DISABLED = (1U << 14U),
    WP_ERASE_SKIP = (1U << 15U),
    CID_CSD_OVERWRITE = (1U << 16U),
    OVERRUN = (1U << 17U),
    UNDERRUN = (1U << 18U),
    ERROR = (1U << 19U),
    CC_ERROR = (1U << 20U),
    CARD_ECC_FAILED = (1U << 21U),
    ILLEGAL_COMMAND = (1U << 22U),
    COM_CRC_ERROR = (1U << 23U),
    LOCK_UNLOCK_FAILED = (1U << 24U),
    CARD_IS_LOCKED = (1U << 25U),
    WP_VIOLATION = (1U << 26U),
    ERASE_PARAM = (1U << 27U),
    ERASE_SEQ_ERROR = (1U << 28U),
    BLOCK_LEN_ERROR = (1U << 29U),
    ADDRESS_ERROR = (1U << 30U),
    OUT_OF_RANGE = (1U << 31U)
};

enum class Operational_State {
    BUSY,
    READY,
    ERROR
};

enum class SDIO_Error_Type : uint8_t {
    OK,
    ERROR,
    BUSY,
    UNSUPPORTED_FUNCTION,
    INVALID_OPERATION,
    INVALID_PARAMETER,
    INVALID_VOLTAGE,
    START_BIT_ERROR,
    RX_FIFO_OVERRUN,
    TX_FIFO_UNDERRUN,
    DATA_TIMEOUT,
    RESPONSE_TIMEOUT,
    DATA_CRC_ERROR,
    COMMAND_CRC_ERROR,
    AUTHENTICATION_ERROR,
    ERASE_SEQUENCE_RESET,
    ECC_DISABLED,
    ERASE_SKIP,
    CSD_OVERWRITE,
    UNKNOWN_ERROR,
    CARD_CONTROLLER_ERROR,
    ECC_FAILED,
    INVALID_COMMAND,
    ILLEGAL_COMMAND,
    LOCK_UNLOCK_FAILED,
    WRITE_PROTECT_VIOLATION,
    INVALID_ERASE_BLOCKS,
    ERASE_SEQUENCE_ERROR,
    INVALID_BLOCK_LENGTH,
    INVALID_ADDRESS,
    COMMAND_OUT_OF_RANGE,
    INITIALIZATION_FAILED,
    INVALID_SELECTION,
    DMA_INSTANCE_ERROR,
    SELECT_DESELECT_FAILED,
    CMD0_FAILED,
    CMD2_FAILED,
    CMD3_FAILED,
    ACMD6_FAILED,
    CMD7_FAILED,
    CMD9_FAILED,
    CMD12_FAILED,
    ACMD13_FAILED,
    CMD13_FAILED,
    CMD16_FAILED,
    CMD17_FAILED,
    CMD18_FAILED,
    ACMD23_FAILED,
    CMD24_FAILED,
    CMD25_FAILED,
    CMD32_FAILED,
    CMD33_FAILED,
    CMD38_FAILED,
    ACMD41_FAILED,
    ACMD51_FAILED,
    CMD55_FAILED
};

struct Error_Map {
    uint32_t mask;
    SDIO_Error_Type errorType;
};

static inline constexpr std::array<Error_Map, 17> errorTableR1 {{
    {static_cast<uint32_t>(R1_Status::OUT_OF_RANGE), SDIO_Error_Type::COMMAND_OUT_OF_RANGE},
    {static_cast<uint32_t>(R1_Status::ADDRESS_ERROR), SDIO_Error_Type::INVALID_ADDRESS},
    {static_cast<uint32_t>(R1_Status::BLOCK_LEN_ERROR), SDIO_Error_Type::INVALID_BLOCK_LENGTH},
    {static_cast<uint32_t>(R1_Status::ERASE_SEQ_ERROR), SDIO_Error_Type::ERASE_SEQUENCE_ERROR},
    {static_cast<uint32_t>(R1_Status::ERASE_PARAM), SDIO_Error_Type::INVALID_ERASE_BLOCKS},
    {static_cast<uint32_t>(R1_Status::WP_VIOLATION), SDIO_Error_Type::WRITE_PROTECT_VIOLATION},
    {static_cast<uint32_t>(R1_Status::LOCK_UNLOCK_FAILED), SDIO_Error_Type::LOCK_UNLOCK_FAILED},
    {static_cast<uint32_t>(R1_Status::COM_CRC_ERROR), SDIO_Error_Type::COMMAND_CRC_ERROR},
    {static_cast<uint32_t>(R1_Status::ILLEGAL_COMMAND), SDIO_Error_Type::ILLEGAL_COMMAND},
    {static_cast<uint32_t>(R1_Status::CARD_ECC_FAILED), SDIO_Error_Type::ECC_FAILED},
    {static_cast<uint32_t>(R1_Status::CC_ERROR), SDIO_Error_Type::CARD_CONTROLLER_ERROR},
    {static_cast<uint32_t>(R1_Status::ERROR), SDIO_Error_Type::ERROR},
    {static_cast<uint32_t>(R1_Status::CID_CSD_OVERWRITE), SDIO_Error_Type::CSD_OVERWRITE},
    {static_cast<uint32_t>(R1_Status::WP_ERASE_SKIP), SDIO_Error_Type::ERASE_SKIP},
    {static_cast<uint32_t>(R1_Status::CARD_ECC_DISABLED), SDIO_Error_Type::ECC_DISABLED},
    {static_cast<uint32_t>(R1_Status::ERASE_RESET), SDIO_Error_Type::ERASE_SEQUENCE_RESET},
    {static_cast<uint32_t>(R1_Status::AKE_SEQ_ERROR), SDIO_Error_Type::AUTHENTICATION_ERROR},
}};

enum class R6_Status : uint32_t {
    ERROR = (1U << 13U),
    ILLEGAL_COMMAND = (1U << 14U),
    COM_CRC_ERROR = (1U << 15U)
};

static inline constexpr std::array<Error_Map, 3> errorTableR6 {{
    {static_cast<uint32_t>(R6_Status::ERROR), SDIO_Error_Type::ERROR},
    {static_cast<uint32_t>(R6_Status::ILLEGAL_COMMAND), SDIO_Error_Type::ILLEGAL_COMMAND},
    {static_cast<uint32_t>(R6_Status::COM_CRC_ERROR), SDIO_Error_Type::COMMAND_CRC_ERROR},
}};


///////////////////////////// STRUCTURES /////////////////////////////

struct SDIO_Config {
    uint32_t desired_clock;
    Clock_Edge clock_edge;
    Bus_Width width;
    bool enable_bypass;
    bool enable_powersave;
    bool enable_hwclock;
};

//struct SDCard_Info {
//    Card_Type card_type;
//    Interface_Version sdio_version;
//    uint32_t card_class;
//    uint32_t capacity_blocks;
//    uint32_t block_size;
//    uint32_t logical_capacity_blocks;
//    uint32_t logical_block_size;
//    uint16_t relative_address;
//};

struct Card_Identification {
    volatile uint8_t manufacture_id;
    volatile uint16_t oem_id;
    volatile uint32_t name0;
    volatile uint8_t name1;
    volatile uint8_t revision;
    volatile uint32_t serial_number;
    volatile uint8_t reserved0;
    volatile uint16_t manufacture_date;
    volatile uint8_t checksum;
    volatile uint8_t reserved1;
};

// Subset of CSD
struct Card_CSD {
    volatile uint8_t transfer_speed;
    volatile uint16_t card_command_class;
    volatile uint8_t read_block_length;
    volatile uint32_t device_size;           // CSD V1.0: 12 bits, CSD V2.0: 22 bits
    volatile uint8_t device_size_multiplier; // CSD V1.0
    volatile uint8_t sector_size;
    volatile uint8_t speed_factor;
    volatile uint8_t write_block_length;
    volatile uint8_t checksum;
};

struct Card_Info {
    Card_Identification cid;
    Card_CSD csd;
    Card_Type type;
    uint32_t capacity;
    uint32_t block_size;
    uint16_t relative_address;
};

struct DMA_Instance_Config {
    dma::DMA_Base dma_base;
    dma::DMA_Channel dma_channel;
};

struct Command_Type {
    Command_Index cmd_index;
    uint32_t cmd_arg;
    Command_Response response;
    Wait_Type wait_type;
    bool send_cmd;
};

struct Data_Type {
    uint32_t timeout;
    uint32_t length;
    uint32_t block_size;
    Transfer_Mode mode;
    Transfer_Direction direction;
    bool send_data;
};


///////////////////////////// CONSTANTS /////////////////////////////

inline constexpr uint32_t KILOBYTE = 1024U;
inline constexpr uint32_t BLOCK_SIZE = 512U;                    // Block size for high-capacity cards
inline constexpr uint32_t Check_Pattern = 0x000001AAU;
inline constexpr uint32_t Voltage_Window = 0x80100000U;         // Host 3.3V request
inline constexpr uint32_t Max_Voltage_Checks = 0x0000FFFFU;     // Maximum number of voltage validation checks
inline constexpr uint32_t Max_Data_Length = 0x01FFFFFFU;        // Maximum length of data
inline constexpr uint32_t Data_Timeout = 0xFFFFFFFFU;           // Data timeout of the state machine
inline constexpr uint32_t Default_Init_Clock = 400000U;         // Max init clock 400KHz but in practice is not stable, so use 300Khz
inline constexpr uint32_t Default_Desired_Clock = 25000000U;    // Maximum possible clock speed
inline constexpr uint32_t SDCARD_Standard = 0x00000000U;        // SDCard standard capacity
inline constexpr uint32_t SDCARD_HCS = 0x40000000U;             // HCS bit = 1 for voltage validation
inline constexpr uint32_t Switch_1_8V_Capacity = 0x01000000U;
inline constexpr uint32_t Card_Locked = 0x02000000U;
inline constexpr uint32_t SCR_Width_4 = 0x00040000U;
inline constexpr uint32_t SCR_Width_1 = 0x00010000U;
inline constexpr uint32_t All_R1_Error_Bits = 0xFDF9E008U;
inline constexpr uint32_t FIFO_Half_Words = 0x00000008U;
inline constexpr uint32_t FIFO_Half_Bytes = 0x00000020U;
inline constexpr uint32_t PasswordKey1 = 0x01020600U;
inline constexpr uint32_t PasswordKey2 = 0x03040506U;
inline constexpr uint32_t CardStateMask = 0x0000000FU;
//inline constexpr uint32_t Clear_All_Cmd_Flags = 0x00C002F5U;
//inline constexpr uint32_t Clear_All_Dat_Flags = 0x00C0073AU;
inline constexpr uint32_t R6_Error_Bits = 0x0000E000U;
inline constexpr uint8_t RCA_Shift = 0x10U;


///////////////////////////// INITIALIZATION DEFAULTS /////////////////////////////

static inline const SDIO_Config default_config = {
    Default_Init_Clock,
    Clock_Edge::RISING_EDGE,
    Bus_Width::WIDTH_1BIT,
    false,
    false,
    false
};

static inline const DMA_Instance_Config default_dma_config = {
    dma::DMA_Base::DMA1_BASE,
    dma::DMA_Channel::CHANNEL3
};

} // namespace sdio
