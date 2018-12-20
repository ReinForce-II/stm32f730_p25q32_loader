/**
 * Copyright 2018, Reinforce-II
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include "main.h"
#include "quadspi.h"
#ifdef __DEBUG_LOG__
#include "usart.h"
#endif
#include "gpio.h"

#define FLASH_OFFSET 0x90000000
#define FLASH_SIZE 0x400000
#define FLASH_SECTOR_SIZE 4096
#define FLASH_PAGE_SIZE 256

void SystemClock_Config(void);

__attribute__((used)) static void LOG_D(const char *format, ...);

__attribute__((used)) static QSPI_CommandTypeDef cmd_read = {
    .Instruction = 0x3b,
    .AddressSize = QSPI_ADDRESS_24_BITS,
    .DummyCycles = 8,
    .InstructionMode = QSPI_INSTRUCTION_1_LINE,
    .AddressMode = QSPI_ADDRESS_1_LINE,
    .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
    .DataMode = QSPI_DATA_2_LINES,
    .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
};

__attribute__((used)) static QSPI_CommandTypeDef cmd_wren = {
    .Instruction = 0x06,
    .DummyCycles = 0,
    .InstructionMode = QSPI_INSTRUCTION_1_LINE,
    .AddressMode = QSPI_ADDRESS_NONE,
    .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
    .DataMode = QSPI_DATA_NONE,
    .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
};

__attribute__((used)) static QSPI_CommandTypeDef cmd_se = {
    .Instruction = 0x20,
    .AddressSize = QSPI_ADDRESS_24_BITS,
    .DummyCycles = 0,
    .InstructionMode = QSPI_INSTRUCTION_1_LINE,
    .AddressMode = QSPI_ADDRESS_1_LINE,
    .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
    .DataMode = QSPI_DATA_NONE,
    .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
};

__attribute__((used)) static QSPI_CommandTypeDef cmd_ce = {
    .Instruction = 0x60,
    .DummyCycles = 0,
    .InstructionMode = QSPI_INSTRUCTION_1_LINE,
    .AddressMode = QSPI_ADDRESS_NONE,
    .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
    .DataMode = QSPI_DATA_NONE,
    .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
};

__attribute__((used)) static QSPI_CommandTypeDef cmd_gbulk = {
    .Instruction = 0x98,
    .DummyCycles = 0,
    .InstructionMode = QSPI_INSTRUCTION_1_LINE,
    .AddressMode = QSPI_ADDRESS_NONE,
    .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
    .DataMode = QSPI_DATA_NONE,
    .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
};

__attribute__((used)) static QSPI_CommandTypeDef cmd_rdsr = {
    .Instruction = 0x05,
    .DummyCycles = 0,
    .InstructionMode = QSPI_INSTRUCTION_1_LINE,
    .AddressMode = QSPI_ADDRESS_NONE,
    .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
    .NbData = 1,
    .DataMode = QSPI_DATA_1_LINE,
    .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
};

__attribute__((used)) static QSPI_CommandTypeDef cmd_pp = {
    .Instruction = 0xA2,
    .AddressSize = QSPI_ADDRESS_24_BITS,
    .DummyCycles = 0,
    .InstructionMode = QSPI_INSTRUCTION_1_LINE,
    .AddressMode = QSPI_ADDRESS_1_LINE,
    .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
    .DataMode = QSPI_DATA_2_LINES,
    .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
};

__attribute__((used)) static void delay_ms(uint32_t n) {
    for (n = n * SystemCoreClock / 1000 / 3; n > 0; n--) {
        __asm volatile("nop");
    }
}

__attribute__((used)) static void wait_for_qspi_ready(void) {
    while (HAL_QSPI_GetState(&hqspi) != HAL_QSPI_STATE_READY) {
        delay_ms(1);
    }
}

__attribute__((used)) static void global_unlock(void) {
    HAL_QSPI_Command(&hqspi, &cmd_wren, 3000);
    wait_for_qspi_ready();
    HAL_QSPI_Command(&hqspi, &cmd_gbulk, 3000);
    wait_for_qspi_ready();
}

__attribute__((used)) static uint8_t get_status(void) {
    uint8_t ret;
    HAL_QSPI_Command(&hqspi, &cmd_rdsr, 3000);
    HAL_QSPI_Receive(&hqspi, &ret, 3000);
    wait_for_qspi_ready();
    return ret;
}

/**
 * @brief   Init
 * @param   None
 * @retval  1   :   Operation succeeded
 * @retval  0   :   Operation failed
 */
__attribute__((used)) int Init(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_QUADSPI_Init();
#ifdef __DEBUG_LOG__
    MX_USART6_UART_Init();
#endif
    wait_for_qspi_ready();
    LOG_D("External loader init done\n");
    return 1;
}

/**
 * @brief   Read
 * @param   address :   start address
 * @param   size    :   size in bytes
 * @param   buffer  :   point read buffer
 * @retval  1   :   Operation succeeded
 * @retval  0   :   Operation failed
 */
__attribute__((used)) int Read(uint32_t address, uint32_t size,
                               uint8_t *buffer) {
    LOG_D("Start reading, addr: %, size: %\n", address, size);
    if (address < FLASH_OFFSET || address + size > FLASH_OFFSET + FLASH_SIZE) {
        return 0;
    }
    cmd_read.Address = address - FLASH_OFFSET;
    cmd_read.NbData = size;
    HAL_QSPI_Command(&hqspi, &cmd_read, 3000);
    HAL_QSPI_Receive(&hqspi, buffer, 3000);
    wait_for_qspi_ready();
    LOG_D("External loader read done\n");
    return 1;
}

/**
 * @brief   Write
 * @param   address :   start address
 * @param   size    :   size in bytes
 * @param   buffer  :   write buffer
 * @retval  1   :   Operation succeeded
 * @retval  0   :   Operation failed
 */
__attribute__((used)) int Write(uint32_t address, uint32_t size,
                                uint8_t *buffer) {
    LOG_D("Start writing, addr: %, size: %\n", address, size);
    if (address < FLASH_OFFSET || address + size > FLASH_OFFSET + FLASH_SIZE) {
        return 0;
    }
    uint32_t current_address = address - FLASH_OFFSET;
    uint32_t remaining_size = size;
    global_unlock();
    while (1) {
        cmd_pp.Address = current_address;
        cmd_pp.NbData =
            remaining_size > FLASH_PAGE_SIZE ? FLASH_PAGE_SIZE : remaining_size;
        HAL_QSPI_Command(&hqspi, &cmd_wren, 3000);
        wait_for_qspi_ready();
        HAL_QSPI_Command(&hqspi, &cmd_pp, 3000);
        HAL_QSPI_Transmit(&hqspi, buffer, 3000);
        wait_for_qspi_ready();
        while (get_status() & 0x01) {
            // Busy
        }
        if (remaining_size <= FLASH_PAGE_SIZE) {
            break;
        } else {
            buffer += FLASH_PAGE_SIZE;
            current_address += FLASH_PAGE_SIZE;
            remaining_size -= FLASH_PAGE_SIZE;
        }
    }
    LOG_D("External loader write done\n");
    return 1;
}

/**
 * @brief   sector erase.
 * @param   erase_start_addr :  erase start address
 * @param   erase_end_addr   :  erase end address
 * @retval  1   :   Operation succeeded
 * @retval  0   :   Operation failed
 */
__attribute__((used)) int SectorErase(uint32_t erase_start_addr,
                                      uint32_t erase_end_addr) {
    LOG_D("Start sector erasing, erase_start_addr: %, erase_end_addr: "
          "%\n",
          erase_start_addr, erase_end_addr);
    if (erase_start_addr < FLASH_OFFSET ||
        erase_end_addr > FLASH_OFFSET + FLASH_SIZE) {
        return 0;
    }
    uint32_t current_address = erase_start_addr - FLASH_OFFSET;
    uint32_t remaining_size = erase_end_addr - erase_start_addr;
    global_unlock();
    while (1) {
        cmd_se.Address = current_address;
        HAL_QSPI_Command(&hqspi, &cmd_wren, 3000);
        wait_for_qspi_ready();
        HAL_QSPI_Command(&hqspi, &cmd_se, 3000);
        wait_for_qspi_ready();
        while (get_status() & 0x01) {
            // Busy
        }
        if (remaining_size <= FLASH_SECTOR_SIZE) {
            break;
        } else {
            current_address += FLASH_SECTOR_SIZE;
            remaining_size -= FLASH_SECTOR_SIZE;
        }
    }
    LOG_D("External loader sector erase done\n");
    return 1;
}

/**
 * @brief   chip erase.
 * @param   None
 * @retval  1   :   Operation succeeded
 * @retval  0   :   Operation failed
 */
__attribute__((used)) int MassErase() {
    LOG_D("Start mass erasing\n");
    global_unlock();
    HAL_QSPI_Command(&hqspi, &cmd_wren, 3000);
    wait_for_qspi_ready();
    HAL_QSPI_Command(&hqspi, &cmd_ce, 3000);
    wait_for_qspi_ready();
    while (get_status() & 0x01) {
        // Busy
    }
    LOG_D("External loader mass erase done\n");
    return 1;
}

/**
 * @brief   HAL_InitTick Stub
 * @param   TickPriority
 * @retval  HAL_OK
 */
__attribute__((used)) HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
    return HAL_OK;
}

__attribute__((used)) static void i2hs(uint32_t num, uint8_t *hs) {
    const uint8_t lut[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
                             '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
    for (uint32_t i = 0; i < 8; i++) {
        hs[7 - i] = lut[num & 0xf];
        num >>= 4;
    }
}

__attribute__((used)) static void LOG_D(const char *format, ...) {
#ifdef __DEBUG_LOG__
    uint8_t hs[8];
    uint32_t num;
    va_list vl;
    va_start(vl, format);
    while (*format) {
        switch (*format) {
        case '%':
            num = va_arg(vl, uint32_t);
            i2hs(num, hs);
            HAL_UART_Transmit(&huart6, hs, 8, 3000);
            break;
        default:
            HAL_UART_Transmit(&huart6, (uint8_t *)format, 1, 3000);
            break;
        }
        format++;
    }
    va_end(vl);
#else
    (void)format;
#endif
}
