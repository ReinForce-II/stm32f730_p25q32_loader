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

#define MCU_FLASH 1
#define NAND_FLASH 2
#define NOR_FLASH 3
#define SRAM 4
#define PSRAM 5
#define PC_CARD 6
#define SPI_FLASH 7
#define I2C_FLASH 8
#define SDRAM 9
#define I2C_EEPROM 10

#define SECTOR_NUM 10 // Max Number of Sector types

#include <stdint.h>

#pragma pack(4)
struct DeviceSectors {
    uint32_t SectorNum;  // Number of Sectors
    uint32_t SectorSize; // Sector Size in Bytes
};

#pragma pack(4)
struct StorageInfo {
    char DeviceName[100];        // Device Name and Description
    uint16_t DeviceType;         // Device Type: ONCHIP, EXT8BIT, EXT16BIT, ...
    uint32_t DeviceStartAddress; // Default Device Start Address
    uint32_t DeviceSize;         // Total Size of Device
    uint32_t PageSize;           // Programming Page Size
    uint8_t EraseValue;          // Content of Erased Memory
    struct DeviceSectors sectors[SECTOR_NUM];
};

#pragma pack(4)
__attribute__((used)) struct StorageInfo const StorageInfo = {
    .DeviceName = "P25Q32H_STM32F7_QSPI", // Device Name + version number
    .DeviceType = SPI_FLASH,              // Device Type
    .DeviceStartAddress = 0x90000000,     // Device Start Address
    .DeviceSize = 0x00400000, // Device Size in Bytes (4MBytes/32Mbits)
    .PageSize = 0x00000100,   // Programming Page Size (256 Bytes)
    .EraseValue = 0xFF,       // Initial Content of Erased Memory
                              // Specify Size and Address of Sectors
    .sectors =
        {
            {
                .SectorNum = 0x00000400,  // Sector Num : 1024
                .SectorSize = 0x00001000, // Sector Size: 4KBytes
            },
            {
                .SectorNum = 0,
                .SectorSize = 0,
            },
        },
};
