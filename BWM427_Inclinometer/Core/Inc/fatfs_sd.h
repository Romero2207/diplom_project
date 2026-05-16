#ifndef FATFS_SD_H
#define FATFS_SD_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "diskio.h"

// Используем пины, которые ты настроил в CubeMX
#define SD_CS_PORT SD_CS_GPIO_Port
#define SD_CS_PIN  SD_CS_Pin

DSTATUS SD_disk_initialize(BYTE pdrv);
DSTATUS SD_disk_status(BYTE pdrv);
DRESULT SD_disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void* buff);

#endif
