#include "fatfs_sd.h"

extern SPI_HandleTypeDef hspi2;

#define SD_CS_LOW()  HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH() HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET)

#define CMD0   (0)
#define CMD8   (8)
#define CMD17  (17)
#define CMD24  (24)
#define CMD55  (55)
#define ACMD41 (0x80 + 41)

static volatile DSTATUS Stat = STA_NOINIT;

static void SPI_TxByte(BYTE data) {
	BYTE dummy_rx;
	HAL_SPI_TransmitReceive(&hspi2, &data, &dummy_rx, 1, 100);
}

static BYTE SPI_RxByte(void) {
	BYTE data = 0;
	BYTE dummy_tx = 0xFF;
	HAL_SPI_TransmitReceive(&hspi2, &dummy_tx, &data, 1, 100);
	return data;
}

static BYTE SD_SendCmd(BYTE cmd, DWORD arg) {
	BYTE n, res;
	if (cmd & 0x80) {
		cmd &= 0x7F;
		res = SD_SendCmd(CMD55, 0);
		if (res > 1)
			return res;
	}
	SD_CS_LOW();
	SPI_TxByte(0xFF);
	SPI_TxByte(cmd | 0x40);
	SPI_TxByte((BYTE) (arg >> 24));
	SPI_TxByte((BYTE) (arg >> 16));
	SPI_TxByte((BYTE) (arg >> 8));
	SPI_TxByte((BYTE) arg);
	n = 0x01;
	if (cmd == CMD0)
		n = 0x95;
	if (cmd == CMD8)
		n = 0x87;
	SPI_TxByte(n);

	// Таймаут ожидания ответа от карты 2000 байт
	uint16_t retry = 2000;
	do {
		res = SPI_RxByte();
	} while ((res & 0x80) && --retry);
	return res;
}

DSTATUS SD_disk_initialize(BYTE pdrv) {
    BYTE n, ty, ocr[4];
    if (pdrv) return STA_NOINIT;

    Stat = STA_NOINIT;
    SD_CS_HIGH();
    HAL_Delay(10); // Ждем устаканивания контактов слота

    // 80 тактов пустышек для гарантированного запуска внутреннего осциллятора SD-карты
    for (n = 80; n; n--) SPI_TxByte(0xFF);

    ty = 0;

    uint8_t is_idle = 0;
    for (BYTE r = 0; r < 10; r++) {
        if (SD_SendCmd(CMD0, 0) == 1) {
            is_idle = 1;
            break; // Карта успешно проснулась!
        }
        SD_CS_HIGH();      // Обязательно передергиваем Chip Select
        SPI_TxByte(0xFF);  // Выгоняем мусор из шины
        HAL_Delay(10);     // Даем паузу перед новой попыткой
    }

    if (is_idle) { // Если карта наконец-то ответила
        if (SD_SendCmd(CMD8, 0x1AA) == 1) {
            for (n = 0; n < 4; n++) ocr[n] = SPI_RxByte();
            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
                uint16_t timeout = 1000;
                while (SD_SendCmd(ACMD41, 1UL << 30) && timeout--) HAL_Delay(1);
                if (timeout > 0) ty = 1;
            }
        } else {
            uint16_t timeout = 1000;
            while (SD_SendCmd(ACMD41, 0) && timeout--) HAL_Delay(1);
            if (timeout > 0) ty = 1;
        }
    }

    SD_CS_HIGH();
    SPI_RxByte();

    if (ty) Stat &= ~STA_NOINIT;
    else Stat = STA_NOINIT;

    return Stat;
}
DSTATUS SD_disk_status(BYTE pdrv) {
	return Stat;
}

DRESULT SD_disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count) {
	if (pdrv || !count)
		return RES_PARERR;
	if (Stat & STA_NOINIT)
		return RES_NOTRDY;

	for (UINT c = 0; c < count; c++) {
		if (SD_SendCmd(CMD17, sector + c) == 0) {
			// Точный временной таймаут 500 мс на ожидание маркера данных
			uint32_t start_tick = HAL_GetTick();
			while (SPI_RxByte() != 0xFE) {
				if (HAL_GetTick() - start_tick > 500) {
					SD_CS_HIGH();
					return RES_ERROR;
				}
			}

			for (int i = 0; i < 512; i++) {
				buff[c * 512 + i] = SPI_RxByte();
			}

			SPI_RxByte();
			SPI_RxByte();
		} else {
			SD_CS_HIGH();
			return RES_ERROR;
		}
	}

	SD_CS_HIGH();
	SPI_RxByte();
	return RES_OK;
}

DRESULT SD_disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count) {
	if (pdrv || !count)
		return RES_PARERR;
	if (Stat & STA_NOINIT)
		return RES_NOTRDY;

	for (UINT c = 0; c < count; c++) {
		if (SD_SendCmd(CMD24, sector + c) == 0) {
			SPI_TxByte(0xFE);

			for (int i = 0; i < 512; i++) {
				SPI_TxByte(buff[c * 512 + i]);
			}

			SPI_TxByte(0xFF);
			SPI_TxByte(0xFF);

			if ((SPI_RxByte() & 0x1F) == 0x05) {
				// Точный таймаут 500 мс на физическую запись во флеш-память
				uint32_t start_tick = HAL_GetTick();
				while (SPI_RxByte() == 0) {
					if (HAL_GetTick() - start_tick > 500) {
						SD_CS_HIGH();
						return RES_ERROR;
					}
				}
			} else {
				SD_CS_HIGH();
				return RES_ERROR;
			}
		} else {
			SD_CS_HIGH();
			return RES_ERROR;
		}
	}

	SD_CS_HIGH();
	SPI_RxByte();
	return RES_OK;
}

DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
	if (pdrv)
		return RES_PARERR;
	if (Stat & STA_NOINIT)
		return RES_NOTRDY;

	DRESULT res = RES_ERROR;
	switch (cmd) {
	case CTRL_SYNC:
		SD_CS_LOW();
		uint32_t start_tick = HAL_GetTick();
		while (SPI_RxByte() != 0xFF) {
			if (HAL_GetTick() - start_tick > 500) {
				SD_CS_HIGH();
				return RES_ERROR;
			}
		}
		res = RES_OK;
		break;

	case GET_SECTOR_SIZE:
		*(WORD*) buff = 512;
		res = RES_OK;
		break;
	}
	SD_CS_HIGH();
	SPI_RxByte();
	return res;
}
