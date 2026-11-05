#ifndef EXT_FLASH_H
#define EXT_FLASH_H
#include <nrfx.h>
#include <stdbool.h>
#include <stdint.h>
//#include "gd5flash.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ SPI NAND CMD
 */
#define sFLASH_CHIP_ID                0xC881
#define sFLASH_CMD_NOR_RESET          0xFF 
#define sFLASH_CMD_Get_Feature        0x0F      /* GET Feature */
#define sFLASH_CMD_Set_Feature        0x1F      /* Set Feature */

#define sFLASH_CMD_NAND_BE            0xD8
#define sFLASH_CMD_READ_TO_CACHE      0x13      /*Read to cache*/
#define sFLASH_CMD_READ_FROM_CACHE    0x03      /*!< Read from cache */
#define sFLASH_CMD_NOR_RDID           0x9F

#define sFLASH_CMD_NAND_PRLD          0x02
#define sFLASH_CMD_NAND_WREN          0x06
#define sFLASH_CMD_NAND_W_Execute     0x10      /*!< Write to Memory instruction */

#define sFLASH_DUMMY_BYTE             0x55

#define  EXT_BLOCKNUM           1024
#define  EXT_PAGES_IN_BLOCK     64
#define  EXT_PAGESIZE           2048
#define  EXT_PAGESIZE_WHOLE     2176
#define  EXT_MAXPAGENUM            (EXT_BLOCKNUM*EXT_PAGES_IN_BLOCK)
#define  EXT_PACKETSIZE         512

#define ERROR 0
#define INFO 1

#define  EXT_WRITEIN_LEN        512
#define  EXT_DATAPAGE_START     256//64*4
#define  EXT_LISTPAGE_START     0
#define  EXT_LIST_DATAADDR_SP   5

#define  EXT_MAXROWNUM_SEC      128
#define  EXT_MAXROWNUM_HRV      64
#define  EXT_MAXROWNUM_FILE     (3*EXT_PAGES_IN_BLOCK)
#define  EXT_REPORTS_MAX_COUNT  255
#define  EXT_MAXBYTECNT_SEC        (EXT_MAXROWNUM_SEC*EXT_PAGESIZE)
#define  EXT_MAXBYTECNT_HRV        (EXT_MAXROWNUM_HRV*EXT_PAGESIZE)
//#define  EXT_MAXBYTECNT_ACC        (EXT_MAXROWNUM_ACC*EXT_PAGESIZE)

#define LOG_FLASH(log_level, ...)       \
    do {                                \
        if (log_level == ERROR) {       \
            NRF_LOG_ERROR(__VA_ARGS__); \
        } else if (log_level == INFO) { \
            NRF_LOG_INFO(__VA_ARGS__);  \
        }                               \
    } while (0);

enum flash_error_codes {
    FLASH_SUCCESS = 0,
    FLASH_DETECT_FAIL = -1,
    FLASH_TRANSFER_ERROR = -2,
    FLASH_TIMEOUT = -3,
    FLASH_INVALID_PARAMS = -4,
    FLASH_MISC_FAILURE = -5,
};

typedef struct spi_transfer {
    uint8_t opcode;
    uint8_t dummy_cycles;
    uint8_t dummy_cyles_pos;
    uint8_t *addr;
    uint8_t addr_len;
    uint8_t *tx_buf;
    uint8_t *rx_buf;
    uint32_t tx_len;
    uint32_t rx_len;
} spi_transfer_t;

//typedef struct extflashRuntimesInfo_s {
//    int8_t spi_line;
//} extflashRtInfo_t;

//typedef struct flash_device {
//    uint32_t (*spi_xfer)(const uint8_t *tx_buf, size_t tx_len, uint8_t *rx_buf,
//        size_t rx_len);
//    void (*sleep)(uint32_t ms);
//    uint32_t flash_size;
//    uint16_t num_of_pages_per_block;
//    uint16_t num_of_blocks;
//    uint16_t page_size;
//    uint16_t block_size;
//    uint16_t num_ecc_bytes_per_page;

//    uint32_t id;
//} flash_device_t;
//extern flash_device_t flash_dev;

//typedef struct{
//    uint16_t Address_Row;
//    uint16_t Address_Column;
//} Ext_Address_t;

/*
id[31:16]: Manufacturer-ID;
id[15:6]: Device-ID;
*/
#define W25N01GV_MANUFACTURER_ID (0xEF << 0)           /*(MF7 - MF0) : EFh */
#define W25N01GV_DEVICE_ID ((0xAA << 8) | (0x21 << 0)) /* (ID15 - ID0) : AA21h */
#define W25N01GV_WHO_AM_I ((W25N01GV_MANUFACTURER_ID << 16) | (W25N01GV_DEVICE_ID << 0))


//extflashRtInfo_t get_extflashRtInfo(void);

//void ext_flash_spi_init(void);
//void ext_flash_spi_uninit(void);

//void spi_flash_init(void);
void spi_uninit();
//void ext_flash_powerdown(void);

//bool extflash_touch(void);

uint8_t extFlashBlockErase(uint32_t);

//uint8_t extFlashPageProgram(uint8_t *pBuffer, uint32_t RA, uint16_t CA, uint16_t NumByteToWrite);

uint8_t extFlashRead(uint32_t pageAddr, uint16_t ca, uint8_t *readData, uint16_t Datalen);

//void write_in_extflash(uint8_t *inbuf, int inlen);

//void ext_list_header_init(uint8_t flag);

bool extflash_init(void);

void extflash_uinit(void);

uint8_t ext_get_ecc(void);

uint8_t extflash_compare_block(uint32_t block);

bool extflash_chip_ok();

//uint8_t extflash_clear_datapage(uint16_t report_id);

//uint16_t extflash_get_reportcnt(void);

//void ext_new_report_addr(void);

//void ext_remove_reportids(uint16_t report_id);

//void ext_get_reportids(uint8_t *buff, uint16_t len);

//uint16_t sFLASH_NAND_ReadID(void);

#ifdef __cplusplus
}
#endif
#endif /* EXT_FLASH_H */