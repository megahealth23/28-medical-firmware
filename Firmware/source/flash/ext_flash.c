#include "ext_flash.h"
//#include "gd5flash.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "nrfx_spim.h"
#include "ring_bsp.h"
#include "reports_def.h"

#define SPIFLASH_WRITEEN_CMD 0x6
#define SPIFLASH_READSR_CMD 0x5
#define SPIFLASH_READDATA_CMD 0x3

//volatile extflashRtInfo_t extflashRtInfo = {
//    .spi_line = 0,
//};

static const nrfx_spim_t ext_spi = NRFX_SPIM_INSTANCE(3); /**< SPI instance. */
static volatile bool ext_spi_xfer_done;                   /**< Flag used to indicate that SPI instance completed the transfer. */
static bool m_exflash_spi_status = false;                 /**< SPI bus status. */
static uint16_t extflash_chip_id = 0;
static bool extflash_m_pwr = false;
// static uint32_t get_validblocks(uint16_t *last_lutidx, bblut_t *last_bblut);

//flash_device_t flash_dev;

#define MAX_SPI_BUFFER_SIZE 2048

#define CS_L() nrf_gpio_pin_clear(EXT_FLASH_SPI_CS); // enalble
#define CS_H() nrf_gpio_pin_set(EXT_FLASH_SPI_CS);   // disable

/* FIXME Find a better way to handle this without global buf */
uint8_t spi_xfer_buf[MAX_SPI_BUFFER_SIZE];
uint8_t spi_rcv_buf[MAX_SPI_BUFFER_SIZE];

uint8_t ext_read_buf[EXT_PACKETSIZE] = {0};
/**
 * @brief SPI user event handler.
 * @param event
 */
static nrf_drv_spi_evt_handler_t spi_event_handler(nrfx_spim_evt_t const *p_event,
    void *p_context) {
    ext_spi_xfer_done = true;
    //    NRF_LOG_INFO("Transfer completed.");
}

//typedef struct{
//    uint16_t Address_Row;
//    uint16_t Address_Column;
//} Ext_Address_t;

//typedef struct{
//    uint8_t type;
//    Ext_Address_t Address_Data;
//    Ext_Address_t Address_List;
//} Ext_Address_para;

//typedef struct{
//    uint8_t type;
//    uint8_t delete_flag;
//    Ext_Address_t data_start_addr;
//} ext_file_sec_directory_t;
//typedef struct{
//    uint8_t type;
//    Ext_Address_t data_start_addr;
//} ext_file_directory_t;

//typedef struct{
//    ext_file_sec_directory_t ext_sec_directory;
//    ext_file_directory_t ext_hrv_directory;
//    ext_file_directory_t ext_acc_directory;
//} Ext_File_Directory_Para;
//Ext_File_Directory_Para ext_file_directory;

//Ext_Address_para ext_addr_sec;
//Ext_Address_para ext_addr_hrv;
//Ext_Address_para ext_addr_acc;

uint8_t ext_report_ids[32];
//typedef struct{
//    Ext_Report_Addr_t ext_sec_addr;
//    Ext_Report_Addr_t ext_hrv_addr;
//    Ext_Report_Addr_t ext_acc_addr;
//} Ext_read_report_t;
/*
ss_pin需要自己控制；
NRF_DRV_SPI_INSTANCE设置1，对应sdk_config.h中的SPI1;
irq_priority中断优先级一定要设置为3，默认配置是7，不然蓝牙协议栈起来后会没效果！

*/
#define USER_SPI_CONFIG_IRQ_PRIORITY_HIGH 3

#define USRE_NRF_DRV_SPI_FLSH_CONFIG                       \
    {                                                      \
        .sck_pin = EXT_FLASH_SPI_SCK /*SPI_SCK_PIN*/,      \
        .mosi_pin = EXT_FLASH_SPI_MOSI /*SPI_MOSI_PIN*/,   \
        .miso_pin = EXT_FLASH_SPI_MISO /*SPI_MISO_PIN*/,   \
        .ss_pin = NRF_DRV_SPI_PIN_NOT_USED,                \
        .irq_priority = USER_SPI_CONFIG_IRQ_PRIORITY_HIGH, \
        .orc = 0xFF,                                       \
        .frequency = NRF_SPI_FREQ_4M,                      \
        .mode = NRF_DRV_SPI_MODE_0,                        \
        .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,      \
    }

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(1); /**< SPI instance. */

void spi_flash_init(void) {
    nrf_drv_spi_config_t const spi_config = USRE_NRF_DRV_SPI_FLSH_CONFIG;
    if (m_exflash_spi_status)
        return;
    // spi_config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, 
      (nrf_drv_spi_evt_handler_t)spi_event_handler, (void *)NULL));

    m_exflash_spi_status = true;
//    NRF_LOG_INFO("SPI flash init...");
}

void spi_uninit() {
    if (!m_exflash_spi_status)
        return;

    nrf_drv_spi_uninit(&spi);
//    NRF_LOG_INFO("SPI flash uninit...");
    m_exflash_spi_status = false;
}

static uint8_t spi_flash_read_one_byte(void) {
    uint8_t len = 1;

    spi_xfer_buf[0] = 0xFF;
    ext_spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_xfer_buf, len, spi_rcv_buf, len));
    while (!ext_spi_xfer_done)
        ;
    return (spi_rcv_buf[0]);
}

static void spi_flash_read_bytes(uint8_t len) {
    memset(spi_xfer_buf, 0, MAX_SPI_BUFFER_SIZE);
    memset(spi_rcv_buf, 0, MAX_SPI_BUFFER_SIZE);
    ext_spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_xfer_buf, len, spi_rcv_buf, len));
    while (!ext_spi_xfer_done)
        ;
}

static uint8_t spi_flash_write_one_byte(uint8_t data) {
    uint8_t len = 1;
    ret_code_t ret_code;

    spi_xfer_buf[0] = data;
    ext_spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_xfer_buf, len, spi_rcv_buf, len));
    while (!ext_spi_xfer_done)
        ;
    return (spi_rcv_buf[0]);
}

static uint8_t spi_flash_write_bytes(uint8_t len) {
    //memset(spi_xfer_buf, 0, MAX_SPI_BUFFER_SIZE);
    memset(spi_rcv_buf, 0, MAX_SPI_BUFFER_SIZE);
    ext_spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_xfer_buf, len, spi_rcv_buf, len));
    while (!ext_spi_xfer_done)
        ;
}

void spi_flash_write_enable(void) {
    uint8_t rslt;
    CS_L();
    rslt = spi_flash_write_one_byte(SPIFLASH_WRITEEN_CMD);
    // if(rslt != 0)
    //    NRF_LOG_INFO("spi_flash_write_enable rslt=%d", rslt);
    CS_H();
}

/*****************************************************************************
** 文件名称：static uint8_t spi_flash_read_status_reg(void)
** 功    能：“读状态寄存器”命令，retValue为读到的数值
** 修改日志：
** 附    录：当CS拉低之后，把05H从DI引脚输入到Flash芯片，CLK上升沿，数据写入Flash，当Flash收到05H后，会把“状态寄存器”的值，从D0引脚输出，CLK下降沿输出。
******************************************************************************/

static uint8_t spi_flash_read_status_reg(void) {
    uint8_t retValue = 0;

    CS_L();
    spi_flash_write_one_byte(SPIFLASH_READSR_CMD);
    retValue = spi_flash_read_one_byte();
    CS_H();
    return retValue;
}

/*****************************************************************************
** 文件名称：uint8_t spi_flash_read_data(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t ReadBytesNum)
** 功    能：读Flash的某地址ReadAddr，读多大ReadByteNum，的数值。
** 修改日志：
******************************************************************************/
uint8_t spi_flash_read_data(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t ReadBytesNum) {
    uint8_t len;

    spi_xfer_buf[0] = SPIFLASH_READDATA_CMD;
    spi_xfer_buf[1] = (uint8_t)((ReadAddr & 0x00ff0000) >> 16);
    spi_xfer_buf[2] = (uint8_t)((ReadAddr & 0x0000ff00) >> 8);
    spi_xfer_buf[3] = (uint8_t)ReadAddr;

    len = ReadBytesNum + 4;

    CS_L();
    ext_spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_xfer_buf, len, spi_rcv_buf, len));
    while (!ext_spi_xfer_done)
        ;
    CS_H();

    memcpy(pBuffer, &spi_rcv_buf[4], ReadBytesNum);

    return NRF_SUCCESS;
}

void NandSetFeature(uint8_t Add, unsigned char uldata) // SPINaned GetFeatures
{
    CS_L();
    spi_flash_write_one_byte(sFLASH_CMD_Set_Feature);
    spi_flash_write_one_byte(Add);
    spi_flash_write_one_byte(uldata);
    CS_H();
}

void ext_flash_spi_init(void) {
    ret_code_t rslt;

    if (m_exflash_spi_status)
        return;

    // nrf_gpio_cfg_default(EXT_FLASH_SPI_SCK); //   WP && SCK
    spi_flash_init();

    nrf_gpio_cfg(
        EXT_FLASH_SPI_CS,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        GPIO_PIN_CNF_PULL_Pullup,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);
    // CS_H();
     nrf_gpio_pin_set(EXT_FLASH_SPI_CS);

    nrf_gpio_cfg(
        EXT_FLASH_SPI_SCK,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_cfg(
        EXT_FLASH_SPI_HOLD,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        GPIO_PIN_CNF_PULL_Pullup,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_cfg(
        EXT_FLASH_SPI_WP,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        GPIO_PIN_CNF_PULL_Pullup,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_pin_set(EXT_FLASH_SPI_HOLD);
    nrf_gpio_pin_set(EXT_FLASH_SPI_WP);

    NandSetFeature(0xB0, 0x10); //Feature  B0H: ECC_EN = 1;
	
    NandSetFeature(0xA0, 0x0);
}

//void ext_flash_spi_uninit(void) {

//    if (!m_exflash_spi_status)
//        return;

//    // nrfx_spim_uninit(&ext_spi);
//    //   memset((void *)&flash_dev, 0, sizeof(flash_dev));

//    nrf_gpio_cfg(
//        EXT_FLASH_SPI_SCK,
//        NRF_GPIO_PIN_DIR_OUTPUT,
//        NRF_GPIO_PIN_INPUT_DISCONNECT,
//        NRF_GPIO_PIN_NOPULL,
//        NRF_GPIO_PIN_S0S1,
//        NRF_GPIO_PIN_NOSENSE);
//    nrf_gpio_pin_set(EXT_FLASH_SPI_SCK);

//    nrf_gpio_cfg(
//        EXT_FLASH_SPI_WP,
//        NRF_GPIO_PIN_DIR_OUTPUT,
//        NRF_GPIO_PIN_INPUT_DISCONNECT,
//        NRF_GPIO_PIN_NOPULL,
//        NRF_GPIO_PIN_S0S1,
//        NRF_GPIO_PIN_NOSENSE);
//    nrf_gpio_pin_set(EXT_FLASH_SPI_WP);

//    nrf_gpio_cfg_default(EXT_FLASH_SPI_HOLD);

//    m_exflash_spi_status = false;
//}

uint8_t NandGetFeature(uint8_t Add) // SPINand GetFeatures
{
    uint8_t feature;
    CS_L();
    spi_flash_write_one_byte(sFLASH_CMD_Get_Feature);
    spi_flash_write_one_byte(Add);
    feature = spi_flash_write_one_byte(sFLASH_DUMMY_BYTE);
    CS_H();
    return feature;
}

uint8_t ext_get_ecc(void)
{
    uint8_t registerval1 = NandGetFeature(0xC0);
    uint8_t registerval2 = NandGetFeature(0xF0);
    uint8_t val = 0;

    uint8_t eccs1 = (registerval1 & 0x20) >> 5;
    uint8_t eccs0 = (registerval1 & 0x10) >> 4;
    uint8_t eccse1 = (registerval2 & 0x20) >> 5;
    uint8_t eccse0 = (registerval2 & 0x10) >> 5;
    NRF_LOG_INFO("ext_get_ecc 0xC0 = %d, 0xF0 = %d, ECCS1=%d,ECCS0=%d,ECCSE1=%d,ECCSE0=%d", registerval1,registerval2,eccs1,eccs0,eccse1,eccse0);
    if(eccs1 == 1)
    {
        if(eccs0 == 1) {
            val = 8;
        } else {
            val = 9;
        }
    } else {
        if(eccs0 == 0) {
            val = 0;
        } else {
            val = (eccs1 << 3) + (eccs0 << 2) + (eccse1 << 1) + eccse0;
        }
    }
    return val;
}

uint8_t Spi_Nand_Wait_Busy(uint8_t prgFlag) // Waiting
{
    unsigned char timeout = 0;
    uint8_t oip;
    oip = NandGetFeature(0xC0);
    if (prgFlag == 0) {
        while ((oip = NandGetFeature(0xC0) & 0x01) == 0x01) {
            nrf_delay_us(5);
            if (timeout++ > 1000) {
                return timeout;
            }
        }
    } else {
        while ((oip = NandGetFeature(0xC0) & 0x03) == 0x03) {
            nrf_delay_us(10);
            if (timeout++ > 1000) {
                return timeout;
            }
        }
        nrf_delay_us(200);
        while ((oip = NandGetFeature(0xC0) & 0x01) == 0x01) {
            nrf_delay_us(10);
            if (timeout++ > 1000) {
                return timeout;
            }
        }
        //        NRF_LOG_INFO("Spi_Nand_Wait_Busy Timeout=%d", timeout);
    }
    return timeout;
}

uint8_t sFLASH_NAND_BlockErase(uint32_t Blockaddr) {
    uint8_t timeout;
    /*!< Send write enable instruction */
    spi_flash_write_enable();
    /*!< Select the FLASH: Chip Select low */
    CS_L();
    /*!< Send Block Erase instruction */
    spi_flash_write_one_byte(sFLASH_CMD_NAND_BE);
    /*!< Send SectorAddr high nibble address byte */
    spi_flash_write_one_byte((Blockaddr & 0xFF0000) >> 16);
    /*!< Send SectorAddr medium nibble address byte */
    spi_flash_write_one_byte((Blockaddr & 0xFF00) >> 8);
    /*!< Send SectorAddr low nibble address byte */
    spi_flash_write_one_byte(Blockaddr & 0xFF);
    /*!< Deselect the FLASH: Chip Select high */
    CS_H();

    timeout = Spi_Nand_Wait_Busy(0);
    if (timeout != 0)
        NRF_LOG_INFO("sFLASH_NAND_BlockErase timeout=%d, addr:%d", timeout,Blockaddr);

    return NandGetFeature(0xC0);
}

uint8_t sFLASH_NAND_Execute(uint32_t WriteAddr) {
    uint8_t timeout;
    /*!< Select the FLASH: Chip Select low */
    CS_L();
    /*!< Send "Write to Memory " instruction */
    spi_flash_write_one_byte(sFLASH_CMD_NAND_W_Execute);
    /*!< Send ReadAddr high nibble address byte to read from */
    spi_flash_write_one_byte((WriteAddr & 0xFF0000) >> 16);
    /*!< Send WriteAddr medium nibble address byte to write to */
    spi_flash_write_one_byte((WriteAddr & 0xFF00) >> 8);
    /*!< Send WriteAddr low nibble address byte to write to */
    spi_flash_write_one_byte(WriteAddr & 0xFF);

    /*!< Deselect the FLASH: Chip Select high */
    CS_H();

    timeout = Spi_Nand_Wait_Busy(1); // pFlag =1, program flash.
    //if (timeout >1 ) //!= 0)
    //    NRF_LOG_INFO("sFLASH_NAND_Execute Spi_Nand_Wait_Busy=%d", timeout);    
    //	nrf_delay_ms(2);

    return NandGetFeature(0xC0);
}

void sFLASH_NAND_Pageprogram(uint8_t *pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite) {
    /*!< Select the FLASH: Chip Select low */
    CS_L();
    /*!< Send "Write to Memory " instruction */
    spi_flash_write_one_byte(sFLASH_CMD_NAND_PRLD);
    /*!< Send WriteAddr medium nibble address byte to write to */
    spi_flash_write_one_byte((WriteAddr & 0xFF00) >> 8);
    /*!< Send WriteAddr low nibble address byte to write to */
    spi_flash_write_one_byte(WriteAddr & 0xFF);

    //uint16_t last_cnt = NumByteToWrite;
    //while(last_cnt){
    //    uint8_t read_cnt = last_cnt > MAX_SPI_BUFFER_SIZE ? MAX_SPI_BUFFER_SIZE : last_cnt;
    //    memmove(spi_xfer_buf, pBuffer, read_cnt);
    //    spi_flash_write_bytes(read_cnt);
    //    pBuffer += read_cnt;
    //    last_cnt = last_cnt > MAX_SPI_BUFFER_SIZE ? last_cnt - MAX_SPI_BUFFER_SIZE : 0;
    //}
    /*!< while there is data to be written on the FLASH */
    while (NumByteToWrite--) {
        /*!< Send the current byte */
        spi_flash_write_one_byte(*pBuffer);
        /*!< Point on the next byte to be written */
        pBuffer++;
    }
    /*!< Deselect the FLASH: Chip Select high */
    CS_H();
}

uint8_t Nand_read_to_cache(uint32_t ReadAddr) // PageRead
{
    uint8_t rslt;
    uint8_t timeout;
    CS_L();
    spi_flash_write_one_byte(sFLASH_CMD_READ_TO_CACHE);    // PageRead to cache
    spi_flash_write_one_byte((uint8_t)((ReadAddr) >> 16)); // send the Page/Block Add
    spi_flash_write_one_byte((uint8_t)((ReadAddr) >> 8));
    spi_flash_write_one_byte((uint8_t)ReadAddr);
    CS_H();
    timeout = Spi_Nand_Wait_Busy(0);
    if (timeout >1 ) //!= 0)
        NRF_LOG_INFO("Nand_read_to_cache Spi_Nand_Wait_Busy=%d", timeout);
    rslt = NandGetFeature(0xC0);
    return rslt;
}

uint8_t extFlashBlockErase(uint32_t Blockaddr) {
    uint8_t rslt;
    rslt = sFLASH_NAND_BlockErase(Blockaddr);
    //	sFLASH_NAND_BlockErase
    return rslt;
}

/* Read From Cache for A/B version */
void Spi_Nand_Read_from_cache_b(uint8_t *pBuffer, uint16_t ReadAddr, uint16_t DataLen) // ReadCache
{
    CS_L();
    spi_flash_write_one_byte(sFLASH_CMD_READ_FROM_CACHE); // PageRead from cache
    spi_flash_write_one_byte((ReadAddr & 0xFF00) >> 8);   // send the Page/Block Add
    spi_flash_write_one_byte(ReadAddr & 0xFF);
    spi_flash_write_one_byte(sFLASH_DUMMY_BYTE);

    while (DataLen--) /*!< while there is data to be read */
    {
        /*!< Read a byte from the FLASH */
        *pBuffer = spi_flash_write_one_byte(sFLASH_DUMMY_BYTE);
        /*!< Point to the next location where the byte read will be saved */
        pBuffer++;
    }
    //uint16_t last_cnt = DataLen;
    //while(last_cnt){
    //    uint16_t read_cnt = last_cnt > MAX_SPI_BUFFER_SIZE ? MAX_SPI_BUFFER_SIZE : last_cnt;
    //    spi_flash_read_bytes(read_cnt);
    //    memmove(pBuffer+DataLen-last_cnt, spi_rcv_buf, read_cnt);
    //    last_cnt = last_cnt > MAX_SPI_BUFFER_SIZE ? last_cnt - MAX_SPI_BUFFER_SIZE : 0;
    //}
    CS_H();
}

uint8_t extFlashRead(uint32_t pageAddr, uint16_t ca, uint8_t *readData, uint16_t Datalen) {
    uint8_t rslt;
    rslt = Nand_read_to_cache(pageAddr);

    Spi_Nand_Read_from_cache_b(readData, ca, Datalen);
    return rslt;
}

uint8_t extFlashPageProgram(uint8_t *pBuffer, uint32_t RA, uint16_t CA, uint16_t NumByteToWrite) {
    uint8_t rslt;
    if(RA >= EXT_MAXPAGENUM)
    {
        return 0xee;
    }
    //    sFLASH_NAND_Pageprogram(pBuffer,  WriteAddr, NumByteToWrite);
    sFLASH_NAND_Pageprogram(pBuffer, CA, NumByteToWrite);

    /*!< Send write enable instruction */
    spi_flash_write_enable();

    rslt = sFLASH_NAND_Execute(RA);
    //	NRF_LOG_INFO("sFLASH_NAND_Execute rslt=%d", rslt);

    return rslt;
}
#if 0
void ext_flash_powerdown(void) {
    nrf_gpio_cfg(
        EXT_FLASH_SPI_CS,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_set(EXT_FLASH_SPI_CS);

    nrf_gpio_cfg(
        EXT_FLASH_SPI_SCK,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_set(EXT_FLASH_SPI_SCK);

    nrf_gpio_cfg(
        EXT_FLASH_SPI_WP,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_set(EXT_FLASH_SPI_WP);
}
#endif
#define sFLASH_CMD_NOR_RDID 0x9F

uint16_t sFLASH_NAND_ReadID(void) {
    uint8_t data1 = 0;
    uint8_t data2 = 0;
    uint8_t data3 = 0;
    uint8_t r, r1;
    // uint8_t data4 = 0; //    r1 = spi_flash_read_status_reg();
    uint16_t data;
    CS_L();
    r = spi_flash_write_one_byte(sFLASH_CMD_NOR_RDID);
    data1 = spi_flash_write_one_byte(0);
    data2 = spi_flash_write_one_byte(sFLASH_DUMMY_BYTE);
    data3 = spi_flash_write_one_byte(sFLASH_DUMMY_BYTE);
    CS_H();
    data = (data2 << 8) | data3;
    return data;
}

void sFLASH_NandReset(void) // NandFlash Reset
{
    uint8_t r;
    CS_L();
    r = spi_flash_write_one_byte(sFLASH_CMD_NOR_RESET);
    CS_H();
    NRF_LOG_INFO("sFLASH_CMD_NOR_RESET: %d", r);
}

/*
usage:
extflashRtInfo_t extflash_rtInfo = get_extflashRtInfo();
NRF_LOG_INFO("extflash: spi_line: %d\n", extflash_rtInfo.spi_line);
*/
//extflashRtInfo_t get_extflashRtInfo(void) {
//    return extflashRtInfo;
//}

/* see:
 *  1)  .\EEGdocs\Preventing inadvertent erase of bad block marker for QspiNAND Products.pdf
 *  2)  .\EEGdocs\Bad Block Management for QspiNAND flash.pdf
 *      lut_idx        LBA                PBA
 *          0:      |  (xxx)  |    ->     (1023)
 *          1:      |  (xxx)  |    ->     (1022)
 *          2:      |  (xxx)  |    ->     (1021)
 *          3:      |  (xxx)  |    ->     (1020)
 *          4:      |  (xxx)  |    ->     (1019)
 *          5:      |  (xxx)  |    ->     (1018)
 *                   ... ...
 *         20:      |  (xxx)  |    ->     (1004)
 */

#define GD5F1GM7UE_ID 0xC891
#define GD5F1GM7RE_ID 0xC881 /*1.8v*/
#if 0
bool extflash_touch(void) {
    uint16_t id;
    bool r = false;
    FLASH_H(); // Enable flash power.

    ext_flash_spi_init(); // includ spi_init();
    sFLASH_NandReset();
    nrf_delay_ms(5);
    id = sFLASH_NAND_ReadID();

    //FLASH_L(); // Disable flash power.
    //spi_uninit();

    NRF_LOG_INFO("NAND ID=%d", id);
    if (GD5F1GM7RE_ID == id)
        r = true;

    return r;
}
#endif
//uint8_t Spi_Nand_Write_Num(uint32_t block_num, uint32_t page_num, uint32_t page_add, uint8_t *pBuffer, uint16_t DataLen) {
//    unsigned long block_add;
//    block_add = ((block_num & 0x3ff) << 6) | (page_num & 0x3f);
//    sFLASH_NAND_Pageprogram(pBuffer, page_add, DataLen);
//    return sFLASH_NAND_Execute(block_add);
//}

void ext_flash_enable(void) {
  nrf_gpio_cfg(
              VFLASH_PIN,
              NRF_GPIO_PIN_DIR_OUTPUT,
              NRF_GPIO_PIN_INPUT_DISCONNECT,
              GPIO_PIN_CNF_PULL_Pullup,
              NRF_GPIO_PIN_S0S1,
              NRF_GPIO_PIN_NOSENSE);
  FLASH_H(); // Enable flash power. 
}

uint32_t spi_flash_read_id(void)
{
    uint32_t temp = 0, temp0 = 0, temp1 = 0, temp2 = 0;

    /* select the flash: chip select low */
    CS_L();

    /* send "RDID " instruction */
    spi_flash_write_one_byte(sFLASH_CMD_NOR_RDID);

    /* read a byte from the flash */
    temp0 = spi_flash_read_one_byte();

    /* read a byte from the flash */
    temp1 = spi_flash_read_one_byte();

    /* read a byte from the flash */
    temp2 = spi_flash_read_one_byte();

    /* deselect the flash: chip select high */
    CS_H();

    temp = (temp0 << 16) | (temp1 << 8) | temp2;

    return temp;
}

uint8_t ext_badblock_check(uint32_t RA)
{
    uint8_t crcval = 0xff;
    extFlashRead(RA, 2048, &crcval, 1);

    uint8_t ect_error_read = ext_get_ecc();
    if(ect_error_read > 8) {
        NRF_LOG_INFO("write EXT addr:%d Bit errors greater than ECC capability(8 bits) and not corrected!", RA);
    } else if (ect_error_read > 0) {
        NRF_LOG_INFO("write EXT addr:%d Bit errors %d were detected and corrected!", RA,ect_error_read);
    }
    if(crcval == 0 || ect_error_read > 8)
    {
        return 0;   //bad page
    }
    return 1;
}

//uint32_t Address_Column;
//uint32_t Address_Row;
#if 0
uint8_t sflash_buffer_write(uint8_t* pbuffer, Ext_Address_t* write_addr, uint16_t num_byte_to_write)
{
    
    if(write_addr->Address_Row >= EXT_MAXPAGENUM) {
      NRF_LOG_INFO("EXT FLASH is overflowed!!!\n\r");
      return 0xff;
    }

    uint8_t timeout = 0;
    uint8_t num_of_page = 0, num_of_single = 0, addr = 0, count = 0, temp = 0;

    addr          = write_addr % SPI_FLASH_PAGE_SIZE;
    count         = SPI_FLASH_PAGE_SIZE - addr;
    num_of_page   = num_byte_to_write / SPI_FLASH_PAGE_SIZE;
    num_of_single = num_byte_to_write % SPI_FLASH_PAGE_SIZE;

     /* write_addr is SPI_FLASH_PAGE_SIZE aligned  */
    if(0 == addr){
        /* num_byte_to_write < SPI_FLASH_PAGE_SIZE */
        if(0 == num_of_page)
            timeout = sFLASH_Pageprogram(pbuffer, write_addr, num_byte_to_write);
        /* num_byte_to_write > SPI_FLASH_PAGE_SIZE */
        else{
            while(num_of_page--){
                timeout = sFLASH_Pageprogram(pbuffer,write_addr,SPI_FLASH_PAGE_SIZE);
                write_addr += SPI_FLASH_PAGE_SIZE;
                pbuffer += SPI_FLASH_PAGE_SIZE;
            }
            if(num_of_single > 0) {
                timeout = sFLASH_Pageprogram(pbuffer,write_addr,num_of_single);
            }
        }
    }else{
        /* write_addr is not SPI_FLASH_PAGE_SIZE aligned  */
        if(0 == num_of_page){
            /* (num_byte_to_write + write_addr) > SPI_FLASH_PAGE_SIZE */
            if(num_of_single > count){
                temp = num_of_single - count;
                timeout = sFLASH_Pageprogram(pbuffer,write_addr,count);
                write_addr += count;
                pbuffer += count;
                timeout = sFLASH_Pageprogram(pbuffer,write_addr,temp);
            }else
                timeout = sFLASH_Pageprogram(pbuffer,write_addr,num_byte_to_write);
        }else{
            /* num_byte_to_write > SPI_FLASH_PAGE_SIZE */
            num_byte_to_write -= count;
            num_of_page = num_byte_to_write / SPI_FLASH_PAGE_SIZE;
            num_of_single = num_byte_to_write % SPI_FLASH_PAGE_SIZE;

            timeout = sFLASH_Pageprogram(pbuffer,write_addr, count);
            write_addr += count;
            pbuffer += count;

            while(num_of_page--){
                timeout = sFLASH_Pageprogram(pbuffer,write_addr,SPI_FLASH_PAGE_SIZE);
                write_addr += SPI_FLASH_PAGE_SIZE;
                pbuffer += SPI_FLASH_PAGE_SIZE;
            }

            if(0 != num_of_single)
                timeout = sFLASH_Pageprogram(pbuffer,write_addr,num_of_single);
        }
    }
    return timeout;
}
#endif
#if 0
void ext_writein_listheader()
{
    uint16_t addr_row = ext_addr_sec.Address_List.Address_Row;
    uint8_t rslt = extFlashPageProgram(&ext_file_directory.ext_sec_directory, addr_row, 0, sizeof(ext_file_sec_directory_t));
    ext_addr_sec.Address_List.Address_Column += sizeof(ext_file_sec_directory_t);

    rslt = extFlashPageProgram(&ext_file_directory.ext_hrv_directory, addr_row + 1, 0, sizeof(ext_file_directory_t));
    ext_addr_hrv.Address_List.Address_Column += sizeof(ext_file_directory_t);

    rslt = extFlashPageProgram(&ext_file_directory.ext_acc_directory, addr_row + 2, 0, sizeof(ext_file_directory_t));
    ext_addr_acc.Address_List.Address_Column += sizeof(ext_file_directory_t);
}
#endif

//uint8_t testbuf1[512] = {0};
uint32_t ext_flash_buffer_write(uint16_t id, uint32_t offset, uint8_t *inbuf, uint16_t num_byte_to_write, E_REPORT_DATA_TYPE data_type)
{
    uint8_t rslt = 0;
    uint16_t start_addr_row = (id - 1) * EXT_MAXROWNUM_FILE;
    uint16_t max_row = start_addr_row + EXT_MAXROWNUM_SEC;
    if(data_type == e_report_data_min_sec){
        if(offset >= EXT_MAXBYTECNT_SEC){
            NRF_LOG_INFO("report_id: %d ext write in sec overflowed! offset: %d", id, offset);
            return offset;
        }
    } else if(data_type == e_report_data_hrv){
        if(offset >= EXT_MAXBYTECNT_HRV){
            NRF_LOG_INFO("report_id: %d ext write in hrv overflowed! offset: %d", id, offset);
            return offset;
        }
        start_addr_row += EXT_MAXROWNUM_SEC;
        max_row = start_addr_row + EXT_MAXROWNUM_HRV;
    } 
    uint16_t write_addr_row = start_addr_row + offset / EXT_PAGESIZE;
    uint16_t write_addr_col = offset % EXT_PAGESIZE;
    
// check if is 1.page turning or 2.bad block or 3.erase block
    if(write_addr_col == 0)
    {
        while (write_addr_row < (max_row - 1))  //check bad page
        {
            if(ext_badblock_check(write_addr_row) == 1)
                break;
            else 
                NRF_LOG_INFO("the %d page is bad block!", write_addr_row);

            write_addr_row++;
            offset += EXT_PAGESIZE;
        } 
        //if(write_addr_row % EXT_PAGES_IN_BLOCK == 0)  //erase block
        //{
        //  extFlashBlockErase(write_addr_row);
        //}
    }

// write in rawdata
    memset(&ext_read_buf, 0, EXT_PACKETSIZE*sizeof(uint8_t));
    //if(data_type == e_report_data_min_sec && offset == 0){
    //    memcpy(ext_read_buf, inbuf, num_byte_to_write);
    //} else {
        *((uint16_t*)(&ext_read_buf[0])) = num_byte_to_write;
        memcpy(ext_read_buf + 2, inbuf, num_byte_to_write);
    //}
    if(write_addr_col + EXT_WRITEIN_LEN <= EXT_PAGESIZE)
    {
        rslt = extFlashPageProgram(ext_read_buf, write_addr_row, write_addr_col, EXT_WRITEIN_LEN);
    } else {
        NRF_LOG_INFO("ext_flash_buffer_write failed!!!");
    }

    offset += EXT_WRITEIN_LEN;

    if(data_type == e_report_data_min_sec)
    {
        NRF_LOG_INFO("SEC data ext write in %d data into id:%d, row:%d, colunm:%d, offset:%d\r\n", num_byte_to_write,id,write_addr_row,write_addr_col,offset);
    } else if(data_type == e_report_data_hrv) {
        NRF_LOG_INFO("HRV data ext write in %d data into id:%d, row:%d, colunm:%d, offset:%d\r\n", num_byte_to_write,id,write_addr_row,write_addr_col,offset);
    } 
    return offset;
}

bool extflash_init(void)
{
    if (extflash_m_pwr) { extflash_m_pwr = false; }
//start ext flash
    ext_flash_enable();
    nrf_delay_ms(10);

    ext_flash_spi_init(); // includ spi_init();
    nrf_delay_ms(10);

    extflash_chip_id = spi_flash_read_id();
    //module_ext_flash.chip_id = spi_flash_read_id();
    //if(extflash_chip_id == sFLASH_CHIP_ID){
        extflash_m_pwr = true;
    //}
    return extflash_m_pwr;
}

void extflash_uinit(void)
{
    if (!extflash_m_pwr)
        return;

    FLASH_L();
    spi_uninit();
    extflash_m_pwr = false;
    extflash_chip_id = 0;
}

uint8_t read_data[EXT_PACKETSIZE] = {0};
uint8_t write_data[EXT_PACKETSIZE] = {0};
uint8_t extflash_compare_page(uint32_t row)
{
    uint8_t bad_packet = 0;
    //extFlashBlockErase(row);
    //nrf_delay_ms(10);
    for (int i = 0; i < EXT_PACKETSIZE; i++) {
       write_data[i] = i;
    }
    memset(read_data, 0, EXT_PACKETSIZE);
    for(int i = 0; i < EXT_PAGESIZE; i += EXT_PACKETSIZE)
    {
        uint8_t rslt = extFlashPageProgram(write_data, row, i, EXT_PACKETSIZE);
        nrf_delay_ms(2);
        rslt = extFlashRead(row, i, read_data, EXT_PACKETSIZE);
        nrf_delay_ms(2);
        if (memcmp(read_data, write_data, EXT_PACKETSIZE) != 0) { // Write check
            bad_packet++;
        }
    }
    wdt_feed();
    return bad_packet;
}

uint8_t extflash_compare_block(uint32_t block)
{
    if(block == 0 || block > 1024)
    {
        return 0;
    }
    uint32_t start_row = (block - 1) * EXT_PAGES_IN_BLOCK;
    extFlashBlockErase(start_row);
    uint32_t bad_page = 0;
    for(int i = start_row; i < start_row + EXT_PAGES_IN_BLOCK; i++)
    {
        if((extflash_compare_page(i)) > 0)
        {
            bad_page++;
            NRF_LOG_INFO("extflash_compare_block row %d is bad page, total bad is %d\n\r",i,bad_page);
        }
    }
    return bad_page;
}

bool extflash_chip_ok()
{
    uint32_t flash_id = spi_flash_read_id();
    //NRF_LOG_INFO("\n\rThe Flash_ID:0x%X\n\r",flash_id);
    if(sFLASH_CHIP_ID == flash_id) {
        return true;
    }
    return false;
}

uint8_t extflash_clear_datapage(uint16_t report_id) 
{
    uint16_t addr_sp = (report_id - 1)*EXT_MAXROWNUM_FILE;
    
    for(int i = addr_sp; i < addr_sp + EXT_MAXROWNUM_FILE; i+=EXT_PAGES_IN_BLOCK)
    {
        extFlashBlockErase(i); // data delete
    }
}