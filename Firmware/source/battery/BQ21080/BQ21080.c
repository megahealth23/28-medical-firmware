#include "BQ21080\BQ21080.h"

#include "I2C.h"
#include "mTask.h"

#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "battery_def.h"

//  /*0h-STATO*/, /*1h-STAT1*/, /*2h-FLAGO*/, /*3h-VBATCTRL*/,
//  /*4h-ICHGCTRL*/, /*5h-CHARGECTRLO*/, /*6h-CHARGECTRL1*/, /*7h-ICCTRL*/,
//  /*8h-TMRILIM*/, /*9h-SHIPRST*/, /*Ah-SYSREG*/, /*Bh-TSCONTROL*/,  /*Ch-MASKID*/,

#if  BQ21080_ENABLE_CHARGE_MODE
/**********************     CHARGE/ADAPTER MODE Setting  ********************************/
#define 	CHARGE_ADAPTER_PARM_CNT       			(0x08)
static BQ21080_addr_t charge_adapter_m_addrs[CHARGE_ADAPTER_PARM_CNT] = { 
    BQ21080_VBAT_CTRL_ADDR/*3h-VBATCTRL*/,
    BQ21080_ICHG_CTRL_ADDR/*4h-ICHGCTRL*/,
    BQ21080_CHARGECTRL0_ADDR/*5h-CHARGECTRLO*/, 
    BQ21080_CHARGECTRL1_ADDR/*6h-CHARGECTRL1*/, 
    BQ21080_IC_CTRL_ADDR/*7h-ICCTRL*/,
    BQ21080_TMR_ILIM_ADDR/*8h-TMRILIM*/,
    BQ21080_SYS_REG_ADDR/*Ah-SYSREG*/, 
    BQ21080_MASK_ID_ADDR/*Ch-MASKID*/,
};

static BQ21080_Reg_t charge_adapter_m_vals[CHARGE_ADAPTER_PARM_CNT]  = {
    SET_VBATREG(4300)/*3h-VBATCTRL*/,
    SET_ICHG(14) /*4h-ICHGCTRL*/, 
    (0x00<<BQ21080_IPRECHG_POS) | (0x02<<BQ21080_ITERM_1_0_POS) | (0x03<<BQ21080_VINDPM_1_0_POS) | (0x03<<BQ21080_THERM_REG_1_0_POS)/*5h-CHARGECTRLO*/, 
    (0x03<<BQ21080_IBAT_OCP_1_0_POS) | (0x03<<BQ21080_BUVLO_2_0_POS) | (0x00<<BQ21080_CHG_STATUS_INT_POS) | (0x03)/*6h-CHARGECTRL1*/, 
    (0x00<<BQ21080_TS_EN_POS)|(0x00<<BQ21080_VLOWV_SEL_POS)|(0x00<<BQ21080_VRCH_0_POS)|(0x00<<BQ21080_2XTMR_EN_POS)|(0x01<<BQ21080_SAFETY_TIMER_1_0_POS)|(0x01<<BQ21080_WATCHDOG_SEL_1_0_POS)/*7h-ICCTRL*/,
    (0x01<<BQ21080_MR_LPRESS_1_0_POS)|(0x00<<BQ21080_MR_RESET_VIN_POS)|(0x02<<BQ21080_AUTOWAKE_1_0_POS)|(0x02<<BQ21080_ILIM_2_0_POS)/*8h-TMRILIM*/, 
    (0x02<<BQ21080_SYS_REG_CTRL_2_0_POS)|(0x00<<BQ21080_SYS_MODE_1_0_POS)|(0x01<<BQ21080_WATCHDOG_15S_ENABLE_POS)|(0x01<<BQ21080_VDPPM_DIS_POS)/*Ah-SYSREG*/, 
    (0x01<<BQ21080_TS_INT_POS)|(0x01<<BQ21080_TREG_INT_POS)|(0x01<<BQ21080_BAT_INT_POS)|(0x01<<BQ21080_PG_INT_POS)/*Ch-MASKID*/
};
#endif

#if  BQ21080_ENABLE_BATTERY_MODE
/**********************     BATTERY MODE Setting  ********************************/
#define 	BATTERY_PARM_CNT       			(0x00)
static BQ21080_addr_t battery_m_addrs[BATTERY_PARM_CNT] = {
//  /*0h-STATO*/, /*1h-STAT1*/, /*2h-FLAGO*/, /*3h-VBATCTRL*/,
//  /*4h-ICHGCTRL*/, /*5h-CHARGECTRLO*/, /*6h-CHARGECTRL1*/, /*7h-ICCTRL*/,
//  /*8h-TMRILIM*/, /*9h-SHIPRST*/, /*Ah-SYSREG*/, /*Bh-TSCONTROL*/,  /*Ch-MASKID*/,
};

static BQ21080_Reg_t battery_m_vals[BATTERY_PARM_CNT]  = {
//  /*0h-STATO*/, /*1h-STAT1*/, /*2h-FLAGO*/, /*3h-VBATCTRL*/,
//  /*4h-ICHGCTRL*/, /*5h-CHARGECTRLO*/, /*6h-CHARGECTRL1*/, /*7h-ICCTRL*/,
//  /*8h-TMRILIM*/, /*9h-SHIPRST*/, /*Ah-SYSREG*/, /*Bh-TSCONTROL*/,  /*Ch-MASKID*/,
};
#endif

#if BQ21080_ENABLE_SHIP_MODE
/**********************     SHIP MODE Setting  ********************************/
#define 	SHIP_PARM_CNT       			(0x01)
static BQ21080_addr_t ship_m_addrs[SHIP_PARM_CNT] = {
    BQ21080_SHIP_RST_ADDR/*9h-SHIPRST*/
};

static BQ21080_Reg_t ship_m_vals[SHIP_PARM_CNT]  = {
    (0x01<<BQ21080_EN_RST_SHIP_1_0_POS)/*9h-SHIPRST*/
};
#endif

#if  BQ21080_ENABLE_SHUTDOWN_MODE
/**********************     SHUTDOWN MODE Setting  ********************************/
#define 	SHUTDOWN_PARM_CNT       		(0x00)
static BQ21080_addr_t shutdown_m_addrs[SHUTDOWN_PARM_CNT] = { 
//  /*0h-STATO*/, /*1h-STAT1*/, /*2h-FLAGO*/, /*3h-VBATCTRL*/,
//  /*4h-ICHGCTRL*/, /*5h-CHARGECTRLO*/, /*6h-CHARGECTRL1*/, /*7h-ICCTRL*/,
//  /*8h-TMRILIM*/, /*9h-SHIPRST*/, /*Ah-SYSREG*/, /*Bh-TSCONTROL*/,  /*Ch-MASKID*/,
};

static BQ21080_Reg_t shutdown_m_vals[SHUTDOWN_PARM_CNT]  = {
//  /*0h-STATO*/, /*1h-STAT1*/, /*2h-FLAGO*/, /*3h-VBATCTRL*/,
//  /*4h-ICHGCTRL*/, /*5h-CHARGECTRLO*/, /*6h-CHARGECTRL1*/, /*7h-ICCTRL*/,
//  /*8h-TMRILIM*/, /*9h-SHIPRST*/, /*Ah-SYSREG*/, /*Bh-TSCONTROL*/,  /*Ch-MASKID*/,
};
#endif

/*!
 * @brief This API reads the data from the given register address
 * of sensor.
 */
int8_t bq21080_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct charg_chip_dev *dev)
{
    /* Variable to define temporary buffer */
    uint8_t rx_data[1];
    int8_t rslt = BATTERY_OK;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->I2C_read == NULL) )
    {
        rslt = BATTERY_E_NULL_PTR;
    }
    else if (len == 0)
    {
        rslt = BATTERY_READ_WRITE_LENGHT_INVALID;
    }
    else
    {

        rslt = dev->I2C_read(dev->id, reg_addr, rx_data, len);

        if (rslt == BATTERY_OK)  {
          memcpy(data, rx_data, len);
        }
        else  {
          rslt = BATTERY_E_COM_FAIL;
        }
    }

    return rslt;
}


/*!
 * @brief This API writes the given data to the register address
 * of sensor.
 */
int8_t bq21080_set_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct charg_chip_dev *dev)
{
    int8_t rslt = BATTERY_OK;
    uint8_t count = 0;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->I2C_write == NULL))
    {
        rslt = BATTERY_E_NULL_PTR;
    }
    else if (len == 0)
    {
        rslt = BATTERY_READ_WRITE_LENGHT_INVALID;
    }
    else
    {
        rslt = dev->I2C_write(dev->id, reg_addr, data, len);

        /* Kindly refer bmi160 data sheet section 3.2.4 */
        dev->delay_ms(1);

        if (rslt != BATTERY_OK)
        {
            rslt = BATTERY_E_COM_FAIL;
        }
    }

    return rslt;
}


int8_t bq21080_software_reset(struct charg_chip_dev *dev)
{
    int8_t rslt = BATTERY_OK;
    uint8_t tx_data = BQ21080_REG_RST_MASK;

    if (rslt == BATTERY_OK)
    {     
/*  
*      When a software reset is issued either through a watchdog action configurable through the WATCHDOG_SEL 
*  bits or register reset configurable through the REG_RST bit, the device will reset all of the registers to the 
*  defaults. Any bits loaded through OTP memory are also loaded. If the device was waiting to go to shipmode (all 
*  conditions for entering ship are fulfilled except adapter removal), a hardware or software reset will cancel the 
*  pending shipmode request. If the shipmode request was written through I2C, the host can cancel the ship entry 
*  by clearing the bit before shipmode entry has happened.
*/
        rslt = bq21080_set_regs(BQ21080_SHIP_RST_ADDR, &tx_data , 1, dev);
        dev->dev_mode = RING_CHGCHIP_NONE_MODE;
        dev->delay_ms(500);
    }

    return rslt;
}


int8_t bq21080_hardware_reset(struct charg_chip_dev *dev)
{
    int8_t rslt = BATTERY_OK;
    uint8_t tx_data = BQ21080_EN_RST_SHIP_1_0_MASK;

    if (rslt == BATTERY_OK)
    {     
/*  
*      When a software reset is issued either through a watchdog action configurable through the WATCHDOG_SEL 
*  bits or register reset configurable through the REG_RST bit, the device will reset all of the registers to the 
*  defaults. Any bits loaded through OTP memory are also loaded. If the device was waiting to go to shipmode (all 
*  conditions for entering ship are fulfilled except adapter removal), a hardware or software reset will cancel the 
*  pending shipmode request. If the shipmode request was written through I2C, the host can cancel the ship entry 
*  by clearing the bit before shipmode entry has happened.
*/
        rslt = bq21080_set_regs(BQ21080_SHIP_RST_ADDR, &tx_data , 1, dev);
        dev->delay_ms(1);
    }

    return rslt;
}

int8_t bq21080_init(struct charg_chip_dev *dev)
{
    int8_t rslt = BATTERY_OK;
    uint8_t rx_data = 0xFF;
    uint16_t try_cnt = 150*12;
    dev->id = BQ21080_I2C_ADDR;

    /* Assign chip id as zero */
    dev->chip_id = 0xFF;

    //bq21080_software_reset(dev);
    while ((try_cnt--) && ((dev->chip_id & 0x0f) != BQ21080_CHIP_ID))
    {
        /* Read chip_id */
        rslt = bq21080_get_regs(BQ21080_MASK_ID_ADDR, &rx_data , 1, dev);
        dev->chip_id = rx_data; 
        printf("bq21080_init chip id : 0x%x, try cnt is :%d\r\n", dev->chip_id, try_cnt);
        dev->delay_ms(100);
    }
    if ((rslt == BATTERY_OK) && ((dev->chip_id & 0x0f) == BQ21080_CHIP_ID)) {
        rslt = BATTERY_OK;
    }  else  {
        return BATTERY_E_DEV_NOT_FOUND;
    }

    rslt = bq21080_get_cur_charg_status(dev, &dev->charg_status);

    return rslt;
}


/*!
 * @brief This API configures the power mode, range and bandwidth
 * of sensor.
 */
int8_t bq21080_set_mode(uint8_t mode, struct charg_chip_dev *dev)
{
    int8_t rslt = BATTERY_OK;
    struct bq21080_cfg cfg = {0};

    uint8_t ite = 0;
    uint8_t txdata[1] = { 0 };
			
	switch(mode)
	{
    case RING_CHGCHIP_NORMAL:
        cfg.cfg_cnt = CHARGE_ADAPTER_PARM_CNT;
        cfg.addrs = charge_adapter_m_addrs;
        cfg.regs = charge_adapter_m_vals;
    break;		
    case RING_CHGCHIP_TRANSPORT:
        cfg.cfg_cnt = SHIP_PARM_CNT;
        cfg.addrs = ship_m_addrs;
        cfg.regs = ship_m_vals;
    break;
    default:
    rslt = BATTERY_E_COM_FAIL;
    break;
	}

    if((rslt == BATTERY_OK) && (cfg.cfg_cnt>0) && (NULL != cfg.addrs) && (NULL != cfg.regs))  {
        for(ite = 0; ite < cfg.cfg_cnt; ite++)
        {
            txdata[0] = (uint8_t)cfg.regs[ite];
            rslt = dev->I2C_write(dev->id, cfg.addrs[ite], &txdata[0], 1);
        }
	}  
    
    if(rslt == BATTERY_OK)
        dev->dev_mode = mode;

    return rslt;
}

void bq21080_show_status(const struct charg_chip_dev *dev){
    uint8_t s = 0;
    uint8_t rslt = bq21080_get_regs(BQ21080_STAT0_ADDR, &s, 1, dev);
    if (rslt != BATTERY_OK)  {
        NRF_LOG_ERROR("bq21080_get_regs stat0 failed!");
    }else
        NRF_LOG_ERROR("bq21080_get_regs stat0: 0x%02X", s);

    s = 0;
    rslt = bq21080_get_regs(BQ21080_STAT1_ADDR, &s, 1, dev);
    if (rslt != BATTERY_OK)  {
        NRF_LOG_ERROR("bq21080_get_regs stat1 failed!");
    }else
        NRF_LOG_ERROR("bq21080_get_regs stat1: 0x%02X", s);

    s = 0;
    rslt = bq21080_get_regs(BQ21080_FLAG0_ADDR, &s, 1, dev);
    if (rslt != BATTERY_OK)  {
        NRF_LOG_ERROR("bq21080_get_regs flag0 failed!");
    }else
        NRF_LOG_ERROR("bq21080_get_regs flag0: 0x%02X", s);

    s = 0;
    rslt = bq21080_get_regs(BQ21080_ICHG_CTRL_ADDR, &s, 1, dev);
    if (rslt != BATTERY_OK)  {
        NRF_LOG_ERROR("bq21080_get_regs IChg failed!");
    }else
        NRF_LOG_ERROR("bq21080_get_regs IChg: 0x%02X", s);
}

void bq21080_get_status(const struct charg_chip_dev *dev, uint8_t *s0, uint8_t *s1, uint8_t *f0, uint8_t *i1){
    uint8_t rslt = bq21080_get_regs(BQ21080_STAT0_ADDR, s0, 1, dev);
    if (rslt != BATTERY_OK)  {
        NRF_LOG_ERROR("bq21080_get_regs stat0 failed!");
    }else
        NRF_LOG_ERROR("bq21080_get_regs stat0: 0x%02X", *s0);

    rslt = bq21080_get_regs(BQ21080_STAT1_ADDR, s1, 1, dev);
    if (rslt != BATTERY_OK)  {
        NRF_LOG_ERROR("bq21080_get_regs stat1 failed!");
    }else
        NRF_LOG_ERROR("bq21080_get_regs stat1: 0x%02X", *s1);

    rslt = bq21080_get_regs(BQ21080_FLAG0_ADDR, f0, 1, dev);
    if (rslt != BATTERY_OK)  {
        NRF_LOG_ERROR("bq21080_get_regs flag0 failed!");
    }else
        NRF_LOG_ERROR("bq21080_get_regs flag0: 0x%02X", *f0);

    rslt = bq21080_get_regs(BQ21080_ICHG_CTRL_ADDR, i1, 1, dev);
    if (rslt != BATTERY_OK)  {
        NRF_LOG_ERROR("bq21080_get_regs IChg failed!");
    }else
        NRF_LOG_ERROR("bq21080_get_regs IChg: 0x%02X", *i1);
}

int8_t bq21080_get_statflag(bq21080_statflag_t *p_statflag, const struct charg_chip_dev *dev)
{
    int8_t rslt = BATTERY_OK;

    if (NULL == p_statflag)  {
        rslt = BATTERY_E_NULL_PTR;
        goto bq21080_get_statflag_end;
    }

    memset(p_statflag, 0x00, sizeof(bq21080_statflag_t));

    rslt = bq21080_get_regs(BQ21080_STAT0_ADDR, &(p_statflag->stat0_val), 1, dev);
    if (rslt != BATTERY_OK)  {
        goto bq21080_get_statflag_end;
    }

    rslt = bq21080_get_regs(BQ21080_STAT1_ADDR, &(p_statflag->stat1_val), 1, dev);
    if (rslt != BATTERY_OK)  {
        goto bq21080_get_statflag_end;
    }

    rslt = bq21080_get_regs(BQ21080_FLAG0_ADDR, &(p_statflag->flag0_val), 1, dev);
    if (rslt != BATTERY_OK)  {
        goto bq21080_get_statflag_end;
    }

bq21080_get_statflag_end:
    return rslt; 
}

uint8_t bq_stat0 = 0;
int8_t bq21080_get_cur_charg_status(struct charg_chip_dev *dev, uint8_t *status){
    int8_t rslt = BATTERY_OK;

    bq21080_stat0_t stat0 = {0};
    rslt = bq21080_get_regs(BQ21080_STAT0_ADDR, &stat0, 1, dev);
    if (rslt != BATTERY_OK)  {
        return rslt;
    }

    memcpy(&bq_stat0, &stat0, sizeof(bq_stat0));
    if(stat0.vin_pgood_stat == 0)
        *status = e_no_charg;
    else{
        switch(stat0.chg_stat){
        case BQ21080_CHG_STAT_NOT_CHGING:
            *status = e_no_charg;
        break;
        case BQ21080_CHG_STAT_CONST_CURRENT:
            *status = e_charging;
        break;
        case BQ21080_CHG_STAT_CONST_VOLTAGE:
            *status = e_charging;
        break;
        case BQ21080_CHG_STAT_CHGING_DONE:
            *status = e_charg_done;
        break;
        default :
            return BATTERY_E_OUT_OF_RANGE;
        }
    }
    
    NRF_LOG_ERROR("bq21080_get_cur_charg_status stat0: 0x%02X", bq_stat0);
    return rslt;
}

void bq21080_wdt_handle(struct charg_chip_dev *dev){
    bq21080_stat0_t stat0 = {0};
    bq21080_get_regs(BQ21080_STAT0_ADDR, &stat0, 1, dev);
}

bool bq21080_ok(struct charg_chip_dev *dev){
    if  (BQ21080_CHIP_ID == (dev->chip_id & 0x0f)){
        return true;
    } else {
        return false;
    }
}

int8_t bq21080_set_charge_intensity(uint8_t intensity, struct charg_chip_dev *dev){
    int8_t rslt = BATTERY_OK;
    uint8_t tx_data = SET_ICHG(intensity);

    if (rslt == BATTERY_OK)
    {     
        rslt = bq21080_set_regs(BQ21080_ICHG_CTRL_ADDR, &tx_data , 1, dev);
        dev->delay_ms(1);
    }

    return rslt;
}

int8_t bq21080_set_charge_enable(uint8_t enable, struct charg_chip_dev *dev){
    int8_t rslt = BATTERY_OK;
    uint8_t tx_data = 0;

    rslt = bq21080_get_regs(BQ21080_ICHG_CTRL_ADDR, &tx_data , 1, dev);
    if(enable != 0){
        tx_data |= 0x80;
    }else{
        tx_data &= 0x7F;
    }

    NRF_LOG_INFO("set_charge_enable write 0x%02x", tx_data);
    if (rslt == BATTERY_OK)
    {     
        rslt = bq21080_set_regs(BQ21080_ICHG_CTRL_ADDR, &tx_data , 1, dev);
        dev->delay_ms(1);
    }

    uint8_t rx = 0;
    rslt = bq21080_get_regs(BQ21080_ICHG_CTRL_ADDR, &rx , 1, dev);
    NRF_LOG_INFO("set_charge_enable read 0x%02x", rx);

    return rslt;
}
