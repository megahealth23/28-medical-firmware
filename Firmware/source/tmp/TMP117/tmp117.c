/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file tmp117.c
* @author:li tianzheng
* @modifing:
* @date 2020.10.15
* @version
* @brief
*/

#include "TMP117\tmp117.h"

/*!
 * @brief This API resets and restarts the device.
 * All register values are overwritten with default parameters.
 *
 * @param[in] dev  : Structure instance of tmp117_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t tmp117_soft_reset(struct tmp117_dev* dev);

/*!
 * @brief This API sets the default configuration parameters of tmp117.
 *
 * @param[in] dev         : Structure instance of tmp117_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static void default_param_settg(struct tmp117_dev* dev);

/*!
 * @brief This API set the mode configuration.
 *
 * @param[in] dev         : Structure instance of tmp117_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_mode_conf(struct tmp117_dev* dev);


/**********************     ShutDown Mode (SD) Setting  ********************************/
#define SD_PARM_CNT (1)

static TMP117_addr_t sd_m_addrs[SD_PARM_CNT] = { 0x01 /*(1) Configuration*/ };

static TMP117_reg_t sd_m_vals[SD_PARM_CNT] = { (TMP117_SHUTDOWN_MODE) /*(1) Configuration*/ };

/**********************     One-Shot Mode (OS) Setting  ********************************/
#define OS_PARM_CNT (5)

static TMP117_addr_t os_m_addrs[OS_PARM_CNT] = { 0x01 /*(1) Configuration*/, 0x02 /*(2)THigh_Limit*/,
												0x03 /*(3)TLow_Limit*/, 0x04 /*(4)EEPROM_UL*/, 0x07 /*(5)Temp_Offset*/ };

static TMP117_reg_t os_m_vals[OS_PARM_CNT] = { (TMP117_ONE_SHOT_MODE | TMP117_CFG_POL_ACTIVE_LOW_SEL | TMP117_CFG_ALERT_PIN_STATUS_SEL | TMP117_CFG_AVG_8) /*(1) Configuration*/,
											  ((0x60 << 8) | (0x00 << 0)) /*(2)THigh_Limit*/,
											  ((0x80 << 8) | (0x00 << 0)) /*(3)TLow_Limit*/,
											  0x00 /*(4)EEPROM_UL*/, 0x00 /*(5)Temp_Offset*/ };

/*********************** User function definitions ****************************/

/*!
 *  @brief This API is the entry point for sensor.It performs
 *  the selection of I2C read mechanism according to the
 *  selected interface and reads the chip-id of tmp117 sensor.
 */
int8_t tmp117_init(struct tmp117_dev* dev)
{
	int8_t rslt;
	uint8_t chip_id[2];
	uint8_t try = 3;

	if ((NULL == dev) || (NULL == dev->read))
	{
		rslt = TMP117_E_NULL_PTR;
	}
	else
	{
		/* Assign chip id as zero */
		dev->chip_id = 0;

		while ((try --) && (TMP117_CHIP_ID != dev->chip_id))
		{
			/* Read chip_id */
			rslt = dev->read(dev->id, TMP117_DEVICE_ID_ADDR, chip_id, 2);
			dev->chip_id = ((chip_id[0] << 8) | chip_id[1]);
		}
		if ((TMP117_OK == rslt) && (TMP117_CHIP_ID == dev->chip_id))
		{
			/* Soft reset */
			rslt = tmp117_soft_reset(dev);
		}
		else
		{
			rslt = TMP117_E_DEV_NOT_FOUND;
		}
	}

	return rslt;
}

/*!
 * @brief This API configures the power mode, range and bandwidth
 * of sensor.
 */
int8_t tmp117_set_sens_conf(struct tmp117_dev* dev)
{
	int8_t rslt = TMP117_OK;

	/* Null-pointer check */
	if (NULL == dev)
	{
		rslt = TMP117_E_NULL_PTR;
	}
	else
	{
		switch (dev->dev_mode)
		{
		case TMP117_DEV_SHUTDOWN_MODE:
			dev->mode_cfg.cfg_cnt = SD_PARM_CNT;
			dev->mode_cfg.addrs = sd_m_addrs;
			dev->mode_cfg.regs = sd_m_vals;
			break;
		case TMP117_DEV_ONE_SHOT_MODE:
			dev->mode_cfg.cfg_cnt = OS_PARM_CNT;
			dev->mode_cfg.addrs = os_m_addrs;
			dev->mode_cfg.regs = os_m_vals;
			break;
		default:
			rslt = TMP117_E_COM_FAIL;
			break;
		}
		if( TMP117_OK == rslt)
			 rslt = set_mode_conf(dev);
	}

	return rslt;
}

/*!
 * @brief This API reads sensor data, stores it in
 * the tmp117_sensor_data structure pointer passed by the user.
 */
int8_t tmp117_get_sensor_data( bool local_convert, struct tmp117_dev* dev)
{
	int8_t rslt = TMP117_OK;
	uint8_t rxdata[2] = { 0 };

	/* Null-pointer check */
	if ((NULL == dev) || (NULL == dev->read) || (NULL == dev->sensor_data))
	{
		rslt = TMP117_E_NULL_PTR;
	}
	else
	{
		/* read Temperature sensor data */
		rslt = dev->read(dev->id, TMP117_CONFIGURATION_ADDR, &rxdata[0], 2);
		dev->sensor_data->temp_val = ((rxdata[0] << 8) | rxdata[1]) & 0xF000;

		/* read Temperature sensor data */
		rslt = dev->read(dev->id, TMP117_TEMP_RESULT_ADDR, &rxdata[0], 2);
		dev->sensor_data->temp_val = ((rxdata[0] << 8) | rxdata[1]);

		if(local_convert)
			dev->sensor_data->celcius = ((((rxdata[0] << 8) | rxdata[1])) * 0.0078125);
		else
			dev->sensor_data->celcius = 0.0;
	}

	return rslt;
}

/*!
 * @brief This API sets the default configuration parameters of therm & therm.
 * Also maintain the previous state of configurations.
 */
static void default_param_settg(struct tmp117_dev* dev)
{
	dev->mode_cfg.cfg_cnt = SD_PARM_CNT;
	dev->mode_cfg.addrs = sd_m_addrs;
	dev->mode_cfg.regs = sd_m_vals;
}

/*!
 * @brief This API set the mode configuration.
 */
static int8_t set_mode_conf(struct tmp117_dev* dev)
	{
	int8_t rslt;
	int8_t ite;
	uint8_t txdata[2] = { 0 };
	uint8_t init_ovtime = 5; // define times used to rewrite;

	/* Null-pointer check */
	if ((NULL == dev) || (NULL == dev->write))
	{
		rslt = TMP117_E_NULL_PTR;
	}
	else
	{
		do
		{
			for (ite = 0; ite < dev->mode_cfg.cfg_cnt; ite++)
			{
				txdata[0] = (dev->mode_cfg.regs[ite] & 0xFF00) >> 8;
				txdata[1] = (dev->mode_cfg.regs[ite] & 0x00FF) >> 0;
				rslt = dev->write(dev->id, dev->mode_cfg.addrs[ite], &txdata[0], 2);
				if (TMP117_OK != rslt)
					break;
			}
			
			if(dev->mode_cfg.cfg_cnt == ite )
				break;
			else
				init_ovtime--;
			
		} while (init_ovtime);

		if (0 == init_ovtime)
			rslt = TMP117_E_DEV_NOT_FOUND;
	}
//set_os_conf_end:
	return rslt;
}

/*!
 * @brief This API resets and restarts the device.
 * All register values are overwritten with default parameters.
 */
static int8_t tmp117_soft_reset(struct tmp117_dev* dev)
{
	int8_t rslt;
	uint8_t txdata[2] = { 0 };
//	uint16_t cmd_data = TMP117_SOFT_RESET_CMD;

	/* Null-pointer check */
	if ((NULL == dev) || (NULL == dev->write) || (NULL == dev->delay_ms))
	{
		rslt = TMP117_E_NULL_PTR;
	}
	else
	{
		/* Reset the device */
		txdata[0] = (TMP117_SOFT_RESET_CMD & 0xFF00) >> 8;
		txdata[1] = (TMP117_SOFT_RESET_CMD & 0x00FF) >> 0;
		rslt = dev->write(dev->id, TMP117_CONFIGURATION_ADDR, &txdata[0], 2);
		dev->delay_ms(TMP117_SOFT_RESET_DELAY_MS);

		if (TMP117_OK == rslt)
		{
			/* Update the default parameters */
			default_param_settg(dev);
		}
	}

	return rslt;
}
