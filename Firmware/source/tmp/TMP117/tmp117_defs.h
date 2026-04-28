/*!
 * @defgroup tmp117_defs
 * @brief
 * @{*/

#ifndef _TMP117_DEFS_H_
#define _TMP117_DEFS_H_

/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
extern "C" {
#endif


/*************************** C types headers *****************************/
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/*************************** Common macros   *****************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)   S8_C(x)
#define UINT8_C(x)  U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)  S16_C(x)
#define UINT16_C(x) U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)  S32_C(x)
#define UINT32_C(x) U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)  S64_C(x)
#define UINT64_C(x) U64_C(x)
#endif

/**@}*/
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *) 0)
#endif
#endif


/** TMP117 Register map */
#define     TMP117_TEMP_RESULT_ADDR    									 (0x00) 
#define     TMP117_CONFIGURATION_ADDR  									 (0x01)
#define     TMP117_THIGH_LIMIT_ADDR  										 (0x02)
#define     TMP117_TLOW_LIMIT_ADDR    									 (0x03)
#define     TMP117_EEPROM_UL_ADDR     									 (0x04)
#define     TMP117_EEPROM1_ADDR      										 (0x05)
#define     TMP117_EEPROM2_ADDR      										 (0x06)
#define     TMP117_TEMP_OFFSET_ADDR   									 (0x07)
#define     TMP117_EEPROM3_ADDR      										 (0x08)
#define     TMP117_DEVICE_ID_ADDR												 (0x0F)

/** Soft reset command */
#define 		TMP117_SOFT_RESET_CMD               				 (0x0002)
#define 		TMP117_SOFT_RESET_DELAY_MS          				 (2)

#define	 		TMP117_CFG_DR_ALERT_PIN_MASK 								 ((0x0001 << 2))   /* ALERT pin select */
#define	 		TMP117_CFG_DR_PIN_STATUS_SEL 								 ((0x0001 << 2))   /* ALERT pin reflects the status of the data ready flag */
#define	 		TMP117_CFG_ALERT_PIN_STATUS_SEL 						 ((0x0000 << 2))   /* ALERT pin reflects the status of the alert flags */

#define	 		TMP117_CFG_POL_MASK 												 ((0x0001 << 3))   /* ALERT pin polarity bit */
#define	 		TMP117_CFG_POL_ACTIVE_HIGH_SEL 							 ((0x0001 << 3))   /* Active high */
#define	 		TMP117_CFG_POL_ACTIVE_LOW_SEL 							 ((0x0000 << 3))   /* Active low */

#define	 		TMP117_CFG_AVG_MASK 												 ((0x0003 << 5))   /* Conversion averaging modes. */
#define	 		TMP117_CFG_AVG_NONE 												 ((0x0000 << 5))   /* No averaging */
#define	 		TMP117_CFG_AVG_8 														 ((0x0001 << 5))   /* 8 Averaged conversions */
#define	 		TMP117_CFG_AVG_32 													 ((0x0002 << 5))   /* 32 averaged conversions */
#define	 		TMP117_CFG_AVG_64 													 ((0x0003 << 5))   /* 64 averaged conversions */

#define	 		TMP117_CFG_CONV_MASK 												 ((0x0007 << 7))   /* Conversion Cycle Time in CC Mode */
/*When the 8 Averaged conversions is setting*/
#define	 		TMP117_CFG_CONV_AVG8_125_MS_SEL							 ((0x0001 << 7))   /* 125 ms */
#define	 		TMP117_CFG_CONV_AVG8_250_MS_SEL							 ((0x0002 << 7))   /* 250 ms */
#define	 		TMP117_CFG_CONV_AVG8_500_MS_SEL							 ((0x0003 << 7))   /* 500 ms */
#define	 		TMP117_CFG_CONV_AVG8_1000_MS_SEL						 ((0x0004 << 7))   /* 1 s */
#define	 		TMP117_CFG_CONV_AVG8_4000_MS_SEL						 ((0x0005 << 7))   /* 4 s */
#define	 		TMP117_CFG_CONV_AVG8_8000_MS_SEL	  				 ((0x0006 << 7))   /* 8 s */
#define	 		TMP117_CFG_CONV_AVG8_16000_MS_SEL						 ((0x0007 << 7))   /* 16 s */

/** Error code definitions */
#define 		TMP117_OK                            					 (0)
#define 		TMP117_E_NULL_PTR                    					 (-1)
#define 		TMP117_E_COM_FAIL                  	  				 (-2)
#define 		TMP117_E_DEV_NOT_FOUND               					 (-3)
#define 		TMP117_READ_WRITE_LENGHT_INVALID     					 (-12)

/*
#define TMP117_E_OUT_OF_RANGE                INT8_C(-4)
#define TMP117_E_INVALID_INPUT               INT8_C(-5)
#define TMP117_E_ACCEL_ODR_BW_INVALID        INT8_C(-6)
#define TMP117_E_GYRO_ODR_BW_INVALID         INT8_C(-7)
#define TMP117_E_LWP_PRE_FLTR_INT_INVALID    INT8_C(-8)
#define TMP117_E_LWP_PRE_FLTR_INVALID        INT8_C(-9)
#define TMP117_E_AUX_NOT_FOUND               INT8_C(-10)
#define TMP117_FOC_FAILURE                   INT8_C(-11)
#define TMP117_READ_WRITE_LENGHT_INVALID     INT8_C(-12)
*/

/*****************************************************************************/
/* type definitions */
typedef int8_t (*tmp117_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*tmp117_delay_fptr_t)(uint32_t period);

typedef uint8_t TMP117_addr_t;
typedef uint16_t TMP117_reg_t;

/*************************** Data structures *********************************/

/*!
 * @brief tmp117 sensor configuration structure
 */
struct tmp117_cfg
{
		 uint8_t cfg_cnt;

		 TMP117_addr_t *addrs;

    /*! Configuration register value */
/**  Functional Modes in Reg mode settings */
#define 	TMP117_CONVERSION_MODE_MASK 			     				 ((0x0003 << 10))   /* ShutdownMode(SD) */
#define 	TMP117_ALERT_THERM_MODE_MASK 			     				 ((0x0003 << 4))   /* ShutdownMode(SD) */
#define 	TMP117_CONTINUOUS_CONVERSION_MODE 		 				 ((0x0000 << 10))   /* Continuous Conversion Mode */
#define 	TMP117_SHUTDOWN_MODE 			             				 ((0x0001 << 10))   /* ShutdownMode(SD) */
#define 	TMP117_ONE_SHOT_MODE 			             				 ((0x0003 << 10))   /* One-ShotMode(OS) */
#define 	TMP117_ALERT_MODE 			             	 				 ((0x0000 << 4))   /* Alert Mode Mode */
#define 	TMP117_THERM_MODE 			             	 				 ((0x0001 << 4))   /* Therm Mode Mode */
			TMP117_reg_t *regs;

};

/*!
 * @brief bmi160 sensor data structure which comprises of accel data
 */
struct tmp117_sensor_data
{
	/*! flag. */
#define	 	TMP117_DEV_STATUS_HIGH_ALERT_FLAG									 ((0x0001 << 15))   /*  */
#define	 	TMP117_DEV_STATUS_LOW_ALERT_FLAG									 ((0x0001 << 14))   /*  */
#define	 	TMP117_DEV_STATUS_DATA_READY_FLAG									 ((0x0001 << 13))   /*  */
#define	 	TMP117_DEV_STATUS_EEPROM_BUSY_FLAG								 ((0x0001 << 12))   /*  */
		uint16_t dev_status;
	
    /*! Temperature result. */
		uint16_t temp_val;

    float celcius;

};

struct tmp117_dev
{
	
/** TMP117 chip identifier,  */
#define 	 TMP117_CHIP_ID                  							 		(0x0117)
    /*! Chip Id */
    uint16_t chip_id;

    /*! Device Id */
    uint8_t id;

/**  Device Modes settings */
#define	 	TMP117_DEV_CONTINUOUS_CONVERSION_MODE							 ((0x0001 << 0))   /*  */
#define	 	TMP117_DEV_SHUTDOWN_MODE													 ((0x0001 << 1))   /*  */
#define	 	TMP117_DEV_ONE_SHOT_MODE 													 ((0x0001 << 2))   /*  */
#define	 	TMP117_DEV_ALERT_MODE															 ((0x0001 << 3))   /*  */
#define	 	TMP117_DEV_THERM_MODE															 ((0x0001 << 4))   /*  */
		uint16_t dev_mode;
	
	    /*! Read function pointer */
    tmp117_com_fptr_t read;

    /*! Write function pointer */
    tmp117_com_fptr_t write;

    /*!  Delay function pointer */
    tmp117_delay_fptr_t delay_ms;
	
	/*! point to specified Mode structure */
	struct tmp117_cfg mode_cfg;

	/*! Temperature result register */
	struct tmp117_sensor_data *sensor_data;
	
};

/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
}
#endif

#endif /* _TMP117_H_ */
/** @}*/
