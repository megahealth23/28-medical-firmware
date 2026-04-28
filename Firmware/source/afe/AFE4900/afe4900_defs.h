/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file afe4900_defs.h
* @author:li tianzheng
* @modifing:
* @date 2020.10.15
* @version
* @brief
*/

#ifndef _AFE4900_DEFS_H_
#define _AFE4900_DEFS_H_

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


/** AFE4900 Register map */
#define 		AFE4900_FIFO_READOUT_ADDR    						    (0xFF)

#define     MODE_MAX_FIFO_PERIOD             (100)
#define     DAILY_FIFO_PERIOD                (100)//(50)//(40)//(80)
#define     EHR_FIFO_PERIOD                  (100)//(50)//(40)//(80)
#define     SPO2_FIFO_PERIOD                 (20)
#define     PTT_FIFO_PERIOD                  (5)

//   reg_addr: 0x42
#define     SPO2_REG_FIFO_PERIOD             (SPO2_FIFO_PERIOD-1)
#define     EHR_REG_FIFO_PERIOD              (EHR_FIFO_PERIOD-1)
#define     DAILY_REG_FIFO_PERIOD              (DAILY_FIFO_PERIOD-1)
#define     PTT_REG_FIFO_PERIOD              (PTT_FIFO_PERIOD-1)


/** Error code definitions */
/** Error code definitions */
#define 		AFE4900_OK                            					 (0)
#define 		AFE4900_E_NULL_PTR                    					 (-1)
#define 		AFE4900_E_COM_FAIL                  	  				 (-2)
#define 		AFE4900_E_DEV_NOT_FOUND               					 (-3)
#define 		AFE4900_READ_WRITE_LENGHT_INVALID     					 (-12)

/*
#define AFE4900_E_OUT_OF_RANGE                INT8_C(-4)
#define AFE4900_E_INVALID_INPUT               INT8_C(-5)
#define AFE4900_E_ACCEL_ODR_BW_INVALID        INT8_C(-6)
#define AFE4900_E_GYRO_ODR_BW_INVALID         INT8_C(-7)
#define AFE4900_E_LWP_PRE_FLTR_INT_INVALID    INT8_C(-8)
#define AFE4900_E_LWP_PRE_FLTR_INVALID        INT8_C(-9)
#define AFE4900_E_AUX_NOT_FOUND               INT8_C(-10)
#define AFE4900_FOC_FAILURE                   INT8_C(-11)
#define AFE4900_READ_WRITE_LENGHT_INVALID     INT8_C(-12)
*/

/*****************************************************************************/
/* type definitions */
typedef    int8_t      (*afe4900_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef    void        (*afe4900_delay_fptr_t)(uint32_t period);
typedef    void        (*afe4900_void_fptr_t)(void);

typedef    uint8_t      AFE4900_addr_t;
typedef    uint32_t     AFE4900_Reg_t;

/*************************** Data structures *********************************/

/*!
 * @brief afe4900 sensor configuration structure
 */
 struct afe4900_cfg 
 {
		 uint8_t         cfg_cnt;
		 AFE4900_addr_t  *addrs;
		 AFE4900_Reg_t   *regs;
 };

struct afe4900_fifo_frame{
	/*! Data buffer of user defined length is to be mapped here */
    uint8_t    *data;
/*
 | FIFO_PARTITION | FIFO_NPHASE | PHASE1       | PHASE2      | PHASE3      | PHASE4 | PHASE5 | PHASE6      |
 | (0000)B        | 4 		    	| LED2         | Amb2        | LED1        | Amb1   |    /   |   /         |
 | (0001)B        | 1           | (LED1-Amb1)  |   /         |   /         |   /    |    /   |   /         |
 | (0010)B        | 2           | LED1         | Amb1        |   /         |   /    |    /   |   /         |
 | (0011)B        | 1           | (LED2-Amb2)  |   /         |   /         |   /    |   /    |   /         |
 | (0100)B        | 2           | LED2         | Amb2        |   /         |   /    |   /    |   /         |
 | (0101)B        | 2           | (LED2-Amb2)  | (LED1-Amb1) |   /         |   /    |   /    |   /         |
 | (0110)B        | 3           | (LED1-Amb1)  | (LED2-Amb1) | (Amb2-Amb1) |   /    |   /    |   /         |
 | (0111)B        | 6           | LED2         | Amb2        | (LED2-Amb1) | LED1   |  Amb1  | (LED1-Amb1) |
 -----------------------------------------------------------------------------------------------------------
 | (1000)B        | 1           | (LED1+Amb1+  |             |             |        |        |             | 
 |                |             | LED2+Amb2)/4 |             |             |        |        |             |
*
*(1) Data marked as Ambient2 are replaced by LED3 when the third LED is enabled. Data marked as Ambient1 are replaced by LED4 when the fourth LED is enabled.
*(2) When decimation mode data are stored in the FIFO, the phases correspond to the corresponding decimation mode outputs.
*(3) When in PTT mode, the ECG signal is available in the phase marked as Ambient1.
*
* The format of fifo_partition: 	Bit[7:4]:FIFO_NPHASE,   Bit[3:0]:FIFO_PARTITION
*/
#define  	AFE4900_FIFO_PARTITION_PH4_L2_A2_L1_A1      			     ((0x04<<4) | (0x00<<0)) 
#define  	AFE4900_FIFO_PARTITION_PH1_DIFFL1A1          			     ((0x01<<4) | (0x01<<0)) 
#define  	AFE4900_FIFO_PARTITION_PH2_L1_A1            			     ((0x02<<4) | (0x02<<0)) 
#define  	AFE4900_FIFO_PARTITION_PH1_DIFFL2A2            			    ((0x01<<4) | (0x03<<0)) 
#define  	AFE4900_FIFO_PARTITION_PH2_L2_A2			    		          ((0x02<<4) | (0x04<<0)) 
#define  	AFE4900_FIFO_PARTITION_PH2_DIFFL2A2_DIFFL1A1    		     ((0x02<<4) | (0x05<<0)) 
#define  	AFE4900_FIFO_PARTITION_PH3_DIFFL1A1_DIFFL2A1_DIFFA2A1        ((0x03<<4) | (0x06<<0)) 
#define  	AFE4900_FIFO_PARTITION_PH6_L2_A2_DIFFL2A2_L1_A1_DIFFL1A1     ((0x06<<4) | (0x07<<0)) 
#define  	AFE4900_FIFO_PARTITION_PH1_SUML2A2L1A1_DIV4                  ((0x01<<4) | (0x08<<0)) 
	  uint8_t      fifo_partition;

    uint16_t     maxsize;
		uint16_t     fifo_byte_start_idx;
		uint16_t     fifo_byte_end_idx;
	
 /*! While calling the API  "afe4900_get_fifo_data" , length stores
  *  number of bytes in FIFO to be read (specified by user as input)
  *  and after execution of the API ,number of FIFO data bytes
  *  available is provided as an output to user
  */
	uint16_t     length;

};

/*!
 * @brief bmi160 sensor data structure which comprises of accel data
 */

struct afe4900_sensor_data
{
    /*!  sensor data */
    int32_t phase1;
    /*!  sensor data */
    int32_t phase2;
    /*!  sensor data */
    int32_t phase3;
	/*!  sensor data */
    int32_t phase4;
	
#define		USING_MAX_MULTIPLE_PHASES       0      
#if   USING_MAX_MULTIPLE_PHASES
	/*! Z-axis sensor data */
    int32_t phase5;
	
	/*! Z-axis sensor data */
    int32_t phase6;
#endif
};

struct afe4900_dev
{
    /*! Chip Id, without use */
#define		AFE4900_DESIGN_ID_ADDR       (0x28)
    uint32_t chip_id;

    /*! Device Id */
    uint8_t id;

/**  Device Modes settings */
//   dev_mode:   
//          dev_mode[15:8]: major mode
//          dev_mode[15:8]: minor mode
#define        AFE4900_DEV_MAJOR_MODE_MASK          (0xFF00)
#define        AFE4900_DEV_POWER_DOWN_MODE          ((0x0001 << 15))
#define        AFE4900_DEV_PPG_ONLY_MODE            ((0x0001 << 14))
#define        AFE4900_DEV_PTT_MODE                 ((0x0001 << 13))
#define        AFE4900_DEV_ECG_ONLY_MODE            ((0x0001 << 12))
#define        AFE4900_DEV_MINOR_MODE_MASK          (0x00FF)
#define        AFE4900_DEV_PPG_SPO2_MODE            ((0x0001 << 7))
#define        AFE4900_DEV_PPG_SPORT_MODE           ((0x0001 << 6))
#define        AFE4900_DEV_PPG_DAILY_MODE           ((0x0001 << 5))
#define        AFE4900_DEV_PPG_BP_MODE              ((0x0001 << 4))
		uint16_t dev_mode;

	    /*! Read function pointer */
    afe4900_com_fptr_t read;

    /*! Write function pointer */
    afe4900_com_fptr_t write;

    /*!  Delay function pointer */
    afe4900_delay_fptr_t delay_ms;
	
    /*!  Software reset device */
	afe4900_void_fptr_t sw_reset;

	/*! Structure to configure PTT Mode */
	struct afe4900_cfg ptt_cfg;
	
	/*! Structure to configure PTT Mode */
	struct afe4900_cfg ppg_cfg;

	/*! Temperature result register */
	struct afe4900_fifo_frame *fifo;
	
	    /*! User set read/write length */
    uint16_t read_write_len;
};
/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
}
#endif

#endif /* _AFE4900_DEFS_H_ */
/** @}*/
