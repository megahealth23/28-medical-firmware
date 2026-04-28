#ifndef __BATTERY_DEF_H__
#define __BATTERY_DEF_H__

typedef struct charg_chip_dev charg_chip_dev;

typedef int8_t (*charg_i8_fptr_u8_u8_u8p_u16)(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
typedef void (*charg_v_fptr_u32)(uint32_t ms);
typedef int8_t (*charg_i8_fptr_stp)(struct charg_chip_dev *dev); 
typedef int8_t (*charg_i8_fptr_u8_stp)(uint8_t mode, struct charg_chip_dev *dev);
typedef int8_t (*charg_i8_fptr_u8p)(struct charg_chip_dev *dev, uint8_t* pu8); 
typedef int8_t (*charg_i8_fptr_u8p4)(struct charg_chip_dev *dev, uint8_t* u1, uint8_t* u2, uint8_t* u3, uint8_t* u4); 
typedef void (*charg_v_fptr_stp)(struct charg_chip_dev *dev);
typedef bool (*charg_b_fptr_stp)(struct charg_chip_dev *dev);

/** Error code definitions */
#define BATTERY_OK                            INT8_C(0)
#define BATTERY_E_NULL_PTR                    INT8_C(-1)
#define BATTERY_E_COM_FAIL                    INT8_C(-2)
#define BATTERY_E_DEV_NOT_FOUND               INT8_C(-3)
#define BATTERY_E_OUT_OF_RANGE                INT8_C(-4)
#define BATTERY_E_INVALID_INPUT               INT8_C(-5)
#define BATTERY_E_ACCEL_ODR_BW_INVALID        INT8_C(-6)
#define BATTERY_E_GYRO_ODR_BW_INVALID         INT8_C(-7)
#define BATTERY_E_LWP_PRE_FLTR_INT_INVALID    INT8_C(-8)
#define BATTERY_E_LWP_PRE_FLTR_INVALID        INT8_C(-9)
#define BATTERY_E_AUX_NOT_FOUND               INT8_C(-10)
#define BATTERY_FOC_FAILURE                   INT8_C(-11)
#define BATTERY_READ_WRITE_LENGHT_INVALID     INT8_C(-12)


#define BATTERY_VOL_MAX     (4.2)
#define BATTERY_VOL_10_PER  (3.6)
#define BATTERY_VOL_5_PER   (3.2)

typedef enum {
    e_charging,
    e_charg_done,
    e_no_charg
} E_CHARG_STATUS;


enum dev_power_state
{
    DEV_NORMAL,		//device normal work
    DEV_CHGING,		//device charging
    DEV_CHGFULL,	//device charge full
    DEV_LOWPWR,		//device low power
    DEV_FAULTS,		//device faults
    DEV_SHUTDOWN,	//device shutdown	
};

struct charg_chip_dev
{
    uint8_t chip_id;
    uint8_t id;
    uint16_t dev_mode;
    uint8_t charg_status;
    uint8_t inited;
    charg_i8_fptr_u8_u8_u8p_u16 I2C_read;
    charg_i8_fptr_u8_u8_u8p_u16 I2C_write;
    charg_v_fptr_u32 delay_ms;
    charg_i8_fptr_stp init;
    charg_i8_fptr_u8_stp set_mode;
    charg_i8_fptr_u8p get_cur_charg_status;
    charg_v_fptr_stp wdt_handle;
    charg_b_fptr_stp ok;
    charg_v_fptr_stp show_status;
    charg_i8_fptr_u8p4 get_status;
    charg_i8_fptr_u8_stp set_charge_intensity;
    charg_i8_fptr_u8_stp set_charge_enable;
    charg_i8_fptr_stp hardware_reset;
};

typedef enum
{
    RING_CHGCHIP_NONE_MODE = 0x00,
    RING_CHGCHIP_TRANSPORT = (0x01<<0x00),
    RING_CHGCHIP_NORMAL = (0x01<<0x01),
} ring_chgchip_mode_t;

#define DEF_CHARG_CHIP(chip)    \
    struct charg_chip_dev _chip = {0};

#define NEW_CHARG_CHIP(chip)   \
    _chip.I2C_read = charg_I2C_read; \
    _chip.I2C_write = charg_I2C_write;   \
    _chip.delay_ms = charg_delay_ms;   \
    _chip.init = chip##_init;    \
    _chip.set_mode = chip##_set_mode;    \
    _chip.get_cur_charg_status = chip##_get_cur_charg_status;    \
    _chip.wdt_handle = chip##_wdt_handle;   \
    _chip.ok = chip##_ok;   \
    _chip.show_status = chip##_show_status; \
    _chip.get_status = chip##_get_status; \
    _chip.set_charge_intensity = chip##_set_charge_intensity;   \
    _chip.set_charge_enable = chip##_set_charge_enable; \
    _chip.hardware_reset = chip##_hardware_reset; \

#endif