#ifndef  _BQ21080_H_
#define  _BQ21080_H_

#include <stdbool.h>
#include <stdint.h>
#include "battery_def.h"


#ifdef __cplusplus  
extern "C"
{
#endif

/*
*     INT PIN is an open-drain output that signals fault interrupts. When a fault occurs, a 128-¦Ìs active 
*  low pulse is sent out as an interrupt for the host. INT is enabled/disabled using the MASK_INT 
*  bit in the control register. Can be pulled up to the logic rail through a 1-k? to 20-k? resistor.
*/


#define  BQ21080_ENABLE_CHARGE_MODE     (true) 
#define  BQ21080_ENABLE_BATTERY_MODE    (false) 
#define  BQ21080_ENABLE_SHIP_MODE       (true) 
#define  BQ21080_ENABLE_SHUTDOWN_MODE   (false) 

typedef struct bq21080_stat0_s{
    uint8_t vin_pgood_stat             : 1;
    uint8_t thermreg_active_stat       : 1;
    uint8_t vindpm_active_stat         : 1;
    uint8_t vdppm_active_stat          : 1;
    uint8_t ilim_active_stat           : 1;
#define BQ21080_CHG_STAT_NOT_CHGING     (0x00)
#define BQ21080_CHG_STAT_CONST_CURRENT  (0x01)
#define BQ21080_CHG_STAT_CONST_VOLTAGE  (0x02)
#define BQ21080_CHG_STAT_CHGING_DONE    (0x03)
    uint8_t chg_stat                   : 2;
    uint8_t ts_open_stat               : 1;
} bq21080_stat0_t;

typedef struct bq21080_stat1_s{
    uint8_t wake2_flag                 : 1;
    uint8_t wake1_flag                 : 1;
    uint8_t safety_tmr_fault_flag      : 1;
    uint8_t ts_stat                    : 2;
    uint8_t stat1_reserved             : 1;
    uint8_t buvlo_stat                 : 1;
    uint8_t vin_ovp_stat               : 1;
} bq21080_stat1_t;

typedef struct  bq21080_flag0_s{
    uint8_t bat_ocp_fault_flag         : 1;
    uint8_t buvlo_fault_flag           : 1;
    uint8_t vin_ovp_fault_flag         : 1;
    uint8_t thermreg_active_flag       : 1;
    uint8_t vindpm_active_flag         : 1;
    uint8_t vdppm_active_flag          : 1;
    uint8_t ilim_active_flag           : 1;
    uint8_t ts_fault                   : 1;
} bq21080_flag0_t;

typedef struct  bq21080_stat_flag_s{
    union{
        bq21080_stat0_t stat0;
        uint8_t stat0_val;
    };

    union{
        bq21080_stat1_t stat1;
        uint8_t stat1_val;
    };

    union{
        bq21080_flag0_t flag0;
        uint8_t flag0_val;
    };
}bq21080_statflag_t;

/*
*  BQ21080_STAT0_ADDR, Table 8-9. STAT0 Register Field Descriptions¡£
*/
#define  BQ21080_TS_OPEN_STAT_POS        (0x07)
#define  BQ21080_TS_OPEN_STAT_MASK       ((0x01)<<(BQ21080_TS_OPEN_STAT_POS))      // TS Open Status: 
                                                                                   //        1)1b0 = TSMR pin is not Open; 
                                                                                   //        2)1b1 = TSMR pin is Open; 
#define  BQ21080_CHG_STAT_1_0_POS        (0x05)
#define  BQ21080_CHG_STAT_1_0_MASK       ((0x03)<<(CHG_STAT_1_POS))               // Charging Status Indicator:
                                                                                   //        1)2b00 = Not Charging while charging is enabled. ;
                                                                                   //        2)2b01 = Constant Current Charging (Trickle Charge/ Pre Charge or in Fast Charge Mode); 
                                                                                   //        3)2b10 = Constant Voltage Charging.;  
                                                                                   //        4)2b11 = Charge Done or charging is disabled by the host. 
#define  BQ21080_ILIM_ACTIVE_STAT_POS     (0x04)
#define  BQ21080_ILIM_ACTIVE_STAT_MASK ((0x01)<<(BQ21080_CHG_STAT_1_POS))           // Input Curent Limit Active:
                                                                                    //        1)1b0 = Not Active 
                                                                                    //        2)1b1 = Active 
#define  BQ21080_VDPPM_ACTIVE_STAT_POS    (0x03)
#define  BQ21080_VDPPM_ACTIVE_STAT_MASK ((0x01)<<(BQ21080_VDPPM_ACTIVE_STAT_POS))   //  VDPPM Mode Active:
                                                                                    //        1)1b0 = Not Active 
                                                                                    //        2)1b1 = Active 
#define  BQ21080_VINDPM_ACTIVE_STAT_POS   (0x02)
#define  BQ21080_VINDPM_ACTIVE_STAT_MASK ((0x01)<<(BQ21080_VINDPM_ACTIVE_STAT_POS)) //  VINDPM Mode Active:
                                                                                    //        1)1b0 = Not Active 
                                                                                    //        2)1b1 = Active 
#define  BQ21080_THERMREG_ACTIVE_ST_POS   (0x01) 
#define  BQ21080_THERMREG_ACTIVE_ST_MASK ((0x01)<<(BQ21080_THERMREG_ACTIVE_ST_POS)) // Thermal Regulation Active:
                                                                                    //        1)1b0 = Not Active 
                                                                                    //        2)1b1 = Active 
#define  BQ21080_VIN_PGOOD_STAT_POS       (0x00)
#define  BQ21080_VIN_PGOOD_STAT_MASK ((0x01)<<(BQ21080_VIN_PGOOD_STAT_POS))         //  VIN Power Good:
                                                                                    //        1)1b0 = VIN Power Not Good 
                                                                                    //        2)1b1 = VIN Power Good
  
/*
*  BQ21080_STAT1_ADDR, Table 8-10. STAT1 Register Field Descriptions¡£
*/
#define  BQ21080_VIN_OVP_STAT_POS          (0x07)
#define  BQ21080_VIN_OVP_STAT_MASK ((0x01)<<(BQ21080_VIN_OVP_STAT_POS))             // VIN_OVP Fault
                                                                                    //        1)1b0 = Not Active 
                                                                                    //        2)1b1 = Active 
#define  BQ21080_BUVLO_STAT_POS            (0x06)
#define  BQ21080_BUVLO_STAT_MASK ((0x01)<<(BQ21080_BUVLO_STAT_POS))// Battery UVLO Status
                                                                                    //        1)1b0 = Not Active 
                                                                                    //        2)1b1 = Active 
#define  BQ21080_TS_STAT_1_0_POS           (0x03)
#define  BQ21080_TS_STAT_1_0_MASK ((0x03)<<(BQ21080_TS_STAT_1_POS))                 // TS Status
                                                                                    //        1)2b00 = Normal 
                                                                                    //        2)2b01 = VTS < VHOT or VTS > VCOLD(charging suspended) 
                                                                                    //        3)2b10 = VCOOL < VTS < VCOLD (Charging current reduced by value set by TS_Registers) 
                                                                                    //        4)2b11 = VWARM > VTS > VHOT (Charging voltage reduced by value set by TS_Registers) 
#define  BQ21080_SAFETY_TMR_FAULT_FLAG_POS  (0x02)
#define  BQ21080_SAFETY_TMR_FAULT_FLAG_MASK ((0x01)<<(BQ21080_SAFETY_TMR_FAULT_FLAG_POS)) //Safety Timer Expired Fault Cleared only after CE is toggled.
                                                                                          //        1)1b0 = Not Active 
                                                                                          //        2)1b1 = Active 
#define  BQ21080_WAKE1_FLAG_POS             (0x01)
#define  BQ21080_WAKE1_FLAG_MASK ((0x01)<<(BQ21080_WAKE1_FLAG_POS))                 // Wake 1 Timer Flag
                                                                                    //        1)1b0 = Does not meet Wake 1 Condition 
                                                                                    //        2)1b1 = Met Wake 1 Condition
#define  BQ21080_WAKE2_FLAG_POS             (0x00)
#define  BQ21080_WAKE2_FLAG_MASK ((0x01)<<(BQ21080_WAKE2_FLAG_POS))// Wake 2 Timer Flag
                                                                                    //        1)1b0 = Does not meet Wake 2 Condition 
                                                                                    //        2)1b1 = Met Wake2 Condition


/*
*  BQ21080_FLAG0_ADDR,  Table 8-11. FLAG0 Register Field Descriptions.
*/
#define  BQ21080_TS_FAULT_POS                (0x07)
#define  BQ21080_TS_FAULT_MASK ((0x01)<<(BQ21080_TS_FAULT_POS))                     //TS_Fault
                                                                                    //        1)1b0 = No TS Fault detected 
                                                                                    //        2)1b1 = TS Fault detected 
#define  BQ21080_ILIM_ACTIVE_FLAG_POS        (0x06)
#define  BQ21080_ILIM_ACTIVE_FLAG_MASK ((0x01)<<(BQ21080_ILIM_ACTIVE_FLAG_POS))     //ILIM Active
                                                                                    //        1)1b0 = NO ILIM Fault detected 
                                                                                    //        2)1b1 = ILIM Fault detected 
#define  BQ21080_VDPPM_ACTIVE_FLAG_POS       (0x05)
#define  BQ21080_VDPPM_ACTIVE_FLAG_MASK ((0x01)<<(BQ21080_VDPPM_ACTIVE_FLAG_POS))   //VDPPM FLAG
                                                                                    //        1)1b0 = VDPPM fault not detected 
                                                                                    //        2)1b1 = VDPPM fault detected 
#define  BQ21080_VINDPM_ACTIVE_FLAG_POS      (0x04)
#define  BQ21080_VINDPM_ACTIVE_FLAG_MASK ((0x01)<<(BQ21080_VINDPM_ACTIVE_FLAG_POS)) //VINDPM FLAG
                                                                                    //        1)1b0 = VINDPM fault not detected 
                                                                                    //        2)1b1 = VINDPM fault detected 
#define  BQ21080_THERMREG_ACTIVE_FLAG_POS    (0x03)
#define  BQ21080_THERMREG_ACTIVE_FLAG_MASK ((0x01)<<(BQ21080_THERMREG_ACTIVE_FLAG_POS))   //Thermal Regulation FLAG
                                                                                          //        1)1b0 = No thermal regulation detected 
                                                                                          //        2)1b1 = Thermal regulation has occured 
#define  BQ21080_VIN_OVP_FAULT_FLAG_POS      (0x02)
#define  BQ21080_VIN_OVP_FAULT_FLAG_MASK ((0x01)<<(BQ21080_VIN_OVP_FAULT_FLAG_POS)) //VIN_OVP FLAG
                                                                                    //        1)1b0 = VIN_OVP fault not detected 
                                                                                    //        2)1b1 = VIN_OVP fault detected 
#define  BQ21080_BUVLO_FAULT_FLAG_POS        (0x01)
#define  BQ21080_BUVLO_FAULT_FLAG_MASK ((0x01)<<(BQ21080_BUVLO_FAULT_FLAG_POS))     //Battery undervoltage FLAG
                                                                                    //        1)1b0 = Battery undervoltage fault not detected 
                                                                                    //        2)1b1 = Battery undervoltage fault detected 
#define  BQ21080_BAT_OCP_FAULT_POS           (0x00)
#define  BQ21080_BAT_OCP_FAULT_MASK ((0x01)<<(BQ21080_BAT_OCP_FAULT_POS))           //Battery overcurrent protection
                                                                                    //        1)1b0 = Battery overcurrent condition not detected
                                                                                    //        2)1b1 = Battery overcurrent condition detected

                                                                                                                               
/*
*  BQ21080_VBAT_CTRL_ADDR, Table 8-12. VBAT_CTRL Register Field Descriptions.
*/
#define  BQ21080_VBATREG_6_0_POS             (0x00)
#define  BQ21080_VBATREG_6_0_MASK ((0x7F)<<(BQ21080_VBATREG_6_0_POS))   // Battery Regulation Voltage VBATREG= 3.5V + VBATREG_CODE * 10mV. Maximum programmable voltage = 4.65V
#define  SET_VBATREG(mVol)        ((uint8_t)((((mVol)-3500)/10)&(BQ21080_VBATREG_6_0_MASK)))

 
/*
*  BQ21080_ICHG_CTRL_ADDR, Table 8-13. ICHG_CTRL Register Field Descriptions.
*/
#define  BQ21080_CHG_DIS_POS                 (0x07)
#define  BQ21080_CHG_DIS_MASK ((0x01)<<(BQ21080_CHG_DIS_POS))                       //Charge Disable
                                                                                    //        1)1b0 = Battery Charging Enabled 
                                                                                    //        2)1b1 = Battery Charging Disabled 
#define  BQ21080_ICHG_6_0_POS                (0x00)
#define  BQ21080_ICHG_6_0_MASK ((0x7F)<<(BQ21080_ICHG_6_0_POS))  //For ICHG <= 35mA = ICHGCODE +5mA For ICHG > 35mA = 40+ ((ICHGCODE-31)*10)mA. Maximum programmable current = 800mA  

#define GET_REGULAR_ICHG_LE_35MA(mIchg)   (((mIchg)<5)?(5):(mIchg))
#define GET_REGULAR_ICHG_GT_35MA(mIchg)   (((mIchg)<40)?(40):(((uint8_t)(((mIchg)+5)/10))*10))
#define CAL_ICHG_LE_35MA(mIchg)    ((GET_REGULAR_ICHG_LE_35MA(mIchg))-5)
#define CAL_ICHG_GT_35MA(mIchg)    ((((GET_REGULAR_ICHG_GT_35MA(mIchg))-40)/10)+31)
#define SET_ICHG(mIchg)      ((uint8_t)((((mIchg)<=35)?  CAL_ICHG_LE_35MA(mIchg): CAL_ICHG_GT_35MA(mIchg))&(BQ21080_ICHG_6_0_MASK)))      /*mIchg: 1)<=35: from 5 to 35, step is 1;  2)>35: from 40 to 800, step is 10*/

/*
*  BQ21080_CHARGECTRL0_ADDR, Table 8-14. CHARGECTRL0 Register Field Descriptions.
*/  
#define  BQ21080_IPRECHG_POS                 (0x06)
#define  BQ21080_IPRECHG_MASK ((0x01)<<(BQ21080_IPRECHG_POS))                       //Precharge current = x times of term
                                                                                    //        1)1b0 = Precharge is 2x Term 
                                                                                    //        2)1b1 = Precharge is Term 
#define  BQ21080_ITERM_1_0_POS               (0x04)
#define  BQ21080_ITERM_1_0_MASK ((0x03)<<(BQ21080_ITERM_1_0_POS))                   //Termination current = % of Icharge
                                                                                    //        1)2b00 = Disable 
                                                                                    //        2)2b01 = 5% of ICHG 
                                                                                    //        3)2b10 = 10% of ICHG 
                                                                                    //        4)2b11 = 20% of ICHG 
#define  BQ21080_VINDPM_1_0_POS              (0x02)
#define  BQ21080_VINDPM_1_0_MASK ((0x03)<<(BQ21080_VINDPM_1_0_POS))                 // VINDPM Level Selection
                                                                                    //        1)2b00 = 4.2 V 
                                                                                    //        2)2b01 = 4.5 V 
                                                                                    //        3)2b10 = 4.7 V 
                                                                                    //        4)2b11 = Disabled 
#define  BQ21080_THERM_REG_1_0_POS           (0x00)
#define  BQ21080_THERM_REG_1_0_MASK ((0x03)<<(BQ21080_THERM_REG_1_0_POS))           //Thermal Regulation Threshold
                                                                                    //        1)2b00 = 100C 
                                                                                    //        2)2b11 = Disabled


 
/*
*  BQ21080_CHARGECTRL1_ADDR, Table 8-15. CHARGECTRL1 Register Field Descriptions.
*/ 
#define  BQ21080_IBAT_OCP_1_0_POS            (0x06)
#define  BQ21080_IBAT_OCP_1_0_MASK ((0x03)<<(BQ21080_IBAT_OCP_1_0_POS))             // Battery Discharge Current Limit
                                                                                    //        1)2b00 = 500mA 
                                                                                    //        2)2b01 = 1000mA 
                                                                                    //        3)2b10 = 1500mA 
                                                                                    //        4)2b11 = Disabled 
#define  BQ21080_BUVLO_2_0_POS               (0x03)
#define  BQ21080_BUVLO_2_0_MASK ((0x07)<<(BQ21080_BUVLO_2_0_POS))                   //Battery Undervoltage LockOut Falling Threshold.
                                                                                    //        1)3b000 = 3.0V 
                                                                                    //        2)3b001 = 3.0V 
                                                                                    //        3)3b010 = 3.0V 
                                                                                    //        4)3b011 = 2.8V 
                                                                                    //        5)3b100 = 2.6V 
                                                                                    //        6)3b101 = 2.4V
                                                                                    //        7)3b110 = 2.2V 
                                                                                    //        8)3b111 = 2.0V 
#define  BQ21080_CHG_STATUS_INT_POS          (0x02)
#define  BQ21080_CHG_STATUS_INT_MASK ((0x01)<<(BQ21080_CHG_STATUS_INT_POS))         //Mask Charging Status Interrupt
                                                                                    //        1)Enable Charging Status Interrupt anytime there is a charging status change.    
                                                                                    //        2)Mask Charging Status Interrup
#define  BQ21080_ILIM_INT_POS                (0x01)
#define  BQ21080_ILIM_INT_MASK ((0x01)<<(BQ21080_ILIM_INT_POS))                     // Mask ILIM Fault Interrupt
                                                                                    //        1)1b0 = Enable ILIM Interrupt 
                                                                                    //        2)1b1 = Mask ILIM Interrupt 
#define  BQ21080_VDPM_INT_POS                (0x00)
#define  BQ21080_VDPM_INT_MASK ((0x01)<<(BQ21080_VDPM_INT_POS))                     //Mask VINDPM and VDPPM Interrupt
                                                                                    //        1)1b0 = Enable VINDPM and VDPPM Interrupt
                                                                                    //        2)1b1 = Mask VINDPM and VDPPM Interrupt



/*
*  BQ21080_IC_CTRL_ADDR, Table 8-16. IC_CTRL Register Field Descriptions.
*/ 
#define  BQ21080_TS_EN_POS                   (0x07)
#define  BQ21080_TS_EN_MASK ((0x01)<<(BQ21080_TS_EN_POS))                           //TS Auto Function
                                                                                    //        1)1b0 = TS auto function disabled (Only charge control is disabled. TS monitoring is enabled) 
                                                                                    //        2)1b1 = TS auto function enabled 
#define  BQ21080_VLOWV_SEL_POS               (0x06)
#define  BQ21080_VLOWV_SEL_MASK ((0x01)<<())                                        //Precharge Voltage Threshold (VLOWV)
                                                                                    //        1)1b0 = 3V 
                                                                                    //        2)1b1 = 2.8V                                                                                                          
#define  BQ21080_VRCH_0_POS                  (0x05)
#define  BQ21080_VRCH_0_MASK ((0x01)<<(BQ21080_VRCH_0_POS))                         //Recharge Voltage Threshold
                                                                                    //        1)1b0 = 100mV 
                                                                                    //        2)1b1 = 200 mV 
#define  BQ21080_2XTMR_EN_POS                (0x04)
#define  BQ21080_2XTMR_EN_MASK ((0x01)<<(BQ21080_2XTMR_EN_POS))                     // Timer Slow
                                                                                    //        1)1b0 = The timer is not slowed at any time 
                                                                                    //        2)1b1 = The timer is slowed by 2x when in any control other than CC or CV                                     
#define  BQ21080_SAFETY_TIMER_1_0_POS        (0x02)
#define  BQ21080_SAFETY_TIMER_1_0_MASK ((0x03)<<(BQ21080_SAFETY_TIMER_1_0_POS))     //Fast Charge Timer
                                                                                    //        1)2b00 = 3 hour fast charge 
                                                                                    //        2)2b01 = 6 hour fast charge 
                                                                                    //        3)2b10 = 12 hour fast charge  
                                                                                    //        4)2b11 = Disable safety timer                  
#define  BQ21080_WATCHDOG_SEL_1_0_POS        (0x00)
#define  BQ21080_WATCHDOG_SEL_1_0_MASK ((0x03)<<(BQ21080_WATCHDOG_SEL_1_0_POS))     // Watchdog Selection
                                                                                    //        1)2b00 = 160s default register values 
                                                                                    //        2)2b01 = 160s HW_RESET 
                                                                                    //        3)2b10 = 40s HW_RESET
                                                                                    //        4)2b11 = Disable watchdog function 



/*
*  BQ21080_TMR_ILIM_ADDR,  Table 8-17. TMR_ILIM Register Field Descriptions.
*/
#define  BQ21080_MR_LPRESS_1_0_POS            (0x06)
#define  BQ21080_MR_LPRESS_1_0_MASK ((0x03)<<(BQ21080_MR_LPRESS_1_0_POS))           //Push button Long Press duration timer
                                                                                    //        1)2b00 = 5s 
                                                                                    //        2)2b01 = 10s 
                                                                                    //        3)2b10 = 15s 
                                                                                    //        4)2b11 = 20s 
#define  BQ21080_MR_RESET_VIN_POS             (0x05)
#define  BQ21080_MR_RESET_VIN_MASK ((0x01)<<(BQ21080_MR_RESET_VIN_POS))             //Hardware reset condition
                                                                                    //        1)1b0 = Reset sent when long press duration is met 
                                                                                    //        2)1b1 = Reset sent when long press duration is met and VIN_Powergood
#define  BQ21080_AUTOWAKE_1_0_POS             (0x03)
#define  BQ21080_AUTOWAKE_1_0_MASK ((0x03)<<(BQ21080_AUTOWAKE_1_0_POS))             // Auto Wake Up Timer Restart
                                                                                    //        1)2b00 = 0.5s 
                                                                                    //        2)2b01 = 1s 
                                                                                    //        3)2b10 = 2s 
                                                                                    //        4)2b11 = 4s 
#define  BQ21080_ILIM_2_0_POS                 (0x00)
#define  BQ21080_ILIM_2_0_MASK ((0x07)<<(BQ21080_ILIM_2_0_POS))                     //Input Current Limit Setting
                                                                                    //        1)3b000 = 50mA 
                                                                                    //        2)3b001 = 100mA(max.)      
                                                                                    //        3)3b010 = 200mA             
                                                                                    //        4)3b011 = 300mA         
                                                                                    //        5)3b100 = 400mA         
                                                                                    //        6)3b101 = 500mA(max.)      
                                                                                    //        7)3b110 = 700mA      
                                                                                    //        8)3b111 = 1100mA



/*
*  BQ21080_SHIP_RST_ADDR,  Table 8-18. SHIP_RST Register Field Descriptions.
*/
#define  BQ21080_REG_RST_POS                  (0x07)
#define  BQ21080_REG_RST_MASK ((0x01)<<(BQ21080_REG_RST_POS))                       // Software Reset
                                                                                    //        1)1b0 = Do nothing 
                                                                                    //        2)1b1 = Software Reset 
#define  BQ21080_EN_RST_SHIP_1_0_POS          (0x05)
#define  BQ21080_EN_RST_SHIP_1_0_MASK ((0x03)<<(BQ21080_EN_RST_SHIP_1_0_POS))       //Shipmode Enable and Hardware Reset
                                                                                    //        1)2b00 = Do nothing 
                                                                                    //        2)2b01 = Enable shutdown mode with wake on adapter insert only
                                                                                    //        3)2b10 = Enable shipmode with wake on button press or adapter insert 
                                                                                    //        4)2b11 = Hardware Reset
#define  BQ21080_PB_LPRESS_ACTION_1_0_POS     (0x03)
#define  BQ21080_PB_LPRESS_ACTION_1_0_MASK ((0x03)<<(BQ21080_PB_LPRESS_ACTION_1_0_POS))   //Pushbutton long press action
                                                                                          //        1)2b00 = Do nothing
                                                                                          //        2)2b01 = Hardware Reset 
                                                                                          //        3)2b10 = Enable shipmode
                                                                                          //        4)2b11 = Enable shutdown mode 
#define  BQ21080_WAKE1_TMR_POS                (0x02)
#define  BQ21080_WAKE1_TMR_MASK ((0x01)<<(BQ21080_WAKE1_TMR_POS))                   // Wake 1 Timer Set
                                                                                    //        1)1b0 = 300ms 
                                                                                    //        2)1b1 = 1s
#define  BQ21080_WAKE2_TMR_POS                (0x01)
#define  BQ21080_WAKE2_TMR_MASK ((0x01)<<(BQ21080_WAKE2_TMR_POS))                   // Wake 2 Timer Set
                                                                                    //        1)1b0 = 2s
                                                                                    //        2)1b1 = 3s
#define  BQ21080_EN_PUSH_POS                  (0x00)
#define  BQ21080_EN_PUSH_MASK ((0x01)<<(BQ21080_EN_PUSH_POS))                       //Enable Push Button and Reset Function on Battery Only
                                                                                    //        1)1b0 = Disable 
                                                                                    //        2)1b1 = Enable



/*
*  BQ21080_SYS_REG_ADDR, Table 8-19. SYS_REG Register Field Descriptions.
*/ 
#define  BQ21080_SYS_REG_CTRL_2_0_POS         (0x05)
#define  BQ21080_SYS_REG_CTRL_2_0_MASK ((0x07)<<(BQ21080_SYS_REG_CTRL_2_0_POS))     // SYS Regulation Voltgage
                                                                                    //        1)3b000 = Battery Tracking Mode 
                                                                                    //        2)3b001 = 4.4V 
                                                                                    //        3)3b010 = 4.5V 
                                                                                    //        4)3b011 = 4.6V
                                                                                    //        5)3b100 = 4.7V 
                                                                                    //        6)3b101 = 4.8V 
                                                                                    //        7)3b110 = 4.9V 
                                                                                    //        8)3b111 = Pass-Through (VSYS is VIN)
#define  BQ21080_SYS_MODE_1_0_POS             (0x02)
#define  BQ21080_SYS_MODE_1_0_MASK ((0x03)<<(BQ21080_SYS_MODE_1_0_POS))             // Sets how SYS is powered in any state, except SHIPMODE
                                                                                    //        1)2b00 = SYS powered from VIN if present or VBAT 
                                                                                    //        2)2b01 = SYS powered from VBAT only, even if VIN present 
                                                                                    //        3)2b10 = SYS disconnected and left floating 
                                                                                    //        4)2b11 = SYS disconnected with pulldown 
#define  BQ21080_WATCHDOG_15S_ENABLE_POS      (0x01)
#define  BQ21080_WATCHDOG_15S_ENABLE_MASK ((0x01)<<(BQ21080_WATCHDOG_15S_ENABLE_POS)) // I2C Watchdog
                                                                                      //        1)1b0 = Mode Disabled
                                                                                      //        2)1b1 = Do a HW reset after 15s if no I2C transaction after VIN plugged 
#define  BQ21080_VDPPM_DIS_POS                (0x00)
#define  BQ21080_VDPPM_DIS_MASK ((0x01)<<(BQ21080_VDPPM_DIS_POS))                   // Disable VDPPM
                                                                                    //        1)1b0 = Enable VDPPM 
                                                                                    //        2)1b1 = Disable VDPPM



/*
*  BQ21080_TS_CONTROL_ADDR,  Table 8-20. TS_CONTROL Register Field Descriptions.
*/
#define  BQ21080_TS_HOT_POS                  (0x06)
#define  BQ21080_TS_HOT_MASK ((0x03)<<(BQ21080_TS_HOT_POS))                         // TS Hot threshold register
                                                                                    //        1)2b00 = Default 60C 
                                                                                    //        2)2b01 = 65C  
                                                                                    //        3)2b10 = 50C 
                                                                                    //        4)2b11 = 45C
#define  BQ21080_TS_COLD_POS                 (0x04)
#define  BQ21080_TS_COLD_MASK ((0x03)<<(BQ21080_TS_COLD_POS))                       // TS Cold threshold register
                                                                                    //        1)2b00 = Default 0C 
                                                                                    //        2)2b01 = 3C 
                                                                                    //        3)2b10 = 5C 
                                                                                    //        4)2b11 = -3C                                                                                                                                                                           
#define  BQ21080_TS_WARM_POS                 (0x03)
#define  BQ21080_TS_WARM_MASK ((0x01)<<(BQ21080_TS_WARM_POS))                       // TS Warm threshold
                                                                                    //        1)1b0 = Default 45C 
                                                                                    //        2)1b1 = Disabled
#define  BQ21080_TS_COOL_POS                 (0x02)
#define  BQ21080_TS_COOL_MASK ((0x01)<<(BQ21080_TS_COOL_POS))                       // TS Cool threshold register
                                                                                    //        1)1b0 = Default 10C 
                                                                                    //        2)1b1 = Disabled 
#define  BQ21080_TS_ICHG_POS                 (0x01)
#define  BQ21080_TS_ICHG_MASK ((0x01)<<(BQ21080_TS_ICHG_POS))                       // Fast charge current when decreased by TS function
                                                                                    //        1)1b0 = 0.5*ICHG 
                                                                                    //        2)1b1 = 0.2*ICHG   
#define  BQ21080_TS_VRCG_POS                 (0x00)
#define  BQ21080_TS_VRCG_MASK ((0x01)<<(BQ21080_TS_VRCG_POS))                       // Reduced target battery voltage during Warm
                                                                                    //        1)1b0 = VBATREG -100mV 
                                                                                    //        2)1b1 = VBATREG -200mV


/*
*  BQ21080_MASK_ID_ADDR, Table 8-21. MASK_ID Register Field Descriptions.
*/
#define  BQ21080_TS_INT_POS                  (0x07)
#define  BQ21080_TS_INT_MASK ((0x01)<<(BQ21080_TS_INT_POS))                         // Mask TS
                                                                                    //        1)1b0 = Enable TS Interrupt
                                                                                    //        2)1b1 = Mask TS Interrupt 
#define  BQ21080_TREG_INT_POS                (0x06)
#define  BQ21080_TREG_INT_MASK ((0x01)<<(BQ21080_TREG_INT_POS))                     // Mask TREG
                                                                                    //        1)1b0 = Enable TREG Interrupt 
                                                                                    //        2)1b1 = Mask TREG Interrupt 
#define  BQ21080_BAT_INT_POS                 (0x05)
#define  BQ21080_BAT_INT_MASK ((0x01)<<(BQ21080_BAT_INT_POS))                       // Mask BATOCP and BUVLO
                                                                                    //        1)1b0 = Enable BOCP and BUVLO Interrupt
                                                                                    //        2)1b1 = Mask BOCP and BUVLO Interrupt 
#define  BQ21080_PG_INT_POS                  (0x04)
#define  BQ21080_PG_INT_MASK ((0x01)<<(BQ21080_PG_INT_POS))                         // Mask PG and VINOVP
                                                                                    //        1)1b0 = Enable PG and VINOVP Interrupt 
                                                                                    //        2)1b1 = Mask PG and VINOVP Interrupt   
#define  BQ21080_DEVICE_ID_POS               (0x00)
#define  BQ21080_DEVICE_ID_MASK ((0x0F)<<(BQ21080_DEVICE_ID_POS))                   // Device ID, 4b0000 = BQ21080


/*
*  Register Maps,  the memory-mapped registers for the I2C registers.
*/
#define  BQ21080_STAT0_ADDR                 (0x00)     //Charger Status Section 8.5.1.1 
#define  BQ21080_STAT1_ADDR                 (0x01)     //Charger Status and Faults Section 8.5.1.2 
#define  BQ21080_FLAG0_ADDR                 (0x02)     //Charger Flag Registers Section 8.5.1.3 
#define  BQ21080_VBAT_CTRL_ADDR             (0x03)     //Battery Voltage Control Section 8.5.1.4 
#define  BQ21080_ICHG_CTRL_ADDR             (0x04)     //Fast Charge Current Control Section 8.5.1.5 
#define  BQ21080_CHARGECTRL0_ADDR           (0x05)     //Charger Control 0 Section 8.5.1.6 
#define  BQ21080_CHARGECTRL1_ADDR           (0x06)     //Charger Control 1 Section 8.5.1.7 
#define  BQ21080_IC_CTRL_ADDR               (0x07)     //IC Control Section 8.5.1.8 
#define  BQ21080_TMR_ILIM_ADDR              (0x08)     //Timer and Input Current Limit Control Section 8.5.1.9 
#define  BQ21080_SHIP_RST_ADDR              (0x09)     //Shipmode, Reset and Pushbutton Control Section 8.5.1.10 
#define  BQ21080_SYS_REG_ADDR               (0x0A)     //SYS Regulation Voltage Control Section 8.5.1.11 
#define  BQ21080_TS_CONTROL_ADDR            (0x0B)     //TS Control Section 8.5.1.12 
#define  BQ21080_MASK_ID_ADDR               (0x0C)     //MASK and Device ID Section 8.5.1.13


/*
*  BQ21080 I2C address
*  The data transfer protocol for standard and fast modes is exactly the same; therefore, they are referred to as the 
*  F/S-mode in this document. The BQ21080 device 7-bit address is 0x6A (shifted 8-bit address is 0xD4)
*/
#define BQ21080_I2C_ADDR                      (0x6A)

/*
* The device will reset all of the registers to the defaults. Any bits loaded through OTP memory are also loaded.
*
*      Table 8-21. MASK_ID Register Field Descriptions
*     Bit  |      Field      |  Type  |   Reset  |        Description
*      7   |   TS_INT_MASK   |   R/W  |    1b1   | Mask TS: 1)1b0 = Enable TS Interrupt; 2) 1b1 = Mask TS Interrupt;
*      6   |  TREG_INT_MASK  |   R/W  |    1b1   | Mask TREG: 1)1b0 = Enable TREG Interrupt;  2)1b1 = Mask TREG Interrupt;
*      5   |   BAT_INT_MASK  |   R/W  |    1b0   | Mask BATOCP and BUVLO: 1)1b0 = Enable BOCP and BUVLO Interrupt; 2)1b1 = Mask BOCP and BUVLO Interrupt; 
*      4   |   PG_INT_MASK   |   R/W  |    1b0   | Mask PG and VINOVP: 1)1b0 = Enable PG and VINOVP Interrupt; 2)1b1 = Mask PG and VINOVP Interrupt;
*     3-0  |    Device_ID    |   R    |  4b0000  |  Device ID, 4b0000 = BQ21080
*/
#define BQ21080_CHIP_ID                       ((uint8_t)0x00)


/*****************************************************************************/
/* type definitions */
typedef int8_t (*bq21080_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*bq21080_delay_fptr_t)(uint32_t period);

typedef    uint8_t      BQ21080_addr_t;
typedef    uint8_t     BQ21080_Reg_t;

struct bq21080_cfg {
    uint8_t         cfg_cnt;
    BQ21080_addr_t  *addrs;
    BQ21080_Reg_t   *regs;
};

/*
*  8.4 Device Functional Modes
*     The BQ21080 has four main modes of operation: Battery Mode, Ship Mode, Charge/Adapter Mode when a 
*  supply is connected to IN, and Shutdown mode. The table below summarizes the functions that are active for 
*  each operation mode.  
*/  
//          dev_mode[15:8]: major mode         dev_mode[7:0]:  minor mode
#define        BQ21080_DEV_MAJOR_MODE_MASK          (0xFF00)
#define        BQ21080_DEV_MINOR_MODE_MASK          (0x00FF)
#define        BQ21080_DEV_CHARGE_ADAPTER_MODE      ((0x0001 << 15))
#define        BQ21080_DEV_BATTERY_MODE       	    ((0x0001 << 14))
#define        BQ21080_DEV_SHIP_MODE                ((0x0001 << 13))
#define        BQ21080_DEV_SHUTDOWN_MODE 	    ((0x0001 << 12))


int8_t bq21080_set_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct charg_chip_dev *dev);
int8_t bq21080_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct charg_chip_dev *dev);

int8_t bq21080_init(struct charg_chip_dev *dev);
int8_t bq21080_set_sens_conf(struct charg_chip_dev *dev);
int8_t bq21080_set_mode(uint8_t mode, struct charg_chip_dev *dev);
int8_t bq21080_get_statflag(bq21080_statflag_t *p_statflag, const struct charg_chip_dev *dev);
int8_t bq21080_get_cur_charg_status(struct charg_chip_dev *dev, uint8_t *status);
int8_t bq21080_hardware_reset(struct charg_chip_dev *dev);
void bq21080_wdt_handle(struct charg_chip_dev *dev);
bool bq21080_ok(struct charg_chip_dev *dev);
void bq21080_show_status(const struct charg_chip_dev *dev);
void bq21080_get_status(const struct charg_chip_dev *dev, uint8_t *s0, uint8_t *s1, uint8_t *f0, uint8_t *i1);
int8_t bq21080_set_charge_intensity(uint8_t intensity, struct charg_chip_dev *dev);
int8_t bq21080_set_charge_enable(uint8_t enable, struct charg_chip_dev *dev);


#ifdef __cplusplus  
{
#endif

#endif
