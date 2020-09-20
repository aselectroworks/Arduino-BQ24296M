/**************************************************************************/
/*!
  @file     bq24296m.h
  Author: Atsushi Sasaki(https://github.com/aselectroworks)
  License: MIT (see LICENSE)
*/
/**************************************************************************/

#ifndef _BQ24296M_H
#define _BQ24296M_H
    
#include <stdint.h>
#include <stdbool.h>
    
#define BQ24296M_I2C_ADDR       0x6B // 7-bits address

typedef enum {
    INPUT_SRC_CTRL_ADDR = 0, 
    POWER_ON_CONF_ADDR, 
    CHG_CUR_CTRL_ADDR, 
    PRECHG_TERM_CUR_CTRL_ADDR, 
    CHG_VOLT_CTRL_ADDR, 
    CHGTERM_TIMER_CTRL_ADDR, 
    BSTVOLT_THERMREG_CTRL_ADDR, 
    MISC_OP_CTRL_ADDR, 
    SYSTEM_STATUS_ADDR, 
    NEW_FAULT_ADDR, 
    VENDER_PART_REV_STATUS_ADDR, 
} reg_addr_enum; 
typedef union {
    uint8_t raw; 
    struct {
        uint8_t iinlim :3; 
        uint8_t vindpm :4; 
        bool en_hiz: 1; 
    }; 
} INPUT_SRC_CTRL_REG; 
// VINDPM Value
#define VINDPM_3880mV_OFFSET    0b0000
#define VINDPM_80mV             0b0001
#define VINDPM_160mV            0b0010
#define VINDPM_320mV            0b0100
#define VINDPM_6400mV           0b1000
// IINLIM Value
#define IINLIM_100mA_OFFSET     0b000
#define IINLIM_150mA            0b001
#define IINLIM_500mA            0b010
#define IINLIM_900mA            0b011
#define IINLIM_1000mA           0b100
#define IINLIM_1500mA           0b101
#define IINLIM_2000mA           0b110
#define IINLIM_3000mA           0b111
typedef union {
    uint8_t raw; 
    struct {
        uint8_t boost_lim: 1; 
        uint8_t sys_min: 3; 
        bool chg_config: 1; 
        bool otg_config: 1; 
        bool wdt_reset: 1; 
        bool reg_reset: 1; 
    }; 
} POWER_ON_CONF_REG; 
// SYS_MIN Value
#define SYS_MIN_VOLT_3000mV_OFFSET  0b000
#define SYS_MIN_VOLT_100mV          0b001
#define SYS_MIN_VOLT_200mV          0b010
#define SYS_MIN_VOLT_400mV          0b100
// BOOST_LIM Value
#define BOOST_LIM_1000mA    0
#define BOOST_LIM_1500mA    1
typedef union {
    uint8_t raw; 
    struct {
        bool force_20pct: 1; 
        uint8_t bcold: 1; 
        uint8_t ichg: 6; 
    }; 
} CHG_CUR_CTRL_REG; 
// ICHG Value
#define ICHG_512mA_OFFSET   0b000000
#define ICHG_64mA           0b000001
#define ICHG_128mA          0b000010
#define ICHG_256mA          0b000100
#define ICHG_512mA          0b001000
#define ICHG_1024mA         0b010000
#define ICHG_2048mA         0b100000
// BCOLD Value
#define BCOLD_76PERCENT_REGN    0
#define BCOLD_79PERCENT_REGN    1
typedef union {
    uint8_t raw; 
    struct {
        uint8_t iterm: 3; 
        uint8_t rsv: 1; 
        uint8_t iprechg: 4; 
    }; 
} PRECHG_TERM_CUR_CTRL_REG; 
// IPRECHG Value
#define IPRECHG_128mA           0b0001
#define IPRECHG_256mA           0b0010
#define IPRECHG_384mA           0b0011
#define IPRECHG_512mA           0b0100
#define IPRECHG_768mA           0b0101
#define IPRECHG_896mA           0b0110
#define IPRECHG_1024mA          0b0111
#define IPRECHG_1152mA          0b1000
#define IPRECHG_1280mA          0b1001
#define IPRECHG_1408mA          0b1010
#define IPRECHG_1536mA          0b1011
#define IPRECHG_1664mA          0b1100
#define IPRECHG_1792mA          0b1101
#define IPRECHG_1920mA          0b1110
#define IPRECHG_2048mA          0b1111
// ITERM Value
#define ITERM_128mA_OFFSET  0b000
#define ITERM_128mA         0b001
#define ITERM_256mA         0b010
#define ITERM_512mA         0b100
typedef union {
    uint8_t raw; 
    struct {
        uint8_t vrechg: 1; 
        uint8_t batlowv: 1; 
        uint8_t vreg: 6; 
    }; 
} CHG_VOLT_CTRL_REG; 
// VREG Value
#define VREG_3504mV_OFFSET  0b000000
#define VREG_16mV           0b000001
#define VREG_32mV           0b000010
#define VREG_64mV           0b000100
#define VREG_128mV          0b001000
#define VREG_256mV          0b010000
#define VREG_512mV          0b100000
// BATLOWV Value
#define BATLOWV_2800mV  0
#define BATLOWV_3000mV  1
// VRECHG Value
#define VRECHG_100mV    0
#define VRECHG_300mV    1
typedef union {
    uint8_t raw; 
    struct {
        uint8_t rsv0: 1; 
        uint8_t chg_timer: 2; 
        bool en_timer: 1; 
        uint8_t watchdog: 2; 
        uint8_t rsv1: 1; 
        uint8_t en_term: 1; 
    }; 
} CHGTERM_TIMER_CTRL_REG; 
// WATCHDOG Value
#define WATCHDOG_DISABLE    0b00
#define WATCHDOG_8HRS       0b01
#define WATCHDOG_12HRS      0b10
#define WATCHDOG_20HRS      0b11
typedef union {
    uint8_t raw; 
    struct { 
        uint8_t treg: 2; 
        uint8_t bhot: 2; 
        uint8_t boostv: 4; 
    }; 
} BSTVOLT_THERMREG_CTRL_REG; 
// BOOSTV Value
#define BOOSTV_4550mV_OFFSET    0b0000
#define BOOSTV_64mV             0b0001
#define BOOSTV_128mV            0b0010
#define BOOSTV_256mV            0b0100
#define BOOSTV_512mV            0b1000
// BHOT Value
#define BHOT_33PERCENT_REGN     0b00
#define BHOT_36PERCENT_REGN     0b01
#define BHOT_30PERCENT_REGN     0b10
#define BHOT_DISABLE            0b11
// TREG Value
#define TREG_60DEG      0b00
#define TREG_80DEG      0b01
#define TREG_100DEG     0b10
#define TREG_120DEG     0b11
typedef union {
    uint8_t raw; 
    struct { 
        uint8_t int_mask: 2; 
        uint8_t rsv: 3; 
        bool batfet_disable: 1; 
        bool tmr2x_en: 1; 
        bool dpdm_en: 1; 
    }; 
} MISC_OP_CTRL_REG; 
// INT_MASK Value
#define INT_MASK_NOTHING        0b00
#define INT_MASK_CHRG_FAULT     0b01
#define INT_MASK_BAT_FAULT      0b10
typedef union {
    uint8_t raw; 
    struct { 
        bool vsys_stat: 1; 
        bool therm_stat: 1; 
        bool pg_stat: 1; 
        bool dpm_stat: 1; 
        uint8_t chrg_stat: 2; 
        uint8_t vbus_stat: 2; 
    }; 
} SYSTEM_STATUS_REG; 
// VBUS_STAT Value
#define VBUS_STAT_UNKNOWN   0b00
#define VBUS_STAT_USB_HOST  0b01
#define VBUS_STAT_ADAPTER   0b10
#define VBUS_STAT_USB_OTG   0b11
// CHRG_STAT Value
#define CHRG_STAT_NOT_CHARGING  0b00
#define CHRG_STAT_PRE_CHARGING  0b01
#define CHRG_STAT_FAST_CHARGING 0b10
#define CHRG_STAT_CHARGE_DONE   0b11
typedef union {
    uint8_t raw; 
    struct { 
        uint8_t ntc_fault: 2; 
        uint8_t rsv: 1; 
        bool bat_fault: 1; 
        uint8_t chrg_fault: 2; 
        bool otg_fault: 1; 
        bool watchdog_fault: 1; 
    }; 
} NEW_FAULT_REG; 
// CHRG_FAULT Value
#define CHRG_FAULT_NORMAL   0b00
#define CHRG_FAULT_INPUT    0b01
#define CHRG_FAULT_THERMAL  0b10
#define CHRG_FAULT_TIMER    0b11
// NTC_FAULT Value
#define NTC_FAULT_NORMAL    0b00
#define NTC_FAULT_COLD      0b01
#define NTC_FAULT_HOT       0b10
typedef union {
    uint8_t raw; 
    struct { 
        uint8_t rev: 3; 
        uint8_t reserverd: 2; 
        uint8_t pn: 3; 
    }; 
} VENDER_PART_REV_STATUS_REG; 

// Uncomment to enable debug messages
//#define BQ24296M_DEBUG

// Define where debug output will be printed
#define DEBUG_PRINTER Serial

// Setup debug printing macros
#ifdef BQ24296M_DEBUG
#define DEBUG_PRINT(...)                  \
    {                                     \
        DEBUG_PRINTER.print(__VA_ARGS__); \
    }
#define DEBUG_PRINTLN(...)                  \
    {                                       \
        DEBUG_PRINTER.println(__VA_ARGS__); \
    }
#else
#define DEBUG_PRINT(...) \
    {                    \
    }
#define DEBUG_PRINTLN(...) \
    {                      \
    }
#endif

/**************************************************************************/
/*!
    @brief  BQ24296M LiPo Charger driver
*/
/**************************************************************************/
class BQ24296M
{
public: 
    BQ24296M(int8_t ce_n, int8_t int_n, int8_t psel, int8_t pg_n, int8_t stat, int8_t otg); 
#ifdef ESP32 || ESP8266
    BQ24296M(int8_t sda, int8_t scl, int8_t ce_n, int8_t int_n, int8_t psel, int8_t pg_n, int8_t stat, int8_t otg); 
#endif
    virtual ~BQ24296M(); 

    void begin(void);
    
    void setInputControl(uint8_t voltage, uint8_t current); 
    void setChargeEnable(bool enable); 
    void setMinimumSystemVoltage(uint8_t voltage); 
    void setChargeCurrent(uint8_t current, uint8_t currentpre, bool force20pct); 
    void setTerminationCurrent(uint8_t current); 
    void setChargeVoltage(uint8_t voltage); 

    uint8_t getStatus(); 
    uint8_t getFaults(); 

    uint8_t getRevision(); 

private: 
    int8_t _psel = -1; 
    int8_t _pg_n = -1; 
    int8_t _stat = -1; 
    int8_t _scl = -1; 
    int8_t _sda = -1; 
    int8_t _int_n = -1; 
    int8_t _otg = -1; 
    int8_t _ce_n = -1; 

    uint8_t readByte(uint8_t addr); 
    uint8_t readMultiByte(uint8_t addr, uint8_t size); 
    void writeByte(uint8_t addr, uint8_t data); 
    void writeMultiByte(uint8_t addr, uint8_t* data, uint8_t size); 
};

#endif
/* [] END OF FILE */
