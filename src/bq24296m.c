/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "bq24296m.h"
#include <stdio.h>
#include <Wire.h>

BQ24296M::BQ24296M(int8_t ce_n, int8_t int_n, int8_t psel, int8_t pg_n, int8_t stat, int8_t otg): 
    _psel(psel),  
    _pg_n(pg_n),  
    _stat(stat), 
    _int_n(int_n), 
    _otg(otg), 
    _ce_n(ce_n)
{ 
    DEBUG_PRINT("[BQ24296M] Call Contructor"); 
}
#if defined(ESP32) || defined(ESP8266)
BQ24296M::BQ24296M(int8_t sda, int8_t scl, int8_t ce_n, int8_t int_n, int8_t psel, int8_t pg_n, int8_t stat, int8_t otg): 
    _psel(psel),  
    _pg_n(pg_n),  
    _stat(stat), 
    _scl(scl), 
    _sda(sda), 
    _int_n(int_n), 
    _otg(otg), 
    _ce_n(ce_n)
{
    DEBUG_PRINT("[BQ24296M] Call Contructor"); 
}
#endif
BQ24296M::~BQ24296M() {
    
}

void BQ24296M::begin() {
    // Initialize I2C
    if(_sda != -1 && _scl != -1) {
        Wire.begin(_sda, _scl); 
    }
    else {
        Wire.begin(); 
    } 
    // Initialize GPIOs
    if(_psel != -1) {
        pinMode(_psel, OUTPUT); 
        digitalWrite(_psel, LOW); 
    }
    if(_pg_n != -1) {
        pinMode(_pg_n, INPUT_PULLUP); 
    }
    if(_stat != -1) {
        pinMode(_stat, INPUT_PULLUP); 
    }
    if(_int_n != -1) {
        pinMode(_int_n, INPUT_PULLUP); 
    }
    if(_otg != -1) {
        pinMode(_otg, OUTPUT); 
        digitalWrite(_otg, HIGH); 
    }
    if(_ce_n != -1) {
        pinMode(_ce_n, OUTPUT); 
        digitalWrite(_ce_n, LOW); 
    }
}

void BQ24296M::setInputControl(uint8_t voltage, uint8_t current) {
    INPUT_SRC_CTRL_REG reg; 
    reg.raw = readByte(INPUT_SRC_CTRL_ADDR); 
    reg.vindpm = voltage; 
    reg.iinlim = current; 
    writeByte(INPUT_SRC_CTRL_ADDR, reg.raw); 
}
void BQ24296M::setChargeEnable(bool enable) {
    POWER_ON_CONF_REG reg; 
    reg.raw = readByte(POWER_ON_CONF_ADDR); 
    reg.chg_config = enable; 
    writeByte(POWER_ON_CONF_ADDR, reg.raw); 
}
void BQ24296M::setMinimumSystemVoltage(uint8_t voltage) {
    POWER_ON_CONF_REG reg; 
    reg.raw = readByte(POWER_ON_CONF_ADDR); 
    reg.sys_min = voltage; 
    writeByte(POWER_ON_CONF_ADDR, reg.raw); 
}
void BQ24296M::setChargeCurrent(uint8_t current, uint8_t currentpre, bool force20pct) {
    CHG_CUR_CTRL_REG regA; 
    regA.raw = readByte(CHG_CUR_CTRL_ADDR); 
    regA.ichg = current; 
    regA.force_20pct = force20pct; 
    writeByte(CHG_CUR_CTRL_ADDR, regA.raw); 
    
    PRECHG_TERM_CUR_CTRL_REG regB; 
    regB.raw = readByte(PRECHG_TERM_CUR_CTRL_ADDR); 
    regB.iprechg = currentpre; 
    writeByte(PRECHG_TERM_CUR_CTRL_ADDR, regB.raw); 
}
void BQ24296M::setTerminationCurrent(uint8_t current) {
    PRECHG_TERM_CUR_CTRL_REG reg; 
    reg.raw = readByte(PRECHG_TERM_CUR_CTRL_ADDR); 
    reg.iterm = current; 
    writeByte(PRECHG_TERM_CUR_CTRL_ADDR, reg.raw); 
}
void BQ24296M::setChargeVoltage(uint8_t voltage) {
    CHG_VOLT_CTRL_REG reg; 
    reg.raw = readByte(CHG_VOLT_CTRL_ADDR); 
    reg.vreg = voltage; 
    writeByte(CHG_VOLT_CTRL_ADDR, reg.raw); 

}

uint8_t BQ24296M::getStatus() {
    return readByte(SYSTEM_STATUS_ADDR); 
}
uint8_t BQ24296M::getFaults() {
    return readByte(NEW_FAULT_ADDR); 
}

uint8_t BQ24296M::getRevision() {
    return readByte(VENDER_PART_REV_STATUS_ADDR); 
}



// Private Function
uint8_t BQ24296M::readByte(uint8_t addr) {
    uint8_t rdData; 
    uint8_t rdDataCount; 
    do {
        Wire.beginTransmission(BQ24296M_I2C_ADDR); 
        Wire.write(addr); 
        Wire.endTransmission(false); // Restart
        delay(10); 
        rdDataCount = Wire.requestFrom(BQ24296M_I2C_ADDR, 1); 
    } while(rdDataCount == 0); 
    while(Wire.available()) {
        rdData = Wire.read(); 
    }
    return rdData; 

}
void BQ24296M::writeByte(uint8_t addr, uint8_t data) {
    DEBUG_PRINTLN("")
    DEBUG_PRINT("writeI2C reg 0x")
    DEBUG_PRINT(addr, HEX)
    DEBUG_PRINT(" -> 0x") DEBUG_PRINTLN(data, HEX)
	
	Wire.beginTransmission(BQ24296M_I2C_ADDR); 
    Wire.write(addr); 
    Wire.write(data); 
    Wire.endTransmission(); 
}

/* [] END OF FILE */
