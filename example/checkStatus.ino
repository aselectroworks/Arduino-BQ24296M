#include "bq24296m.h"
#include <stdio.h>

#define I2C_SDA 22
#define I2C_SCL 23
#define RST_N_PIN 21
#define INT_N_PIN 34
#define PGOOD_N_PIN 39

BQ24296M bq24296m(I2C_SDA, I2C_SCL, /*ce_n=*/-1, /*int_n=*/-1, /*psel=*/-1, /*pg_n=*/PGOOD_N_PIN, /*stat=*/-1, /*otg=*/-1); 

void setup() {
    Serial.begin(115200); 

    bq24296m.begin(); 

    VENDER_PART_REV_STATUS_REG rev_reg; 
    rev_reg.raw = bq24296m.getRevision(); 
    DEBUG_PRINTLN("")
    DEBUG_PRINT("PN : 0x") DEBUG_PRINTLN(rev_reg.pn, HEX)
    DEBUG_PRINT("REV: 0x") DEBUG_PRINTLN(rev_reg.rev, HEX)
}

bool charge_enable = false; 
void loop() {

    // Check Status Registor
    SYSTEM_STATUS_REG status; 
    status.raw = bq24296m.getStatus(); 
    Serial.print(" VBUS_STAT : 0x"); Serial.print(status.vbus_stat, HEX); 
    Serial.print(" CHRG_STAT : 0x"); Serial.print(status.chrg_stat, HEX); 
    Serial.print(" DPM_STAT  : 0x"); Serial.print(status.chrg_stat, HEX); 
    Serial.print(" PG_STAT   : 0x"); Serial.print(status.pg_stat, HEX); 
    Serial.print(" THERM_STAT: 0x"); Serial.print(status.therm_stat, HEX); 
    Serial.print(" VSYS_STAT : 0x"); Serial.println(status.vsys_stat, HEX); 

    delay(1000); 

    // Toggle Charge Enable
    Serial.print(" Set Charge Enable: "); Serial.println(charge_enable, HEX); 
    bq24296m.setChargeEnable(charge_enable); 
    charge_enable = (charge_enable) ? false : true; 
}
