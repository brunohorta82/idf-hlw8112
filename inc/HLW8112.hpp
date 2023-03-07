#pragma once
#include "freertos/Freertos.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#define REG_SYSCON_ADDR 0x00
#define REG_EMUCON_ADDR 0x01
#define REG_HFCONST_ADDR 0x02
#define REG_EMUCON2_ADDR 0x13
#define REG_ANGLE_ADDR 0x22
#define REG_UFREQ_ADDR 0x23
#define REG_RMSIA_ADDR 0x24
#define REG_RMSIB_ADDR 0x25
#define REG_RMSU_ADDR 0x26
#define REG_PF_ADDR 0x27
#define REG_ENERGY_PA_ADDR 0x28
#define REG_ENERGY_PB_ADDR 0x29
#define REG_POWER_PA_ADDR 0x2C
#define REG_POWER_PB_ADDR 0x2D

#define REG_SAGCYC_ADDR 0x17
#define REG_SAGLVL_ADDR 0x18
#define REG_OVLVL_ADDR 0x19

#define REG_INT_ADDR 0x1D
#define REG_IE_ADDR 0x40
#define REG_IF_ADDR 0x41
#define REG_RIF_ADDR 0x42

#define REG_RDATA_ADDR 0x44

#define REG_CHECKSUM_ADDR 0x6f
#define REG_RMS_IAC_ADDR 0x70
#define REG_RMS_IBC_ADDR 0x71
#define REG_RMS_UC_ADDR 0x72
#define REG_POWER_PAC_ADDR 0x73
#define REG_POWER_PBC_ADDR 0x74
#define REG_POWER_SC_ADDR 0x75
#define REG_ENERGY_AC_ADDR 0x76
#define REG_ENERGY_BC_ADDR 0x77

#define D_TIME1_50MS 50
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define VOLTAGE_DIVIDER 1880000         // Voltage divider Upstream resistors 470K*4  1880K
#define VOLTAGE_DIVIDER_DOWNSTREAM 1000 // Voltage divider downstream resistors  1K
#define SHUNT_RESISTOR 0.1
#define PW_METRICS_PUBLISH_TIME_MS 2000
#define SYNC_TIME_MS 1000
class PowerReadings
{
public:
    int meterId;
    float voltage;
    float current;
    float power;
    time_t timestamp;
};

void Init_HLW8112();
unsigned char HLW8112_checkSum_Read(unsigned char u8_Reg_length);
int Calculate_HLW8112_MeterData(struct PowerReadings *rA, struct PowerReadings *rB);
