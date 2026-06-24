#ifndef SRC_FIT_HEADER_FIT_DPS368_EE_H_
#define SRC_FIT_HEADER_FIT_DPS368_EE_H_


#define DPS368_SLAVE_ADDR 0x77

#define PSR_B2   0x00
#define PSR_B1   0x01
#define PSR_B0   0x02
#define TMP_B2   0x03
#define TMP_B1   0x04
#define TMP_B0   0x05
#define PRS_CFG  0x06
#define TMP_CFG  0x07
#define MEAS_CFG 0x08
#define CFG_REG  0x09

#define PRESSURE_MEASURE 0x01
#define TEMP_MEASURE     0x02


void initDPS368();
void readCoeff();
void startMeasureDPS368(uint8_t tempOSR,uint8_t pressureOSR);
void processDPS368();
void getPressureResult();
void getTempResult();
uint8_t getRDY();
uint8_t getMeasureBlockingTime(uint8_t OSR);
void checkDPS368isReady();
#endif /* SRC_FIT_HEADER_FIT_DPS368_EE_H_ */
