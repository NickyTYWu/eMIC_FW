#ifndef SRC_FIT_HEADER_FIT_SBM100_EE_H_
#define SRC_FIT_HEADER_FIT_SBM100_EE_H_

#define SBM100_SLAVE_ADDR 0x10

void initSBM100();
bool setSBM100REG(uint8_t *reg);
void dumpSBM100REG();
void getSBM100REGFromRAM(uint8_t *reg);
bool isSBM100Exist();
void checkSBM100isReady();
#endif /* SRC_FIT_HEADER_FIT_SBM100_EE_H_ */
