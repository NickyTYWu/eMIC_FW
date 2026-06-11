#include "main.h"
#include "FIT_DebugMessage.h"
#include "FIT_sensirion_hw_i2c_implementation.h"
#include "Command.h"
#include "FIT_SBM100.h"
#include "FIT_SysTick.h"
#include "FIT_M24C08_EE.h"

bool bSBM100Exist=false;

const uint8_t SBM100_REG_Addr[18]={
		0x01,
		0x20,
		0x01,
		0x30,
		0x01,
		0x31,
		0x01,
		0x32,
		0x01,
		0x33,
		0x01,
		0x34,
		0x01,
		0x35,
		0x01,
		0x36,
		0x01,
		0x80,
};

uint8_t SBM100_REG[18]={
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
	    0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x20,
};

void getSBM100REGFromRAM(uint8_t *reg)
{
    memcpy(reg,SBM100_REG,18);
}

void dumpSBM100REG()
{
    if(!isSBM100Exist())
        return;

    for(int i=0;i<18;i+=2)
    {
        uint8_t writeReg[2];

        writeReg[0]=SBM100_REG_Addr[i];
        writeReg[1]=SBM100_REG_Addr[i+1];

        sensirion_i2c_write(SBM100_SLAVE_ADDR, writeReg,2) ;

        sensirion_i2c_read(SBM100_SLAVE_ADDR, &SBM100_REG[i], 2);
    }
}

bool setSBM100REG(uint8_t *reg)
{
    if(!isSBM100Exist())
        return false;

    for(int i=0;i<18;i+=2)
    {
        uint8_t writeReg[4];

        writeReg[0]=SBM100_REG_Addr[i];
        writeReg[1]=SBM100_REG_Addr[i+1];
        writeReg[2]=reg[i];
        writeReg[3]=reg[i+1];

        if(sensirion_i2c_write(SBM100_SLAVE_ADDR, writeReg,4)!=0)
        {
            bSBM100Exist=I2C2_LL_IsDeviceReady(SBM100_SLAVE_ADDR, 10);
            return false;
        }
    }

    return true;
}

bool isSBM100Exist()
{
    return bSBM100Exist;
}

void checkSBM100isReady()
{
    if(!I2C2_LL_IsDeviceReady(SBM100_SLAVE_ADDR, 10))
    {
        bSBM100Exist=false;
        FT_printf("SBM100 is not exist!!");
    }
    else
    {
        bSBM100Exist=true;;
    }
}

void initSBM100()
{
    if(!I2C2_LL_IsDeviceReady(SBM100_SLAVE_ADDR, 10))
    {
        bSBM100Exist=false;
        FT_printf("SBM100 is not exist!!");
    }
    else
    {
        bSBM100Exist=true;

        uint8_t tempRegbuf[19];

        if(readSBM100AllReg(tempRegbuf)==true)
        {
            if(tempRegbuf[18]==0xAA)
            {
                FT_printf("dumpSBM100 Reg:");
                for(int i=0;i<18;i+=1)
                {
                    FT_printf("%x,",i,tempRegbuf[i]);
                }
                FT_printf("\r\n");

                memcpy(SBM100_REG,tempRegbuf,18);

                for(int i=0;i<18;i+=2)
                {
                    uint8_t writeReg[4];

                    writeReg[0]=SBM100_REG_Addr[i];
                    writeReg[1]=SBM100_REG_Addr[i+1];
                    writeReg[2]=SBM100_REG[i];
                    writeReg[3]=SBM100_REG[i+1];

                    sensirion_i2c_write(SBM100_SLAVE_ADDR, writeReg,4) ;
                }
            }
        }
    }
	//dumpSBM100REG();
}
