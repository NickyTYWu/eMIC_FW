#include "main.h"
#include "FIT_DebugMessage.h"
#include "FIT_sensirion_hw_i2c_implementation.h"
#include "Command.h"
#include "FIT_DPS368.h"
#include "FIT_SysTick.h"

bool bDPS368Exist=false;
uint8_t coeffData[19];
uint8_t measureCMD;
bool bStartMeasureDPS368=false;
bool bWaitRdy=false;
uint32_t measureTempBlockingTime;
uint32_t measurePressureBlockingTime;
uint32_t measureStartTickCount;
uint8_t tempResultBuf[3];
uint8_t pressureResultBuf[3];
uint8_t tempOversample;
uint8_t pressureOversample;

bool isDPS368Exist()
{
    return bDPS368Exist;
}

void readTMP_COEF_SRCE()
{
    uint8_t reg[1];
    uint8_t TMP_COEF_SRCE[1];

    reg[0]=0x28;
    sensirion_i2c_write(DPS368_SLAVE_ADDR, reg,1);

    sensirion_i2c_read(DPS368_SLAVE_ADDR, TMP_COEF_SRCE, 1) ;
    FT_printf("TMP_COEF_SRCE:%x,",TMP_COEF_SRCE[0]);
}

void readProductID()
{
    uint8_t reg[1];
    uint8_t productIDBuf[1];

    reg[0]=0x0D;
    sensirion_i2c_write(DPS368_SLAVE_ADDR, reg,1);

    sensirion_i2c_read(DPS368_SLAVE_ADDR, productIDBuf, 1) ;
    FT_printf("ProductID:%x,",productIDBuf[0]);
}

void readCoeff()
{
    uint8_t reg[1];

    FT_printf("Coeff:");

    coeffData[0]=0x01;

    for(int i=0;i<18;i++)
    {

        reg[0]=0x10+i;
        sensirion_i2c_write(DPS368_SLAVE_ADDR, reg,1);

        sensirion_i2c_read(DPS368_SLAVE_ADDR, &coeffData[i+1], 1) ;
        FT_printf("%x,",coeffData[i+1]);
    }

    FT_printf("\r\n");
}

uint8_t getMeasureBlockingTime(uint8_t OSR)
{
    uint8_t blockingTime=4;
    if(OSR==0)
    {
        blockingTime=4;
    }
    else if(OSR==1)
    {
        blockingTime=6;
    }
    else if(OSR==2)
    {
        blockingTime=9;
    }
    else if(OSR==3)
    {
        blockingTime=15;
    }
    else if(OSR==4)
    {
        blockingTime=28;
    }
    else if(OSR==5)
    {
        blockingTime=54;
    }
    else if(OSR==6)
    {
        blockingTime=105;
    }
    else if(OSR==7)
    {
        blockingTime=207;
    }

    return blockingTime;
}

void startMeasureDPS368(uint8_t tempOSR,uint8_t pressureOSR)
{

    if(bStartMeasureDPS368||!bDPS368Exist)
        return;

    uint8_t reg[2];

    reg[0]=TMP_CFG;
    reg[1]=tempOSR|0x80;

    sensirion_i2c_write(DPS368_SLAVE_ADDR, reg,2);

    reg[0]=PRS_CFG;
    reg[1]=pressureOSR;

    sensirion_i2c_write(DPS368_SLAVE_ADDR, reg,2);

    reg[0]=CFG_REG;
    reg[1]=0;

    if(tempOSR>=4)
    {
        reg[1]|=0x08;
    }

    if(pressureOSR>=4)
    {
        reg[1]|=0x04;
    }

    sensirion_i2c_write(DPS368_SLAVE_ADDR, reg,2);

    bStartMeasureDPS368=true;
    measureCMD=TEMP_MEASURE;

    bWaitRdy=false;

    tempOversample=tempOSR;
    pressureOversample=pressureOSR;

    measureTempBlockingTime=getMeasureBlockingTime(tempOSR);
    measurePressureBlockingTime=getMeasureBlockingTime(pressureOSR);
}

uint8_t getRDY()
{
    uint8_t reg[1];
    uint8_t result[1];

    reg[0]=MEAS_CFG;
    sensirion_i2c_write(DPS368_SLAVE_ADDR, reg,1);

    sensirion_i2c_read(DPS368_SLAVE_ADDR, result, 1) ;

    return result[0];
}

void getTempResult()
{
    uint8_t reg[1];

    for(int i=0;i<3;i++)
    {
        reg[0]=TMP_B2+i;
        sensirion_i2c_write(DPS368_SLAVE_ADDR, reg,1);

        sensirion_i2c_read(DPS368_SLAVE_ADDR, &tempResultBuf[i], 1) ;
    }
}

void getPressureResult()
{
    uint8_t reg[1];

    for(int i=0;i<3;i++)
    {
        reg[0]=PSR_B2+i;
        sensirion_i2c_write(DPS368_SLAVE_ADDR, reg,1);

        sensirion_i2c_read(DPS368_SLAVE_ADDR, &pressureResultBuf[i], 1) ;
    }
}

void processDPS368()
{
    if(bStartMeasureDPS368)
    {
        if(!bWaitRdy)
        {
            uint8_t reg[2];

            reg[0]=MEAS_CFG;
            reg[1]=measureCMD;

            sensirion_i2c_write(DPS368_SLAVE_ADDR, reg,2);

            measureStartTickCount=HAL_GetTick();

            bWaitRdy=true;
        }
        else
        {
            if(measureCMD==TEMP_MEASURE)
            {
                if(HAL_GetTick()-measureStartTickCount>=measureTempBlockingTime)
                {
                    if(getRDY()|0x20)
                    {
                        getTempResult();
                        bWaitRdy=false;
                        measureCMD=PRESSURE_MEASURE;
                    }
                }
            }
            else if(measureCMD==PRESSURE_MEASURE)
            {
                if(HAL_GetTick()-measureStartTickCount>=measurePressureBlockingTime)
                {
                    if(getRDY()|0x10)
                    {
                        getPressureResult();
                        measureCMD=0;
                        bWaitRdy=false;
                        bStartMeasureDPS368=false;
                        responseReadDPS368TempPressure(tempResultBuf,pressureResultBuf,tempOversample,pressureOversample);
                    }
                }
            }
        }
    }
}


void initDPS368()
{
    if(!I2C2_LL_IsDeviceReady(DPS368_SLAVE_ADDR,10))
    {
        bDPS368Exist=false;
        coeffData[0]=0x00;
        FT_printf("DPS368 is not exist!!");
    }
    else
    {
        bDPS368Exist=true;
        readTMP_COEF_SRCE();
        readProductID();
        readCoeff();
    }
}

