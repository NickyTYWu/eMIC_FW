/*
 * FIT_M24C08_EE.c
 *
 *  Created on: May 19, 2025
 *      Author: fit0354
 */
#include "main.h"
#include "FIT_PCMD3140.h"
#include "FIT_M24C08_EE.h"
#include "FIT_eeprom_pcmd3140_hw_i2c_implemention.h"
#include "FIT_DebugMessage.h"
#include "FIT_sensirion_hw_i2c_implementation.h"
#include "Command.h"

int8_t FIT_EE_Read(uint16_t address,uint8_t *data,uint16_t size)
{
	uint8_t wbuf[1];
	uint8_t slaveAddr=(EE_BASE_SLAVE_ADDR|(address&0x300)>>7);
	int8_t result=0;

	LL_GPIO_ResetOutputPin(WC_GPIO_Port, WC_Pin);

	wbuf[0]=address&0xff;

	result=eeprom_pcmd3140_i2c_write(slaveAddr,wbuf,1);
	LL_mDelay(1);

	if(result!=0)
	{
		FT_printf("FIT_EE_Read write address fail\r\n");
		return result;
	}

	result=eeprom_pcmd3140_i2c_read(slaveAddr,&data[0],size);
	LL_mDelay(1);

	if(result!=0)
	{
		FT_printf("FIT_EE_Read read fail===\r\n");
		return result;
	}
	LL_GPIO_SetOutputPin(WC_GPIO_Port, WC_Pin);
	return result;
}

int8_t FIT_EE_Write(uint16_t address,uint8_t *data,uint16_t size)
{
	uint8_t wbuf[size+1];

	uint8_t slaveAddr=(EE_BASE_SLAVE_ADDR|(address&0x300)>>7);
	int8_t result=0;

	LL_GPIO_ResetOutputPin(WC_GPIO_Port, WC_Pin);
	wbuf[0]=address&0xff;
	memcpy(&wbuf[1],data,size);

	result=eeprom_pcmd3140_i2c_write(slaveAddr,wbuf,size+1);

	if(result!=0)
	{
		FT_printf("FIT_EE_Write fail\r\n");
		return result;
	}

	LL_mDelay(1);
	LL_GPIO_SetOutputPin(WC_GPIO_Port, WC_Pin);
	return result;
}

bool writeEE(uint16_t addr,uint8_t *data,uint8_t size)
{
	uint16_t writeAddress=addr;
	uint16_t len = size;

	while (len > 0) {
		uint8_t offset = writeAddress % 128;

		uint8_t page_space = 16 - (offset % 16);
		uint8_t write_len = (len < page_space) ? len : page_space;

		if(FIT_EE_Write(writeAddress,data,write_len)!=0)
				return false;

		writeAddress += write_len;
		data += write_len;
		len -= write_len;
	}

	return true;
}

bool writeModelNumber(uint8_t infoID,uint8_t *data,uint8_t size)
{

	uint16_t writeAddress;
	bool result=true;

	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_MODE_NUMBER_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_MODE_NUMBER_ADDR;
	}
	else
	{
		return false;
	}

	result=writeEE(writeAddress,data,size);

	return result;
}



void readModelNumber(uint8_t infoID,uint8_t *data)
{

	uint16_t writeAddress=CARTRIDGE1_MODE_NUMBER_ADDR;
	uint8_t tempBuf[CARTRIDGE1_MODE_NUMBER_SIZE];


	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_MODE_NUMBER_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_MODE_NUMBER_ADDR;
	}

	FIT_EE_Read(writeAddress,tempBuf,CARTRIDGE1_MODE_NUMBER_SIZE);

	memcpy(data,tempBuf,CARTRIDGE1_MODE_NUMBER_SIZE);
}

bool writeVersionLetter(uint8_t infoID,uint8_t *data,uint8_t size)
{

	uint16_t writeAddress;
    bool result=true;

	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_VERSION_LETTER_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_VERSION_LETTER_ADDR;
	}
	else
	{
		return false;
	}

	result=writeEE(writeAddress,data,size);

	return result;
}

void readVersionLetter(uint8_t infoID,uint8_t *data)
{

	uint16_t writeAddress=CARTRIDGE1_VERSION_LETTER_ADDR;
	uint8_t tempBuf[CARTRIDGE1_VERSION_LETTER_SIZE];


	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_VERSION_LETTER_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_VERSION_LETTER_ADDR;
	}

	FIT_EE_Read(writeAddress,tempBuf,CARTRIDGE1_VERSION_LETTER_SIZE);

	memcpy(data,tempBuf,CARTRIDGE1_VERSION_LETTER_SIZE);
}

bool writeSerialNumber(uint8_t infoID,uint8_t *data,uint8_t size)
{

	uint16_t writeAddress;
	bool result=true;

	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_SERIAL_NUMBER_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_SERIAL_NUMBER_ADDR;
	}
	else if(infoID==INFOR_ID_SYSTEM)
	{
		writeAddress=SYSTEM_SERIAL_NUMBER_ADDR;
	}
	else
	{
		return false;
	}

	result=writeEE(writeAddress,data,size);

	return result;
}

void readSerialNumber(uint8_t infoID,uint8_t *data)
{

	uint16_t writeAddress=CARTRIDGE1_SERIAL_NUMBER_ADDR;
	uint8_t tempBuf[CARTRIDGE1_SERIAL_NUMBER_SIZE];


	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_SERIAL_NUMBER_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_SERIAL_NUMBER_ADDR;
	}
	else if(infoID==INFOR_ID_SYSTEM)
	{
		writeAddress=SYSTEM_SERIAL_NUMBER_ADDR;
	}

	FIT_EE_Read(writeAddress,tempBuf,CARTRIDGE1_SERIAL_NUMBER_SIZE);

	memcpy(data,tempBuf,CARTRIDGE1_SERIAL_NUMBER_SIZE);
}

bool writeSensitivity(uint8_t infoID,uint8_t *data,uint8_t size)
{

	uint16_t writeAddress;
	bool result=true;

	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_SENSITIVITY_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_SENSITIVITY_ADDR;
	}
	else if(infoID==INFOR_ID_SYSTEM)
	{
		writeAddress=SYSTEM_SENSITIVITY_ADDR;
	}
	else
	{
		return false;
	}

	result=writeEE(writeAddress,data,size);

	return result;
}

void readSensitivity(uint8_t infoID,uint8_t *data)
{

	uint16_t writeAddress=CARTRIDGE1_SENSITIVITY_ADDR;
	uint8_t tempBuf[CARTRIDGE1_SENSITIVITY_SIZE];


	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_SENSITIVITY_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_SENSITIVITY_ADDR;
	}
	else if(infoID==INFOR_ID_SYSTEM)
	{
		writeAddress=SYSTEM_SENSITIVITY_ADDR;
	}

	FIT_EE_Read(writeAddress,tempBuf,CARTRIDGE1_SENSITIVITY_SIZE);

	memcpy(data,tempBuf,CARTRIDGE1_SENSITIVITY_SIZE);
}

bool writeReferenceFrequency(uint8_t infoID,uint8_t *data,uint8_t size)
{

	uint16_t writeAddress;
	bool result=true;

	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_REFERENCE_FREQUENCY_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_REFERENCE_FREQUENCY_ADDR;
	}
	else
	{
		return false;
	}

	result=writeEE(writeAddress,data,size);

	return result;
}

void readReferenceFrequency(uint8_t infoID,uint8_t *data)
{

	uint16_t writeAddress=CARTRIDGE1_REFERENCE_FREQUENCY_ADDR;
	uint8_t tempBuf[CARTRIDGE1_REFERENCE_FREQUENCY_SIZE];


	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_REFERENCE_FREQUENCY_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_REFERENCE_FREQUENCY_ADDR;
	}

	FIT_EE_Read(writeAddress,tempBuf,CARTRIDGE1_REFERENCE_FREQUENCY_SIZE);

	memcpy(data,tempBuf,CARTRIDGE1_REFERENCE_FREQUENCY_SIZE);
}

bool writeUnitsCode(uint8_t infoID,uint8_t *data,uint8_t size)
{

	uint16_t writeAddress;
	bool result=true;

	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_UNITS_CODE_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_UNITS_CODE_ADDR;
	}
	else
	{
		return false;
	}

	result=writeEE(writeAddress,data,size);

	return result;
}

void readUnitsCode(uint8_t infoID,uint8_t *data)
{

	uint16_t writeAddress=CARTRIDGE1_UNITS_CODE_ADDR;
	uint8_t tempBuf[CARTRIDGE1_UNITS_CODE_SIZE];


	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_UNITS_CODE_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_UNITS_CODE_ADDR;
	}

	FIT_EE_Read(writeAddress,tempBuf,CARTRIDGE1_UNITS_CODE_SIZE);

	memcpy(data,tempBuf,CARTRIDGE1_UNITS_CODE_SIZE);
}

bool writeFrequencyRangeMin(uint8_t infoID,uint8_t *data,uint8_t size)
{

	uint16_t writeAddress;
	bool result=true;

	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_FREQUENCY_RANG_MIN_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_FREQUENCY_RANG_MIN_ADDR;
	}
	else
	{
		return false;
	}

	result=writeEE(writeAddress,data,size);

	return result;
}

void readFrequencyRangeMin(uint8_t infoID,uint8_t *data)
{

	uint16_t writeAddress=CARTRIDGE1_FREQUENCY_RANG_MIN_ADDR;
	uint8_t tempBuf[CARTRIDGE1_FREQUENCY_RANG_MIN_SIZE];


	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_FREQUENCY_RANG_MIN_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_FREQUENCY_RANG_MIN_ADDR;
	}

	FIT_EE_Read(writeAddress,tempBuf,CARTRIDGE1_FREQUENCY_RANG_MIN_SIZE);

	memcpy(data,tempBuf,CARTRIDGE1_FREQUENCY_RANG_MIN_SIZE);
}

bool writeFrequencyRangeMax(uint8_t infoID,uint8_t *data,uint8_t size)
{

	uint16_t writeAddress;
	bool result=true;

	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_FREQUENCY_RANG_MAX_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_FREQUENCY_RANG_MAX_ADDR;
	}
	else
	{
		return false;
	}

	result=writeEE(writeAddress,data,size);

	return result;
}

void readFrequencyRangeMax(uint8_t infoID,uint8_t *data)
{

	uint16_t writeAddress=CARTRIDGE1_FREQUENCY_RANG_MAX_ADDR;
	uint8_t tempBuf[CARTRIDGE1_FREQUENCY_RANG_MAX_SIZE];


	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_FREQUENCY_RANG_MAX_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_FREQUENCY_RANG_MAX_ADDR;
	}

	FIT_EE_Read(writeAddress,tempBuf,CARTRIDGE1_FREQUENCY_RANG_MAX_SIZE);

	memcpy(data,tempBuf,CARTRIDGE1_FREQUENCY_RANG_MAX_SIZE);
}

bool writeChannelAssignment(uint8_t infoID,uint8_t *data,uint8_t size)
{

	uint16_t writeAddress;
	bool result=true;

	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_CHANNEL_ASSIGNMEN_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_CHANNEL_ASSIGNMEN_ADDR;
	}
	else
	{
		return false;
	}

	result=writeEE(writeAddress,data,size);

	return result;
}

void readChannelAssignment(uint8_t infoID,uint8_t *data)
{

	uint16_t writeAddress=CARTRIDGE1_CHANNEL_ASSIGNMEN_ADDR;
	uint8_t tempBuf[CARTRIDGE1_CHANNEL_ASSIGNMEN_SIZE];


	if(infoID==INFOR_ID_CARTRIDGE1)
    {
		writeAddress=CARTRIDGE1_CHANNEL_ASSIGNMEN_ADDR;
    }
	else if(infoID==INFOR_ID_CARTRIDGE2)
	{
		writeAddress=CARTRIDGE2_CHANNEL_ASSIGNMEN_ADDR;
	}

	FIT_EE_Read(writeAddress,tempBuf,CARTRIDGE1_CHANNEL_ASSIGNMEN_SIZE);

	memcpy(data,tempBuf,CARTRIDGE1_CHANNEL_ASSIGNMEN_SIZE);
}

bool writeDigitalInterfaceType(uint8_t *data,uint8_t size)
{

	uint16_t writeAddress=SYSTEM_DIGITAL_INTERFACE_TYPE_ADDR;
	bool result=true;

	result=writeEE(writeAddress,data,size);

	return result;
}

void readDigitalInterfaceType(uint8_t infoID,uint8_t *data)
{
	uint8_t tempBuf[SYSTEM_DIGITAL_INTERFACE_TYPE_SIZE];

	FIT_EE_Read(SYSTEM_DIGITAL_INTERFACE_TYPE_ADDR,tempBuf,SYSTEM_DIGITAL_INTERFACE_TYPE_SIZE);

	memcpy(data,tempBuf,SYSTEM_DIGITAL_INTERFACE_TYPE_SIZE);
}

bool writeBitClockFrequency(uint8_t *data,uint8_t size)
{

	uint16_t writeAddress=SYSTEM_BIT_CLOCK_FREQUENCY_ADDR;
	bool result=true;

	result=writeEE(writeAddress,data,size);

	return result;
}

void readBitClockFrequency(uint8_t infoID,uint8_t *data)
{
	uint8_t tempBuf[SYSTEM_BIT_CLOCK_FREQUENCY_SIZE];

	FIT_EE_Read(SYSTEM_BIT_CLOCK_FREQUENCY_ADDR,tempBuf,SYSTEM_BIT_CLOCK_FREQUENCY_SIZE);

	memcpy(data,tempBuf,SYSTEM_BIT_CLOCK_FREQUENCY_SIZE);
}

bool writeWordLength(uint8_t *data,uint8_t size)
{

	uint16_t writeAddress=SYSTEM_WORD_LENGTH_ADDR;
	bool result=true;

	result=writeEE(writeAddress,data,size);

	return result;
}

void readWordLength(uint8_t infoID,uint8_t *data)
{
	uint8_t tempBuf[SYSTEM_WORD_LENGTH_SIZE];

	FIT_EE_Read(SYSTEM_WORD_LENGTH_ADDR,tempBuf,SYSTEM_WORD_LENGTH_SIZE);

	memcpy(data,tempBuf,SYSTEM_WORD_LENGTH_SIZE);
}

bool writeSampleRate(uint8_t *data,uint8_t size)
{

	uint16_t writeAddress=SYSTEM_SAMPLE_RATE_ADDR;
	bool result=true;

	result=writeEE(writeAddress,data,size);

	return result;
}

void readSampleRate(uint8_t infoID,uint8_t *data)
{
	uint8_t tempBuf[SYSTEM_SAMPLE_RATE_SIZE];

	FIT_EE_Read(SYSTEM_SAMPLE_RATE_ADDR,tempBuf,SYSTEM_SAMPLE_RATE_SIZE);

	memcpy(data,tempBuf,SYSTEM_SAMPLE_RATE_SIZE);
}

bool writeCalibrationDate(uint8_t *data,uint8_t size)
{

	uint16_t writeAddress=SYSTEM_CALIBRATION_DATE_ADDR;
	bool result=true;

	result=writeEE(writeAddress,data,size);

	return result;
}

void readCalibrationDate(uint8_t infoID,uint8_t *data)
{
	uint8_t tempBuf[SYSTEM_CALIBRATION_DATE_SIZE];

	FIT_EE_Read(SYSTEM_CALIBRATION_DATE_ADDR,tempBuf,SYSTEM_CALIBRATION_DATE_SIZE);

	memcpy(data,tempBuf,SYSTEM_CALIBRATION_DATE_SIZE);
}


bool writeManufacturerID(uint8_t *data,uint8_t size)
{

	uint16_t writeAddress=SYSTEM_MANUFACTUER_ID_ADDR;
	bool result=true;

	result=writeEE(writeAddress,data,size);

	return result;
}

void readManufacturerID(uint8_t infoID,uint8_t *data)
{
	uint8_t tempBuf[SYSTEM_MANUFACTUER_ID_SIZE];

	FIT_EE_Read(SYSTEM_MANUFACTUER_ID_ADDR,tempBuf,SYSTEM_MANUFACTUER_ID_SIZE);

	memcpy(data,tempBuf,SYSTEM_MANUFACTUER_ID_SIZE);
}

bool writeFirmwareVersion(uint8_t *data,uint8_t size)
{

	uint16_t writeAddress=SYSTEM_FW_VERSION_ADDR;
	bool result=true;

	result=writeEE(writeAddress,data,size);

	return result;
}

void readFirmwareVersion(uint8_t infoID,uint8_t *data)
{
	uint8_t tempBuf[SYSTEM_FW_VERSION_SIZE];

	FIT_EE_Read(SYSTEM_FW_VERSION_ADDR,tempBuf,SYSTEM_FW_VERSION_SIZE);

	memcpy(data,tempBuf,SYSTEM_FW_VERSION_SIZE);
}

bool writeNotes(uint8_t *data,uint8_t size)
{

	uint16_t writeAddress=SYSTEM_NOTE_ADDR;
	bool result=true;

	result=writeEE(writeAddress,data,size);

	return result;
}

void readNotes(uint8_t infoID,uint8_t *data)
{
	uint8_t tempBuf[SYSTEM_NOTE_SIZE];

	FIT_EE_Read(SYSTEM_NOTE_ADDR,tempBuf,SYSTEM_NOTE_SIZE);

	memcpy(data,tempBuf,SYSTEM_NOTE_SIZE);
}


int8_t getPageFromEEprom(uint8_t blockID,uint8_t *pageBuf,uint8_t pageEESize)
{
	uint16_t startAddr=0;
	uint8_t tempBuf[pageEESize];
	uint8_t numberOfByte=0;
	uint8_t checksum=0;
	int offset=0;
	int8_t result=0;

	FT_printf("getPageFromEEprom blockID:%d,EE page size=%d\r\n",blockID,pageEESize);
	if(blockID==BLOCK_ID_PAGE0)
	{
		startAddr=PAGE0_START_ADDR;
		numberOfByte=PAGE0_REG_MAX_SIZE;
	}
	else if(blockID==BLOCK_ID_PAGE2)
	{
		startAddr=PAGE2_START_ADDR;
		numberOfByte=PAGE2_REG_MAX_SIZE;
	}
	else if(blockID==BLOCK_ID_PAGE3)
	{
		startAddr=PAGE3_START_ADDR;
		numberOfByte=PAGE3_REG_MAX_SIZE;
	}
	else if(blockID==BLOCK_ID_PAGE4)
	{
		startAddr=PAGE4_START_ADDR;
		numberOfByte=PAGE4_REG_MAX_SIZE;
	}
	else if(blockID==BLOCK_ID_TBD1)
	{
		startAddr=TBD1_START_ADDR;
		numberOfByte=TBD1_SIZE;
	}
	else if(blockID==BLOCK_ID_TBD2)
	{
		startAddr=TBD2_START_ADDR;
		numberOfByte=TBD2_SIZE;
	}
	else if(blockID==BLOCK_ID_INFO1)
	{
		startAddr=INFO1_START_ADDR;
		numberOfByte=INFO1_SIZE;
	}
	else if(blockID==BLOCK_ID_INFO2)
	{
		startAddr=INFO2_START_ADDR;
		numberOfByte=INFO2_SIZE;
	}

	result=FIT_EE_Read(startAddr,tempBuf,pageEESize);

	checksum= calculateChecksum(tempBuf,numberOfByte);

	FT_printf("getPageFromEEprom checksum:%x %x\r\n",checksum,tempBuf[pageEESize-1]);

	if(checksum!=tempBuf[pageEESize-1]&&blockID!=BLOCK_ID_INFO1&&blockID!=BLOCK_ID_INFO2)
	{
		result=1;
		memset(pageBuf,0x00,pageEESize);
		checksum= calculateChecksum(pageBuf,numberOfByte);
		pageBuf[numberOfByte]=checksum;
	}
	else
	{
	    for(offset=0;offset<numberOfByte;offset++)
        {
    	    pageBuf[offset]=tempBuf[offset];
        }
        pageBuf[offset]=checksum;
	}

	return result;
}

int8_t writePage(uint8_t blockID,uint8_t *pageBuf,uint8_t numberOfByte,uint8_t pageEESize)
{
	uint8_t tempBuf[pageEESize];
	int offset=0;
	uint16_t startAddr=0;
	uint8_t checksum=0;
	int8_t result=0;

	FT_printf("write page blockID:%d,numberOfByte:%d,EE page size%x\r\n",blockID,numberOfByte,pageEESize);

	memset(tempBuf,0xff,pageEESize);

	for(offset=0;offset<numberOfByte;offset++)
	{
		tempBuf[offset]=pageBuf[offset];
	}

	checksum= calculateChecksum(tempBuf,numberOfByte);
	FT_printf("write page checksum:%x %x\r\n",checksum,pageBuf[numberOfByte]);

	if(checksum!=pageBuf[numberOfByte])
	{
		return 2;
	}


	if(blockID==BLOCK_ID_PAGE0)
	{
		startAddr=PAGE0_START_ADDR;
		tempBuf[pageEESize-1]=checksum;
	}
	else if(blockID==BLOCK_ID_PAGE2)
	{
		startAddr=PAGE2_START_ADDR;
		tempBuf[pageEESize-1]=checksum;
	}
	else if(blockID==BLOCK_ID_PAGE3)
	{
		startAddr=PAGE3_START_ADDR;
		tempBuf[pageEESize-1]=checksum;
	}
	else if(blockID==BLOCK_ID_PAGE4)
	{
		startAddr=PAGE4_START_ADDR;
		tempBuf[pageEESize-1]=checksum;
	}
	else if(blockID==BLOCK_ID_TBD1)
	{
		startAddr=TBD1_START_ADDR;
	}
	else if(blockID==BLOCK_ID_TBD2)
	{
		startAddr=TBD2_START_ADDR;
	}
	else if(blockID==BLOCK_ID_INFO1)
	{
		startAddr=INFO1_START_ADDR;
	}
	else if(blockID==BLOCK_ID_INFO2)
	{
		startAddr=INFO2_START_ADDR;
	}

	for(offset=0;offset<pageEESize;offset+=16)
	{
	  result=FIT_EE_Write(startAddr+offset,&tempBuf[offset],16);
	  if(result!=0)
	  {
		  FT_printf("write page fail\r\n");
		  return result;
	  }
	}

	return result;
}

bool writeEnableDebugMsgFlag(uint8_t flag)
{
	uint8_t data[1];

	data[0]=flag;

	if(FIT_EE_Write(DEBUG_MSG_ENABLE_FLAG_ADDR,data,1)!=0)
			return false;

	return true;
}

uint8_t readEnableDebugMsgFlag()
{
	uint8_t data[1];


    FIT_EE_Read(DEBUG_MSG_ENABLE_FLAG_ADDR,data,1);

    return data[0];
}

bool commandWriteOneByteToEE(uint16_t addr,uint8_t *data)
{
	uint8_t pageBuf[129];

    uint16_t startAddr=0;
    uint16_t checkSumAddr=0;
    uint8_t pageBufSize=0;
    uint8_t checksum[1]={0};

    if(FIT_EE_Write(addr,data,1)!=0)
    {
	    return false;
    }


	if(addr>=PAGE0_START_ADDR && addr<PAGE2_START_ADDR)
	{
		pageBufSize=PAGE0_REG_MAX_SIZE;
		startAddr=PAGE0_START_ADDR;
		checkSumAddr=startAddr+PAGE0_SIZE-1;
	}
	else if(addr>=PAGE2_START_ADDR && addr<PAGE3_START_ADDR)
	{
		pageBufSize=PAGE2_REG_MAX_SIZE;
		startAddr=PAGE2_START_ADDR;
		checkSumAddr=startAddr+PAGE2_SIZE-1;
	}
	else if(addr>=PAGE3_START_ADDR && addr<PAGE4_START_ADDR)
	{
		pageBufSize=PAGE3_REG_MAX_SIZE;
		startAddr=PAGE3_START_ADDR;
		checkSumAddr=startAddr+PAGE3_SIZE-1;
	}
	else if(addr>=PAGE4_START_ADDR && addr<TBD1_START_ADDR)
	{
		pageBufSize=PAGE4_REG_MAX_SIZE;
		startAddr=PAGE4_START_ADDR;
		checkSumAddr=startAddr+PAGE4_SIZE-1;
	}

	FIT_EE_Read(startAddr,pageBuf,pageBufSize);

	checksum[0]= calculateChecksum(pageBuf,pageBufSize);

	FIT_EE_Write(checkSumAddr,checksum,1);

	return true;
}

void initEEPROM()
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

	LL_GPIO_SetOutputPin(WC_GPIO_Port, WC_Pin);
	/**/
	GPIO_InitStruct.Pin = WC_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(WC_GPIO_Port, &GPIO_InitStruct);
}

void testEEprom()
{
  //uint8_t tempbuf[1024];
  uint8_t tempbuf1[1024];

  memset(tempbuf1,0xff,1024);

  //for(int i=0;i<1024;i++)
	  //tempbuf[i]=0xff;

  //FIT_EE_Write(0,tempbuf,50);
  //FIT_EE_Write(0,tempbuf,1);
  //FIT_EE_Write(1,tempbuf,1);
  //FIT_EE_Write(2,tempbuf,1);
  //for(int i=0;i<1024;i++)
  	  //tempbuf[i]=i;
  //tempbuf[0]=14;
    //FIT_EE_Write(0,tempbuf,50);
    //FIT_EE_Write(0,tempbuf,10);

    //for(int i=0;i<1024;i++)
     //FIT_EE_Write(i,&tempbuf1[0],1);
    //FIT_EE_Write(1,&tempbuf[0],16);
    //FIT_EE_Write(16,&tempbuf[16],16);
    //FIT_EE_Write(16,&tempbuf[16],16);
    //FIT_EE_Write(0,&tempbuf[0],1);
    //FIT_EE_Write(1,&tempbuf[1],1);
    //FIT_EE_Write(2,&tempbuf[2],1);
    //FIT_EE_Write(2,&tempbuf[2],1);
  //tempbuf1[0]=55;
    //for(int i=0;i<1024;i++)
  //FIT_EE_Read(i,&tempbuf1[i],1);
  FIT_EE_Read(0,&tempbuf1[0],255);
  FIT_EE_Read(256,&tempbuf1[256],255);
  FIT_EE_Read(512,&tempbuf1[512],255);
  FIT_EE_Read(768,&tempbuf1[768],255);
  //FIT_EE_Read(2,&tempbuf1[2],1);

  for(int i=0;i<1024;i++)
  {
	  FT_printf("addr:%d data:%d\r\n",i,tempbuf1[i]);
  }

  //FT_printf("DEVICE_ID_START_ADDR:%d\r\n",DEVICE_ID_START_ADDR);
  FT_printf("PAGE0_START_ADDR:%d\r\n",PAGE0_START_ADDR);
  //FT_printf("PAGE1_START_ADDR:%d\r\n",PAGE1_START_ADDR);
  FT_printf("PAGE2_START_ADDR:%d\r\n",PAGE2_START_ADDR);
  FT_printf("PAGE3_START_ADDR:%d\r\n",PAGE3_START_ADDR);
  FT_printf("PAGE4_START_ADDR:%d\r\n",PAGE4_START_ADDR);

}




