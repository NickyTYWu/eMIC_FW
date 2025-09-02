#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include "main.h"
#include "string.h"
#include "ftm_common.h"
#include "ftm_wpptest.h"
#include "FIT_M24C08_EE.h"
#include "FIT_DebugMessage.h"
#include "FIT_PCMD3140.h"
#include "FIT_sensirion_hw_i2c_implementation.h"


uint32_t ftm_timeout = 4000; //unit ms

extern int version;

int FTM_versions(CMD_DATA* cmd)
{

	return OK;
}

int FTM_alive_test(CMD_DATA* cmd)
{
	return OK;
}

int FTM_fw_log_enable(CMD_DATA* cmd)
{
	uint8_t bEnable=0x00;

	if(cmd->param[0]=='1')
	{
		bEnable=0x01;
	}

	if(enableFWLog(bEnable))
	{

	    return OK;
	}

	return ERROR;
}

int FTM_cartridge_info(CMD_DATA* cmd)
{
	
	uint8_t buf[32]={0};
    float value;
    int16_t v;
	readModelNumber(INFOR_ID_CARTRIDGE1,buf);

	FTM_printf("Cartridge 1 Model Number:%S\r\n",(wchar_t*)buf);

    memset(buf,0,32);
    readVersionLetter(INFOR_ID_CARTRIDGE1,buf);
    FTM_printf("Cartridge 1 Version Letter:%S\r\n",(wchar_t*)buf);

    memset(buf,0,32);
    readSerialNumber(INFOR_ID_CARTRIDGE1,buf);
    FTM_printf("Cartridge 1 Serial Number:%S\r\n",(wchar_t*)buf);

	memset(buf,0,32);
	readSensitivity(INFOR_ID_CARTRIDGE1,buf);
	memcpy(&value,buf,sizeof(float));

	FTM_printf("Cartridge 1 Sensitivity:%.2f\r\n",value);

	memset(buf,0,32);
	readReferenceFrequency(INFOR_ID_CARTRIDGE1,buf);
	memcpy(&value,buf,sizeof(float));

	FTM_printf("Cartridge 1 Reference Frequency:%.2f\r\n",value);

	memset(buf,0,32);
	readUnitsCode(INFOR_ID_CARTRIDGE1,buf);

	FTM_printf("Cartridge 1 Units Code:0x%02X\r\n",buf[0]);

	memset(buf,0,32);
	readFrequencyRangeMin(INFOR_ID_CARTRIDGE1,buf);
	memcpy(&v,buf,sizeof(int16_t));

	FTM_printf("Cartridge 1 Frequency Range Min:%d\r\n",v);

	memset(buf,0,32);
	readFrequencyRangeMax(INFOR_ID_CARTRIDGE1,buf);
	memcpy(&v,buf,sizeof(int16_t));

	FTM_printf("Cartridge 1 Frequency Range Max:%d\r\n",v);

	memset(buf,0,32);
	readChannelAssignment(INFOR_ID_CARTRIDGE1,buf);

	FTM_printf("Cartridge 1 Channel Assignment:0x%02X\r\n",buf[0]);

	readModelNumber(INFOR_ID_CARTRIDGE2,buf);

	FTM_printf("Cartridge 2 Model Number:%S\r\n",(wchar_t*)buf);

	memset(buf,0,32);
	readVersionLetter(INFOR_ID_CARTRIDGE2,buf);
	FTM_printf("Cartridge 2 Version Letter:%S\r\n",(wchar_t*)buf);

	memset(buf,0,32);
	readSerialNumber(INFOR_ID_CARTRIDGE2,buf);
	FTM_printf("Cartridge 2 Serial Number:%S\r\n",(wchar_t*)buf);

	memset(buf,0,32);
	readSensitivity(INFOR_ID_CARTRIDGE2,buf);
	memcpy(&value,buf,sizeof(float));

	FTM_printf("Cartridge 2 Sensitivity:%.2f\r\n",value);

	memset(buf,0,32);
	readReferenceFrequency(INFOR_ID_CARTRIDGE2,buf);
	memcpy(&v,buf,sizeof(float));

	FTM_printf("Cartridge 2 Reference Frequency:%.2f\r\n",value);

	memset(buf,0,32);
	readUnitsCode(INFOR_ID_CARTRIDGE2,buf);

	FTM_printf("Cartridge 2 Units Code:0x%02X\r\n",buf[0]);

	memset(buf,0,32);
	readFrequencyRangeMin(INFOR_ID_CARTRIDGE2,buf);
	memcpy(&v,buf,sizeof(int16_t));

	FTM_printf("Cartridge 2 Frequency Range Min:%d\r\n",v);

	memset(buf,0,32);
	readFrequencyRangeMax(INFOR_ID_CARTRIDGE2,buf);
	memcpy(&v,buf,sizeof(int16_t));

	FTM_printf("Cartridge 2 Frequency Range Max:%d\r\n",v);

	memset(buf,0,32);
	readChannelAssignment(INFOR_ID_CARTRIDGE2,buf);

	FTM_printf("Cartridge 2 Channel Assignment:0x%02X\r\n",buf[0]);

	return OK ;
}

int hex_digit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

int parse_ascii_values(const char* buf, uint16_t* out, int max_count) {
    int count = 0;
    const char* p = buf;

    while (*p && count < max_count) {
        // 跳過空白
        while (*p == ' ') p++;

        int value = 0;
        if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
            // 十六進制
            p += 2;
            int digit;
            while ((digit = hex_digit(*p)) >= 0) {
                value = (value << 4) + digit;
                p++;
            }
        } else if (*p >= '0' && *p <= '9') {
            // 十進制
            while (*p >= '0' && *p <= '9') {
                value = value * 10 + (*p - '0');
                p++;
            }
        } else {
            break; // 非法字元
        }

        out[count++] = (uint8_t)value;
    }

    return count;
}

int ascii_to_float(const char* str, float* out_value) {
    float result = 0.0f;
    //float fraction = 0.0f;
    int sign = 1;
    int has_dot = 0;
    float div = 10.0f;

    // 跳過空白
    while (*str == ' ') str++;

    if (*str == '-') {
        sign = -1;
        str++;
    } else if (*str == '+') {
        str++;
    }

    if (!(*str >= '0' && *str <= '9') && *str != '.') return 0;

    while (*str) {
        if (*str == '.') {
            if (has_dot) return 0;  // 多個小數點
            has_dot = 1;
            str++;
            continue;
        }

        if (*str >= '0' && *str <= '9') {
            if (!has_dot) {
                result = result * 10.0f + (*str - '0');
            } else {
                result = result + (*str - '0') / div;
                div *= 10.0f;
            }
        } else {
            return 0;  // 非數字非法字元
        }
        str++;
    }

    *out_value = sign * result;
    return 1;
}

int ascii_to_int(const char* str, int* out_value) {
    int sign = 1;
    int result = 0;
    int base = 10;

    // 跳過前導空白
    while (*str == ' ') str++;

    // 處理正負號
    if (*str == '-') {
        sign = -1;
        str++;
    } else if (*str == '+') {
        str++;
    }

    // 檢查是否為 0x / 0X 開頭（十六進位）
    if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X')) {
        base = 16;
        str += 2;
    }

    // 判斷第一個合法數字是否存在
    if ((base == 10 && (*str < '0' || *str > '9')) ||
        (base == 16 && !((*str >= '0' && *str <= '9') ||
                         (*str >= 'A' && *str <= 'F') ||
                         (*str >= 'a' && *str <= 'f')))) {
        return 0;
    }

    // 開始轉換數值
    while (*str) {
        int digit;

        if (*str >= '0' && *str <= '9') {
            digit = *str - '0';
        } else if (base == 16 && *str >= 'a' && *str <= 'f') {
            digit = *str - 'a' + 10;
        } else if (base == 16 && *str >= 'A' && *str <= 'F') {
            digit = *str - 'A' + 10;
        } else {
            return 0;  // 非法字元
        }

        if (digit >= base) return 0;  // 超過合法範圍（多餘保險）

        result = result * base + digit;
        str++;
    }

    *out_value = sign * result;
    return 1;
}

// 將兩個 ASCII 字元轉成 BCD (e.g. '2','5' → 0x25)
uint8_t ascii2bcd(char high, char low) {
    return ((high - '0') << 4) | (low - '0');
}

// str: 必須為長度至少12的合法 ASCII 日期 "YYMMDDhhmmss"
int parse_date_ascii_to_bcd(const char* str, uint8_t bcd[6]) {
    for (int i = 0; i < 6; ++i) {
        char hi = str[i * 2];
        char lo = str[i * 2 + 1];
        if (hi < '0' || hi > '9' || lo < '0' || lo > '9') {
            return 0; // 非法輸入
        }
        bcd[i] = ascii2bcd(hi, lo);
    }
    return 1; // 轉換成功
}

int FTM_w_sys_manuf_id(CMD_DATA* cmd)
{
	int value;

	if(ascii_to_int(cmd->param,&value))
	{
		uint8_t buf[2];

		memcpy(buf,&value,2);

		writeManufacturerID(buf,2);
	}
	else
	{
		FTM_printf("emic:Not a valid  number\r\n");

		return ERROR;

	}

	return OK ;
}

int FTM_w_sys_cal_date(CMD_DATA* cmd)
{
	uint8_t bcd[6];

	if(parse_date_ascii_to_bcd(cmd->param,bcd))
	{
		writeCalibrationDate(bcd,6);
	}
	else
	{
		FTM_printf("emic:Not a valid Date(YYMMDDhhmmss)\r\n");

	    return ERROR;
	}

	return OK;
}

int FTM_w_sys_serial_number(CMD_DATA* cmd)
{
	return write_serial_number(INFOR_ID_SYSTEM,cmd);
}

int FTM_w_sys_sensitivity(CMD_DATA* cmd)
{
	return write_sensitivity(INFOR_ID_SYSTEM,cmd);
}

int FTM_w_sys_sample_rate(CMD_DATA* cmd)
{
	float value;

	if(ascii_to_float(cmd->param,&value))
	{
		uint8_t buf[4]={0};
		memcpy(buf,&value,4);
		writeSampleRate(buf,4);
	}
	else
	{
		FTM_printf("emic:Not a valid floating point number\r\n");

		return ERROR;

	}

	return OK ;
}

int FTM_w_sys_word_length(CMD_DATA* cmd)
{
	int value;

	if(ascii_to_int(cmd->param,&value))
	{
		uint8_t buf[1];

		buf[0]=(uint8_t)value;

		writeWordLength(buf,1);
	}
	else
	{
		FTM_printf("Not a valid number\r\n");

		return ERROR;

	}

	return OK ;
}

int FTM_w_sys_bit_clock_freq(CMD_DATA* cmd)
{
	float value;

	if(ascii_to_float(cmd->param,&value))
	{
		uint8_t buf[4]={0};
		memcpy(buf,&value,4);
		writeBitClockFrequency(buf,4);
	}
	else
	{
		FTM_printf("emic:Not a valid floating point number\r\n");

		return ERROR;

	}

	return OK ;
}

int FTM_w_sys_digit_type(CMD_DATA* cmd)
{
	uint8_t param_len=	strlen(cmd->param);

	if(param_len>=4)
	{
		FTM_printf("emic:error version latter length(%d).\r\n",param_len);
		return ERROR;
	}
	else
	{
		writeDigitalInterfaceType((uint8_t*)cmd->param,param_len);
	}

	return OK ;
}


int writeChAssign(uint8_t infoID,CMD_DATA* cmd)
{
	int value;

	if(ascii_to_int(cmd->param,&value))
	{
		uint8_t buf[1];

		buf[0]=(uint8_t)value;

		writeChannelAssignment(infoID,buf,1);
	}
	else
	{
		FTM_printf("emic:Not a valid number\r\n");

		return ERROR;

	}

	return OK ;
}

int write_freq_max(uint8_t infoID,CMD_DATA* cmd)
{
	int value;

	if(ascii_to_int(cmd->param,&value))
	{
		uint8_t buf[2];

		memcpy(buf,&value,2);

		writeFrequencyRangeMax(infoID,buf,2);
	}
	else
	{
		FTM_printf("emic:Not a valid  number\r\n");

		return ERROR;

	}

	return OK ;
}

int write_freq_min(uint8_t infoID,CMD_DATA* cmd)
{
	int value;

	if(ascii_to_int(cmd->param,&value))
	{
		uint8_t buf[2];

		memcpy(buf,&value,2);

		writeFrequencyRangeMin(infoID,buf,2);
	}
	else
	{
		FTM_printf("emic:Not a valid  number\r\n");

		return ERROR;

	}

	return OK ;
}

int write_uints_code(uint8_t infoID,CMD_DATA* cmd)
{
	int value;

	if(ascii_to_int(cmd->param,&value))
	{
		uint8_t buf[1];

		buf[0]=(uint8_t)value;

		writeUnitsCode(infoID,buf,1);
	}
	else
	{
		FTM_printf("emic:Not a valid number\r\n");

		return ERROR;

	}

	return OK ;
}

int write_ref_freq(uint8_t infoID,CMD_DATA* cmd)
{
	float value;

	if(ascii_to_float(cmd->param,&value))
	{
		uint8_t buf[4]={0};
		memcpy(buf,&value,4);
		writeReferenceFrequency(infoID,buf,4);
	}
	else
	{
		FTM_printf("emic:Not a valid floating point number\r\n");

		return ERROR;

	}

	return OK ;
}

int write_sensitivity(uint8_t infoID,CMD_DATA* cmd)
{
	float value;

	    if(ascii_to_float(cmd->param,&value))
	    {
	    	uint8_t buf[4]={0};
			memcpy(buf,&value,4);
			writeSensitivity(infoID,buf,4);
	    }
	    else
	    {
	    	FTM_printf("emic:Not a valid floating point number\r\n");

	    	return ERROR;

	    }

	    return OK ;
}

int write_serial_number(uint8_t infoID,CMD_DATA* cmd)
{
	uint8_t param_len=	strlen(cmd->param);

	if(param_len!=16)
	{
		FTM_printf("emic:error serial number length(%d).\r\n",param_len);
		return ERROR;
	}
	else
	{
		writeSerialNumber(infoID,(uint8_t*)cmd->param,param_len);
	}
	return OK ;
}

int write_version_latter(uint8_t infoID,CMD_DATA* cmd)
{
	uint8_t param_len=	strlen(cmd->param);

	if(param_len!=1)
	{
		FTM_printf("emic:error version latter length(%d).\r\n",param_len);
		return ERROR;
	}
	else
	{
		writeVersionLetter(infoID,(uint8_t*)cmd->param,param_len);
	}
	return OK ;
}

int write_model_number(uint8_t infoID,CMD_DATA* cmd)
{
	uint8_t param_len=	strlen(cmd->param);

	if(param_len!=8)
	{
		FTM_printf("emic:error model number length(%d).\r\n",param_len);
		return ERROR;
	}
	else
	{
		writeModelNumber(infoID,(uint8_t*)cmd->param,param_len);
	}
	return OK ;
}

int FTM_w_cart2_ch_assign(CMD_DATA* cmd)
{
	return writeChAssign(INFOR_ID_CARTRIDGE2,cmd);
}

int FTM_w_cart2_freq_max(CMD_DATA* cmd)
{
	return write_freq_max(INFOR_ID_CARTRIDGE2,cmd);
}

int FTM_w_cart2_freq_min(CMD_DATA* cmd)
{
	return write_freq_min(INFOR_ID_CARTRIDGE2,cmd);
}

int FTM_w_cart2_uints_code(CMD_DATA* cmd)
{
	return write_uints_code(INFOR_ID_CARTRIDGE2,cmd);
}

int FTM_w_cart2_ref_freq(CMD_DATA* cmd)
{
	return write_ref_freq(INFOR_ID_CARTRIDGE2,cmd);
}

int FTM_w_cart2_sensitivity(CMD_DATA* cmd)
{
	return write_sensitivity(INFOR_ID_CARTRIDGE2,cmd);
}

int FTM_w_cart2_serial_number(CMD_DATA* cmd)
{
	return write_serial_number(INFOR_ID_CARTRIDGE2,cmd);
}


int FTM_w_cart2_version_latter(CMD_DATA* cmd)
{
	return write_version_latter(INFOR_ID_CARTRIDGE2,cmd);
}

int FTM_w_cart2_model_number(CMD_DATA* cmd)
{
	return write_model_number(INFOR_ID_CARTRIDGE2,cmd);
}

int FTM_w_cart1_ch_assign(CMD_DATA* cmd)
{
	return writeChAssign(INFOR_ID_CARTRIDGE1,cmd);
}

int FTM_w_cart1_freq_max(CMD_DATA* cmd)
{
	return write_freq_max(INFOR_ID_CARTRIDGE1,cmd);
}

int FTM_w_cart1_freq_min(CMD_DATA* cmd)
{
	return write_freq_min(INFOR_ID_CARTRIDGE1,cmd);
}

int FTM_w_cart1_uints_code(CMD_DATA* cmd)
{
	return write_uints_code(INFOR_ID_CARTRIDGE1,cmd);
}

int FTM_w_cart1_ref_freq(CMD_DATA* cmd)
{
	return write_ref_freq(INFOR_ID_CARTRIDGE1,cmd);
}

int FTM_w_cart1_sensitivity(CMD_DATA* cmd)
{
	return write_sensitivity(INFOR_ID_CARTRIDGE1,cmd);
}

int FTM_w_cart1_serial_number(CMD_DATA* cmd)
{
	return write_serial_number(INFOR_ID_CARTRIDGE1,cmd);
}


int FTM_w_cart1_version_latter(CMD_DATA* cmd)
{
	return write_version_latter(INFOR_ID_CARTRIDGE2,cmd);
}

int FTM_w_cart1_model_number(CMD_DATA* cmd)
{
	return write_model_number(INFOR_ID_CARTRIDGE1,cmd);
}

uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

void bcd_to_datetime_string(uint8_t bcd[6], char *outStr, size_t maxLen) {
    uint8_t sec  = bcd_to_dec(bcd[5]);
    uint8_t min  = bcd_to_dec(bcd[4]);
    uint8_t hour = bcd_to_dec(bcd[3]);
    uint8_t day  = bcd_to_dec(bcd[2]);
    uint8_t mon  = bcd_to_dec(bcd[1]);
    uint8_t year = bcd_to_dec(bcd[0]);

    snprintf(outStr, maxLen, "20%02d-%02d-%02d %02d:%02d:%02d",
             year, mon, day, hour, min, sec);
}



int FTM_system_info(CMD_DATA* cmd)
{

	uint8_t buf[32]={0};
	uint8_t bcd[6]={0};
    float value;
    int16_t v;

    readDigitalInterfaceType(INFOR_ID_SYSTEM,buf);
    FTM_printf("System Digital Interface Type:%S\r\n",(wchar_t*)buf);

    memset(buf,0,32);
    readBitClockFrequency(INFOR_ID_SYSTEM,buf);
    memcpy(&value,buf,sizeof(float));
    FTM_printf("System Bit Clock Frequency:%.2f\r\n",value);

    memset(buf,0,32);
    readWordLength(INFOR_ID_SYSTEM,buf);
    FTM_printf("System Word Length:%d\r\n",buf[0]);

	memset(buf,0,32);
	readSampleRate(INFOR_ID_SYSTEM,buf);
	memcpy(&value,buf,sizeof(float));
	FTM_printf("System Sample Rate:%.2f\r\n",value);

	memset(buf,0,32);
	readSerialNumber(INFOR_ID_SYSTEM,buf);
	FTM_printf("System Serial Number::%S\r\n",(wchar_t*)buf);

	memset(buf,0,32);
	readSensitivity(INFOR_ID_SYSTEM,buf);
	memcpy(&value,buf,sizeof(float));
	FTM_printf("System Sensitivity:%.2f\r\n",value);

	memset(buf,0,32);
	readCalibrationDate(INFOR_ID_SYSTEM,bcd);
	bcd_to_datetime_string(bcd,(char*)buf,sizeof(buf));
	FTM_printf("System Calibration Date:%S\r\n",(wchar_t*)buf);

	memset(buf,0,32);
	readManufacturerID(INFOR_ID_SYSTEM,buf);
	memcpy(&v,buf,sizeof(int16_t));
	FTM_printf("System Manufacturer ID:0x%04X\r\n",v);

	FTM_printf("Firewarm Version:0x%04X\r\n",version);

	return OK ;
}

int FTM_w_note(CMD_DATA* cmd)
{
	uint8_t param_len=	strlen(cmd->param);

	if(param_len>=128)
	{
		FTM_printf("emic:error note max length is 128 byte(%d).\r\n",param_len);
		return ERROR;
	}
	else
	{
		writeNotes((uint8_t*)cmd->param,128);
	}

	return OK ;
}

int FTM_note(CMD_DATA* cmd)
{
	uint8_t buf[128]={0};

	readNotes(INFOR_ID_SYSTEM,buf);

	FTM_printf("Note:%S\r\n",(wchar_t*)buf);

	return OK ;
}

int FTM_w_pcmd(CMD_DATA* cmd)
{
	uint16_t result[3];

	int num=parse_ascii_values(cmd->param , result, 3);

	FTM_printf("num:%d\r\n",num);

	if(num!=3)
	{
		FTM_printf("emic:error parameter.\r\n");
		return ERROR;
	}
	else
	{
		if(!writeRegToPCDM3140WithPageParameters((uint8_t*)result))
			return ERROR;

	}

	return OK ;
}

int FTM_r_pcmd(CMD_DATA* cmd)
{
    uint16_t result[2],readResult;
	int num=parse_ascii_values(cmd->param , result, 2);

	FTM_printf("num:%d\r\n",num);

	if(num!=2)
	{
		FTM_printf("emic:error parameter.\r\n");
		return ERROR;
	}
	else
	{
		//FTM_printf("param=%x %x\r\n",result[0],result[1]);
		readResult=readRegFromPCDM3140WithPageParameters((uint8_t*)result);

		FTM_printf("emic:Page%d REG:0x%02X Value=0x%02X\r\n",result[0],result[1],readResult);
	}

	return OK ;
}

int FTM_r_ee(CMD_DATA* cmd)
{
	int addr=0;

	if(ascii_to_int(cmd->param,&addr))
	{
		uint8_t RESPONSE_CMD_BUF[1];

		FIT_EE_Read((uint16_t)addr,RESPONSE_CMD_BUF,1);

		FTM_printf("address:0x%04X value:0x%02X\r\n",addr,RESPONSE_CMD_BUF[0]);
	}
	else
	{
		FTM_printf("emic:Not a valid address\r\n");

		return ERROR;

	}

	return OK ;

}

int FTM_w_ee(CMD_DATA* cmd)
{
	uint16_t result[2];
	uint8_t data[1];

	int num=parse_ascii_values(cmd->param , result, 2);

	if(num==2)
	{
		data[0]=result[1]&0xff;

		commandWriteOneByteToEE(result[0],data);
	}
	else
	{
		FTM_printf("emic:error parameter.\r\n");

		return ERROR;

	}

	return OK ;

}

int FTM_get_th(CMD_DATA* cmd)
{
	int32_t temperature, humidity;

	getTemperatureAndHumidity(&temperature,&humidity);

	FTM_printf("emic:temperature:%.2f,humidity:%.2f\r\n",temperature/1000.0,humidity/1000.0);

	return OK ;
}

