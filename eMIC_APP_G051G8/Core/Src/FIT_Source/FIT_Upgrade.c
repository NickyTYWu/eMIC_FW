 #include "main.h"
 #include "FIT_FLASH.h"
 #include "FIT_DebugMessage.h"

#define SIZE_OF_FLASH_SECTOR             2048

uint32_t Total_Image_Size=0;
uint32_t upgradeStartAddress=0;
uint32_t tempUpgradeStartAddress=0;
bool bUpgradeStart=false;

uint16_t number_of_data_in_flash_buffer;
uint8_t flash_buffer[2048];
uint8_t tempBuf[64];


bool total_image_checksum_calculate(uint32_t total_checksum)
{
    uint32_t estimated_total_checksum=0;

    uint8_t *image = (uint8_t*)upgradeStartAddress;

    for(int i=0;i<Total_Image_Size;i++)
    {
        estimated_total_checksum+=image[i];
    }

    FT_printf("estimated_total_checksum:%lx estimated_total_checksum:%lx\r\n",estimated_total_checksum,total_checksum);

    if (estimated_total_checksum == total_checksum)
    {
		    return true;
    }

    return false;
}

uint32_t check_image_checkSum(uint8_t *body_buffer)
{
    uint32_t file_size = 0, totalChecksum = 0;

    file_size = ((body_buffer[0]<<24)|(body_buffer[1]<<16)|(body_buffer[2]<<8)|(body_buffer[3]));

    totalChecksum = ((body_buffer[4]<<24)|(body_buffer[5]<<16)|(body_buffer[6]<<8)|(body_buffer[7]));

    if(total_image_checksum_calculate(totalChecksum))
    {
        return file_size;
    }

    return 0;
}

bool UPGRADE_END(uint8_t *body_buffer,uint16_t len)
{
    uint32_t image_size = 0;

    bool ReturnValue=false;
    if(number_of_data_in_flash_buffer)
    {
        mem_flash_write(tempUpgradeStartAddress,flash_buffer,number_of_data_in_flash_buffer);
        tempUpgradeStartAddress+=number_of_data_in_flash_buffer;
    }
    //printf("END OTA_UPGRADE_START:%x\r\n",OTA_SD_START_ADDR);
    image_size=check_image_checkSum(body_buffer);

    if(image_size>0)
    {
        ReturnValue=true;
    }

    return ReturnValue;
}

void UPGRADE_START(uint8_t *body_buffer,uint16_t len)
{
    memcpy(&flash_buffer[number_of_data_in_flash_buffer],body_buffer,len);

    number_of_data_in_flash_buffer+=len;

    if(number_of_data_in_flash_buffer>=SIZE_OF_FLASH_SECTOR)
    {
        mem_flash_write(tempUpgradeStartAddress,flash_buffer,number_of_data_in_flash_buffer);

        tempUpgradeStartAddress+=number_of_data_in_flash_buffer;

        number_of_data_in_flash_buffer=0;
    }
}

bool UPGRADE_INIT(uint8_t *CMDBuf)
{
    Total_Image_Size=((CMDBuf[0]<<24)|(CMDBuf[1]<<16)|(CMDBuf[2]<<8)|(CMDBuf[3]));
    tempUpgradeStartAddress=upgradeStartAddress=((CMDBuf[4]<<24)|(CMDBuf[5]<<16)|(CMDBuf[6]<<8)|(CMDBuf[7]));

    number_of_data_in_flash_buffer=0;

    bUpgradeStart=true;

    return bUpgradeStart;
}
