#ifndef __Command_H__
#define __Command_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************

#ifdef __cplusplus
extern "C" {
#endif


#define hBYTE 0x55
#define flash_base_address 0x08000000

#define MAXBUFFERSIZE   256  


#define CMD_FAIL              0x00
#define CMD_SUCCESS           0x01
#define CMD_CHECKSUMERROR     0x02

#define STARTBYTE         0x00
#define PAYLOADLEN        0x01
#define CMDID             0x02
#define CMDDATA           0x03

#define BLOCK_ID_PAGE0 0x00
#define BLOCK_ID_PAGE2 0x01
#define BLOCK_ID_PAGE3 0x02
#define BLOCK_ID_PAGE4 0x03
#define BLOCK_ID_TBD1  0x04
#define BLOCK_ID_TBD2  0x05
#define BLOCK_ID_INFO1 0x06
#define BLOCK_ID_INFO2 0x07
  
#define ACK_CMD                                          0x00
#define GET_FW_VERSION_CMD                               0x01
#define GET_FW_VERSION_RESPONSE_CMD                      0x02
#define GET_TH_CMD                                       0x03
#define GET_TH_RESPONSE_CMD                              0x04
#define WRITE_EEPROM_BLOCK_CMD                           0x05
#define READ_EEPROM_BLOCK_CMD                            0x06
#define READ_EEPROM_BLOCK_RESPONSE_CMD                   0x07
#define WRITE_EEPROM_CMD                                 0x08
#define READ_EEPROM_CMD                                  0x09
#define READ_EEPROM_RESPONSE_CMD                         0x0A
#define WRITE_PCMD_BLOCK_CMD                             0x0B
#define READ_PCMD_BLOCK_CMD                              0x0C
#define READ_PCMD_BLOCK_RESPONSE_CMD                     0x0D
#define WRITE_PCMD_REG_CMD                               0x0E
#define READ_PCMD_REG_CMD                                0x0F
#define READ_PCMD_REG_RESPONSE_CMD                       0x10
#define WRITE_PCMD_REG_WITH_PAGE_PARAMETERS_CMD          0x11
#define READ_PCMD_REG_WITH_PAGE_PARAMETERS_CMD           0x12
#define READ_PCMD_REG_WITH_PAGE_PARAMETERS_RESPONSE_CMD  0x13
#define WRITE_MODEL_NUMBER_CMD                           0x14
#define READ_MODEL_NUMBER_CMD                            0x15
#define READ_MODEL_NUMBER_RESPONSE_CMD                   0x16
#define WRITE_VERSION_LETTER_CMD                         0x17
#define READ_VERSION_LETTER_CMD                          0x18
#define READ_VERSION_LETTER_RESPONSE_CMD                 0x19
#define WRITE_SERIAL_NUMBER_CMD                          0x1A
#define READ_SERIAL_NUMBER_CMD                           0x1B
#define READ_SERIAL_NUMBER_RESPONSE_CMD                  0x1C
#define WRITE_SENSITIVITY_CMD                            0x1D
#define READ_SENSITIVITY_CMD                             0x1E
#define READ_SENSITIVITY_RESPONSE_CMD                    0x1F
#define WRITE_REFERENCE_FREQUENCY_CMD                    0x20
#define READ_REFERENCE_FREQUENCY_CMD                     0x21
#define READ_REFERENCE_FREQUENCY_RESPONSE_CMD            0x22
#define WRITE_UNITS_CODE_CMD                             0x23
#define READ_UNITS_CODE_CMD                              0x24
#define READ_UNITS_CODE_RESPONSE_CMD                     0x25
#define WRITE_FREQUENCY_RANG_MIN_CMD                     0x26
#define READ_FREQUENCY_RANG_MIN_CMD                      0x27
#define READ_FREQUENCY_RANG_MIN_RESPONSE_CMD             0x28
#define WRITE_FREQUENCY_RANG_MAX_CMD                     0x29
#define READ_FREQUENCY_RANG_MAX_CMD                      0x2A
#define READ_FREQUENCY_RANG_MAX_RESPONSE_CMD             0x2B
#define WRITE_CHANNEL_ASSIGNMEN_CMD                      0x2C
#define READ_CHANNEL_ASSIGNMEN_CMD                       0x2D
#define READ_CHANNEL_ASSIGNMEN_RESPONSE_CMD              0x2E
#define WRITE_DIGITAL_INTERFACE_TYPE_CMD                 0x2F
#define READ_DIGITAL_INTERFACE_TYPE_CMD                  0x30
#define READ_DIGITAL_INTERFACE_TYPE_RESPONSE_CMD         0x31
#define WRITE_BIT_CLOCK_FREQUENCY_CMD                    0x32
#define READ_BIT_CLOCK_FREQUENCY_CMD                     0x33
#define READ_BIT_CLOCK_FREQUENCY_RESPONSE_CMD            0x34
#define WRITE_WORD_LENGTH_CMD                            0x35
#define READ_WORD_LENGTH_CMD                             0x36
#define READ_WORD_LENGTH_RESPONSE_CMD                    0x37
#define WRITE_SAMPLE_RATE_CMD                            0x38
#define READ_SAMPLE_RATE_CMD                             0x39
#define READ_SAMPLE_RATE_RESPONSE_CMD                    0x3A
#define WRITE_CALIBRATION_DATE_CMD                       0x3B
#define READ_CALIBRATION_DATE_CMD                        0x3C
#define READ_CALIBRATION_DATE_RESPONSE_CMD               0x3D
#define WRITE_MANUFACTUER_ID_CMD                         0x3E
#define READ_MANUFACTUER_ID_CMD                          0x3F
#define READ_MANUFACTUER_ID_RESPONSE_CMD                 0x40
#define WRITE_FW_VERSION_CMD                             0x41
#define READ_FW_VERSION_CMD                              0x42
#define READ_FW_VERSION_RESPONSE_CMD                     0x43
#define WRITE_NOTE_CMD                                   0x44
#define READ_NOTE_CMD                                    0x45
#define READ_NOTE_RESPONSE_CMD                           0x46
#define REBOOT_CMD                                       0xAA
#define GATMODE_CMD                                      0xB0
#define OTA_ENTER_UPGRADE_MODE_CMD                       0xB1
#define OTA_UPGRADE_INIT_CMD                             0xB2
#define OTA_UPGRADE_START_CMD                            0xB3
#define OTA_UPGRADE_END_CMD                              0xB4
#define DEBUG_MSG_RESPONE_CMD                            0xFF

  
  
  
  
  
  
  
//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
