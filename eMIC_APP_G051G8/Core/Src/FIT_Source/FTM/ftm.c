/****************************************************************************
 * include
 ****************************************************************************/

#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "ftm.h"
#include "ftm_common.h"
#include "ftm_wpptest.h"
#include "ftm_ringbuff.h"
#include "FIT_DebugMessage.h"

/****************************************************************************
 * define
 ****************************************************************************/

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define C2C(c0,c1) ((c0 << 8) | c1)

uint16_t rxindex;
RingBuffer rb;

typedef int (*CMD_FUNCTION)(CMD_DATA* );

struct ftm_state_s ftm_state_p;

typedef struct {
    char*		name;
	int             num_param;
	CMD_FUNCTION    func;
} CMD_TABLE;

CMD_TABLE cmd_table_group[] = 
{	
	{"emic cart info"                            ,  0  , FTM_cartridge_info                 },
	{"emic system info"                          ,  0  , FTM_system_info                    },
	{"emic get th"                               ,  0  , FTM_get_th                         },
	{"emic note"                                 ,  0  , FTM_note                           },
	{"emic fw log enable"                        ,  1  , FTM_fw_log_enable                  },
	{"emic set cart1 model number"               ,  1  , FTM_w_cart1_model_number           },
	{"emic set cart1 version latter"             ,  1  , FTM_w_cart1_version_latter         },
	{"emic set cart1 sn"                         ,  1  , FTM_w_cart1_serial_number          },
	{"emic set cart1 sensitivity"                ,  1  , FTM_w_cart1_sensitivity            },
	{"emic set cart1 ref freq"                   ,  1  , FTM_w_cart1_ref_freq               },
	{"emic set cart1 uints code"                 ,  1  , FTM_w_cart1_uints_code             },
	{"emic set cart1 freq range min"             ,  1  , FTM_w_cart1_freq_min               },
	{"emic set cart1 freq range max"             ,  1  , FTM_w_cart1_freq_max               },
	{"emic set cart1 ch assign     "             ,  1  , FTM_w_cart1_ch_assign              },
	{"emic set cart2 model number"               ,  1  , FTM_w_cart2_model_number           },
	{"emic set cart2 version latter"             ,  1  , FTM_w_cart2_version_latter         },
	{"emic set cart2 sn"                         ,  1  , FTM_w_cart2_serial_number          },
	{"emic set cart2 sensitivity"                ,  1  , FTM_w_cart2_sensitivity            },
	{"emic set cart2 ref freq"                   ,  1  , FTM_w_cart2_ref_freq               },
	{"emic set cart2 uints code"                 ,  1  , FTM_w_cart2_uints_code             },
	{"emic set cart2 freq range min"             ,  1  , FTM_w_cart2_freq_min               },
	{"emic set cart2 freq range max"             ,  1  , FTM_w_cart2_freq_max               },
	{"emic set cart2 ch assign"                  ,  1  , FTM_w_cart2_ch_assign              },
	{"emic set sys digit type"                   ,  1  , FTM_w_sys_digit_type               },
	{"emic set sys bit clock freq"               ,  1  , FTM_w_sys_bit_clock_freq           },
	{"emic set sys word length"                  ,  1  , FTM_w_sys_word_length              },
	{"emic set sys sample rate"                  ,  1  , FTM_w_sys_sample_rate              },
	{"emic set sys sn"                           ,  1  , FTM_w_sys_serial_number            },
	{"emic set sys sensitivity"                  ,  1  , FTM_w_sys_sensitivity              },
	{"emic set sys cal date"                     ,  1  , FTM_w_sys_cal_date                 },
	{"emic set sys manuf id"                     ,  1  , FTM_w_sys_manuf_id                 },
	{"emic set note"                             ,  1  , FTM_w_note                         },
	{"pcmd read"                                 ,  1  , FTM_r_pcmd                         },
	{"pcmd write"                                ,  1  , FTM_w_pcmd                         },
	{"ee read"                                   ,  1  , FTM_r_ee                           },
	{"ee write"                                  ,  1  , FTM_w_ee                           },

};

static int ftmd_de_capitalize(char *c )
{
	/* 'a': 0x61, 'A': 0x41 */
	if ( (*c) < 'a')
	(*c) += 'a' - 'A';

	return OK;
}

static void write_response(const int ret, char* result)
{
	switch (ret)
	{
		case OK:
		strcat(result, "emic:OK\r\n");
		break;
		case ERROR:
		strcat(result, "emic:ERROR\r\n");
		break;
		case TIMEOUT:
		strcat(result, "emic:TIMEOUT\r\n");
		break;
		case NOTSUPPORT:
		strcat(result, "emic:NOT SUPPORT\r\n");
		break;
		default:
		strcat(result, "emic:NOT SUPPORT\r\n");
		break;
	}
	
	FTM_printf("%s",result);
}

static int cmd_func_group(const char* cmd, CMD_DATA* cmd_data, CMD_TABLE* cmd_table, int array_size)
{
	char *cmd_name,*cmd_param;
	int len_cmd_name, len_cmd_param;

	for (int i = 0; i < array_size; i++)
	{
		cmd_name = (cmd_table+i)->name;
		len_cmd_name = strlen(cmd_name);
		
		if (strncasecmp(cmd, cmd_name, len_cmd_name) == 0)
		{
			cmd_param = (char*)cmd + len_cmd_name;
			len_cmd_param = strlen(cmd_param);
			/* if there are no parameters, len_cmd_param should be 0 */
			if ((cmd_table+i)->num_param == 0)
			{
				if (len_cmd_param > 0)	break;
			}
			else
			{
				if (len_cmd_param >= 128 || len_cmd_param <= 1 || cmd_param[0] != ' ') 	break;
				strncpy(cmd_data->param, cmd_param+1, len_cmd_param-1);
			}
			return ((cmd_table+i)->func(cmd_data));
		}

	}
	//printf("ERROR: No this command!!!\n");

	return NOTSUPPORT;
}

static void ftm_cmd_determine(char* cmd)
{
	CMD_DATA cmd_data = {{0}};
	int len_cmd = strlen(cmd);
	int ret = NOTSUPPORT;
	int array_size;
	
	if( len_cmd == 0 )
	{
		ret = OK;
		write_response(ret, cmd_data.result);
	}
	else
	{
		ftmd_de_capitalize(&cmd[0]);
		ftmd_de_capitalize(&cmd[1]);
		
		switch(C2C(cmd[0],cmd[1]))
		{	
			case C2C('e','m'):
			case C2C('p','c'):
			case C2C('e','e'):
			array_size = ARRAY_SIZE(cmd_table_group);
			ret = cmd_func_group(cmd, &cmd_data, &cmd_table_group[0], array_size);
			break;
		}
		write_response(ret, cmd_data.result);
	}
}

void ftm_clear_cmd_line()
{
	rxindex = 0;
	for (uint8_t i = 0; i < FTMD_CMD_LINE_MAX_SIZE; i++) ftm_state_p.ftm_cmd_line[i] = 0; // Clear the string buffer
}

void ftm_get_conmmand(uint8_t rxBuffer)
{
	//static uint16_t rxindex;
	static uint8_t	last_rxBuffer;
	
	if (((rxBuffer == FTMD_CR) || (rxBuffer == FTMD_LF))) // If Enter
	{
		ftm_state_p.ftm_cmd_line[rxindex] = 0;
		int len_cmd = strlen(ftm_state_p.ftm_cmd_line);
		if( !((last_rxBuffer == FTMD_CR) && (rxBuffer == FTMD_LF))&&len_cmd!=0)
		{
			/* call FTM (-,-) here */
			//printf("%s",ftm_state_p.ftm_cmd_line);
			ftm_cmd_determine(ftm_state_p.ftm_cmd_line);
		}
		ftm_clear_cmd_line();//cear the string buffer
	}
	else
	{
		ftm_state_p.ftm_cmd_line[rxindex] = rxBuffer; // Add that character to the string
		rxindex++;
		if (rxindex > FTMD_CMD_LINE_MAX_SIZE) // User typing too much, we can't have commands that big
		{
			ftm_clear_cmd_line(); // Clear the string buffer
		}
	}
	last_rxBuffer = rxBuffer;
}

void func_init(void)
{
	cbInit(&rb,FTMD_RX_DUMP_SIZE);
}

void Receive_command(void)
{
	ElemType Read_Byte;
	
	while(!cbIsEmpty(&rb))
	{
		cbRead(&rb,&Read_Byte);

		ftm_get_conmmand(Read_Byte);
	}
}

