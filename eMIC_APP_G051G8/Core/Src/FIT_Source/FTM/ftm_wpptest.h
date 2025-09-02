#ifndef FTM_WPPTEST_H_
#define FTM_WPPTEST_H_

#define ftm_print printf

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct {
	char param[CMD_PARAM_SIZE];
	char result[CMD_RESULT_SIZE];
} CMD_DATA;

/****************************************************************************
 * FTM test item
 ****************************************************************************/
int hex_digit(char c);
int parse_ascii_values(const char* buf, uint16_t* out, int max_count);
uint8_t ascii2bcd(char high, char low);
int parse_date_ascii_to_bcd(const char* str, uint8_t bcd[6]);
int ascii_to_float(const char* str, float* out_value);
int ascii_to_int(const char* str, int* out_value);
int writeChAssign(uint8_t infoID,CMD_DATA* cmd);
int write_freq_max(uint8_t infoID,CMD_DATA* cmd);
int write_freq_min(uint8_t infoID,CMD_DATA* cmd);
int write_uints_code(uint8_t infoID,CMD_DATA* cmd);
int write_ref_freq(uint8_t infoID,CMD_DATA* cmd);
int write_sensitivity(uint8_t infoID,CMD_DATA* cmd);
int write_serial_number(uint8_t infoID,CMD_DATA* cmd);
int write_version_latter(uint8_t infoID,CMD_DATA* cmd);
int write_model_number(uint8_t infoID,CMD_DATA* cmd);
int FTM_api_version(CMD_DATA* );
int FTM_alive_test(CMD_DATA* );
int FTM_cartridge_info(CMD_DATA* cmd);
int FTM_get_th(CMD_DATA* cmd);
int FTM_note(CMD_DATA* cmd);
int FTM_w_note(CMD_DATA* cmd);
int FTM_system_info(CMD_DATA* cmd);
int FTM_fw_log_enable(CMD_DATA* cmd);
int FTM_w_cart1_model_number(CMD_DATA* cmd);
int FTM_w_cart1_version_latter(CMD_DATA* cmd);
int FTM_w_cart1_serial_number(CMD_DATA* cmd);
int FTM_w_cart1_sensitivity(CMD_DATA* cmd);
int FTM_w_cart1_ref_freq(CMD_DATA* cmd);
int FTM_w_cart1_uints_code(CMD_DATA* cmd);
int FTM_w_cart1_freq_min(CMD_DATA* cmd);
int FTM_w_cart1_freq_max(CMD_DATA* cmd);
int FTM_w_cart1_ch_assign(CMD_DATA* cmd);
int FTM_w_cart2_model_number(CMD_DATA* cmd);
int FTM_w_cart2_version_latter(CMD_DATA* cmd);
int FTM_w_cart2_serial_number(CMD_DATA* cmd);
int FTM_w_cart2_sensitivity(CMD_DATA* cmd);
int FTM_w_cart2_ref_freq(CMD_DATA* cmd);
int FTM_w_cart2_uints_code(CMD_DATA* cmd);
int FTM_w_cart2_freq_min(CMD_DATA* cmd);
int FTM_w_cart2_freq_max(CMD_DATA* cmd);
int FTM_w_cart2_ch_assign(CMD_DATA* cmd);
int FTM_w_sys_digit_type(CMD_DATA* cmd);
int FTM_w_sys_bit_clock_freq(CMD_DATA* cmd);
int FTM_w_sys_word_length(CMD_DATA* cmd);
int FTM_w_sys_sample_rate(CMD_DATA* cmd);
int FTM_w_sys_serial_number(CMD_DATA* cmd);
int FTM_w_sys_sensitivity(CMD_DATA* cmd);
int FTM_w_sys_cal_date(CMD_DATA* cmd);
int FTM_w_sys_manuf_id(CMD_DATA* cmd);
int FTM_r_pcmd(CMD_DATA* cmd);
int FTM_w_pcmd(CMD_DATA* cmd);
int FTM_r_ee(CMD_DATA* cmd);
int FTM_w_ee(CMD_DATA* cmd);

#endif /* FTM_WPPTEST_H_ */
