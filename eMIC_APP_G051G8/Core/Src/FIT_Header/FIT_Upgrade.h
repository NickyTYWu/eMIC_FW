/*
 * FIT_Upgrade.h
 *
 *  Created on: 2021?4?10?
 *      Author: fit0354
 */

bool total_image_checksum_calculate(uint32_t total_checksum);
uint32_t check_image_checkSum(uint8_t *body_buffer);
bool UPGRADE_END(uint8_t *body_buffer,uint16_t len);
bool UPGRADE_START(uint8_t *body_buffer,uint16_t len);
bool UPGRADE_INIT(uint8_t *CMDBuf);
