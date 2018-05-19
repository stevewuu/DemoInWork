/*
 * bl.h
 *
 *  Created on: 2017Äê8ÔÂ5ÈÕ
 *      Author: 4337
 */

#ifndef BL_H_
#define BL_H_

#include "device_registers.h"

void bootup_application(uint32_t appEntry, uint32_t appStack);
uint8_t check_sum(uint8_t *data,uint8_t data_size);
void download_app(void);
#endif /* BL_H_ */
