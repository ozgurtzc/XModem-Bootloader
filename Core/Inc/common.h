/*
 * common.h
 *
 *  Created on: Sep 11, 2021
 *      Author: OZGUR
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#define SOF '$'
#define EOF '#'

#define DATA_LENGTH 128

typedef enum
{
    IDLE = 0,
	SOF_RECEIVED,
	DATA_RECEIVED,
} RX_STATES;

#endif /* INC_COMMON_H_ */
