/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Wonik Robotics.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Wonik Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * 	@file candef.h
 *  @brief definition of constants used by CAN API
 *
 *  Created on: 		July 29, 2016
 *  Added to Project: 	July 29, 2016
 *  Author: 			Sean Yi
 *  Maintained by:      Sean Yi(seanyi@wonikrobotics.com)
 */

#ifndef __ALLEGROHAND_CANDEF_H__
#define __ALLEGROHAND_CANDEF_H__

#ifdef USING_NAMESPACE_CANAPI
#   define CANAPI_BEGIN namespace CANAPI {
#   define CANAPI_END };
#else
#   define CANAPI_BEGIN
#   define CANAPI_END
#   define CANAPI
#endif

CANAPI_BEGIN

#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (1)
#endif

///////////////////////////////////////////////
///  Define CAN message structure
typedef struct{
    unsigned char	msg_type;
    unsigned long	msg_id;         // message identifier
    unsigned char	data_length;    //
    char			data[8];        // data array
} can_msg;

///////////////////////////////////////////////
///  Define CAN Constants
#define TX_QUEUE_SIZE       (32)
#define RX_QUEUE_SIZE       (32)
#define TX_TIMEOUT          (5)
#define RX_TIMEOUT          (5)
#define mbxID               (0)
#define BASE_ID             (0)
#define MAX_BUS             (256)

#define		STD             (bool)0
#define		EXT             (bool)1

///////////////////////////////////////////////
///  Define Control Board ID number
#define ID_COMMON			0x01
#define ID_DEVICE_MAIN		0x02
#define ID_DEVICE_SUB_01	0x03
#define ID_DEVICE_SUB_02	0x04
#define ID_DEVICE_SUB_03	0x05
#define ID_DEVICE_SUB_04	0x06
#define ID_DEVICE_SUB_05	0x07
#define ID_DEVICE_SUB_06	0x08
#define ID_DEVICE_SUB_07	0x09
#define ID_DEVICE_SUB_08	0x0a
#define ID_DEVICE_SUB_09	0x0b
#define ID_DEVICE_SUB_10	0x0c

////////////////////////////////////////////////
///  Define CAN Command
#define ID_CMD_SET_SYSTEM_ON			0x01
#define ID_CMD_SET_SYSTEM_OFF			0x02
#define ID_CMD_SET_PERIOD				0x03
#define ID_CMD_SET_MODE_JOINT			0x04
#define ID_CMD_SET_MODE_TASK			0x05
#define ID_CMD_SET_TORQUE_1				0x06
#define ID_CMD_SET_TORQUE_2				0x07
#define ID_CMD_SET_TORQUE_3				0x08
#define ID_CMD_SET_TORQUE_4				0x09
#define ID_CMD_SET_POSITION_1			0x0a
#define ID_CMD_SET_POSITION_2			0x0b
#define ID_CMD_SET_POSITION_3			0x0c
#define ID_CMD_SET_POSITION_4			0x0d
#define ID_CMD_QUERY_STATE_DATA			0x0e
#define ID_CMD_QUERY_CONTROL_DATA		0x0f
#define ID_CMD_QUERY_ID					0x10
#define ID_CMD_AHRS_SET					0x11
#define ID_CMD_AHRS_POSE				0x12
#define ID_CMD_AHRS_ACC					0x13
#define ID_CMD_AHRS_GYRO				0x14
#define ID_CMD_AHRS_MAG					0x15

////////////////////////////////////////////////
///  Define Control Channel in Device
#define MOTOR_CH_1			0x01
#define MOTOR_CH_2			0x02
#define MOTOR_CH_3			0x03
#define MOTOR_CH_4			0x04
#define MOTOR_CH_ALL		0x05
#define DEVICE_SYSTEM		0x06

////////////////////////////////////////////////
/// Define State
#define STATE_CH1			0x01
#define STATE_CH2			0x02
#define STATE_CH3			0x04
#define STATE_CH4			0x08
#define STATE_ALL			0x10
#define STATE_SYSTEM		0x80

////////////////////////////////////////////////
///   Define Motor State
#define SYSTEM_ON		0x01
#define SYSTEM_OFF		0x02
#define MOTOR_RUN		0x04
#define MOTOR_STOP		0x08

#define MOTOR_DIR_CW	0x01
#define MOTOR_DIR_CCW	0x02

#define ON	1
#define OFF 0

////////////////////////////////////////////////
///   Define AHRS Data Rate&Mask
#define AHRS_RATE_1Hz	0x00
#define AHRS_RATE_10Hz	0x01
#define AHRS_RATE_20Hz	0x02
#define AHRS_RATE_50Hz	0x03
#define AHRS_RATE_100Hz	0x04

#define AHRS_MASK_POSE	0x01
#define AHRS_MASK_ACC	0x02
#define AHRS_MASK_GYRO	0x04
#define AHRS_MASK_MAG	0x08

////////////////////////////////////////////////

CANAPI_END

#endif // __ALLEGROHAND_CANDEF_H__

