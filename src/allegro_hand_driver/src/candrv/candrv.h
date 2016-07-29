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

#ifndef __ALLEGROHAND_CANDRV_H__
#define __ALLEGROHAND_CANDRV_H__

/*
 * 	@file candrv.h
 *  @brief API for communication over CAN bus
 *  @detailed The API for communicating with the various motor controllers
 *          over the CAN bus interface on the robot hand
 *
 *  Created on: 		July 29, 2016
 *  Added to Project: 	July 29, 2016
 *  Author: 			Sean Yi
 *  Maintained by:      Sean Yi(seanyi@wonikrobotics.com)
 */

#include "candef.h"

CANAPI_BEGIN

/******************/
/* CAN device API */
/******************/
int can_open(void*& ch);
int can_open_with_name(void*& ch, const char* dev_name);
int can_open_ex(void*& ch, int type, int index);

int can_reset(void* ch);
int can_close(void* ch);
int can_flush(void* ch);

int sys_init(void* ch, int period_msec);
int sys_start(void* ch);
int sys_stop(void* ch);

int ahrs_init(void* ch, unsigned char rate, unsigned char mask); // since v3.0
int query_id(void* ch); // since v3.0

int write_current(void* ch, int findex, short* pwm);

int can_write_message(void* ch, char cmd, char src, char des, int len, unsigned char* data, int blocking, int timeout_usec);
int can_read_message(void* ch, char* cmd, char* src, char* des, int* len, unsigned char* data, int blocking, int timeout_usec);

CANAPI_END

#endif // __ALLEGROHAND_CANDRV_H__
