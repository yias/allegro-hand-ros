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
 * 	@file pcan.cpp
 *  @brief CAN API implementation to support PEAK CAN interface
 *
 *  Created on: 		July 29, 2016
 *  Added to Project: 	July 29, 2016
 *  Author: 			Sean Yi
 *  Maintained by:      Sean Yi(seanyi@wonikrobotics.com)
 */

/*======================*/
/*       Includes       */
/*======================*/
//system headers
#include <stdio.h>
#include <errno.h>
#ifndef _WIN32
#include <inttypes.h>
#include <pthread.h>
#include <syslog.h>
#include <unistd.h>
#include <fcntl.h>
#else
#include <windows.h>
#endif
#include <malloc.h>
#include <assert.h>
//project headers
extern "C" {
#ifndef _WIN32
#include "libpcan/libpcan.h"
#else
#include "Peak/PCANBasic.h"
#endif
}
#include "candef.h"
#include "candrv.h"

#include "ros/ros.h" // for ROS_ERROR, ROS_INFO


CANAPI_BEGIN

/*=====================*/
/*       Defines       */
/*=====================*/
//macros
#define isAlpha(c) ( ((c >= 'A') && (c <= 'Z')) ? 1 : 0 )
#define isSpace(c) ( (c == ' ') ? 1 : 0 )
#define isDigit(c) ( ((c >= '0') && (c <= '9')) ? 1 : 0 )
#define ADDR2NODE(x) ((((x) >> 5) & 0x001F) - BASE_ID)
#define NODE2ADDR(x) (((mbxID + BASE_ID) << 5) | ((x) + BASE_ID))
#define GROUPID(n)   (((mbxID + BASE_ID) << 5) | (0x0400 + (n)))
#define BROADCAST    (GROUPID(0))
#define Border(Value,Min,Max)  (Value<Min)?Min:((Value>Max)?Max:Value)

/*=========================================*/
/*       Global file-scope variables       */
/*=========================================*/

/*==========================================*/
/*       Private functions prototypes       */
/*==========================================*/
int canInit(void* ch)
{
    int err;
    int i;
    char CanVersion[VERSIONSTRING_LEN];
    TPCANRdMsg CanMsg;

    err = CAN_VersionInfo(ch, CanVersion);
    if (err) {
        ROS_ERROR("CAN: Error in CAN_VersionInfo()");
    } else {
        ROS_INFO("CAN: Version info: %s", CanVersion);
    }

    ROS_INFO("CAN: Initializing device");
    // init to an user defined bit rate
    err = CAN_Init(ch, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
    if (err) {
        ROS_ERROR("CAN: Error in CAN_Init()");
        return err;
    }

    ROS_INFO("CAN: Clearing the CAN buffer");
    for (i = 0; i < 100; i++) {
        LINUX_CAN_Read_Timeout(ch, &CanMsg, 1000);
    }

    return 0; // PCAN_ERROR_OK
}

int canReadMsg(void* ch, can_msg& msg, int blocking, int timeout_usec)
{
    int err;
    int i;
    TPCANRdMsg CanMsg;

    if (blocking || timeout_usec < 0) {
        err = LINUX_CAN_Read(ch, &CanMsg);
    }
    else {
        err = LINUX_CAN_Read_Timeout(ch, &CanMsg, timeout_usec);
        if (CAN_ERR_QRCVEMPTY == err) // when receive queue is empty...
            return err;
    }

    if (err) {
        ROS_ERROR("CAN: CAN_Read() failed with error %x", err);
        return err;
    }

    msg.msg_id = CanMsg.Msg.ID;
    msg.data_length = CanMsg.Msg.LEN;
    for(i = 0; i < CanMsg.Msg.LEN; i++)
        msg.data[i] = CanMsg.Msg.DATA[i];

    return 0;
}

int canSendMsg(void* ch, can_msg& msg, int blocking, int timeout_usec)
{
    int err;
    int i;
    TPCANMsg CanMsg;
    CanMsg.ID = msg.msg_id;
    CanMsg.MSGTYPE = MSGTYPE_STANDARD;//msg.msg_type;
    CanMsg.LEN = msg.data_length;
    for(i = 0; i < msg.data_length; i++)
        CanMsg.DATA[i] = msg.data[i];

    if (blocking || timeout_usec < 0)
        err = CAN_Write(ch, &CanMsg);
    else
        err = LINUX_CAN_Write_Timeout(ch, &CanMsg, timeout_usec);

    if (err)
        ROS_ERROR("CAN: CAN_Write() failed with error %d", err);

    return err;
}

/*========================================*/
/*       Public functions (CAN API)       */
/*========================================*/
int can_open(void*& ch)
{
    ROS_ERROR("CAN: Error! Unsupported function call, can_open(void*&)");
    return -1;
}

int can_open_with_name(void*& ch, const char* dev_name)
{
    int err;

    ROS_INFO("CAN: Opening device on channel [%s]", dev_name);
    ch = LINUX_CAN_Open(dev_name, O_RDWR);
    if (!ch) {
        ROS_ERROR("CAN: Error in CAN_Open() on channel %s", dev_name);
        return -1;
    }
    else {
        ROS_INFO("CAN: Channel %s is opend successfully", dev_name);
    }

    err = canInit(ch);
    return err;
}

int can_open_ex(void*& ch, int type, int index)
{
    ROS_ERROR("CAN: Error! Unsupported function call, can_open(void*&, int, int)");
    return -1;
}

int can_reset(void* ch)
{
    return -1;
}

int can_flush(void* ch)
{
    int i;
    TPCANRdMsg CanMsg;

    for (i = 0; i < 100; i++) {
        LINUX_CAN_Read_Timeout(ch, &CanMsg, 1000);
    }

    return 0;
}

int can_close(void* ch)
{
    int err;

    err = CAN_Close(ch);
    if (err) {
        ROS_ERROR("CAN: Error in CAN_Close()");
        return -1;
    }

    return 0; // PCAN_ERROR_OK
}

int sys_init(void* ch, int period_msec)
{
    int err;
    can_msg msg;

    msg.msg_id = ((unsigned long)ID_CMD_SET_PERIOD<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
    msg.data_length = 1;
    msg.data[0] = (unsigned char)period_msec;
    err = canSendMsg(ch, msg, TRUE, 0);
    if (err) return err;
    usleep(10000);

    msg.msg_id = ((unsigned long)ID_CMD_SET_MODE_TASK<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
    msg.data_length = 0;
    err = canSendMsg(ch, msg, TRUE, 0);
    if (err) return err;
    usleep(10000);

    msg.msg_id = ((unsigned long)ID_CMD_QUERY_STATE_DATA<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
    msg.data_length = 0;
    err = canSendMsg(ch, msg, TRUE, 0);
    if (err) return err;
    usleep(10000);

    return 0;
}

int sys_start(void* ch)
{
    int err;
    can_msg msg;

    msg.msg_id = ((unsigned long)ID_CMD_QUERY_STATE_DATA<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
    msg.data_length = 0;
    err = canSendMsg(ch, msg, TRUE, 0);
    if (err) return err;
    usleep(10000);

    msg.msg_id = ((unsigned long)ID_CMD_SET_SYSTEM_ON<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
    msg.data_length = 0;
    err = canSendMsg(ch, msg, TRUE, 0);
    if (err) return err;
    usleep(10000);

    return 0;
}

int sys_stop(void* ch)
{
    int err;
    can_msg msg;

    msg.msg_id = ((unsigned long)ID_CMD_SET_SYSTEM_OFF<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
    msg.data_length = 0;
    err = canSendMsg(ch, msg, TRUE, 0);
    //usleep(10000);

    return err;
}

int query_id(void* ch)
{
    int err;
    can_msg msg;
    msg.msg_id = ((unsigned long)ID_CMD_QUERY_ID<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
    msg.data_length = 0;
    err = canSendMsg(ch, msg, TRUE, 0);
    return err;
}

int ahrs_init(void* ch, unsigned char rate, unsigned char mask)
{
    int err;
    can_msg msg;
    msg.msg_id = ((unsigned long)ID_CMD_AHRS_SET<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
    msg.data_length = 2;
    msg.data[0] = (unsigned char)rate;
    msg.data[1] = (unsigned char)mask;
    err = canSendMsg(ch, msg, TRUE, 0);
    return err;
}

int write_current(void* ch, int findex, short* pwm)
{
    int err;
    can_msg msg;

    if (findex < 0 || findex > 3)
        return -1;

    msg.msg_id = ((unsigned long)(ID_CMD_SET_TORQUE_1 + findex)<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
    msg.data_length = 8;
    msg.data[0] = (unsigned char)( (pwm[0] >> 8) & 0x00ff);
    msg.data[1] = (unsigned char)(pwm[0] & 0x00ff);
    msg.data[2] = (unsigned char)( (pwm[1] >> 8) & 0x00ff);
    msg.data[3] = (unsigned char)(pwm[1] & 0x00ff);
    msg.data[4] = (unsigned char)( (pwm[2] >> 8) & 0x00ff);
    msg.data[5] = (unsigned char)(pwm[2] & 0x00ff);
    msg.data[6] = (unsigned char)( (pwm[3] >> 8) & 0x00ff);
    msg.data[7] = (unsigned char)(pwm[3] & 0x00ff);
    err = canSendMsg(ch, msg, TRUE, 0);

    return err;
}

int can_write_message(void* ch, char cmd, char src, char des, int len, unsigned char* data, int blocking, int timeout_usec)
{
    int err;
    int i;
    can_msg msg;

    msg.msg_id = ((unsigned long)cmd<<6) | ((unsigned long)des <<3) | ((unsigned long)src);
    msg.msg_type = MSGTYPE_STANDARD;
    msg.data_length = len;
    for (i = 0; i < len; i++) {
        msg.data[i] = data[i];
    }
    err = canSendMsg(ch, msg, blocking, timeout_usec);

    return err;
}

int can_read_message(void* ch, char* cmd, char* src, char* des, int* len, unsigned char* data, int blocking, int timeout_usec)
{
    int err;
    can_msg msg;

    err = canReadMsg(ch, msg, blocking, timeout_usec);
    if (err)
        return err;

    *cmd = (char)( (msg.msg_id >> 6) & 0x1f );
    *des = (char)( (msg.msg_id >> 3) & 0x07 );
    *src = (char)( msg.msg_id & 0x07);
    *len = msg.data_length;
    for(int nd=0; nd<msg.data_length; nd++) data[nd] = msg.data[nd];

    /*printf("    cmd:=%xh src:=%xh des:=%xh (len:=%d)", *cmd, *src, *des, *len);
    for(int nd=0; nd<msg.data_length; nd++) printf(" %3d ", msg.data[nd]);
    printf("\n");*/

    return 0;
}

CANAPI_END
