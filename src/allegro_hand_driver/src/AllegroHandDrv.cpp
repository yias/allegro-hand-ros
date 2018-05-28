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
 *  @file AllegroHandDrv.cpp
 *  @brief Allegro Hand Driver
 *
 *  Created on:         Nov 15, 2012
 *  Added to Project:   Jan 17, 2013
 *  Author:             Sean Yi, K.C.Chang, Seungsu Kim, & Alex Alspach
 *  Maintained by:      Sean Yi(seanyi@wonikrobotics.com)
 */

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string>
#include "ros/ros.h"
#include "candrv/candrv.h"
#include "allegro_hand_driver/AllegroHandDrv.h"

using namespace std;

#define PWM_LIMIT_ROLL 250.0*1.5
#define PWM_LIMIT_NEAR 450.0*1.5
#define PWM_LIMIT_MIDDLE 300.0*1.5
#define PWM_LIMIT_FAR 190.0*1.5

#define PWM_LIMIT_THUMB_ROLL 350.0*1.5
#define PWM_LIMIT_THUMB_NEAR 270.0*1.5
#define PWM_LIMIT_THUMB_MIDDLE 180.0*1.5
#define PWM_LIMIT_THUMB_FAR 180.0*1.5

#define PWM_LIMIT_GLOBAL_8V 800.0 // maximum: 1200
#define PWM_LIMIT_GLOBAL_24V 500.0

namespace allegro
{

AllegroHandDrv::AllegroHandDrv()
    : _can_handle(0)
    , _curr_position_get(0)
    , _emergency_stop(false)
{

    if (ros::param::has("~zero")) {
        _emergency_stop = false;
        ROS_INFO("CAN: Joint zeros and directions loaded from parameter server.");
    }
    else {
        ROS_ERROR("Encoder/Motor offsets and directions not loaded.");
        ROS_ERROR("Check launch file is loading /parameters/zero.yaml.");
        ROS_ERROR("Shutting down...");
        _emergency_stop = true;
    }

    // This version number is used to in setting the finger motor CAN channels
    // the channels used differ from versions 1.0 to 2.0

    ros::param::get("~hand_info/version", _hand_version);
    if (_hand_version == 3.0)
        _tau_cov_const = 1200.0;
    else
        _tau_cov_const = 800.0;
    ROS_INFO("Hand Version: %2.1f", _hand_version);

    if (ros::param::has("~hand_info/input_voltage")) {
        ros::param::get("~hand_info/input_voltage", _input_voltage);
        if (_input_voltage == 24.0)
            _pwm_max_global = PWM_LIMIT_GLOBAL_24V;
        else
            _pwm_max_global = PWM_LIMIT_GLOBAL_8V;
    }
    else {
        ROS_INFO("Input Voltage is not defined");
        ROS_INFO("Input Voltage is set as its default value");
        _input_voltage = 8.0;
        _pwm_max_global = PWM_LIMIT_GLOBAL_8V;
    }
    ROS_INFO("Input Voltage: %.1f Volts", _input_voltage);
    ROS_INFO("Max. PWM Input: %.1f", _pwm_max_global);

    _pwm_max[eJOINTNAME_INDEX_0] = min(_pwm_max_global, PWM_LIMIT_ROLL);
    _pwm_max[eJOINTNAME_INDEX_1] = min(_pwm_max_global, PWM_LIMIT_NEAR);
    _pwm_max[eJOINTNAME_INDEX_2] = min(_pwm_max_global, PWM_LIMIT_MIDDLE);
    _pwm_max[eJOINTNAME_INDEX_3] = min(_pwm_max_global, PWM_LIMIT_FAR);

    _pwm_max[eJOINTNAME_MIDDLE_0] = min(_pwm_max_global, PWM_LIMIT_ROLL);
    _pwm_max[eJOINTNAME_MIDDLE_1] = min(_pwm_max_global, PWM_LIMIT_NEAR);
    _pwm_max[eJOINTNAME_MIDDLE_2] = min(_pwm_max_global, PWM_LIMIT_MIDDLE);
    _pwm_max[eJOINTNAME_MIDDLE_3] = min(_pwm_max_global, PWM_LIMIT_FAR);

    _pwm_max[eJOINTNAME_PINKY_0] = min(_pwm_max_global, PWM_LIMIT_ROLL);
    _pwm_max[eJOINTNAME_PINKY_1] = min(_pwm_max_global, PWM_LIMIT_NEAR);
    _pwm_max[eJOINTNAME_PINKY_2] = min(_pwm_max_global, PWM_LIMIT_MIDDLE);
    _pwm_max[eJOINTNAME_PINKY_3] = min(_pwm_max_global, PWM_LIMIT_FAR);

    _pwm_max[eJOINTNAME_THUMB_0] = min(_pwm_max_global, PWM_LIMIT_THUMB_ROLL);
    _pwm_max[eJOINTNAME_THUMB_1] = min(_pwm_max_global, PWM_LIMIT_THUMB_NEAR);
    _pwm_max[eJOINTNAME_THUMB_2] = min(_pwm_max_global, PWM_LIMIT_THUMB_MIDDLE);
    _pwm_max[eJOINTNAME_THUMB_3] = min(_pwm_max_global, PWM_LIMIT_THUMB_FAR);

    ros::param::get("~zero/encoder_offset/j00", _encoder_offset[eJOINTNAME_INDEX_0]);
    ros::param::get("~zero/encoder_offset/j01", _encoder_offset[eJOINTNAME_INDEX_1]);
    ros::param::get("~zero/encoder_offset/j02", _encoder_offset[eJOINTNAME_INDEX_2]);
    ros::param::get("~zero/encoder_offset/j03", _encoder_offset[eJOINTNAME_INDEX_3]);
    ros::param::get("~zero/encoder_offset/j10", _encoder_offset[eJOINTNAME_MIDDLE_0]);
    ros::param::get("~zero/encoder_offset/j11", _encoder_offset[eJOINTNAME_MIDDLE_1]);
    ros::param::get("~zero/encoder_offset/j12", _encoder_offset[eJOINTNAME_MIDDLE_2]);
    ros::param::get("~zero/encoder_offset/j13", _encoder_offset[eJOINTNAME_MIDDLE_3]);
    ros::param::get("~zero/encoder_offset/j20", _encoder_offset[eJOINTNAME_PINKY_0]);
    ros::param::get("~zero/encoder_offset/j21", _encoder_offset[eJOINTNAME_PINKY_1]);
    ros::param::get("~zero/encoder_offset/j22", _encoder_offset[eJOINTNAME_PINKY_2]);
    ros::param::get("~zero/encoder_offset/j23", _encoder_offset[eJOINTNAME_PINKY_3]);
    ros::param::get("~zero/encoder_offset/j30", _encoder_offset[eJOINTNAME_THUMB_0]);
    ros::param::get("~zero/encoder_offset/j31", _encoder_offset[eJOINTNAME_THUMB_1]);
    ros::param::get("~zero/encoder_offset/j32", _encoder_offset[eJOINTNAME_THUMB_2]);
    ros::param::get("~zero/encoder_offset/j33", _encoder_offset[eJOINTNAME_THUMB_3]);

    ros::param::get("~zero/encoder_direction/j00", _encoder_direction[eJOINTNAME_INDEX_0]);
    ros::param::get("~zero/encoder_direction/j01", _encoder_direction[eJOINTNAME_INDEX_1]);
    ros::param::get("~zero/encoder_direction/j02", _encoder_direction[eJOINTNAME_INDEX_2]);
    ros::param::get("~zero/encoder_direction/j03", _encoder_direction[eJOINTNAME_INDEX_3]);
    ros::param::get("~zero/encoder_direction/j10", _encoder_direction[eJOINTNAME_MIDDLE_0]);
    ros::param::get("~zero/encoder_direction/j11", _encoder_direction[eJOINTNAME_MIDDLE_1]);
    ros::param::get("~zero/encoder_direction/j12", _encoder_direction[eJOINTNAME_MIDDLE_2]);
    ros::param::get("~zero/encoder_direction/j13", _encoder_direction[eJOINTNAME_MIDDLE_3]);
    ros::param::get("~zero/encoder_direction/j20", _encoder_direction[eJOINTNAME_PINKY_0]);
    ros::param::get("~zero/encoder_direction/j21", _encoder_direction[eJOINTNAME_PINKY_1]);
    ros::param::get("~zero/encoder_direction/j22", _encoder_direction[eJOINTNAME_PINKY_2]);
    ros::param::get("~zero/encoder_direction/j23", _encoder_direction[eJOINTNAME_PINKY_3]);
    ros::param::get("~zero/encoder_direction/j30", _encoder_direction[eJOINTNAME_THUMB_0]);
    ros::param::get("~zero/encoder_direction/j31", _encoder_direction[eJOINTNAME_THUMB_1]);
    ros::param::get("~zero/encoder_direction/j32", _encoder_direction[eJOINTNAME_THUMB_2]);
    ros::param::get("~zero/encoder_direction/j33", _encoder_direction[eJOINTNAME_THUMB_3]);

    ros::param::get("~zero/motor_direction/j00", _motor_direction[eJOINTNAME_INDEX_0]);
    ros::param::get("~zero/motor_direction/j01", _motor_direction[eJOINTNAME_INDEX_1]);
    ros::param::get("~zero/motor_direction/j02", _motor_direction[eJOINTNAME_INDEX_2]);
    ros::param::get("~zero/motor_direction/j03", _motor_direction[eJOINTNAME_INDEX_3]);
    ros::param::get("~zero/motor_direction/j10", _motor_direction[eJOINTNAME_MIDDLE_0]);
    ros::param::get("~zero/motor_direction/j11", _motor_direction[eJOINTNAME_MIDDLE_1]);
    ros::param::get("~zero/motor_direction/j12", _motor_direction[eJOINTNAME_MIDDLE_2]);
    ros::param::get("~zero/motor_direction/j13", _motor_direction[eJOINTNAME_MIDDLE_3]);
    ros::param::get("~zero/motor_direction/j20", _motor_direction[eJOINTNAME_PINKY_0]);
    ros::param::get("~zero/motor_direction/j21", _motor_direction[eJOINTNAME_PINKY_1]);
    ros::param::get("~zero/motor_direction/j22", _motor_direction[eJOINTNAME_PINKY_2]);
    ros::param::get("~zero/motor_direction/j23", _motor_direction[eJOINTNAME_PINKY_3]);
    ros::param::get("~zero/motor_direction/j30", _motor_direction[eJOINTNAME_THUMB_0]);
    ros::param::get("~zero/motor_direction/j31", _motor_direction[eJOINTNAME_THUMB_1]);
    ros::param::get("~zero/motor_direction/j32", _motor_direction[eJOINTNAME_THUMB_2]);
    ros::param::get("~zero/motor_direction/j33", _motor_direction[eJOINTNAME_THUMB_3]);
}

AllegroHandDrv::~AllegroHandDrv()
{
    if (_can_handle != 0) {
        ROS_INFO("CAN: System Off");
        CANAPI::sys_stop(_can_handle);
        usleep(10000);
        ROS_INFO("CAN: Close CAN channel");
        CANAPI::can_close(_can_handle);
    }
}

// trim from end. see http://stackoverflow.com/a/217605/256798
static inline std::string &rtrim(std::string &s)
{
    s.erase(std::find_if(
        s.rbegin(), s.rend(),
        std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

bool AllegroHandDrv::init(int mode)
{
    string CAN_CH;
    ros::param::get("~comm/CAN_CH", CAN_CH);
    rtrim(CAN_CH);  // Ensure the ROS parameter has no trailing whitespace.

    if (CAN_CH.empty()) {
        ROS_ERROR("Invalid (empty) CAN channel, cannot proceed. Check PCAN comms.");
        return false;
    }

    if (CANAPI::can_open_with_name(_can_handle, CAN_CH.c_str())) {
        _can_handle = 0;
        return false;
    }

    ROS_INFO("CAN: Flush CAN receive buffer");
    CANAPI::can_flush(_can_handle);
    usleep(100);

    ROS_INFO("CAN: System Off");
    CANAPI::sys_stop(_can_handle);
    usleep(100);

    ROS_INFO("CAN: Query firmware information");
    CANAPI::query_id(_can_handle);
    usleep(100);

    //ROS_INFO("CAN: Enable embedded AHRS sensor");
    //CANAPI::ahrs_init(_can_handle, AHRS_RATE_1Hz, AHRS_MASK_POSE);
    //usleep(100);

    ROS_INFO("CAN: Setting loop period(:= 3ms) and initialize system");
    CANAPI::sys_init(_can_handle, 3);

    ROS_INFO("CAN: System ON");
    CANAPI::sys_start(_can_handle);
    usleep(100);

    ROS_INFO("CAN: Communicating");

    return true;
}

int AllegroHandDrv::update(void)
{
    if (_emergency_stop)
        return -1;

    _readDevices();
    usleep(10);
    _writeDevices();

    if (_emergency_stop) {
        ROS_ERROR("Emergency stop in update()");
        return -1;
    }

    return 0;
}

void AllegroHandDrv::setTorque(double *torque)
{
    if (_hand_version == 1.0) {
        // for Allegro Hand v1.0
        for (int findex = 0; findex < 4; findex++) {
            _desired_torque[4*findex+0] = torque[4*findex+0];
            _desired_torque[4*findex+1] = torque[4*findex+1];
            _desired_torque[4*findex+2] = torque[4*findex+2];
            _desired_torque[4*findex+3] = torque[4*findex+3];
        }
    }
    else if (_hand_version >= 2.0) {
        // for Allegro Hand v2.0
        for (int findex = 0; findex < 4; findex++) {
            _desired_torque[4*findex+3] = torque[4*findex+0];
            _desired_torque[4*findex+2] = torque[4*findex+1];
            _desired_torque[4*findex+1] = torque[4*findex+2];
            _desired_torque[4*findex+0] = torque[4*findex+3];
        }
    }
    else {
        ROS_ERROR("CAN: Can not determine proper finger CAN channels. Check the Allegro Hand version number in 'zero.yaml'");
        return;
    }
}

void AllegroHandDrv::getJointInfo(double *position)
{
    for (int i = 0; i < DOF_JOINTS; i++) {
        position[i] = _curr_position[i];
    }
}

void AllegroHandDrv::_readDevices()
{
    int err;
    char cmd, src, des;
    int len;
    unsigned char data[8];

    err = CANAPI::can_read_message(_can_handle, &cmd, &src, &des, &len, data, 0, 0);
    while (!err) {
        _parseMessage(cmd, src, des, len, data);
        err = CANAPI::can_read_message(_can_handle, &cmd, &src, &des, &len, data, 0, 0);
    }
}

void AllegroHandDrv::_writeDevices()
{
    double pwmDouble[DOF_JOINTS];
    short pwm[DOF_JOINTS];

    if (!(_curr_position_get == (0x01 | 0x02 | 0x04 | 0x08)))
        return;

    // convert to torque to pwm
    for (int i = 0; i < DOF_JOINTS; i++) {
        pwmDouble[i] = _desired_torque[i] * 1.0 * (double) _motor_direction[i] * _tau_cov_const;

        // limitation should be less than 800
        if (pwmDouble[i] > _pwm_max[i]) {
            pwmDouble[i] = _pwm_max[i];
        }
        else if (pwmDouble[i] < -_pwm_max[i]) {
            pwmDouble[i] = -_pwm_max[i];
        }

        pwm[i] = (short) pwmDouble[i];
    }

    for (int findex = 0; findex < 4; findex++) {
        CANAPI::write_current(_can_handle, findex, &pwm[findex*4]);
        //ROS_INFO("write torque %d: %d %d %d %d", findex, pwm[findex*4+0], pwm[findex*4+1], pwm[findex*4+2], pwm[findex*4+3]);
    }

    _curr_position_get = 0;
}

void AllegroHandDrv::_parseMessage(char cmd, char src, char des, int len, unsigned char* data)
{
    int tmppos[4];
    int lIndexBase;

    switch (cmd) {
    case ID_CMD_QUERY_CONTROL_DATA:
        if (src >= ID_DEVICE_SUB_01 && src <= ID_DEVICE_SUB_04) {

            tmppos[0] = (int) (data[0] | (data[1] << 8));
            tmppos[1] = (int) (data[2] | (data[3] << 8));
            tmppos[2] = (int) (data[4] | (data[5] << 8));
            tmppos[3] = (int) (data[6] | (data[7] << 8));

            lIndexBase = 4 * (src - ID_DEVICE_SUB_01);

            _curr_position[lIndexBase+0] = (double)_encoder_direction[lIndexBase+0] * (double)(tmppos[0] - 32768 - _encoder_offset[lIndexBase+0]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
            _curr_position[lIndexBase+1] = (double)_encoder_direction[lIndexBase+1] * (double)(tmppos[1] - 32768 - _encoder_offset[lIndexBase+1]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
            _curr_position[lIndexBase+2] = (double)_encoder_direction[lIndexBase+2] * (double)(tmppos[2] - 32768 - _encoder_offset[lIndexBase+2]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
            _curr_position[lIndexBase+3] = (double)_encoder_direction[lIndexBase+3] * (double)(tmppos[3] - 32768 - _encoder_offset[lIndexBase+3]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);

            _curr_position_get |= (0x01 << (src - ID_DEVICE_SUB_01));
            //ROS_INFO("get position %d: %.1f, %.1f, %.1f, %.1f", (src - ID_DEVICE_SUB_01), _curr_position[lIndexBase+0], _curr_position[lIndexBase+1], _curr_position[lIndexBase+2], _curr_position[lIndexBase+3]);
        }
        else {
            ROS_WARN("No subdevice match!");
        }
        break;

    case ID_CMD_QUERY_STATE_DATA:
        break;

    case ID_CMD_QUERY_ID:
        ROS_INFO("CAN: Allegro Hand PCB revision (%02x%02xh)", data[3], data[2]);
        ROS_INFO("CAN: Allegro Hand firmware version (%02x%02xh)", data[5], data[4]);
        ROS_INFO("CAN: Allegro Hand hardware type (%s)", (data[7] == 0 ? "Left" : "Right"));
        break;

    case ID_CMD_AHRS_POSE:
        ROS_INFO("CAN: AHRS Roll  = %02x%02xh", data[0], data[1]);
        ROS_INFO("CAN: AHRS Pitch = %02x%02xh", data[2], data[3]);
        ROS_INFO("CAN: AHRS Yaw   = %02x%02xh", data[4], data[5]);
        break;

    case ID_CMD_AHRS_ACC:
        ROS_INFO("CAN: AHRS Acc(x) = %02x%02xh", data[0], data[1]);
        ROS_INFO("CAN: AHRS Acc(y) = %02x%02xh", data[2], data[3]);
        ROS_INFO("CAN: AHRS Acc(z) = %02x%02xh", data[4], data[5]);
        break;

    case ID_CMD_AHRS_GYRO:
        ROS_INFO("CAN: AHRS Angular Vel(x) = %02x%02xh", data[0], data[1]);
        ROS_INFO("CAN: AHRS Angular Vel(y) = %02x%02xh", data[2], data[3]);
        ROS_INFO("CAN: AHRS Angular Vel(z) = %02x%02xh", data[4], data[5]);
        break;

    case ID_CMD_AHRS_MAG:
        ROS_INFO("CAN: AHRS Magnetic Field(x) = %02x%02xh", data[0], data[1]);
        ROS_INFO("CAN: AHRS Magnetic Field(x) = %02x%02xh", data[2], data[3]);
        ROS_INFO("CAN: AHRS Magnetic Field(x) = %02x%02xh", data[4], data[5]);
        break;

    default:
        ROS_WARN("unknown command %d, src %d, to %d, len %d", cmd, src, des, len);
        for(int nd=0; nd<len; nd++)
            printf("%d \n ", data[nd]);
        return;
    }
}

} // namespace allegro
