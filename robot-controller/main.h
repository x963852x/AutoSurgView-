/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of main

**************************************************************************/


#pragma once

#include <fstream>
#include <ctime>
#include <cstdlib>
#include <filesystem>

#include "motor_work.h"
#include "lidar.h"
#include "ssbot_control.h"
#include "udp.h"
#include "joystick.h"
#include "button.h"
#include "LiftingColumn.h"
#include "visual.h"
#include "udp_control.h"
#include "rs_camera_work.h"
#include "occlusion_avoidance.h"

#include "ssbot_config.h"

void robot_init();	//机器人运动部件参数初始化
void arm_init();	//机械臂关节电机初始化函数
void Lidar_init();
BOOL WINAPI HandlerRoutine(DWORD dwCtrlType);	//程序关闭绑定函数
