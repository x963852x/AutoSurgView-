/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of joystick listen and control

**************************************************************************/

#pragma once

#include <Windows.h>
#include <joystickapi.h>
#include <cstddef>
#include <iostream>
#include <atomic>
#include <condition_variable>
#include <thread>

#include "motor_work.h"
#include "camera.h"

#include "ssbot_config.h"

#define ERR_NODRIVER 0;
#define ERR_NODEVICE 1;

int joystick1_listen();	//手柄1监听、控制程序
int joystick2_listen();	//手柄2监听、控制程序