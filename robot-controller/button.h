/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of button listen

**************************************************************************/

#pragma once

#include <cstddef>
#include <iostream>
#include <atomic>
#include <condition_variable>
#include <thread>

#include "yanHuaDriver.h"
#include "LiftingColumn.h"

//蜂鸣器状态变量
#define BEEP_FREE 0;
#define BEEP_ONE 1;
#define BEEP_TWO 2;
#define BEEP_THREE 3;

void button_listen();	//机器人面板按钮监听函数