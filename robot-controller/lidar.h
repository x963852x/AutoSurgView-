/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of lidar scan

**************************************************************************/

#pragma once

#include <vector>
#include <thread>

#include "rplidar.h"
#include "opencv2/opencv.hpp"
#include "motor_work.h"
#include "button.h"
#include "ssbot_config.h"
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
//激光雷达工作/转动状态
#define LIDAR_FREE 0
#define LIDAR_WORK 1
#define LIDAR_STOP 2
#define LIDAR_START 3
#define PI 3.1415926

using std::vector;
using std::cout;
using std::endl;
using namespace sl;
//雷达采样点结构体
struct lidar_point
{
	double dis;		//距离 单位 mm
	double angle;	//角度 单位 °
};

void lidar_scan();	//激光雷达扫描程序