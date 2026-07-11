#pragma once
#include <stdio.h>
#include <atomic>
#include <thread>
#include <fstream>
#include <algorithm>
#include <mutex>
#include "LiftingColumn.h"
#include "opencv2/opencv.hpp"
#include "lidar.h"
#include "motor_work.h"
#include "yanHuaDriver.h"
#include "camera.h"
#include "visual.h"
#include "camera.h"
#include "rs_camera_work.h"
#include "udp.h"
#include "occlusion_avoidance.h"

void key_control();	//键盘控制函数
void lidar_start();	//激光雷达开始
void lidar_stop();	//激光雷达停止

void io_control();	//面板控制

void key_control_simulation();	//键盘控制函数——仿真

void oa_start();	//开启遮挡规避

void oa_end();	//停止遮挡规避