#pragma once

//因socket编译问题，将udp_init()函数独立置于udp.cpp文件中

#include "udp.h"

#include "camera.h"
#include "motor.h"
#include "LiftingColumn.h"

#include "motor_work.h"
#include "ssbot_control.h"
#include "rs_camera_work.h"

#include "occlusion_avoidance.h"

#pragma region 宏定义变量
#define DEBUG_ERROR_STOP_BUTTON 0	//急停旋钮未旋起
#define DEBUG_ERROR_OMPL_OUT_RANGE 1	//机械臂非法初始位置
#define DEBUG_ERROR_OMPL_OUT_RANGE_FREE 11	//机械臂已释放，可手动调整
#define DEBUG_ERROR_OMPL_OUT_RANGE_LEGAL 12	//电机已调整至合法位置
#define DEBUG_ERROR_LIDAR 2
#define DEBUG_ERROR_3D_CAMERA 3
#define DEBUG_ERROR_TARGET_CAMERA_FAIL 4	//靶标图像识别失败
#define DEBUG_ERROR_TARGET_IK_FAIL 5	//机器人目标拍摄姿态求解失败
#define DEBUG_ERROR_TRAJ_OVERTIME 14	//路径规划超时失败
#define DEBUG_ERROR_TRAJ_INVALID_GOAL_OR_START 15	//路径规划因非法初始或目标位置失败

#define DEBUG_MOTOR_INIT_SUCCESS 6	//电机初始化成功
#define DEBUG_LIDAR_INIT_SUCCESS 7	//靶标图像识别成功
#define DEBUG_3D_CAMERA_INIT_SUCCESS 8	//3D相机初始化成功
#define INIT_SUCCESS_WITH_RS_FUN_DISABLED 13	//初始化成功,无3D相机功能
#define DEBUG_INFO_TARGET_CAMERA_SUCCESS 9	//机器人目标拍摄姿态求解成功
#define DEBUG_INFO_GET_PICTURE_FINISHED 21  //AprilTag靶标图像采集完成
#define DEBUG_INFO_TARGET_IK_SUCCESS 10
#define DEBUG_INFO_EXPAND_RECYVLE_SUCCESS 20 //机器人展开回收正常结束

#define DEBUG_INFO_TRAJ_SUCCESS 16
#define DEBUG_INFO_ARM_STOP 17
#define DEBUG_INFO_HEARTBEAT_UI 18
#define DEBUG_INFO_HEARTBEAT_UI_WITHOUT_RS 19

#define DEBUG_RL_OCCLUSION_AVOIDANCE_START 22
#define DEBUG_RL_OCCLUSION_AVOIDANCE_END 23
#pragma endregion

void udp_control();	//UDP控制函数，接收来自上位机和红云主机的信息，控制机器人动作
void udp_refresh();	//UDP更新函数，向上位机发送机器人运动部件参数，向红云主机发送信息
void udp_send_debug_info(int16_t error_index);	//UDP发送函数，向上位机发送机器人调试信息
void heart_beat_butel();//机器人与5G主机心跳通信
void heart_beat_monitor();//机器人与上位机心跳通信
void heart_beat();//机器人通信心跳功能函数，内含两个子线程
