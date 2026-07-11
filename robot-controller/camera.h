/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of camera control

**************************************************************************/

#pragma once

#include <thread>
#include <mutex>
#include <iostream>

#include "serial/serial.h"
#include "motor_work.h"
#include "math.h"

#include "ssbot_config.h"

//相机运动状态
#define CAMERA_FREE 0
#define CAMERA_MOVING 1

//相机运动信号状态
enum CAMERA_O
{
	CAMERA_U = 1,
	CAMERA_D = 2,
	CAMERA_L = 3,
	CAMERA_R = 4,
	CAMERA_P = 5,
	CAMERA_N = 6,
};

//相机类
class camera
{
public:
	camera();	//相机类默认构造函数
	void setPanMotor_with_speed(double speed); //相机类成员函数：云台俯仰电机模式运动，用于视觉伺服对准
	void setTilMotor_with_speed(double speed); //相机类成员函数：云台侧倾电机模式运动，用于视觉伺服对准
	void setPanMotor2Servo(); //相机类成员函数：云台俯仰电机模式转舵机模式
	void setTilMotor2Servo(); //相机类成员函数：云台侧倾电机模式转舵机模式
	void setPan_with_speed(double Pan, double speed = 0);	//相机类成员函数：云台电机俯仰运动指令，带有速度限制
	void setTilt_with_speed(double Tilt, double speed = 0);	//相机类成员函数：云台电机侧倾运动指令，带有速度限制
	void setRoll_with_speed(double Roll, double speed = 0);	//相机类成员函数：云台电机侧倾运动指令，带有速度限制
	void setWrist(double Pan, double Tilt, double Roll);	//相机类成员函数：云台电机运动指令
	bool getPan(double& Pan);	//相机类成员函数：云台电机读取俯仰角度指令
	bool getTilt(double& Tilt);	//相机类成员函数：云台电机读取侧倾角度指令
	bool getRoll(double& Roll);	//相机类成员函数：云台电机读取自旋角度指令
	bool getWrist(double& Pan, double& Tilt, double& Roll);	//相机类成员函数：云台电机读取角度指令
	void move(CAMERA_O orian, int speed = 12);	//相机类成员函数：云台电机运动指令 用于响应手柄控制
	void stop(int LastState = 0);	//相机类成员函数：云台电机停止指令
	void stopPan();	//相机类成员函数：云台电机俯仰停止指令
	void stopTilt();	//相机类成员函数：云台电机侧倾停止指令
	void stopRoll();	//相机类成员函数：云台电机自旋停止指令
	void zoom(int orian);	//相机类成员函数：相机模组缩放指令
	void focus(int orian);	//相机类成员函数：相机聚焦控制指令
	void exposure(int orian);	//相机类成员函数：相机曝光控制指令
	void light(int orian);	//相机类成员函数：相机无影灯程序控制指令

	uint8_t CheckSum(uint8_t buf[]);

private:
	double pan;
	double tilt;
	double roll;
};

void camera_work();	//相机伺服程序
void camera_init();