/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of button listen

**************************************************************************/

#include "button.h"

using std::cout;
using std::endl;

#pragma region 全局变量
int zoomState;			//缩放控制按键状态
int safeState;			//安全控制按键状态
int autoState = 0;			//自动运行状态
int zoomRed;			//录制状态
int startState;			//开始状态
int laststartState;
int beepState;			//蜂鸣器状态


int LCState = -1;		//立柱控制信号当前状态
int LastLCState = -1;	//立柱控制信号上一状态
char ResetFlag = '0';

mutex lc_mt; //立柱升降控制和按键监听交替执行锁
condition_variable lc_var;
bool lc_mt_tag = true;
#pragma endregion

//按钮监听程序
void button_listen()
{
	int upLast = 0;		//立柱上升按键上一状态
	int downLast = 0;	//立柱下降按键上一状态

	auto cardI = StartCardDi();	//IO采集卡对象

	int upState;		//立柱上升按键当前状态
	int downState;		//立柱下降按键当前状态

	//轮询采集按键状态
	while (1)
	{
		unique_lock<mutex> lock(lc_mt);
		lc_var.wait(lock, [] {return lc_mt_tag; });
		lc_mt_tag = false;
		lc_var.notify_one();

		upState = CardReadBite(cardI, 0);	//读取立柱上升按钮状态
		downState = CardReadBite(cardI, 1);	//读取立柱下降按钮状态

		if (upState)
		{
			LCState = LC_UP;	//立柱控制信号当前状态置为上升
			upLast = 1;	//立柱上升按键上一状态置为按下
		}
		else if (downState)	//立柱下降按键优先级低于上升按键，即二者同时按下，立柱将上升
		{
			LCState = LC_DOWN;	//立柱控制信号当前状态置为下降
			downLast = 1;	//立柱下降按键上一状态置为按下
		}
		else if (upState != upLast || downState != downLast)	//用于检测按钮松开
		{
			LCState = LC_STOP;	//立柱控制信号当前状态置为停止
			upLast = 0;		//立柱上升按键上一状态置为松开
			downLast = 0;	//立柱下降按键上一状态置为松开
		}

		if (CardReadBite(cardI, 2))	//读取相机视野放大按钮状态
		{
			zoomState = 'U';	//缩放控制按键状态置为放大
		}
		else if (CardReadBite(cardI, 3))	//读取相机视野缩小按钮状态
		{
			zoomState = 'D';	//缩放控制按键状态置为缩小
		}
		else
		{
			zoomState = 'S';	//缩放控制按键状态置为停止
		}
		if (CardReadBite(cardI, 4))	//读取录制按键状态
		{
			//startState = 1;	//录制按键状态置为按下
			ResetFlag = '1';      //重启下位机按键按下
		}
		else
		{
			startState = 0;	//录制按键状态置为松开
		}
		if (CardReadBite(cardI, 5))	//读取安全按键状态
		{
			safeState = 1;	//安全按键状态置为按下
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			safeState = 0;	//安全按键状态置为松开
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	EndCardDi(cardI);
}