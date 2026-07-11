/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of joystick listen and control

**************************************************************************/

#include "joystick.h"

using std::cout;
using std::endl;

extern std::map<std::string, std::string> parameters;

std::atomic<int> joy1State;	//手柄1状态
std::atomic<int> joy2State;	//手柄2状态

//外部变量
extern int safeState;	//安全控制按键状态
extern camera Cam;		//全局相机对象
extern int MOVE_TYPE;	//机械臂运动类型

extern int ptz_speed;
extern int ptz_speed_joy;

//手柄 ID
UINT JOYSTICK1;
UINT JOYSTICK2;

//手柄1监听、控制程序
//手柄1可控制机械臂末端平移
int joystick1_listen()
{
	JOYSTICK1 = atoi(parameters["joystick_1"].c_str());
	JOYSTICK2 = atoi(parameters["joystick_2"].c_str());

	JOYINFO joyinfo;
	UINT wNumDevs, wDeviceID;
	BOOL bDevAttached;

	if ((wNumDevs = joyGetNumDevs()) == 0)
	{
		return ERR_NODRIVER;
	}
	cout << wNumDevs << endl;

	bDevAttached = joyGetPos(JOYSTICK1, &joyinfo) != JOYERR_UNPLUGGED;
	if (bDevAttached)
	{
		cout << "手柄1连接" << endl;
	}
	else
	{
		cout << "手柄1未连接" << endl;
	}

	int thisState = 'S';	//当前手柄状态
	int lastState = 'S';	//先前手柄状态

	while (bDevAttached)
	{
		//获取手柄位置，并将 当前手柄状态 设置为相应值
		MMRESULT joyreturn = joyGetPos(JOYSTICK1, &joyinfo);

		//if (joyreturn == JOYERR_NOERROR)
		//{
		//	printf("0x%09d ", joyinfo.wXpos);
		//	printf("0x%09d ", joyinfo.wYpos);
		//	//printf("0x%09X ", joyinfoex.dwZpos);
		//	//printf("0x%09X ", joyinfoex.dwPOV);
		//	//printf("0x%09X ", joyinfoex.dwButtons);
		//	printf("\n");
		//}
		//else
		//{
		//	switch (joyreturn)
		//	{
		//	case JOYERR_PARMS:
		//		printf("bad parameters\n");
		//		break;
		//	case JOYERR_NOCANDO:
		//		printf("request not completed\n");
		//		break;
		//	case JOYERR_UNPLUGGED:
		//		printf("joystick is unplugged\n");
		//		break;
		//	default:
		//		printf("未知错误\n");
		//		break;
		//	}
		//}

		if (joyinfo.wXpos < 20000)
		{
			thisState = 'L';
		}
		else if (joyinfo.wXpos > 45000)
		{
			thisState = 'R';
		}
		else if (joyinfo.wYpos < 20000)
		{
			thisState = 'U';
		}
		else if (joyinfo.wYpos > 45000)
		{
			thisState = 'D';
		}
		else
		{
			thisState = 'S';
		}

		//若手柄状态发生改变
		if (thisState != lastState)
		{
			printf("手柄1状态 %c 1\n", thisState);

			//设置手柄1状态
			joy1State = thisState;

			if (safeState)	//若安全按钮按下
			{
				//根据手柄1状态控制机械臂做出相应动作
				switch (joy1State)
				{
				case 'U':
					arm_up();
					break;
				case 'D':
					arm_down();
					break;
				case 'L':
					arm_left();
					break;
				case 'R':
					arm_right();
					break;
				}
			}
			else
				arm_stop();

			if (joy1State == 'S' && MOVE_TYPE == MOVE_TYPE_JOYSTICK)
			{
				arm_stop();
			}
		}
		lastState = thisState;
		std::this_thread::sleep_for(std::chrono::microseconds(1));
	}

	return 0;
}

//手柄2监听、控制程序
//手柄2可控制云台电机微调
int joystick2_listen()
{
	JOYINFO joyinfo;
	UINT wNumDevs, wDeviceID;
	BOOL bDevAttached;

	if ((wNumDevs = joyGetNumDevs()) == 0)
	{
		return ERR_NODRIVER;
	}
	cout << wNumDevs << endl;

	bDevAttached = joyGetPos(JOYSTICK2, &joyinfo) != JOYERR_UNPLUGGED;
	if (bDevAttached)
	{
		cout << "手柄2连接" << endl;
	}
	else
	{
		cout << "手柄2未连接" << endl;
	}

	int thisState = 'S';	//当前手柄状态
	int lastState = 'S';	//先前手柄状态

	while (bDevAttached)
	{
		//获取手柄位置，并将 当前手柄状态 设置为相应值
		joyGetPos(JOYSTICK2, &joyinfo);
		if (joyinfo.wXpos < 20000)
		{
			thisState = 'L';
		}
		else if (joyinfo.wXpos > 45000)
		{
			thisState = 'R';
		}
		else if (joyinfo.wYpos < 20000)
		{
			thisState = 'U';
		}
		else if (joyinfo.wYpos > 45000)
		{
			thisState = 'D';
		}
		else if (joyinfo.wZpos < 20000)
		{
			thisState = 'P';
		}
		else if (joyinfo.wZpos > 45000)
		{
			thisState = 'N';
		}
		else
		{
			thisState = 'S';
		}

		//若手柄状态发生改变
		if (thisState != lastState)
		{
			printf("手柄2状态 %c 2\n", thisState);
			//设置手柄1状态
			joy2State = thisState;

			//根据手柄1状态控制机械臂做出相应动作
			if (safeState)	//若安全按钮按下
			{
				//根据手柄1状态控制机械臂做出相应动作
				switch (joy2State)
				{
				case 'U':
					Cam.move(CAMERA_U, ptz_speed_joy);
					break;
				case 'D':
					Cam.move(CAMERA_D, ptz_speed_joy);
					break;
				case 'R':
					Cam.move(CAMERA_R, ptz_speed_joy);
					break;
				case 'L':
					Cam.move(CAMERA_L, ptz_speed_joy);
					break;
				case 'P':
					Cam.move(CAMERA_P, ptz_speed_joy);
					break;
				case 'N':
					Cam.move(CAMERA_N, ptz_speed_joy);
					break;
				case 'S':
					Cam.stop();
					break;
				default:
					break;
				}
			}
			else
				Cam.stop();
		}

		lastState = thisState;
		std::this_thread::sleep_for(std::chrono::microseconds(1));
	}
	return 0;
}