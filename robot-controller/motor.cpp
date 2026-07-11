/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of motor

**************************************************************************/

#include "motor.h"

using std::cout;
using std::endl;

#pragma region 全局变量源自宏定义变量
//关节电机角度偏置，单位°
double FIRST_MOTOR_OFFSET;		//关节电机1角度偏置
double SECOND_MOTOR_OFFSET;	//关节电机2角度偏置
double THIRD_MOTOR_OFFSET;	//关节电机3角度偏置

//关节电机角度展开，单位°
double FIRST_MOTOR_OPEN;		//关节电机1角度展开
double SECOND_MOTOR_OPEN;	//关节电机2角度展开
double THIRD_MOTOR_OPEN;	//关节电机3角度展开

//关节电机角度收拢，单位°
double FIRST_MOTOR_CLOSE;		//关节电机1角度收拢
double SECOND_MOTOR_CLOSE;	//关节电机2角度收拢
double THIRD_MOTOR_CLOSE;	//关节电机3角度收拢

//轨迹插值速度限制
double SPEED_MAX;		//多关节运动速度 °/s
double SPEED_MAX_DEBUG_HIGH;	//单关节调试运动，高速情况 °/s，应用于机器人触摸屏控制
double SPEED_MAX_DEBUG_LOW;	//单关节调试运动，高速情况 °/s，应用于机器人触摸屏控制
#pragma endregion


#pragma region 全局变量
bool open_can_success = false;	//CAN通信设备是否连接成功

//电机是否初始化变量
bool motor_1_enabled = false;
bool motor_2_enabled = false;
bool motor_3_enabled = false;
#pragma endregion

#pragma region 关节电机类
//关节电机类默认构造函数
motor::motor()
{
	open_can_success = motor::openCAN();	//打开CAN通信设备
}

//关节电机类默认析构函数
motor::~motor()
{
	motor::closeCAN();	//关闭CAN通信设备
}

//关节电机类成员函数：打开CAN通信设备
//@return true if opencan success
bool motor::openCAN()
{
	INIT_CONFIG vic;
	vic.Timing0 = 0;
	vic.Timing1 = 0x14;
	vic.Mode = 0;
	DWORD dwRel;
	dwRel = OpenDevice(nDeviceType, nDeviceInd, nReserved);
	if (dwRel != STATUS_OK)
	{
		printf("opencan error");
		return FALSE;
	}
	dwRel = InitCAN(nDeviceType, nDeviceInd, nCANInd, &vic);
	if (dwRel == STATUS_ERR)
	{
		CloseDevice(nDeviceType, nDeviceInd);
		printf("Error");
		return FALSE;
	}
	dwRel = StartCAN(nDeviceType, nDeviceInd, nCANInd);
	if (dwRel == STATUS_ERR)
	{
		CloseDevice(nDeviceType, nDeviceInd);
		printf("Error");
		return FALSE;
	}
	printf("opencan sucess\n");
	return true;
}

//关节电机类成员函数：关闭CAN通信设备
//@return true if closecan success
void motor::closeCAN()
{
	CloseDevice(nDeviceType, nDeviceInd);
}

// 关节电机类成员函数：使能关节电机
// @param number 将使能的关节电机ID
// @return true if enable motor success
bool motor::enable(int number)
{
	disable(number);
	Sleep(100);
	int time = 0;
	while (time < MAX_CAN_TIME)
	{
		time++;

		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		DWORD reT;
		CAN_OBJ vco;
		ZeroMemory(&vco, sizeof(CAN_OBJ));
		vco.ID = 0x00 + number;
		vco.SendType = 0;
		vco.RemoteFlag = 0;
		vco.ExternFlag = 0;
		vco.DataLen = 2;
		vco.Data[0] = 0x2A;
		vco.Data[1] = 0x01;
		reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, vco.DataLen);
		Sleep(1000);
		DWORD dwRel;
		CAN_OBJ vcoR[1];
		dwRel = Receive(nDeviceType, nDeviceInd, nCANInd, vcoR, 1, 10);

		if (dwRel == 0 || reT == 0 || vcoR->ID != 0x00 + number || vcoR->Data[0] != 0X2A)
		{
			continue;
		}
		if (vcoR->Data[1] == 0X01)
		{
			switch (number)
			{
			case(FIRST_MOTOR_CAN):
				motor_1_enabled = true;
				break;
			case(SECOND_MOTOR_CAN):
				motor_2_enabled = true;
				break;
			case(THIRD_MOTOR_CAN):
				motor_3_enabled = true;
				break;
			default:
				cout << "Motor number out of range" << endl;
				break;
			}
			return true;
		}
		else
		{
			cout << "Motor " << number << " enable failed" << endl;
			return false;
		}
	}
	cout << "Motor " << number << " enable failed for max_can_time" << endl;
	return false;	
}

// 关节电机类成员函数：关节电机心跳通信
// @param number 通信关节电机ID
// @return true if handshake motor success
bool motor::handshake(int number)
{
	while (true)
	{
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		DWORD reT;
		CAN_OBJ vco;
		ZeroMemory(&vco, sizeof(CAN_OBJ));
		vco.ID = 0x00 + number;
		vco.SendType = 0;
		vco.RemoteFlag = 0;
		vco.ExternFlag = 0;
		vco.DataLen = 1;
		vco.Data[0] = 0;
		reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, vco.DataLen);
		DWORD dwRel;
		CAN_OBJ vcoR[1];
		dwRel = Receive(nDeviceType, nDeviceInd, nCANInd, vcoR, 1, 10);
		if (dwRel == 0 || reT == 0 || vcoR->ID != 0x00 + number || vcoR->Data[0] != 0X00)
		{
			continue;
		}
		if (vcoR->Data[1] == 0X01)
			return true;
		else
			return false;
	}

}

// 关节电机类成员函数：失能关节电机
// @param number 将失能的关节电机ID
// @return true if disable motor success
void motor::disable(int number)
{
	ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
	DWORD reT;
	CAN_OBJ vco;
	ZeroMemory(&vco, sizeof(CAN_OBJ));
	vco.ID = 0x00 + number;
	vco.SendType = 0;
	vco.RemoteFlag = 0;
	vco.ExternFlag = 0;
	vco.DataLen = 2;
	vco.Data[0] = 0x2A;
	vco.Data[1] = 0x00;
	reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, vco.DataLen);

	switch (number)
	{
	case(FIRST_MOTOR_CAN):
		motor_1_enabled = false;
		break;
	case(SECOND_MOTOR_CAN):
		motor_2_enabled = false;
		break;
	case(THIRD_MOTOR_CAN):
		motor_3_enabled = false;
		break;
	default:
		cout << "Motor number out of range" << endl;
		break;
	}
}

// 关节电机类成员函数：设置关节电机位置限制
// @param number 将设置位置限制的关节电机ID
// @return true if set position limit success
bool motor::setPositionLimit(int number)
{
	int time = 0;
	bool suc = false;

	double limit_max = 0;	//位置限制最大值
	double limit_min = 0;	//位置限制最小值

	//根据ID选择位置限制值
	if (number == FIRST_MOTOR_CAN)
	{
		limit_max = AngletoPosition(-1 * (FIRST_MOTOR_MIN - FIRST_MOTOR_OFFSET), number);
		limit_min = AngletoPosition(-1 * (FIRST_MOTOR_MAX - FIRST_MOTOR_OFFSET), number);
	}
	else if (number == SECOND_MOTOR_CAN)
	{
		limit_max = AngletoPosition(-1 * (SECOND_MOTOR_MIN + SECOND_MOTOR_OFFSET), number);
		limit_min = AngletoPosition(-1 * (SECOND_MOTOR_MAX + SECOND_MOTOR_OFFSET), number);
	}
	else if (number == THIRD_MOTOR_CAN)
	{
		limit_max = AngletoPosition(THIRD_MOTOR_MAX - THIRD_MOTOR_OFFSET, number);
		limit_min = AngletoPosition(THIRD_MOTOR_MIN - THIRD_MOTOR_OFFSET, number);
	}

	while (time < MAX_CAN_TIME)
	{
		time++;
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		DWORD reT;
		CAN_OBJ vco;
		ZeroMemory(&vco, sizeof(CAN_OBJ));
		vco.ID = 0x00 + number;
		vco.SendType = 0;
		vco.RemoteFlag = 0;
		vco.ExternFlag = 0;
		vco.DataLen = 5;

		int ratio = 101;

		vco.Data[0] = 0x83;
		int position_IQ24 = DtoIQ24(limit_max);
		vco.Data[1] = position_IQ24 >> 24 & 0xFF;
		vco.Data[2] = position_IQ24 >> 16 & 0xFF;
		vco.Data[3] = position_IQ24 >> 8 & 0xFF;
		vco.Data[4] = position_IQ24 & 0xFF;
		reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, vco.DataLen);
		DWORD dwRel;
		CAN_OBJ vcoR[1];
		dwRel = Receive(nDeviceType, nDeviceInd, nCANInd, vcoR, 1, 10);
		if (dwRel == 0 || reT == 0 || vcoR->ID != 0x00 + number || vcoR->Data[0] != 0X83)
		{
			continue;
		}
		if (vcoR->Data[1] == 0X01)
		{
			suc = true;
			break;
		}
		else
		{
			cout << "Motor " << number << " set position limit failed" << endl;
			return false;
		}
	}
	
	if (!suc)
	{
		cout << "Motor " << number << " set position limit failed for max_can_time" << endl;
		return false;
	}

	time = 0;
	while (time < MAX_CAN_TIME)
	{
		time++;
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		DWORD reT;
		CAN_OBJ vco;
		ZeroMemory(&vco, sizeof(CAN_OBJ));
		vco.ID = 0x00 + number;
		vco.SendType = 0;
		vco.RemoteFlag = 0;
		vco.ExternFlag = 0;
		vco.DataLen = 5;

		int ratio = 101;
		//if (number == THIRD_MOTOR_CAN)
		//{
		//	ratio = 51;
		//}

		vco.Data[0] = 0x84;
		int position_IQ24 = DtoIQ24(limit_min);
		vco.Data[1] = position_IQ24 >> 24 & 0xFF;
		vco.Data[2] = position_IQ24 >> 16 & 0xFF;
		vco.Data[3] = position_IQ24 >> 8 & 0xFF;
		vco.Data[4] = position_IQ24 & 0xFF;
		reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, vco.DataLen);

		DWORD dwRel;
		CAN_OBJ vcoR[1];
		dwRel = Receive(nDeviceType, nDeviceInd, nCANInd, vcoR, 1, 10);
		if (dwRel == 0 || reT == 0 || vcoR->ID != 0x00 + number || vcoR->Data[0] != 0X84)
		{
			continue;
		}
		if (vcoR->Data[1] == 0X01)
		{
			return true;
		}
		else
		{
			cout << "Motor " << number << " set position limit failed" << endl;
			return false;
		}
	}

	cout << "Motor " << number << " set position limit failed for max_can_time" << endl;
	return false;
}

// 关节电机类成员函数：设置关节电机速度限制
// @param number 将设置速度限制的关节电机ID
// @return true if set speed limit success
bool motor::setSpeedLimit(int number)
{
	int time = 0;
	while (time < MAX_CAN_TIME)
	{
		time++;
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		DWORD reT;
		CAN_OBJ vco;
		ZeroMemory(&vco, sizeof(CAN_OBJ));
		vco.ID = 0x00 + number;
		vco.SendType = 0;
		vco.RemoteFlag = 0;
		vco.ExternFlag = 0;
		vco.DataLen = 5;

		double rpm = 6;

		int ratio = 101;

		vco.Data[0] = 0x5A;
		int rpm_IQ24 = DtoIQ24(rpm * ratio / 6000);
		vco.Data[1] = rpm_IQ24 >> 24 & 0xFF;
		vco.Data[2] = rpm_IQ24 >> 16 & 0xFF;
		vco.Data[3] = rpm_IQ24 >> 8 & 0xFF;
		vco.Data[4] = rpm_IQ24 & 0xFF;
		reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, vco.DataLen);

		DWORD dwRel;
		CAN_OBJ vcoR[1];
		dwRel = Receive(nDeviceType, nDeviceInd, nCANInd, vcoR, 1, 10);
		if (dwRel == 0 || reT == 0 || vcoR->ID != 0x00 + number || vcoR->Data[0] != 0X5A)
		{
			continue;
		}
		if (vcoR->Data[1] == 0X01)
		{
			return true;
		}
		else
		{
			cout << "Motor " << number << " set speed limit failed" << endl;
			return false;
		}
	}
	cout << "Motor " << number << " set speed limit failed for max_can_time" << endl;
	return false;
}

// 关节电机类成员函数：设置关节电机控制模式
// @param mode 控制模式
// @param number 将设置控制模式的关节电机ID
// @return true if set mode success
bool motor::setMode(int mode, int number)
{
	int time = 0;
	while (time < MAX_CAN_TIME)
	{
		time++;

		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		DWORD reT;
		CAN_OBJ vco;
		ZeroMemory(&vco, sizeof(CAN_OBJ));
		vco.ID = 0x00 + number;
		vco.SendType = 0;
		vco.RemoteFlag = 0;
		vco.ExternFlag = 0;
		vco.DataLen = 2;
		vco.Data[0] = 0x07;
		vco.Data[1] = 0x00 + mode;
		reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, vco.DataLen);

		DWORD dwRel;
		CAN_OBJ vcoR[1];
		dwRel = Receive(nDeviceType, nDeviceInd, nCANInd, vcoR, 1, 10);

		if (dwRel == 0 || reT == 0 || vcoR->ID != 0x00 + number || vcoR->Data[0] != 0X07)
		{
			continue;
		}
		if (vcoR->Data[1] == 0X01)
		{
			return true;
		}
		else
		{
			cout << "Motor " << number << " set mode failed" << endl;
			return false;
		}
	}
	cout << "Motor " << number << " set mode failed for max_can_time" << endl;
	return false;
}

// 关节电机类成员函数：获取关节电机位置值
// @param number 将获取位置值的关节电机ID
// @return 关节电机位置值，单位r（转）
double motor::getPosition(int number)
{
	double position = 0;
	int maxtime = 100;
	int time = 0;
	while (true)
	{
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		DWORD reT;
		CAN_OBJ vco;
		ZeroMemory(&vco, sizeof(CAN_OBJ));
		vco.ID = 0x00 + number;
		vco.SendType = 1;
		vco.RemoteFlag = 0;
		vco.ExternFlag = 0;
		vco.DataLen = 1;
		vco.Data[0] = 0x06;
		reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, 6);
		DWORD dwRel;
		CAN_OBJ vcoR[1];
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		dwRel = Receive(nDeviceType, nDeviceInd, nCANInd, vcoR, 1, 10);

		if (time > maxtime)
			return 0;

		if (dwRel == 0 || reT == 0 || vcoR->ID != 0x00 + number || vcoR->Data[0] != 0x06)
		{
			time++;
			continue;
		}
		int data = vcoR->Data[1] << 24 | vcoR->Data[2] << 16 | vcoR->Data[3] << 8 | vcoR->Data[4];
		position = IQ24toD(data);
		return position;
	}
}

// 关节电机类成员函数：设置关节电机位置值
// @param position	期望关节电机位置值，单位r（转）
// @param number 将设置位置值的关节电机ID
void motor::setPosition(double position, int number)
{

	ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
	DWORD reT;
	CAN_OBJ vco;
	ZeroMemory(&vco, sizeof(CAN_OBJ));
	vco.ID = 0x00 + number;
	vco.SendType = 0;
	vco.RemoteFlag = 0;
	vco.ExternFlag = 0;
	vco.DataLen = 5;
	vco.Data[0] = 0x0A;
	
	int position_IQ24 = DtoIQ24(position);
	vco.Data[1] = position_IQ24 >> 24 & 0xFF;
	vco.Data[2] = position_IQ24 >> 16 & 0xFF;
	vco.Data[3] = position_IQ24 >> 8 & 0xFF;
	vco.Data[4] = position_IQ24 & 0xFF;

	reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, vco.DataLen);
}

// 关节电机类成员函数：获取关节电机速度限制值
// @param number 将获取速度限制值的关节电机ID
// @return 关节电机速度限制值，单位r/s（转/秒）
double motor::getSpeed(int number)
{
	double speed = 0;
	while (true)
	{
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		DWORD reT;
		CAN_OBJ vco;
		ZeroMemory(&vco, sizeof(CAN_OBJ));
		vco.ID = 0x00 + number;
		vco.SendType = 1;
		vco.RemoteFlag = 0;
		vco.ExternFlag = 0;
		vco.DataLen = 1;
		vco.Data[0] = 0x05;
		reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, 6);
		DWORD dwRel;
		CAN_OBJ vcoR[1];
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		dwRel = Receive(nDeviceType, nDeviceInd, nCANInd, vcoR, 1, 10);
		if (dwRel == 0 || reT == 0 || vcoR->ID != 0x00 + number || vcoR->Data[0] != 0x05)
		{
			continue;
		}
		int data = vcoR->Data[1] << 24 | vcoR->Data[2] << 16 | vcoR->Data[3] << 8 | vcoR->Data[4];
		speed = IQ24toD(data)*6000;
		return speed;
	}
}

// 关节电机类成员函数：设置关节电机速度限制值
// @param position	期望关节电机速度限制值，单位r/s（转/秒）
// @param number 将设置速度限制值的关节电机ID
bool motor::setSpeed(double rpm, int number)
{
	int time = 0;
	while (time < MAX_CAN_TIME)
	{
		time++;
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		DWORD reT;
		CAN_OBJ vco;
		ZeroMemory(&vco, sizeof(CAN_OBJ));
		vco.ID = 0x00 + number;
		vco.SendType = 0;
		vco.RemoteFlag = 0;
		vco.ExternFlag = 0;
		vco.DataLen = 5;
		vco.Data[0] = 0x1F;
		int ratio = 101;

		int rpm_IQ24 = DtoIQ24(rpm * ratio / 960);
		vco.Data[1] = rpm_IQ24 >> 24 & 0xFF;
		vco.Data[2] = rpm_IQ24 >> 16 & 0xFF;
		vco.Data[3] = rpm_IQ24 >> 8 & 0xFF;
		vco.Data[4] = rpm_IQ24 & 0xFF;

		reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, vco.DataLen);
		DWORD dwRel;
		CAN_OBJ vcoR[1];
		dwRel = Receive(nDeviceType, nDeviceInd, nCANInd, vcoR, 1, 10);
		if (dwRel == 0 || reT == 0 || vcoR->ID != 0x00 + number || vcoR->Data[0] != 0X1F)
		{
			continue;
		}
		if (vcoR->Data[1] == 0X01)
		{
			return true;
		}
		else
		{
			cout << "Motor " << number << " set speed failed" << endl;
			return false;
		}
	}
	cout << "Motor " << number << " set speed failed for max_can_time" << endl;
	return false;
}

// 关节电机类成员函数：设置关节电机速度限制值
// @param position	期望关节电机速度限制值，单位r/s（转/秒）
// @param number 将设置速度限制值的关节电机ID
bool motor::setSpeedModeSpeed(double rpm, int number)
{
	int time = 0;
	while (time < MAX_CAN_TIME)
	{
		time++;
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		DWORD reT;
		CAN_OBJ vco;
		ZeroMemory(&vco, sizeof(CAN_OBJ));
		vco.ID = 0x00 + number;
		vco.SendType = 0;
		vco.RemoteFlag = 0;
		vco.ExternFlag = 0;
		vco.DataLen = 5;
		vco.Data[0] = 0x09;
		int ratio = 101;

		/*int rpm_IQ24 = DtoIQ24(rpm * ratio / 960);*/
		int rpm_IQ24 = DtoIQ24(rpm / 6000);
		vco.Data[1] = rpm_IQ24 >> 24 & 0xFF;
		vco.Data[2] = rpm_IQ24 >> 16 & 0xFF;
		vco.Data[3] = rpm_IQ24 >> 8 & 0xFF;
		vco.Data[4] = rpm_IQ24 & 0xFF;

		reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, vco.DataLen);
		DWORD dwRel;
		CAN_OBJ vcoR[1];
		dwRel = Receive(nDeviceType, nDeviceInd, nCANInd, vcoR, 1, 10);

		if (dwRel == 0 || reT == 0 || vcoR->ID != 0x00 + number || vcoR->Data[0] != 0X1F)
		{
			continue;
		}
		if (vcoR->Data[1] == 0X01)
		{
			return true;
		}
		else
		{
			cout << "Motor " << number << " set speed failed" << endl;
			return false;
		}
	}
	cout << "Motor " << number << " set speed failed for max_can_time" << endl;
	return false;
}

// 关节电机类成员函数：关节电机初始化
// @param number 将初始化的关节电机ID
// @param mode 关节电机运动模式
// @param rpm 关节电机速度限制值，单位r/s（转/秒）
// @return true if init successfully
bool motor::set_param(int number, int mode, double rpm)
{
	if (!enable(number))	//使能
	{
		return false;
	}
	if (! setPositionLimit(number))	//设置位置限制
	{
		return false;
	}
	if (!setMode(mode, number))	//设置控制模式
	{
		return false;
	}
	if (!setSpeed(rpm, number))	//设置速度限制
	{
		return false;
	}
	cout << "Motor " << number << " set param successed" << endl;
	return true;
}

// 关节电机类成员函数：设置关节电机角度值
// @param angle	期望关节电机角度值，单位°
// @param number 将设置角度值的关节电机ID
void motor::setAngle(double angle,int number)
{
	if (number == FIRST_MOTOR_CAN)
	{
		/*angle -= FIRST_MOTOR_OFFSET;
		angle = -angle;*/
		angle = -angle;
		angle += FIRST_MOTOR_OFFSET;
	}
	else if (number == SECOND_MOTOR_CAN)
	{
		/*angle = -angle;
		angle -= SECOND_MOTOR_OFFSET;*/
		angle = -angle;
		angle += SECOND_MOTOR_OFFSET;
	}
	else if (number == THIRD_MOTOR_CAN)
	{
		//angle += THIRD_MOTOR_OFFSET;
		angle += THIRD_MOTOR_OFFSET;
	}

	//将角度转化为r（转）后，设置关节电机位置
	setPosition(AngletoPosition(angle,number),number);
}

// 关节电机类成员函数：获取关节电机角度值
// @param number 将获取角度值的关节电机ID
// @return 关节电机角度值，单位°
double motor::getAngle(int number)
{
	//获取关节电机位置值r（转）后，转化为角度
	double angle = PositiontoAngle(getPosition(number), number);
	//cout << number << angle << endl;
	if (number == FIRST_MOTOR_CAN)
	{
		/*angle = -angle;
		angle += FIRST_MOTOR_OFFSET;*/
		angle -= FIRST_MOTOR_OFFSET;
		angle = -angle;
	}
	else if (number == SECOND_MOTOR_CAN)
	{
		/*angle = -angle;
		angle -= SECOND_MOTOR_OFFSET;*/
		angle -= SECOND_MOTOR_OFFSET;
		angle = -angle;
	}
	else if (number == THIRD_MOTOR_CAN)
	{
		//angle += THIRD_MOTOR_OFFSET;
		angle -= THIRD_MOTOR_OFFSET;
	}
	return angle;
}

// 关节电机类成员函数：关节电机急停
// @param number 急停的关节电机ID
void motor::stop(int number)
{
	ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
	DWORD reT;
	CAN_OBJ vco;
	ZeroMemory(&vco, sizeof(CAN_OBJ));
	vco.ID = 0x00 + number;
	vco.SendType = 0;
	vco.RemoteFlag = 0;
	vco.ExternFlag = 0;
	vco.DataLen = 2;
	vco.Data[0] = 0x07;
	vco.Data[1] = 0x03;
	reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, 2);
}

// 关节电机类成员函数：设置关节电机PID参数P
// @param p	期望关节电机PID参数P
// @param number 将设置参数的关节电机ID
// @return true if set proportional success
bool motor::setProportional(double p, int number)
{
	int time = 0;
	while (time < MAX_CAN_TIME)
	{
		time++;
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		DWORD reT;
		CAN_OBJ vco;
		ZeroMemory(&vco, sizeof(CAN_OBJ));
		vco.ID = 0x00 + number;
		vco.SendType = 0;
		vco.RemoteFlag = 0;
		vco.ExternFlag = 0;
		vco.DataLen = 5;

		vco.Data[0] = 0x12;
		int p_IQ24 = DtoIQ24(p);
		vco.Data[1] = p_IQ24 >> 24 & 0xFF;
		vco.Data[2] = p_IQ24 >> 16 & 0xFF;
		vco.Data[3] = p_IQ24 >> 8 & 0xFF;
		vco.Data[4] = p_IQ24 & 0xFF;
		reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, vco.DataLen);

		DWORD dwRel;
		CAN_OBJ vcoR[1];
		dwRel = Receive(nDeviceType, nDeviceInd, nCANInd, vcoR, 1, 10);
		if (dwRel == 0 || reT == 0 || vcoR->ID != 0x00 + number || vcoR->Data[0] != 0X12)
		{
			continue;
		}
		if (vcoR->Data[1] == 0X01)
		{
			cout << "Motor " << number << " set proportional " << p << endl;
			return true;
		}
		else
		{
			cout << "Motor " << number << " set proportional failed" << endl;
			return false;
		}
	}
	cout << "Motor " << number << " set proportional failed for max_can_time" << endl;
	return false;
}

// 关节电机类成员函数：设置关节电机PID参数I
// @param p	期望关节电机PID参数I
// @param number 将设置参数的的关节电机ID
// @return true if set integral success
bool motor::setIntegral(double i, int number)
{
	int time = 0;
	while (time < MAX_CAN_TIME)
	{
		time++;
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		DWORD reT;
		CAN_OBJ vco;
		ZeroMemory(&vco, sizeof(CAN_OBJ));
		vco.ID = 0x00 + number;
		vco.SendType = 0;
		vco.RemoteFlag = 0;
		vco.ExternFlag = 0;
		vco.DataLen = 5;

		vco.Data[0] = 0x13;
		int i_IQ24 = DtoIQ24(i);
		vco.Data[1] = i_IQ24 >> 24 & 0xFF;
		vco.Data[2] = i_IQ24 >> 16 & 0xFF;
		vco.Data[3] = i_IQ24 >> 8 & 0xFF;
		vco.Data[4] = i_IQ24 & 0xFF;
		reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, vco.DataLen);

		DWORD dwRel;
		CAN_OBJ vcoR[1];
		dwRel = Receive(nDeviceType, nDeviceInd, nCANInd, vcoR, 1, 10);
		if (dwRel == 0 || reT == 0 || vcoR->ID != 0x00 + number || vcoR->Data[0] != 0X13)
		{
			continue;
		}
		if (vcoR->Data[1] == 0X01)
		{
			cout << "Motor " << number << " set integral " << i << endl;
			return true;
		}
		else
		{
			cout << "Motor " << number << " set integral failed" << endl;
			return false;
		}
	}
	cout << "Motor " << number << " set integral failed for max_can_time" << endl;
	return false;
}

// 关节电机类成员函数：获取关节电机PID参数P
// @param number 将获取参数的的关节电机ID
// @return 关节电机PID参数P
double motor::getProportional(int number)
{
	double p = 0;
	while (true)
	{
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		DWORD reT;
		CAN_OBJ vco;
		ZeroMemory(&vco, sizeof(CAN_OBJ));
		vco.ID = 0x00 + number;
		vco.SendType = 1;
		vco.RemoteFlag = 0;
		vco.ExternFlag = 0;
		vco.DataLen = 1;
		vco.Data[0] = 0x19;
		reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, 6);
		DWORD dwRel;
		CAN_OBJ vcoR[1];
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		dwRel = Receive(nDeviceType, nDeviceInd, nCANInd, vcoR, 1, 10);
		if (dwRel == 0 || reT == 0 || vcoR->ID != 0x00 + number || vcoR->Data[0] != 0x19)
		{
			continue;
		}
		int data = vcoR->Data[1] << 24 | vcoR->Data[2] << 16 | vcoR->Data[3] << 8 | vcoR->Data[4];
		p = IQ24toD(data);
		return p;
	}
}

// 关节电机类成员函数：获取关节电机PID参数I
// @param number 将获取参数的关节电机ID
// @return 关节电机PID参数I
double motor::getIntegral(int number)
{
	double i = 0;
	while (true)
	{
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		DWORD reT;
		CAN_OBJ vco;
		ZeroMemory(&vco, sizeof(CAN_OBJ));
		vco.ID = 0x00 + number;
		vco.SendType = 1;
		vco.RemoteFlag = 0;
		vco.ExternFlag = 0;
		vco.DataLen = 1;
		vco.Data[0] = 0x1A;
		reT = Transmit(nDeviceType, nDeviceInd, nCANInd, &vco, 6);
		DWORD dwRel;
		CAN_OBJ vcoR[1];
		ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
		dwRel = Receive(nDeviceType, nDeviceInd, nCANInd, vcoR, 1, 10);
		if (dwRel == 0 || reT == 0 || vcoR->ID != 0x00 + number || vcoR->Data[0] != 0x1A)
		{
			continue;
		}
		int data = vcoR->Data[1] << 24 | vcoR->Data[2] << 16 | vcoR->Data[3] << 8 | vcoR->Data[4];
		i = IQ24toD(data);
		return i;
	}
}
#pragma endregion

#pragma region 关节电机功能函数
//角度转化为r（转）
double AngletoPosition(double angle, int number)
{
	int ratio = 101;

	return angle / 360 * ratio;
}

//r（转）转化为角度
double PositiontoAngle(double position, int number)
{
	int ratio = 101;

	return position / ratio * 360;
}
#pragma endregion