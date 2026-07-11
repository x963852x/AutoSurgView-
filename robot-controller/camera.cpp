/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of camera control

**************************************************************************/

#include "camera.h"

using std::unique_lock;
using std::mutex;

#pragma region 全局变量
int CAMERA_TILT_OFFSET;
int CAMERA_PAN_OFFSET;
int CAMERA_ROLL_OFFSET;

int ptz_speed = 12;
int ptz_speed_joy = 10;
int ptz_speed_max = 60;

double ptz_angle_max_pan = 40;
double ptz_angle_max_tilt = 40;
double ptz_angle_max_roll = 40;

extern std::map<std::string, std::string> parameters;

camera Cam;		//全局相机对象
int CAMERA_STATE = CAMERA_FREE;		//全局相机状态

std::mutex CAM_mutex;		//全局相机互斥锁——摄像机模组
std::mutex MOT_mutex;		//全局相机互斥锁——云台电机

serial::Serial s_cam;	//相机串口对象——摄像机模组
serial::Serial s_mot;	//相机串口对象——电机模组

std::vector<double> joint_now(6);	//全局机械臂当前值 [0-2]关节电机 [3-5]云台电机
std::vector<double> joint_tar(6);	//全局机械臂目标值 [0-2]关节电机 [3-5]云台电机
#pragma endregion

#pragma region 外部引用变量
extern int ARM_STATE;
extern bool FOCUS_STATE;
extern int OP_STATUS;
#pragma endregion

//相机类默认构造函数
camera::camera()
{

}

#pragma region 云台运动
//俯仰电机模式控制
//@param speed 目标速度 正值 单位 °/s
void camera::setPanMotor_with_speed(double speed)
{
	//speed = speed / 0.24;
	int16_t speed16 = (int16_t)speed;
	uint16_t uspeed16 = (uint16_t)speed16;

	uint8_t cw[10] = { 0x55, 0x55, 0x02, 0x07, 0x1D, 0x01, 0x00, 0x00,
		0x00, 0x00 };

	cw[7] = *((uint8_t*)(&uspeed16));
	cw[8] = *((uint8_t*)(&uspeed16) + 1);
	cw[9] = CheckSum(cw);

	MOT_mutex.lock();
	s_mot.flushOutput();
	s_mot.write(cw, 18);
	s_mot.flushOutput();
	MOT_mutex.unlock();
}

//侧倾电机模式控制
//@param speed 目标速度 正值 单位 °/s
void camera::setTilMotor_with_speed(double speed)
{
	//speed = speed / 0.24;
	int16_t speed16 = (int16_t)speed;
	uint16_t uspeed16 = (uint16_t)speed16;

	uint8_t cw[10] = { 0x55, 0x55, 0x01, 0x07, 0x1D, 0x01, 0x00, 0x00,
		0x00, 0x00 };

	cw[7] = *((uint8_t*)(&uspeed16));
	cw[8] = *((uint8_t*)(&uspeed16) + 1);
	cw[9] = CheckSum(cw);

	MOT_mutex.lock();
	s_mot.flushOutput();
	s_mot.write(cw, 18);
	s_mot.flushOutput();
	MOT_mutex.unlock();
}

//俯仰电机模式转舵机模式
void camera::setPanMotor2Servo()
{
	uint8_t cw[10] = { 0x55, 0x55, 0x02, 0x07, 0x1D, 0x00, 0x00, 0x00,
		0x00, 0x00 };

	cw[9] = CheckSum(cw);

	MOT_mutex.lock();
	s_mot.flushOutput();
	s_mot.write(cw, 18);
	s_mot.flushOutput();
	MOT_mutex.unlock();
}

//侧倾电机模式转舵机模式
void camera::setTilMotor2Servo()
{
	uint8_t cw[10] = { 0x55, 0x55, 0x01, 0x07, 0x1D, 0x00, 0x00, 0x00,
		0x00, 0x00 };

	cw[9] = CheckSum(cw);

	MOT_mutex.lock();
	s_mot.flushOutput();
	s_mot.write(cw, 18);
	s_mot.flushOutput();
	MOT_mutex.unlock();
}

//舵机模式控制
//相机类成员函数：云台电机俯仰运动指令，带有速度限制
//@param Pan 目标位置 单位 °
//@param speed 目标速度 正值 单位 °/s
void camera::setPan_with_speed(double Pan, double speed)
{
	CAMERA_STATE = CAMERA_MOVING;

	double Pan_now;
	double time = 0;

	if (speed != 0 && getPan(Pan_now))
	{
		time = abs(Pan - Pan_now) / speed * 1000;
	}

	int16_t pan16 = Pan / 0.24 + CAMERA_PAN_OFFSET;
	int16_t time16 = (int16_t)time;

	uint8_t cw[10] = { 0x55, 0x55, 0x02, 0x07, 0x01, 0x00, 0x00, 0x00,
		0x00, 0x00 };

	cw[5] = *(uint8_t*)(&pan16);
	cw[6] = *((uint8_t*)(&pan16) + 1);
	cw[7] = *((uint8_t*)(&time16));
	cw[8] = *((uint8_t*)(&time16) + 1);
	cw[9] = CheckSum(cw);

	MOT_mutex.lock();
	s_mot.flushOutput();
	s_mot.write(cw, 18);
	s_mot.flushOutput();
	MOT_mutex.unlock();
}

//相机类成员函数：云台电机侧倾运动指令，带有速度限制
//@param Tilt 目标位置 单位 °
//@param speed 目标速度 正值 单位 °/s
void camera::setTilt_with_speed(double Tilt, double speed)
{
	CAMERA_STATE = CAMERA_MOVING;

	double Tilt_now;
	double time = 0;

	if (speed != 0 && getTilt(Tilt_now))
	{
		time = abs(Tilt - Tilt_now) / speed * 1000;
	}

	int16_t tilt16 = Tilt / 0.24 + CAMERA_TILT_OFFSET;
	int16_t time16 = (int16_t)time;

	uint8_t cw[10] = { 0x55, 0x55, 0x01, 0x07, 0x01, 0x00, 0x00, 0x00,
		0x00, 0x00 };

	cw[5] = *(uint8_t*)(&tilt16);
	cw[6] = *((uint8_t*)(&tilt16) + 1);
	cw[7] = *((uint8_t*)(&time16));
	cw[8] = *((uint8_t*)(&time16) + 1);
	cw[9] = CheckSum(cw);

	MOT_mutex.lock();
	s_mot.flushOutput();
	s_mot.write(cw, 18);
	s_mot.flushOutput();
	MOT_mutex.unlock();
}

//相机类成员函数：云台电机侧倾运动指令，带有速度限制
//@param Roll 目标位置 单位 °
//@param speed 目标速度 正值 单位 °/s
void camera::setRoll_with_speed(double Roll, double speed)
{
	CAMERA_STATE = CAMERA_MOVING;

	double Roll_now;
	double time = 0;

	if (speed != 0 && getRoll(Roll_now))
	{
		time = abs(Roll - Roll_now) / speed * 1000;
	}

	int16_t roll16 = Roll / 0.24 + CAMERA_ROLL_OFFSET;
	roll16 = std::min((int16_t)1000, std::max((int16_t)0, roll16));
	int16_t time16 = (int16_t)time;

	uint8_t cw[10] = { 0x55, 0x55, 0x03, 0x07, 0x01, 0x00, 0x00, 0x00,
		0x00, 0x00 };

	cw[5] = *(uint8_t*)(&roll16);
	cw[6] = *((uint8_t*)(&roll16) + 1);
	cw[7] = *((uint8_t*)(&time16));
	cw[8] = *((uint8_t*)(&time16) + 1);
	cw[9] = CheckSum(cw);

	MOT_mutex.lock();
	s_mot.flushOutput();
	s_mot.write(cw, 18);
	s_mot.flushOutput();
	MOT_mutex.unlock();
}

//相机类成员函数：云台电机运动指令
//@param Pan 目标位置 单位 ° 
//@param Tilt 目标位置 单位 °
void camera::setWrist(double Pan, double Tilt, double Roll)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	this->setPan_with_speed(Pan, ptz_speed);	//速度默认为12
	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	this->setTilt_with_speed(Tilt, ptz_speed);	//速度默认为12
	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	this->setRoll_with_speed(Roll, ptz_speed);	//速度默认为12
}

//相机类成员函数：云台电机读取俯仰角度指令
//@param Pan 目标位置输出变量 单位 °
bool camera::getPan(double& Pan)
{
	uint8_t cw[6] = { 0x55, 0x55, 0x02, 0x03, 0x1C, 0xDE };

	uint8_t cr[8] = { 0 };

	MOT_mutex.lock();

	int max_times = 3;

	while (max_times--)
	{
		s_mot.flushInput();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		s_mot.write(cw, 6);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		int n = s_mot.available();
		if (n == 8)
		{
			s_mot.read(cr, 8);
			if (cr[2] == 0x02 && cr[4] == 0x1C && cr[7] == CheckSum(cr))
			{
				int16_t cache = (int16_t)((cr[6] & 0x00FF) << 8 | cr[5] & 0x00FF);
				Pan = (cache - CAMERA_PAN_OFFSET) * 0.24;

				this->pan = Pan;

				MOT_mutex.unlock();
				return true;
			}
		}
	}

	MOT_mutex.unlock();

	return false;
}

//相机类成员函数：云台电机读取侧倾角度指令
//@param Tilt 目标位置输出变量 单位 °
bool camera::getTilt(double& Tilt)
{
	uint8_t cw[6] = { 0x55, 0x55, 0x01, 0x03, 0x1C, 0xDF };

	uint8_t cr[8] = { 0 };

	MOT_mutex.lock();

	int max_times = 3;

	while (max_times--)
	{
		s_mot.flushInput();
		s_mot.write(cw, 6);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		int n = s_mot.available();
		if (n == 8)
		{
			s_mot.read(cr, 8);
			if (cr[2] == 0x01 && cr[4] == 0x1C && cr[7] == CheckSum(cr))
			{
				int16_t cache = (int16_t)((cr[6] & 0x00FF) << 8 | cr[5] & 0x00FF);
				Tilt = (cache - CAMERA_TILT_OFFSET) * 0.24;

				this->tilt = Tilt;

				MOT_mutex.unlock();
				return true;
			}
		}
	}

	MOT_mutex.unlock();
	return false;
}

//相机类成员函数：云台电机读取自旋角度指令
//@param Tilt 目标位置输出变量 单位 °
bool camera::getRoll(double& Roll)
{
	uint8_t cw[6] = { 0x55, 0x55, 0x03, 0x03, 0x1C, 0xDD };

	uint8_t cr[8] = { 0 };

	MOT_mutex.lock();

	int max_times = 3;

	while (max_times--)
	{
		s_mot.flushInput();
		s_mot.write(cw, 6);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		int n = s_mot.available();
		if (n == 8)
		{
			s_mot.read(cr, 8);
			if (cr[2] == 0x03 && cr[4] == 0x1C && cr[7] == CheckSum(cr))
			{
				int16_t cache = (int16_t)((cr[6] & 0x00FF) << 8 | cr[5] & 0x00FF);
				Roll = (cache - CAMERA_ROLL_OFFSET) * 0.24;

				this->roll = Roll;

				MOT_mutex.unlock();
				return true;
			}
		}
	}

	MOT_mutex.unlock();
	return false;
}

//相机类成员函数：云台电机读取角度指令
//@param Pan 目标位置输出变量 单位 °
//@param Tilt 目标位置输出变量 单位 °
bool camera::getWrist(double& Pan, double& Tilt, double& Roll)
{
	return getPan(Pan) && getTilt(Tilt) && getRoll(Roll);
}

//相机类成员函数：云台电机运动指令 用于响应手柄控制
//@param orian 相机运动控制信号
//@param speed 相机运动目标速度 单位 °/s
void camera::move(CAMERA_O orian, int speed)
{
	if (orian == CAMERA_U)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		this->setPan_with_speed(ptz_angle_max_pan, speed);
	}
	if (orian == CAMERA_D)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		this->setPan_with_speed(-ptz_angle_max_pan, speed);
	}
	if (orian == CAMERA_L)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		this->setTilt_with_speed(ptz_angle_max_tilt, speed);
	}
	if (orian == CAMERA_R)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		this->setTilt_with_speed(-ptz_angle_max_tilt, speed);
	}
	if (orian == CAMERA_P)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		this->setRoll_with_speed(ptz_angle_max_roll, speed);
	}
	if (orian == CAMERA_N)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		this->setRoll_with_speed(-ptz_angle_max_roll, speed);
	}
	std::cout << "orian = " << orian;
}

//相机类成员函数：云台电机俯仰停止指令
void camera::stopPan()
{
	uint8_t cw[6] = { 0x55, 0x55, 0x02, 0x03, 0x0C, 0xEE };

	MOT_mutex.lock();
	s_mot.write(cw, 6);
	s_mot.write(cw, 6);
	s_mot.flush();
	MOT_mutex.unlock();
}

//相机类成员函数：云台电机侧倾停止指令
void camera::stopTilt()
{
	uint8_t cw[6] = { 0x55, 0x55, 0x01, 0x03, 0x0C, 0xEF };

	MOT_mutex.lock();
	s_mot.write(cw, 6);
	s_mot.write(cw, 6);
	s_mot.flush();
	MOT_mutex.unlock();
}

//相机类成员函数：云台电机自旋停止指令
void camera::stopRoll()
{
	uint8_t cw[6] = { 0x55, 0x55, 0x03, 0x03, 0x0C, 0xED };

	MOT_mutex.lock();
	s_mot.write(cw, 6);
	s_mot.write(cw, 6);
	s_mot.flush();
	MOT_mutex.unlock();
}


//相机类成员函数：云台电机停止指令
void camera::stop(int LastState)
{
	switch (LastState)
	{
	case 0:
		this->stopPan();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		this->stopTilt();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		this->stopRoll();
		break;
	case 'U':
		this->stopPan();
		break;
	case 'D':
		this->stopPan();
		break;
	case 'L':
		this->stopTilt();
		break;
	case 'R':
		this->stopTilt();
		break;
	case 'P':
		this->stopRoll();
		break;
	case 'N':
		this->stopRoll();
		break;
	default:
		break;
	}

	CAMERA_STATE = CAMERA_FREE;
}

uint8_t camera::CheckSum(uint8_t buf[])
{
	uint8_t i;
	uint16_t temp = 0;
	for (i = 2; i < buf[3] + 2; i++) {
		temp += buf[i];
	}
	temp = ~temp;
	i = (uint8_t)temp;
	return i;
}
#pragma endregion

#pragma region 相机光学

//相机类成员函数：相机模组缩放指令
//@param orian 相机缩放控制信号
void camera::zoom(int orian)
{
	uint8_t cw[6] = { 0x81, 0x01, 0x04 ,0x07, 0x00, 0xFF };	//停止缩放
	if (orian == 1)	//视野放大
	{
		cw[4] = 0x35;
	}
	else if (orian == -1)	//视野缩小
	{
		cw[4] = 0x25; 
	}
	s_cam.write(cw, 7);
}


//相机类成员函数：相机聚焦控制指令
//@param orian 相机聚焦控制信号
void camera::focus(int orian)
{
	uint8_t cw[6] = { 0x81, 0x01, 0x04 ,0x38, 0x03, 0xFF };	//停止
	if (orian == 1)
	{
		cw[3] = 0x08;
		cw[4] = 0x02;
	}
	else if (orian == 2)
	{
		cw[3] = 0x08;
		cw[4] = 0x03;
	}
	s_cam.write(cw, 7);
}

//相机类成员函数：相机曝光控制指令
//@param orian 相机曝光控制信号
void camera::exposure(int orian)
{
	uint8_t cw[7] = { 0xFF, 0x01, 0x00 ,0x00, 0x00, 0x00, 0x01 };	//停止
	//亮
	if (orian == 1)
	{
		cw[2] = 0x04;
		cw[6] = 0x05;
	}
	else if (orian == 2)	//暗
	{
		cw[2] = 0x02;
		cw[6] = 0x03;
	}
	s_cam.write(cw, 7);
}

//相机类成员函数：相机无影灯程序控制指令
//@param orian 相机无影灯程序控制信号
void camera::light(int orian)
{
	uint8_t cw[7] = { 0xFF, 0x01, 0x00 ,0x00, 0x00, 0x00, 0x01 };	//停止
	//无影灯模式开
	if (orian == 1)
	{
		cw[3] = 0x07;
		cw[5] = 0x15;
		cw[6] = 0x1D;
	}
	else if (orian == 2)	//无影灯模式关
	{
		cw[3] = 0x07;
		cw[5] = 0x14;
		cw[6] = 0x1C;
	}
	s_cam.write(cw, 7);
}
#pragma endregion

//相机伺服程序
void camera_work()
{
	while (true)
	{
		//在目标锁定状态，且机械臂运动时，实时控制云台电机至指定位置
		//（此情况下，joint_now数组中，云台电机值为期望值，在机械臂follow_traj函数中被实时更改）
		if (ARM_STATE == ARM_FOLLOW_TRAJ && (FOCUS_STATE || OP_STATUS == 1))
		{
			//角度值限幅确认
			if (abs(joint_now[4]) < ptz_angle_max_pan && abs(joint_now[3]) < ptz_angle_max_tilt)
			{
				//发送位置控制指令
				Cam.setWrist(joint_now[4], joint_now[3], joint_now[5]);
			}
			else
			{
				cout << "error" << " Tilt = " << joint_now[3] << ", " << "Pan = " << joint_now[4] << "Roll = " << joint_now[5] << endl;
			}
		}
		//相机在手柄控制运动过程中，实时读取云台电机值
		//（此情况下，joint_now数组中，云台电机值为实际值）
		else if (CAMERA_STATE == CAMERA_MOVING)
		{
			Cam.getPan(joint_now[4]);
			Cam.getTilt(joint_now[3]);
			Cam.getRoll(joint_now[5]);

			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}
}

void camera_init()
{
	s_cam.setPort(parameters["serial_cam"]);	//相机串口对象——摄像机模组
	s_cam.setBaudrate(9600);
	s_cam.open();
	s_mot.setPort(parameters["serial_mot"]);	//相机串口对象——电机模组
	s_mot.setBaudrate(115200);
	s_mot.open();

	CAMERA_TILT_OFFSET = std::atof(parameters["CAMERA_TILT_OFFSET"].c_str());
	CAMERA_PAN_OFFSET = std::atof(parameters["CAMERA_PAN_OFFSET"].c_str());
	CAMERA_ROLL_OFFSET = std::atof(parameters["CAMERA_ROLL_OFFSET"].c_str());

	ptz_speed = std::atof(parameters["PTZ_SPEED"].c_str());
	ptz_speed_joy = std::atof(parameters["PTZ_SPEED_JOY"].c_str());
	ptz_speed_max = std::atof(parameters["PTZ_SPEED_MAX"].c_str());

	ptz_angle_max_pan = std::max(std::min(90.0,std::atof(parameters["PTZ_ANGLE_MAX_PAN"].c_str())),40.0);
	ptz_angle_max_tilt = std::max(std::min(90.0, std::atof(parameters["PTZ_ANGLE_MAX_TILT"].c_str())), 40.0);
	ptz_angle_max_roll = std::max(std::min(120.0, std::atof(parameters["PTZ_ANGLE_MAX_ROLL"].c_str())), 40.0);
}