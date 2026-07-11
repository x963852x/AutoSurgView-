/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of liftingcolumn control

**************************************************************************/

//#include "LiftingColumn.h"
//
//#pragma region 全局变量
//extern std::map<std::string, std::string> parameters;
//
//serial::Serial s_lc;	//升降立柱串口对象
//
//std::mutex S_LC;	//全局线程锁——升降立柱
//
//LiftingColumn LC;	//全局升降立柱对象
//
//int LC_state = LC_FREE;	//全局升降立柱工作状态
//
//double lc_target;		//立柱运动目标
//double lc_now;			//立柱当前位置
//double lc_start;		//立柱运动起始位置
//#pragma endregion
//
//#pragma region 升降立柱类
////升降立柱类默认构造函数
//LiftingColumn::LiftingColumn()
//{
//}
//
//LiftingColumn::~LiftingColumn()
//{
//}
//
//void LiftingColumn::reset()
//{
//	uint8_t cw[16] = {
//	0x1B,
//	0x43, 0x57,
//	0x32, 0x30, 0x30, 0x30,
//	0x30, 0x31,
//	0x30, 0x30, 0x30, 0x31,
//	0x37, 0x45, 0x05
//	};
//
//	uint8_t cr[5] = { 0 };
//
//	S_LC.lock();
//	std::this_thread::sleep_for(std::chrono::milliseconds(100));
//	s_lc.write(cw, 16);
//	s_lc.flush();
//	s_lc.read(cr, 5);
//	S_LC.unlock();
//	LC_state = LC_RESET;
//
//}
//
////升降立柱类成员函数：上升指令
//void LiftingColumn::up()
//{
//	uint8_t cw[20] = {
//	0x1B,
//	0x43, 0x57,
//	0x32, 0x30, 0x31, 0x30,
//	0x30, 0x32,
//	0x30, 0x30, 0x30, 0x34,
//	0x30, 0x30, 0x30, 0x30,
//	0x34, 0x33, 0x05
//	};
//
//	uint8_t cr[5] = { 0 };
//
//	S_LC.lock();
//	std::this_thread::sleep_for(std::chrono::milliseconds(100));
//	s_lc.write(cw, 20);
//	s_lc.flush();
//	s_lc.read(cr, 5);
//	S_LC.unlock();
//	LC_state = LC_DEBUG;
//}
//
////升降立柱类成员函数：下降指令
//void LiftingColumn::down()
//{
//	uint8_t cw[20] = {
//		0x1B,
//		0x43, 0x57,
//		0x32, 0x30, 0x31, 0x30,
//		0x30, 0x32,
//		0x30, 0x30, 0x30, 0x36,
//		0x30, 0x30, 0x30, 0x30,
//		0x34, 0x35, 0x05
//	};
//
//	uint8_t cr[5] = { 0 };
//	
//	S_LC.lock();
//	std::this_thread::sleep_for(std::chrono::milliseconds(100));
//	s_lc.write(cw, 20);
//	s_lc.flush();
//	s_lc.read(cr, 5);
//	S_LC.unlock();
//	LC_state = LC_DEBUG;
//}
//
////升降立柱类成员函数：停止指令
//void LiftingColumn::stop()
//{
//	uint8_t cw[20] = {
//	0x1B,
//	0x43, 0x57,
//	0x32, 0x30, 0x31, 0x30,
//	0x30, 0x32,
//	0x30, 0x30, 0x30, 0x30,
//	0x30, 0x30, 0x30, 0x30,
//	0x33, 0x46, 0x05
//	};
//
//	uint8_t cr[5] = { 0 }; 
//	
//	S_LC.lock();
//	std::this_thread::sleep_for(std::chrono::milliseconds(100));
//	s_lc.write(cw, 20);
//	s_lc.flush();
//	s_lc.read(cr, 5);
//
//	//上键弹起
//	cw[12] = 0x35;
//	cw[17] = 0x34;
//	cw[18] = 0x34;
//	s_lc.write(cw, 20);
//	s_lc.flush();
//	s_lc.read(cr, 5);
//
//	//下键弹起
//	cw[12] = 0x37;
//	cw[17] = 0x34;
//	cw[18] = 0x36;
//	s_lc.write(cw, 20);
//	s_lc.flush();
//	s_lc.read(cr, 5);
//	
//	S_LC.unlock();
//
//	LC_state = LC_FREE;
//}
//
////升降立柱类成员函数：获取立柱高度
////@return height 立柱高度
//double LiftingColumn::get_height()
//{
//	uint8_t cw[12] = {
//	0x1B,	//帧头
//	0x50, 0x52,	//命令字 PR
//	0x31, 0x30, 0x33, 0x30,
//	0x30, 0x31,
//	0x43, 0x37,
//	0x05	//帧尾
//	};
//
//	uint8_t cr[100] = { 0 };
//
//	while (true)
//	{
//		S_LC.lock();
//		std::this_thread::sleep_for(std::chrono::milliseconds(100));
//		s_lc.write(cw, 12);
//
//		std::this_thread::sleep_for(std::chrono::milliseconds(100));
//
//		int n = s_lc.available();
//		if (n)
//		{
//			if (n <= 100)
//			{
//				s_lc.read(cr, n);
//			}
//			else
//			{
//				s_lc.close();
//				std::this_thread::sleep_for(std::chrono::milliseconds(100));
//				s_lc.open();
//
//				std::cout << "立柱串口重启" << std::endl;
//			}
//
//			s_lc.flush();
//			S_LC.unlock();
//		}
//		else
//		{
//			s_lc.flush();
//			S_LC.unlock();
//			continue;
//		}
//
//		if (cr[0] == 0x1B && cr[1] == 0x41)
//		{
//
//			uint8_t high, low;
//			get_CheckSum(cr, 9, high, low);
//
//			if (cr[7] == low && cr[6] == high)
//			{
//				int height = ascii_to_hex(cr[2]);
//				height <<= 4;
//				height += ascii_to_hex(cr[3]);
//				height <<= 4;
//				height += ascii_to_hex(cr[4]);
//				height <<= 4;
//				height += ascii_to_hex(cr[5]);
//
//				std::cout << "height = " << height << std::endl;
//
//				return height * 0.01;
//			}
//		}
//
//		std::cout << "get height error" << std::endl;
//		std::this_thread::sleep_for(std::chrono::milliseconds(400));
//		continue;
//	}
//}
////升降立柱成员函数：运动至指定高度
////@param height 目标高度
//void LiftingColumn::move_to_height(double height)
//{
//	LC.stop();
//
//	int height_send = (int)(height * 100);	//0.1mm单位
//
//	height_send = std::min(17500, std::max(10100, height_send));	//目标高度限幅
//	
//	lc_target = height;
//
//	uint8_t cw[20] = {
//		0x1B,
//		0x43, 0x57,
//		0x32, 0x30, 0x31, 0x30,
//		0x30, 0x32,
//		0x30, 0x30, 0x30, 0x33,
//		0x30, 0x30, 0x30, 0x30,
//		0x33, 0x46, 0x05
//	};
//
//	cw[16] = hex_to_ascii(height_send & 0x0F);
//	cw[15] = hex_to_ascii((height_send >> 4) & 0x0F);
//	cw[14] = hex_to_ascii((height_send >> 8) & 0x0F);
//	cw[13] = hex_to_ascii((height_send >> 12) & 0x0F);
//
//	uint8_t high, low;
//	get_CheckSum(cw, sizeof(cw), high, low);
//	cw[sizeof(cw) - 3] = high;
//	cw[sizeof(cw) - 2] = low;
//
//	S_LC.lock();
//	s_lc.write(cw, 20);
//	s_lc.flush();
//	S_LC.unlock();
//
//	LC_state = LC_MOVING;	//立柱状态设置为运动至目标状态
//	lc_target = height;
//	lc_start = lc_now;	//立柱出发高度设置为当前高度
//}
//
//void LiftingColumn::get_CheckSum(uint8_t buf[], int len, uint8_t& high, uint8_t& low)
//{
//	uint8_t i;
//	uint32_t temp = 0;
//
//	for (i = 1; i < len - 3; i++)
//	{
//		temp += buf[i];
//	}
//
//	temp = temp % 256;
//
//	high = temp >> 4;
//	low = temp & 0x0F;
//
//
//	high = hex_to_ascii(high);
//	low = hex_to_ascii(low);
//}
//
//#pragma endregion
//
//uint8_t ascii_to_hex(uint8_t ascii)
//{
//	if (ascii >= '0' && ascii <= '9')
//	{
//		return ascii - '0';
//	}
//	else if (ascii >= 'A' && ascii <= 'F')
//	{
//		return 0x0A + ascii - 'A';
//	}
//	else
//	{
//		return 0;
//	}
//}
//
//uint8_t hex_to_ascii(uint8_t hex)
//{
//	if (hex < 0x0A)
//	{
//		return hex + '0';
//	}
//	else
//	{
//		return hex - 0x0A + 'A';
//	}
//}
//
//
//
//
////升降立柱伺服程序
//void liftcolumn_work()
//{
//	while (true)
//	{
//		//立柱在运动至目标位置状态
//		if (LC_state == LC_MOVING)
//		{
//			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
//
//			lc_now = LC.get_height();	//读取当前高度
//
//			//若经过目标位置
//			if ((lc_target - lc_now) * (lc_target - lc_start) <= 0 || abs(lc_target - lc_now) <= 1)
//			{
//				LC.stop();	//立柱停止
//			}
//			else
//			{
//				std::this_thread::sleep_for(std::chrono::milliseconds(100));
//			}
//		}
//		//立柱在调试状态（对应机器人面板按键控制）
//		else if (LC_state == LC_DEBUG)
//		{
//			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
//			lc_now = LC.get_height();	//读取当前高度
//			std::this_thread::sleep_for(std::chrono::milliseconds(100));
//		}
//		else if (LC_state == LC_RESET)
//		{
//			lc_now = LC.get_height();	//读取当前高度
//			LC_state = LC_FREE;
//			std::this_thread::sleep_for(std::chrono::milliseconds(100));
//		}
//		else
//		{
//			std::this_thread::sleep_for(std::chrono::seconds(1));
//		}
//	}
//}
//
//void liftcolumn_init()
//{
//	s_lc.setPort(parameters["serial_lc"]);	//相机串口对象——摄像机模组
//	s_lc.setBaudrate(19200);
//	s_lc.open();
//}

#include "LiftingColumn.h"

serial::Serial s_lc;	//升降立柱串口对象
std::mutex S_LC;	//全局线程锁——升降立柱
LiftingColumn LC;	//全局升降立柱对象

extern std::map<std::string, std::string> parameters;
extern int autoState;
int LC_state = LC_FREE;	//全局升降立柱工作状态
int lc_start = -237;
int lc_zero = 1314;
int lc_now;
int lc_target;
LiftingColumn::LiftingColumn()
{
}

LiftingColumn::~LiftingColumn()
{
}
void LiftingColumn::reset()
{
	move_to_height(10);
	LC_state = LC_RESET;

}

//升降立柱类成员函数：上升指令
void LiftingColumn::up()
{
	move_to_height(700);
	LC_state = LC_DEBUG;
}

//升降立柱类成员函数：下降指令
void LiftingColumn::down()
{
	move_to_height(10);
	LC_state = LC_DEBUG;
}

//升降立柱类成员函数：停止指令
void LiftingColumn::stop()
{
	LC_Disable();
	LC_state = LC_FREE;
}
void LiftingColumn::set_to_height(int height)
{
	int now_height = get_height();
	lc_target = height;
	cout << "当前高度为：" << now_height << endl;
	cout << "目标高度为：" << lc_target << endl;
	set_relative_dis(-1 * (height - now_height));
}
void LiftingColumn::move_to_height(int height)
{
	LC_state = LC_MOVING;
	LC_Enable();           //使能
	set_to_height(height); //脉冲数计算
	GHGSTP_Low();          //换步信号先失能
	GHGSTP_High();         //换步信号再使能
	GHGSTP_Low();
}
void LiftingColumn::GHGSTP_High()
{
	vector<uint8_t> cw = { 0x01, 0x06, 0x05, 0x23, 0x00, 0x10, 0x00, 0x00 };
	creat_crc(cw);
	vector<uint8_t> cr;
	S_LC.lock();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	s_lc.write(cw);
	s_lc.flush();
	s_lc.read(cr, cw.size());
	S_LC.unlock();
}
void LiftingColumn::GHGSTP_Low()
{
	vector<uint8_t> cw = { 0x01, 0x06, 0x05, 0x23, 0x00, 0x00, 0x00, 0x00 };
	creat_crc(cw);
	vector<uint8_t> cr;
	S_LC.lock();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	s_lc.write(cw);
	s_lc.flush();
	s_lc.read(cr, cw.size());
	S_LC.unlock();
}
void LiftingColumn::LC_Enable()
{
	vector<uint8_t> cw = { 0x01, 0x06, 0x05, 0x14, 0x00, 0x10, 0x00, 0x00 };
	creat_crc(cw);
	vector<uint8_t> cr;
	S_LC.lock();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	s_lc.write(cw);
	s_lc.flush();
	s_lc.read(cr, cw.size());
	S_LC.unlock();
}
void LiftingColumn::LC_Disable()
{
	vector<uint8_t> cw = { 0x01, 0x06, 0x05, 0x14, 0x00, 0x00, 0x00, 0x00 };
	creat_crc(cw);
	vector<uint8_t> cr;
	S_LC.lock();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	s_lc.write(cw);
	s_lc.flush();
	s_lc.read(cr, cw.size());
	S_LC.unlock();
}
/*
	@dis:距离当前立柱高度运动多少高度--20mm
	@speed:立柱运行速度--1mm/s
*/
void LiftingColumn::set_relative_dis(int dis)
{
	int pluse = dis_2_pulse(dis);
	uint16_t low = (uint16_t)((int16_t)(pluse % 10000));
	uint16_t high = (uint16_t)((int16_t)(pluse / 10000));

	vector<uint8_t> cw = { 0x01, 0x06, 0x04, 0x0A, 0x00, 0x00, 0x00, 0x00 };
	cw[4] = (uint8_t)(low >> 8);
	cw[5] = (uint8_t)low;
	creat_crc(cw);
	vector<uint8_t> cr;
	S_LC.lock();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	s_lc.write(cw);
	s_lc.flush();
	s_lc.read(cr, cw.size());
	S_LC.unlock();

	vector<int> temp_vec = { 0x01, 0x06, 0x04, 0x0B, 0x00, 0x00, 0x00, 0x00 };
	temp_vec[4] = (uint8_t)(high >> 8);
	temp_vec[5] = (uint8_t)high;
	cw.resize(0);
	cr.resize(0);
	cw.insert(cw.end(), temp_vec.begin(), temp_vec.end());
	creat_crc(cw);
	S_LC.lock();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	s_lc.write(cw);
	s_lc.flush();
	s_lc.read(cr, cw.size());
	S_LC.unlock();
}

/*
	@dis:距离当前立柱高度运动多少高度--20mm
	@speed:立柱运行速度--1mm/s
*/
void LiftingColumn::set_speed(double speed)
{
	uint16_t sp = (int16_t)(speed / 5.0 * 600);
	vector<uint8_t> cw = { 0x01, 0x06, 0x04, 0x0C, 0x00, 0x00, 0x00, 0x00 };
	cw[4] = (uint8_t)(sp >> 8);
	cw[5] = (uint8_t)sp;
	creat_crc(cw);
	vector<uint8_t> cr;
	S_LC.lock();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	s_lc.write(cw);
	s_lc.flush();
	s_lc.read(cr, cw.size());
	S_LC.unlock();
}
int LiftingColumn::dis_2_pulse(int dis)
{
	double circle_c = (double)dis / 5.0;
	int pulse_count = (int)(circle_c * (13178.0));
	return pulse_count;
}
int LiftingColumn::get_position()
{
	int ret = 0;
	vector<uint8_t> cw = {
		0x01, 0x03, 0x10, 0x5B, 0x00, 0x01, 0x00, 0x00
	};
	creat_crc(cw);

	vector<uint8_t> cr;
	S_LC.lock();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	s_lc.write(cw);
	s_lc.flush();
	s_lc.read(cr, 7);
	S_LC.unlock();
	int16_t circle_count = 0;
	if (check_crc(cr))
	{
		circle_count = (((uint16_t)cr[3]) << 8) + ((uint16_t)cr[4]);
		//cout << "当前圈数为：" << circle_count << endl;
	}
	else
	{
		cout << "圈数返回错误" << endl;
	}

	vector<int> temp_vec = { 0x01, 0x03, 0x10, 0x03, 0x00, 0x01, 0x00, 0x00 };
	cr.resize(0);
	cw.resize(0);
	cw.insert(cw.end(), temp_vec.begin(), temp_vec.end());
	creat_crc(cw);
	S_LC.lock();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	s_lc.write(cw);
	s_lc.flush();
	s_lc.read(cr, 7);
	S_LC.unlock();
	int16_t mach_angle = 0;
	if (check_crc(cr))
	{
		mach_angle = (((uint16_t)cr[3]) << 8) + ((uint16_t)cr[4]);
		//cout << "当前角度为：" << mach_angle << endl;
		//cout << "当前高度为：" << -(get_position() - lc_start) * (5.0 / 360.0) << endl;
	}
	else
	{
		cout << "角度返回错误" << endl;
	}
	ret = circle_count * 360 + mach_angle;
	return ret;
}
int LiftingColumn::get_height()
{
	double now_height = -(get_position() - lc_start) * (5.0 / 360.0);
	return (int)now_height;
}
void LiftingColumn::set_pulse_per_circle()
{
	uint32_t pulse_per_circle = 13178;
	uint16_t low = ((uint16_t)(pulse_per_circle % 10000));
	uint16_t high = ((uint16_t)(pulse_per_circle / 10000));

	vector<uint8_t> cw = { 0x01, 0x06, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00 };
	cw[4] = (uint8_t)(low >> 8);
	cw[5] = (uint8_t)low;
	creat_crc(cw);
	vector<uint8_t> cr;
	S_LC.lock();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	s_lc.write(cw);
	s_lc.flush();
	s_lc.read(cr, cw.size());
	S_LC.unlock();

	vector<int> temp_vec = { 0x01, 0x06, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00 };
	temp_vec[4] = (uint8_t)(high >> 8);
	temp_vec[5] = (uint8_t)high;
	cw.resize(0);
	cr.resize(0);
	cw.insert(cw.end(), temp_vec.begin(), temp_vec.end());
	creat_crc(cw);
	S_LC.lock();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	s_lc.write(cw);
	s_lc.flush();
	s_lc.read(cr, cw.size());
	S_LC.unlock();
}
void LiftingColumn::set_angle_mode()
{
	vector<uint8_t> cw = { 0x01, 0x06, 0x04, 0x03, 0x01, 0x10, 0x00, 0x00 };
	creat_crc(cw);
	vector<uint8_t> cr;
	S_LC.lock();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	s_lc.write(cw);
	s_lc.flush();
	s_lc.read(cr, cw.size());
	S_LC.unlock();
}
void LiftingColumn::set_effective_segments()
{
	vector<uint8_t> cw = { 0x01, 0x06, 0x04, 0x04, 0x00, 0x01, 0x00, 0x00 };
	creat_crc(cw);
	vector<uint8_t> cr;
	S_LC.lock();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	s_lc.write(cw);
	s_lc.flush();
	s_lc.read(cr, cw.size());
	S_LC.unlock();
}
void LiftingColumn::set_starting_segments()
{
	vector<uint8_t> cw = { 0x01, 0x06, 0x04, 0x08, 0x00, 0x01, 0x00, 0x00 };
	creat_crc(cw);
	vector<uint8_t> cr;
	S_LC.lock();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	s_lc.write(cw);
	s_lc.flush();
	s_lc.read(cr, cw.size());
	S_LC.unlock();
}
uint16_t Crc_Compute(const std::vector<uint8_t>& data) {
	const uint16_t polynomial = 0xA001;
	uint16_t crc = 0xFFFF;

	for (uint8_t byte : data) {
		crc ^= byte;
		for (int i = 0; i < 8; i++) {
			if (crc & 0x0001) {
				crc = (crc >> 1) ^ polynomial;
			}
			else {
				crc >>= 1;
			}
		}
	}
	return crc;
}
bool check_crc(vector<uint8_t>& cr)
{
	size_t cw_size = cr.size();
	uint16_t crc_check = Crc_Compute(vector<uint8_t>(cr.begin(), cr.begin() + (cw_size - 2)));
	uint16_t temp_1 = ((crc_check << 8) + ((crc_check >> 8)));
	uint16_t temp_2 = (((uint16_t)cr[cw_size - 2]) << 8) + ((uint16_t)cr[cw_size - 1]);
	if (temp_1 == temp_2)
	{
		return true;
	}
	return false;
}
void creat_crc(vector<uint8_t>& cw)
{
	size_t cw_size = cw.size();
	uint16_t cw_crc = Crc_Compute(vector<uint8_t>(cw.begin(), cw.begin() + cw_size - 2));
	cw[cw_size - 2] = (uint8_t)(cw_crc);
	cw[cw_size - 1] = (uint8_t)(cw_crc >> 8);
	return;
}
void liftcolumn_init()
{
	s_lc.setPort(parameters["serial_lc"]);
	//s_lc.setPort("com13");
	s_lc.setBaudrate(19200);
	s_lc.setBytesize(serial::eightbits);  // 8字节
	s_lc.setParity(serial::parity_even);  // 偶校验
	s_lc.setStopbits(serial::stopbits_one);  // 1位停止位
	s_lc.open();

	LC.set_pulse_per_circle();  //设置每圈脉冲
	LC.set_angle_mode();        //等待模式设置为：不等待，换步模式为：1，定位模式为：相对定位
	LC.set_effective_segments();//有效段数：设置为1
	LC.set_starting_segments(); //内部位置起始段号：1
	LC.set_speed(20);
	LC.LC_Enable();
}

void liftcolumn_work(void)
{
		while (true)
	{
		//立柱在运动至目标位置状态
		if (LC_state == LC_MOVING)
		{
			//std::this_thread::sleep_for(std::chrono::milliseconds(100));

			lc_now = LC.get_height();	//读取当前高度

			//若经过目标位置
			//if ((lc_target - lc_now) * (lc_target - lc_start) <= 0 || abs(lc_target - lc_now) <= 1)
			
			//if (abs(lc_target - lc_now) <= 5)
			//{
			//	up_flag = 0;
			//	LC.stop();	//立柱失能
			//	/*autoState = 0;*/
			//}
			//else
			//{
			//	std::this_thread::sleep_for(std::chrono::milliseconds(100));
			//}
		}
		//立柱在调试状态（对应机器人面板按键控制）
		else if (LC_state == LC_DEBUG)
		{
			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
			lc_now = LC.get_height();	//读取当前高度
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		else if (LC_state == LC_RESET)
		{
			lc_now = LC.get_height();	//读取当前高度
			LC_state = LC_FREE;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}
}