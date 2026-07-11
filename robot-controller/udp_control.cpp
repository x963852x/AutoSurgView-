#include "udp_control.h"

#pragma region 外部引用变量
extern std::map<std::string, std::string> parameters;

extern SOCKET RecvSocket;
extern SOCKET SendSocket;
extern SOCKET SendSocket2;
extern SOCKET SendSocket_debug_info;
extern SOCKET SendSocketFlag;
extern sockaddr_in RecvAddr;//服务器地址
extern SOCKADDR_IN SendAddr;//服务器地址
extern SOCKADDR_IN SendAddr2;//服务器地址
extern SOCKADDR_IN SendAddr_debug_info;//服务器地址
extern SOCKADDR_IN SendAddrFlag;//服务器地址

//extern char RecvBuf[];
extern  int BufLen;//缓冲区大小
extern int SenderAddrSize;
extern sockaddr_in SenderAddr;

extern camera Cam;
extern int CAMERA_STATE;
extern motor M;
extern int ARM_STATE;
extern LiftingColumn LC;
extern int LC_state;
extern int autoState;
extern int lc_now;
extern int lc_zero;
extern int lc_target;
extern int ARM_OBSTACLE_STOP;

extern double speed_max_debug;
extern std::vector<double> joint_now;

extern int TARGET_STATE;

extern int newSend, sendCount;
extern std::map<std::string, int> redJS;

extern int beepState;

extern struct GYRO_DATA gyro_data;

extern std::ofstream f_log;

extern bool FOCUS_STATE;
extern double oa_height;
extern int OP_STATUS;
extern int VISUALSERVO_STATE;

//关节电机角度收拢，单位°
extern double FIRST_MOTOR_CLOSE;		//关节电机1角度收拢
extern double SECOND_MOTOR_CLOSE;	//关节电机2角度收拢
extern double THIRD_MOTOR_CLOSE;	//关节电机3角度收拢

extern double FIRST_MOTOR_OPEN;		//关节电机1角度展开
extern double SECOND_MOTOR_OPEN;	//关节电机2角度展开
extern double THIRD_MOTOR_OPEN;	   //关节电机3角度展开


//轨迹插值速度限制
extern double SPEED_MAX;		//多关节运动速度 °/s
extern double SPEED_MAX_DEBUG_HIGH;	//单关节调试运动，高速情况 °/s，应用于机器人触摸屏控制
extern double SPEED_MAX_DEBUG_LOW;	//单关节调试运动，高速情况 °/s，应用于机器人触摸屏控制

//下位机重启标志位
extern char ResetFlag;
extern int DETECT_STATE;
#pragma endregion

#pragma region 全局变量
int markState = 0;

int udp_ptz_speed = 10;
#pragma endregion

//UDP控制函数，接收来自上位机和红云主机的信息，控制机器人动作
void udp_control()
{
	while (1)
	{
		char RecvBuf[100] = { 0 };//发送数据的缓冲区

		//监听端口信息（阻塞式）
		recvfrom(RecvSocket, RecvBuf, BufLen, 0, (SOCKADDR*)&SenderAddr, &SenderAddrSize);

		if (RecvBuf[0] != '{')	//非红云主机信息
		{
			printf("SenderIP  :%s\n", inet_ntoa(SenderAddr.sin_addr));
			printf("SenderData:%s\n", RecvBuf);
		}

		//靶标识别
		if (RecvBuf[0] == '1')
		{
			f_log << "[" << get_time() << "] " << "机器人收到靶标识别命令" << endl;

			if (RecvBuf[1] == 'c')	//校验
			{
				//遮挡规避停止
				if (FOCUS_STATE)
				{
					oa_end();
				}
				//运动部件停止
				if (ARM_STATE != ARM_FREE)
				{
					arm_stop();
				}
				if (CAMERA_STATE != CAMERA_FREE)
				{
					Cam.stop();
				}
				if (LC_state != LC_FREE)
				{
					LC.stop();
				}

				if (TARGET_STATE == TARGET_FREE)	//若目标拍摄位置识别功能处于空闲状态
				{
					TARGET_STATE = TARGET_GET_IMAGE_PRE;	//3D Camera开始获取图片
					f_log << "[" << get_time() << "] " << "3D Camera开始获取图片" << endl;
				}
			}
			else
			{
				f_log << "[" << get_time() << "] " << "欧拉角数据校验失败" << endl;
				cout << "校验失败" << endl;
			}
		}
		//机械臂回收
		else if (RecvBuf[0] == '2')
		{
			autoState = 1;
			Cam.setWrist(0, 0, 0);	//云台归位
			arm_close();	//机械臂收拢

			while (ARM_STATE != ARM_FREE)	//等待机械臂运动停止
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}

			//判断机械臂运动是否到位
			if (std::abs(M.getAngle(FIRST_MOTOR_CAN) - (FIRST_MOTOR_CLOSE)) < 5 &&
				std::abs(M.getAngle(SECOND_MOTOR_CAN) - (SECOND_MOTOR_CLOSE)) < 5 &&
				std::abs(M.getAngle(THIRD_MOTOR_CAN) - (THIRD_MOTOR_CLOSE)) < 5)
			{
				LC.move_to_height(10);	//立柱运动
				lidar_stop();	//激光雷达停止
			}

			while (std::abs(lc_now - 10) > 2 && ARM_OBSTACLE_STOP == 0)	//等待立柱停止
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(200));
			}

			LC.stop();

			if (std::abs(lc_now - 10) <= 4 || ARM_OBSTACLE_STOP == 1)	//判断立柱是否运动到位
			{
				autoState = 0;
				ARM_OBSTACLE_STOP = 0;
			}

			udp_send_debug_info(DEBUG_INFO_EXPAND_RECYVLE_SUCCESS);
		}
		////机械臂展开
		else if (RecvBuf[0] == 'z')
		{
			lidar_start();

			int height = atoi(RecvBuf + 1);
			cout << "设定身高为：" << height << "cm" << endl;
			height = height * 10;
			height -= lc_zero;                                //1253
			cout << "立柱换算高度为：" << height << "mm" << endl;

			autoState = 1;
			LC.move_to_height(height);	//立柱运动

			//while (LC_state != LC_FREE || up_flag!=0)	//等待立柱停止
			while (std::abs(lc_now - height) > 2)	//等待立柱停止
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(200));
			}

			LC.stop();

			if (std::abs(lc_now - height) <= 4)	//判断立柱是否运动到位
			{
				arm_open();	//机械臂打开
				Cam.setWrist(0, 0, 0);	//云台归位
			}

			while (ARM_STATE != ARM_FREE)	//等待机械臂运动停止
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}

			if ((std::abs(M.getAngle(FIRST_MOTOR_CAN) - (FIRST_MOTOR_OPEN)) < 5 &&
				std::abs(M.getAngle(SECOND_MOTOR_CAN) - (SECOND_MOTOR_OPEN)) < 5 &&
				std::abs(M.getAngle(THIRD_MOTOR_CAN) - (THIRD_MOTOR_OPEN)) < 5) || ARM_OBSTACLE_STOP == 1)
			{
				autoState = 0;
				ARM_OBSTACLE_STOP = 0;
			}
			udp_send_debug_info(DEBUG_INFO_EXPAND_RECYVLE_SUCCESS);
		}
		//靶标控制录制开始，需与红云主机通信
		else if (RecvBuf[0] == '3')
		{
			while (newSend != 0)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
			redJS.clear();
			redJS["cmdID"] = 2000;
			redJS["opType"] = 1;
			redJS["seqnum"] = sendCount++;
			newSend = NEWSEND_READY;

		}
		//靶标录制结束，需与红云主机通信
		else if (RecvBuf[0] == '4')
		{
			while (newSend != 0)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
			redJS.clear();
			redJS["cmdID"] = 2002;
			if (markState == 0)
			{
				redJS["opType"] = 3;
				markState = 1;
			}
			else
			{
				redJS["opType"] = 4;
				markState = 0;
			}
			redJS["seqnum"] = sendCount++;
			newSend = NEWSEND_READY;
		}

		//视野放大
		else if (RecvBuf[0] == '5')
		{
			Cam.zoom(-1);
		}
		//视野缩小
		else if (RecvBuf[0] == '6')
		{
			Cam.zoom(1);
		}
		//缩放停止
		else if (RecvBuf[0] == '7')
		{
			Cam.zoom(0);
		}

		//机械臂单关节调试
		else if (RecvBuf[0] == 'a')
		{
			switch (RecvBuf[1])
			{
			case '1':
				for (int i = 0; i < 5; i++)
				{
					if (M.setMode(MODE_POSITION, FIRST_MOTOR_CAN))
					{
						break;
					}
				}
				//M.enable(FIRST_MOTOR_CAN);
				break;
			case '2':
				for (int i = 0; i < 5; i++)
				{
					if (M.setMode(MODE_CURRENT, FIRST_MOTOR_CAN))
					{
						break;
					}
				}
				//M.disable(FIRST_MOTOR_CAN);
				break;
			case '3':
				speed_max_debug = SPEED_MAX_DEBUG_HIGH;
				break;
			case '4':
				speed_max_debug = SPEED_MAX_DEBUG_LOW;
				break;
			case '5':
				arm_debug(FIRST_MOTOR_CAN, DEBUG_NEGATIVE);
				break;
			case '6':
				arm_debug(FIRST_MOTOR_CAN, DEBUG_POSITIVE);
				break;
			case '7':
				arm_stop();
				break;
			}
		}
		//触屏控制机械臂末端运动
		else if (RecvBuf[0] == 'e')
		{
		/*if (RecvBuf[1] != 'e')
		{
			string str = RecvBuf;
			int index = str.find("e", 1);
			string x, y;
			y = str.substr(1, index - 1);
			x = str.substr(index + 1, 100);
			int int_x, int_y;
			int_x = stoi(x);
			int_y = stoi(y);
			double direction_x, direction_y, norm;
			norm = sqrt(int_x * int_x + int_y * int_y);
			direction_x = int_x / norm;
			direction_y = int_y / norm;
			arm_touch_move(direction_x, direction_y);
		}
		else
		{
			arm_stop();
		}*/


		//根据手柄1状态控制机械臂做出相应动作
		switch (RecvBuf[1])
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
		case 'S':
			arm_stop();
			break;
		}
		}
		else if (RecvBuf[0] == 's')
		{
			switch (RecvBuf[1])
			{
			case '1':
				for (int i = 0; i < 5; i++)
				{
					if (M.setMode(MODE_POSITION, SECOND_MOTOR_CAN))
					{
						break;
					}
				}
				//M.enable(SECOND_MOTOR_CAN);
				break;
			case '2':
				for (int i = 0; i < 5; i++)
				{
					if (M.setMode(MODE_CURRENT, SECOND_MOTOR_CAN))
					{
						break;
					}
				}
				/*M.disable(SECOND_MOTOR_CAN);*/
				break;
			case '3':
				speed_max_debug = SPEED_MAX_DEBUG_HIGH;
				break;
			case '4':
				speed_max_debug = SPEED_MAX_DEBUG_LOW;
				break;
			case '5':
				arm_debug(SECOND_MOTOR_CAN, DEBUG_NEGATIVE);
				break;
			case '6':
				arm_debug(SECOND_MOTOR_CAN, DEBUG_POSITIVE);
				break;
			case '7':
				arm_stop();
				break;
			}
		}
		else if (RecvBuf[0] == 'd')
		{
			switch (RecvBuf[1])
			{
			case '1':
				for (int i = 0; i < 5; i++)
				{
					if (M.setMode(MODE_POSITION, THIRD_MOTOR_CAN))
					{
						break;
					}
				}
				//M.enable(THIRD_MOTOR_CAN);
				break;
			case '2':
				for (int i = 0; i < 5; i++)
				{
					if (M.setMode(MODE_CURRENT, THIRD_MOTOR_CAN))
					{
						break;
					}
				}
				//M.disable(THIRD_MOTOR_CAN);
				break;
			case '3':
				speed_max_debug = SPEED_MAX_DEBUG_HIGH;
				break;
			case '4':
				speed_max_debug = SPEED_MAX_DEBUG_LOW;
				break;
			case '5':
				arm_debug(THIRD_MOTOR_CAN, DEBUG_NEGATIVE);
				break;
			case '6':
				arm_debug(THIRD_MOTOR_CAN, DEBUG_POSITIVE);
				break;
			case '7':
				arm_stop();
				break;
			}
		}
		else if (RecvBuf[0] == '{')	//红云主机信息
		{
			printf("wow!\n");
			//std::cout<< RecvBuf <<std::endl;
			std::string rawJson(RecvBuf);
			//rawJson += '\0';
			const auto rawJsonLength = static_cast<int>(rawJson.length());
			constexpr bool shouldUseOldWay = false;
			JSONCPP_STRING err;
			Json::Value root;
			Json::CharReaderBuilder builder;
			const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
			if (!reader->parse(rawJson.c_str(), rawJson.c_str() + rawJsonLength, &root, &err))
			{
				std::cout << "error" << std::endl;
			}
			int cache = root["cmdID"].asInt();
			if (cache == 1000)
			{
				switch (root["opType"].asInt())
				{
				case 1:
					std::cout << "zoom in" << std::endl;
					Cam.zoom(1);
					break;
				case 2:
					std::cout << "zoom out" << std::endl;
					Cam.zoom(-1);
					break;
				case 3:
					std::cout << "zoom stop" << std::endl;
					Cam.zoom(0);
					break;
				}
			}
		}
		else if (RecvBuf[0] == 'c')	//相机光学信息
		{
			switch (RecvBuf[1])
			{
			case 'z':
				if (RecvBuf[2] == '0')
					Cam.zoom(0);
				else if (RecvBuf[2] == '1')
					Cam.zoom(1);
				else if (RecvBuf[2] == '2')
					Cam.zoom(-1);
				break;
			case 'f':
				if (RecvBuf[2] == '0')
					Cam.focus(0);
				else if (RecvBuf[2] == '1')
					Cam.focus(1);
				else if (RecvBuf[2] == '2')
					Cam.focus(2);
				break;
			case 'i':
				if (RecvBuf[2] == '0')
					Cam.exposure(0);
				else if (RecvBuf[2] == '1')
					Cam.exposure(1);
				else if (RecvBuf[2] == '2')
					Cam.exposure(2);
				break;
			case 'l':
				if (RecvBuf[2] == '1')
					Cam.light(1);
				else if (RecvBuf[2] == '2')
					Cam.light(2);
				break;
			case 'd':
				if (DETECT_STATE == 0)
					DETECT_STATE = 1;
				break;
			}
		}
		else if (RecvBuf[0] == 'f')	//目标锁定信息
		{
			if (RecvBuf[1] == '1')
			{
				oa_start();	//设置开始遮挡规避
			}
			else if (RecvBuf[1] == '0')
			{
				oa_end();	//设置停止遮挡规避
			}
			else if (RecvBuf[1] == '2')
			{
				double oa_height_temp = atof(RecvBuf + 2) / 100;
				if (oa_height != oa_height_temp)
				{
					oa_height = oa_height_temp;
				}

				set_focus_point();	//设置目标锁定点
			}
			else if (RecvBuf[1] == '3')
			{
				OP_STATUS = 1;
			}
			else if (RecvBuf[1] == '4')
			{
				OP_STATUS = 0;
			}
			else if (RecvBuf[1] == '5')
			{
				VISUALSERVO_STATE = 8;
			}
		}
		//术野云台电机触屏控制
		else if (RecvBuf[0] = 't')
		{
		switch (RecvBuf[1])
		{
		case '1':
			Cam.move(CAMERA_U, udp_ptz_speed);
			break;
		case '2':
			Cam.move(CAMERA_D, udp_ptz_speed);
			break;
		case '3':
			Cam.move(CAMERA_R, udp_ptz_speed);
			break;
		case '4':
			Cam.move(CAMERA_L, udp_ptz_speed);
			break;
		case '5':
			Cam.move(CAMERA_P, udp_ptz_speed);
			break;
		case '6':
			Cam.move(CAMERA_N, udp_ptz_speed);
			break;
		case '7':
			Cam.stop();
			break;
		default:
			Cam.stop();
			break;
		}
		}
	}
	printf("finished receiving,closing socket..\n");
	closesocket(RecvSocket);
	//释放资源，退出
	printf("Exiting.\n");
	WSACleanup();
}

//UDP更新函数，向上位机发送机器人运动部件参数，向红云主机发送信息
void udp_refresh()
{
	//机器人运动部件参数数据包
	//int16_t joint_lizhu = (int16_t)((lc_now - 100) * 10);
	int16_t joint_lizhu = (int16_t)(lc_now);
	int16_t joint_1 = (int16_t)(joint_now[0] * 100);
	int16_t joint_2 = (int16_t)(joint_now[1] * 100);
	int16_t joint_3 = (int16_t)(joint_now[2] * 100);
	int16_t joint_4 = (int16_t)(joint_now[3] * 100);
	int16_t joint_5 = (int16_t)(joint_now[4] * 100);

	char buf[12];

	buf[0] = *(uint8_t*)(&joint_lizhu);
	buf[1] = *((uint8_t*)(&joint_lizhu) + 1);
	buf[2] = *(uint8_t*)(&joint_1);
	buf[3] = *((uint8_t*)(&joint_1) + 1);
	buf[4] = *(uint8_t*)(&joint_2);
	buf[5] = *((uint8_t*)(&joint_2) + 1);
	buf[6] = *(uint8_t*)(&joint_3);
	buf[7] = *((uint8_t*)(&joint_3) + 1);
	buf[8] = *(uint8_t*)(&joint_4);
	buf[9] = *((uint8_t*)(&joint_4) + 1);
	buf[10] = *(uint8_t*)(&joint_5);
	buf[11] = *((uint8_t*)(&joint_5) + 1);

	sendto(SendSocket, buf, 12, 0, (sockaddr*)&SendAddr, 50);

	while (1)
	{
		if (ARM_STATE == ARM_FOLLOW_TRAJ || LC_state != LC_FREE || CAMERA_STATE != CAMERA_FREE)
		{
			//机器人运动过程中，向上位机发送机器人运动部件数据
			//机器人运动部件参数数据包
			//joint_lizhu = (lc_now - 100) * 10;
			joint_lizhu = lc_now;
			joint_1 = (int16_t)(joint_now[0] * 100);
			joint_2 = (int16_t)(joint_now[1] * 100);
			joint_3 = (int16_t)(joint_now[2] * 100);
			joint_4 = (int16_t)(joint_now[3] * 100);
			joint_5 = (int16_t)(joint_now[4] * 100);

			buf[0] = *(uint8_t*)(&joint_lizhu);
			buf[1] = *((uint8_t*)(&joint_lizhu) + 1);
			buf[2] = *(uint8_t*)(&joint_1);
			buf[3] = *((uint8_t*)(&joint_1) + 1);
			buf[4] = *(uint8_t*)(&joint_2);
			buf[5] = *((uint8_t*)(&joint_2) + 1);
			buf[6] = *(uint8_t*)(&joint_3);
			buf[7] = *((uint8_t*)(&joint_3) + 1);
			buf[8] = *(uint8_t*)(&joint_4);
			buf[9] = *((uint8_t*)(&joint_4) + 1);
			buf[10] = *(uint8_t*)(&joint_5);
			buf[11] = *((uint8_t*)(&joint_5) + 1);

			sendto(SendSocket, buf, 12, 0, (sockaddr*)&SendAddr, 50);
		}

		if (newSend == 1)	//若给红云主机发送的消息已准备好
		{
			//初始化socket（通过实验发现，必须每次创建并初始化，随后绑定发送端口，发送完毕后关闭）
			//红云主机地址
			SendSocket2 = socket(AF_INET, SOCK_DGRAM, 0);
			SendAddr2.sin_family = AF_INET;
			SendAddr2.sin_addr.S_un.S_addr = inet_addr("10.42.0.161");
			SendAddr2.sin_port = htons(6060);

			//本机地址
			SOCKADDR_IN   addrMe2;
			addrMe2.sin_family = AF_INET;
			addrMe2.sin_port = ntohs(6060);
			addrMe2.sin_addr.S_un.S_addr = inet_addr("10.42.0.155");

			//绑定
			bind(SendSocket2, (sockaddr*)&addrMe2, sizeof(addrMe2));

			Json::Value root2;
			for (auto node : redJS)
			{
				root2[node.first] = node.second;
			}
			Json::StreamWriterBuilder builder;
			const std::string json_file = Json::writeString(builder, root2);
			const char* SendBuf = json_file.c_str();
			int BufLen = strlen(SendBuf);

			sendto(SendSocket2, SendBuf, BufLen, 0, (SOCKADDR*)&SendAddr2, sizeof(SendAddr2));

			closesocket(SendSocket2);	//关闭

			newSend = NEWSEND_FREE;	//发送状态设为空闲
		}

		//向重启下位机软程序发送标志
		if (ResetFlag == '1')
		{
			sendto(SendSocketFlag, &ResetFlag, 1, 0, (sockaddr*)&SendAddrFlag, 50);
			ResetFlag = '0';
		}
		_sleep(2000);
	}
}

std::mutex udp_send_debug_info_mutex;
//UDP发送函数，向上位机发送机器人调试信息
void udp_send_debug_info(int16_t info_index)
{
	char buf[4];

	buf[0] = 0x0A;
	buf[1] = 0x0B;
	buf[2] = info_index;
	buf[3] = buf[0] + buf[1] + buf[2];

	udp_send_debug_info_mutex.lock();
	sendto(SendSocket_debug_info, buf, 4, 0, (sockaddr*)&SendAddr_debug_info, 50);
	udp_send_debug_info_mutex.unlock();
}

//机器人通信心跳功能函数，内含两个子线程
void heart_beat()
{
	thread t_butel(heart_beat_butel);
	thread t_monitor(heart_beat_monitor);

	t_butel.join();
	t_monitor.join();
}

//机器人与5G主机心跳通信
void heart_beat_butel()
{
	while (true)
	{
		while (newSend != 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		redJS.clear();
		redJS["cmdID"] = 2004;
		redJS["opType"] = 1;
		redJS["seqnum"] = sendCount++;
		newSend = NEWSEND_READY;	//消息准备完成，即将发送
		std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	}
}

//机器人与上位机心跳通信
void heart_beat_monitor()
{
	bool rs_flag = false;
	if (atoi(parameters["rs_fun"].c_str()))
	{
		rs_flag = true;
	}

	while (true)
	{
		if (rs_flag)
		{
			udp_send_debug_info(DEBUG_INFO_HEARTBEAT_UI);
		}
		else
		{
			udp_send_debug_info(DEBUG_INFO_HEARTBEAT_UI_WITHOUT_RS);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	}
}