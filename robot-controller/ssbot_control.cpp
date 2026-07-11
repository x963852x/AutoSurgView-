#include "ssbot_control.h"
#pragma region 全局变量
int CONTROL_KEY = -1;
#pragma endregion

#pragma region 外部引用变量
extern int LIDAR_STATE;

extern double p_3;
extern double i_3;
extern motor M;
extern int LCState, LastLCState;
extern mutex lc_mt; //立柱升降控制和按键监听交替执行锁
extern condition_variable lc_var;
extern bool lc_mt_tag;
extern int zoomState, startState, safeState, autoState, beepState;
extern int laststartState;
extern LiftingColumn LC;
extern camera Cam;

extern arm_collision_model ACM;
extern arm_collision_model ACM_show;

extern cv::Mat bg;

extern std::mutex Lidar_Point;
extern vector<cv::Point2d> Lidar_point;

extern int TARGET_STATE;

extern int newSend, sendCount;
extern std::map<std::string, int> redJS;

extern vector<gloRS*> RSs;
extern int rs_end_left, rs_end_right, rs_top_left, rs_top_right;

extern bool FOCUS_STATE;

extern int ptz_speed_max;

extern int AUTO_FLAG;  //机械臂自动运动控制标志

//被动靶标误差数据记录
extern string err_path;
#pragma endregion

//键盘控制函数
void key_control()
{
	double angle_1, angle_2, angle_3, angle_4, angle_5;	//机械臂关节角度

	cv::Mat control_backgroung = cv::imread("control.jpg");	//键入图片
	cv::imshow("control", control_backgroung);	//显示图片
	while (true)
	{
		CONTROL_KEY = cv::waitKey(0);	//检测按键
		ofstream fout(err_path, ios::out | ios::app);
		switch (CONTROL_KEY)
		{
		//case '1':
		//	arm_init();
		//	break;
		

		case '2':
			arm_disable();	//机械臂失能
			break;
		case '3':
			arm_open();	//机械臂打开
			break;
		case '4':
			arm_stop();	//机械臂急停
			break;
		case '5':
			arm_close();	//机械臂收拢
			break;
		case '6':
			lidar_start();	//激光雷达开始工作
			break;
		case '7':
			lidar_stop();	//激光雷达停止工作
			break;
		case '8':	//云台归位
			Cam.setPan_with_speed(0, ptz_speed_max);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			Cam.setTilt_with_speed(0, ptz_speed_max);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			Cam.setRoll_with_speed(0, ptz_speed_max);
			break;
		case '9':
			set_focus_point();	//设置目标锁定点
			break;
		case '0':	//读取并打印各关节角度
			angle_1 = M.getAngle(FIRST_MOTOR_CAN);
			angle_2 = M.getAngle(SECOND_MOTOR_CAN);
			angle_3 = M.getAngle(THIRD_MOTOR_CAN);
			cout << "angle_1 = " << angle_1 << endl;
			cout << "angle_2 = " << angle_2 << endl;
			cout << "angle_3 = " << angle_3 << endl;
			Cam.getTilt(angle_4);
			Cam.getPan(angle_5);
			cout << "angle_4 = " << angle_4 << endl;
			cout << "angle_5 = " << angle_5 << endl;
			break;
		
		
		//case 'p':
		//	double pan_temp;
		//	double tilt_temp;

		//	Cam.getPan(pan_temp);
		//	Cam.getTilt(tilt_temp);

		//	cout << "pan = " << pan_temp << endl;
		//	cout << "tilt = " << tilt_temp << endl;
		//	break;


		case 'q':
			arm_debug(THIRD_MOTOR_CAN,DEBUG_POSITIVE);	//机械臂第三关节正转
			break;
		case 'w':
			arm_debug(THIRD_MOTOR_CAN, DEBUG_NEGATIVE);	//机械臂第三关节反转
			break;
		case 'a':
			arm_debug(SECOND_MOTOR_CAN, DEBUG_POSITIVE);	//机械臂第二关节正转
			break;
		case 's':
			arm_debug(SECOND_MOTOR_CAN, DEBUG_NEGATIVE);	//机械臂第二关节反转
			break;
		case 'z':
			arm_debug(FIRST_MOTOR_CAN, DEBUG_POSITIVE);	//机械臂第一关节正转
			break;
		case 'x':
			arm_debug(FIRST_MOTOR_CAN, DEBUG_NEGATIVE);	//机械臂第一关节反转
			break;
		case 'n':
			arm_stop();
			break;


		//case 'e':
		//	p_3 = std::max(0.0, p_3 - 0.01);
		//	M.setProportional(p_3, THIRD_MOTOR_CAN);
		//	break;
		//case 'r':
		//	p_3 += 0.01;
		//	M.setProportional(p_3, THIRD_MOTOR_CAN);
		//	break;
		//case 'd':
		//	i_3 = std::max(0.0, i_3 - 0.01);
		//	M.setIntegral(i_3, THIRD_MOTOR_CAN);
		//	break;
		//case 'f':
		//	i_3 += 0.01;
		//	M.setIntegral(i_3, THIRD_MOTOR_CAN);
		//	break;


		case 'c':
			LC.move_to_height(200);	//立柱运动至120cm处
			break;
		case 'v':
			LC.move_to_height(500);	//立柱运动至150cm处
			break;
		case 't':
			oa_start();	//设置开始遮挡规避
			break;
		case 'y':
			oa_end();	//设置停止遮挡规避
			break;
		case 'r':
			LC.reset();	//设置立柱重启
			break;

		case 'm':
			/*cout << "关节1角度为：" << PositiontoAngle(M.getPosition(1), 1) << endl;
			cout << "关节2角度为：" << PositiontoAngle(M.getPosition(2), 2) << endl;
			cout << "关节3角度为：" << PositiontoAngle(M.getPosition(3), 3) << endl;*/
			cout << "关节1电机转数为：" << (M.getPosition(1)) << endl;
			cout << "关节2电机转数为：" << (M.getPosition(2)) << endl;
			cout << "关节3电机转数为：" << (M.getPosition(3)) << endl; 
			break;
		case 'h':
			double real_Rx, real_Ry, real_Rz,order;
			cout << "请输入角度真值" << endl;
			cin >> real_Rx >> real_Ry >> real_Rz>> order;
			fout << "data:" << real_Rx << " " << real_Ry<<" " << real_Rz <<" "<< order << endl;
			fout.close();
			if (TARGET_STATE == TARGET_FREE)	//若目标拍摄位置识别功能处于空闲状态
			{
				TARGET_STATE = TARGET_GET_IMAGE_PRE;	//3D Camera开始获取图片
			}
			break;
		//case '1':
		//{
		//	cv::VideoCapture cap1;
		//	cap1.open(0);
		//	cap1.set(cv::CAP_PROP_FRAME_WIDTH, 1920);//可以
		//	cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
		//	cv::Mat frame1;
		//	cap1 >> frame1;
		//	cv::imwrite("C:\\Users\\Butel\\Desktop\\被动靶标数据测试\\测试图片\\test.jpg", frame1);
		//	break;
		//}
		/*case 'j':
			M.setMode(MODE_SPEED, THIRD_MOTOR_CAN);
			M.setSpeedModeSpeed(2, THIRD_MOTOR_CAN);*/
		/*case 'b':
			AUTO_FLAG = 1;
			break;
		case 'g':
			AUTO_FLAG = 0;
			break;*/
		default:
			break;
		}
	}
}

//键盘控制函数——仿真
void key_control_simulation()
{
	double angle_1 = 0;
	double angle_2 = 0;
	double angle_3 = 0;

	cv::Mat control_backgroung = cv::imread("control.jpg");
	cv::imshow("control", control_backgroung);
	while (true)
	{
		CONTROL_KEY = cv::waitKey(0);
		switch (CONTROL_KEY)
		{
		case '3':
			arm_open_simulation();
			break;
		case '5':
			arm_close_simulation();
			break;
		case '6':
			lidar_start();
			break;
		case '7':
			lidar_stop();
			break;
		case '8':
			TARGET_STATE = TARGET_GET_IMAGE_PRE;
			break;
		default:
			break;
		}
	}
}

//激光雷达停止
void lidar_stop()
{
	if (LIDAR_STATE == LIDAR_WORK)	//若激光雷达正在工作
	{
		LIDAR_STATE = LIDAR_STOP;	//设置激光雷达STOP
	}
}

//激光雷达开始
void lidar_start()
{
	if (LIDAR_STATE == LIDAR_FREE)	//若激光雷达处于空闲状态
	{
		LIDAR_STATE = LIDAR_START;	//设置激光雷达STOP
	}
}

//面板控制
void io_control()
{
	auto cardO = StartCardDo();
	int zoomLast = 'S', redLast = 'S';
	while (true)
	{
		unique_lock<mutex> lock(lc_mt);
		lc_var.wait(lock, [] {return !lc_mt_tag; });
		lc_mt_tag = true;
		lc_var.notify_one();

		//立柱部分
		if (LCState != LastLCState)
		{
			switch (LCState)
			{
			case LC_UP:
				if (!autoState && safeState)
				{
					LC.up();
					cout << "up" << endl;
				}
				break;
			case LC_DOWN:
				if (!autoState && safeState)
				{
					LC.down();
					cout << "down" << endl;
				}
				break;
			case LC_STOP:
				if (!autoState)
				{
					LC.stop();
					cout << "stop" << endl;
				}
				break;
			}
		}
		LastLCState = LCState;

		//画面缩放部分
		if (zoomState != zoomLast)
		{
			if (zoomState == 'U')
			{
				Cam.zoom(1);
			}
			else if (zoomState == 'D')
			{
				Cam.zoom(-1);
			}
			else
			{
				Cam.zoom(0);
			}
			zoomLast = zoomState;
		}

		//视频录制部分
		if (startState != laststartState)
		{
			while (newSend != 0)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}

			redJS.clear();
			redJS["cmdID"] = 2002;

			if (startState)
			{
				redJS["opType"] = 1;
			}
			else
			{
				redJS["opType"] = 2;
			}

			redJS["seqnum"] = sendCount++;
			newSend = NEWSEND_READY;
			laststartState = startState;
		}


		//蜂鸣器部分
		if (beepState != 0)
		{
			while (beepState != 0)
			{
				//CardWriteBite(cardO, 4, 1);
				std::this_thread::sleep_for(std::chrono::milliseconds(200));
				//CardWriteBite(cardO, 4, 0);
				beepState--;
			}
		}
		
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	EndCardDo(cardO);
}

//开启遮挡规避
void oa_start()
{
	RSs[rs_end_left]->set_work_state(0);	//将3D camera曝光参数设为目标锁定状态相应值
	RSs[rs_end_right]->set_work_state(0);	//将3D camera曝光参数设为目标锁定状态相应值
	FOCUS_STATE = true;	//设置目标锁定状态true
	cout << "set FOCUS_STATE true" << endl;
}

//停止遮挡规避
void oa_end()
{
	FOCUS_STATE = false;	//设置目标锁定状态false
	RSs[rs_end_left]->set_work_state(0);	//将3D camera曝光参数设为目标锁定状态相应值
	RSs[rs_end_right]->set_work_state(1);	//将3D camera曝光参数设为目标拍摄位置识别状态相应值
	cout << "set FOCUS_STATE false" << endl;
}