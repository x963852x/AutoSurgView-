/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of motor

**************************************************************************/

#include "main.h"

extern std::map<std::string, std::string> parameters;

//CAN通信部件是否连通
extern bool open_can_success;

//运动部件
extern motor M;
extern LiftingColumn LC;
extern camera Cam;

//运动部件状态参数
extern int ARM_STATE;
extern int lc_now;
extern std::vector<double> joint_now;

//传感器
extern ILidarDriver* lidar1;	//激光雷达——上
extern ILidarDriver* lidar2;	//激光雷达——下

//日志文件
std::ofstream f_log;


int main()
{
	const char* log_env = std::getenv("AUTOSURGVIEW_LOG_DIR");
	std::filesystem::path log_dir = log_env ? log_env : "logs";
	std::filesystem::create_directories(log_dir);
	f_log = std::ofstream(log_dir / (get_time() + ".txt"));

	read_parameters();

	//确认CAN通信部件连接成功
	if (open_can_success)
	{
		camera_init();	//云台相机电机初始化
		liftcolumn_init(); //立柱初始化
		Lidar_init();
		udp_init();	//UDP通信初始化
		arm_init();	//机械臂通信初始化
		rrt_init();	//RRT规划器参数通信初始化
		robot_init();	//机器人运动部件参数初始化

		if (atoi(parameters["rs_fun"].c_str()))
		{
			rs_camera_init();	//3D camera初始化
		}
		else
		{
			udp_send_debug_info(INIT_SUCCESS_WITH_RS_FUN_DISABLED);
		}

		SetConsoleCtrlHandler(HandlerRoutine, TRUE);	//绑定关机程序，在程序结束后执行


		ARM_STATE = ARM_FREE;	//设置机械臂状态为空闲

		thread t1(motor_work);	//机械臂关节电机工作线程
		thread t2(camera_work);	//机械臂云台电机工作线程
		//thread t1(motor_work_simulation);	//机械臂关节电机仿真测试线程，实际工作时注释，不执行
		thread t3(liftcolumn_work);	//升降立柱工作线程

		thread t4;
		if (atoi(parameters["lidar_fun"].c_str()))
		{
			t4 = thread(lidar_scan);	//激光雷达工作线程
		}

		thread t5(key_control);	//键盘控制线程
		//thread t5(key_control_simulation);
		thread t6(joystick1_listen);	//手柄1控制线程
		thread t7(joystick2_listen);	//手柄2控制线程
		thread t8(button_listen);	//机器人面板按键监听线程
		thread t9(io_control);	//机器人面板按键控制线程
		thread t10(visualization);	//可视化线程
		thread t11(udp_control);	//UDP接受与控制线程
		thread t12(udp_refresh);	//UDP发送线程

		thread t13;
		thread t15;

		thread t16(OP_test_thread);

		if (atoi(parameters["rs_fun"].c_str()))
		{
			t13 = thread(rs_camera_work);	//3D camera工作线程，用于执行目标拍摄位置识别任务
			t15 = thread(occlusion_avoidance);	//遮挡规避线程

		}
		thread t14(heart_beat);	//机器人心跳通信线程

		t1.join();
		t2.join();
		t3.join();

		if (atoi(parameters["lidar_fun"].c_str()))
		{
			t4.join();
		}

		t5.join();
		t6.join();
		t7.join();
		t8.join();
		t9.join();
		t10.join();
		t11.join();
		t12.join();

		if (atoi(parameters["rs_fun"].c_str()))
		{
			t13.join();
			t15.join();
		}

		t14.join();
		t16.join();

		arm_disable();	//机械臂关节电机失能
	}

	
	return 0;
}

void robot_init()
{
	cout << "robot_init start" << endl;

	lc_now = LC.get_height();
	//double an1 = (M.getPosition(1), 1);
	//double an2 = (M.getPosition(2), 2);
	//double an3 = (M.getPosition(3), 3);
	//cout << "关节1角度为：" << an1 << endl;
	//cout << "关节2角度为：" << an2 << endl;
	//cout << "关节3角度为：" << an3 << endl;
	joint_now[0] = M.getAngle(FIRST_MOTOR_CAN);
	joint_now[1] = M.getAngle(SECOND_MOTOR_CAN);
	joint_now[2] = M.getAngle(THIRD_MOTOR_CAN);

	double tilt, pan, roll;
	Cam.getPan(pan);
	Cam.getTilt(tilt);
	Cam.getRoll(roll);

	joint_now[3] = tilt;
	joint_now[4] = pan;
	joint_now[5] = roll;

	cout << "robot_init succesed" << endl;
}
void Lidar_init()
{
	lidar1 = *createLidarDriver();
	lidar2 = *createLidarDriver();
	if (!lidar1 || !lidar2) {
		cout << "Lidar_init Fail" << endl;
	}

}
//机械臂初始化
void arm_init()
{
	while (!arm_set_param())
	{
		udp_send_debug_info(DEBUG_ERROR_STOP_BUTTON);
		f_log << "[" << get_time() << "] " << "电机初始化失败" << endl;
	}

	joint_now[0] = M.getAngle(FIRST_MOTOR_CAN);
	joint_now[1] = M.getAngle(SECOND_MOTOR_CAN);
	joint_now[2] = M.getAngle(THIRD_MOTOR_CAN);

	if (!arm_position_test())
	{

		f_log << "[" << get_time() << "] " << "电机处于非法位置" << endl;
		udp_send_debug_info(DEBUG_ERROR_OMPL_OUT_RANGE);

		while (!arm_free()) {};
		f_log << "[" << get_time() << "] " << "电机已释放" << endl;
		udp_send_debug_info(DEBUG_ERROR_OMPL_OUT_RANGE_FREE);

		while (!arm_position_test())
		{
			joint_now[0] = M.getAngle(FIRST_MOTOR_CAN);
			joint_now[1] = M.getAngle(SECOND_MOTOR_CAN);
			joint_now[2] = M.getAngle(THIRD_MOTOR_CAN);

			Sleep(500);
		}

		f_log << "[" << get_time() << "] " << "电机已调整至合法位置" << endl;
		udp_send_debug_info(DEBUG_ERROR_OMPL_OUT_RANGE_LEGAL);

		while (!arm_init_mode()) {};
	}

	f_log << "[" << get_time() << "] " << "电机初始化成功" << endl;
	udp_send_debug_info(DEBUG_MOTOR_INIT_SUCCESS);
	/*M.disable(FIRST_MOTOR_CAN);
	M.disable(SECOND_MOTOR_CAN);
	M.disable(THIRD_MOTOR_CAN);*/

	cout << "arm init successed" << endl;
}


BOOL WINAPI HandlerRoutine(DWORD dwCtrlType)
{
	if (CTRL_CLOSE_EVENT == dwCtrlType)
	{
		lidar1->stop();
		lidar2->stop();
		arm_disable();
		LC.stop();	//立柱失能
	}
	return true;
}
