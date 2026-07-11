/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of motor_work

**************************************************************************/

#include "motor_work.h"
#include <fstream>

#pragma region KDL相关
using KDL::Segment;
using KDL::Joint;
using KDL::ChainFkSolverPos_recursive;
using KDL::ChainIkSolverVel_pinv;
using KDL::ChainIkSolverPos_NR_JL;
using KDL::Frame;
using KDL::Vector;
using KDL::JntArray;
#pragma endregion

#pragma region 全局变量
std::mutex TAR;	//机械臂关节目标角度线程锁

motor M;	//关节电机对象M

std::vector<traj_point>traj;	//机械臂粗轨迹（带时间戳的路径点）
std::vector<traj_point>traj_interpolation;	//机械臂插值后轨迹（可以不带时间戳）

int ARM_STATE = -1; 	//机械臂状态

int MOVE_TYPE = 0;	//机械臂运动类型

bool FOCUS_STATE = false;	//机械臂是否目标锁定

double speed_max_debug = 6;	//机械臂调试最大速度
double a_max = 2;	//机械臂关节电机加速度最大值，单位°/s^2

double line_step = 5;	//笛卡尔空间直线离散点步长 单位mm

int AUTO_FLAG = 0;  //机械臂自动运动控制标志
#pragma region 路径规划器相关变量
RRT_test::RRT_Space  space_move;	//路径规划空间

//路径规划器指针
//静态路径规划器指针
//RRT_test::RRT* rrt_solver_ptr = nullptr;
//RRT_test::RRT_Star* rrt_solver_ptr = nullptr;
//RRT_test::RRT_Connect* rrt_solver_ptr = nullptr;
//RRT_test::RRT_Connect_Star* rrt_solver_ptr = nullptr;
//RRT_test::Q_RRT_Star* rrt_solver_ptr = nullptr;
//动态路径规划器指针
//RRT_test::D_RRT* rrt_solver_ptr = nullptr;
//RRT_test::D_RRT_Star* rrt_solver_ptr = nullptr;
//RRT_test::D_RRT_Connect* rrt_solver_ptr = nullptr;
//RRT_test::D_RRT_Connect_Star* rrt_solver_ptr = nullptr;
//RRT_test::MP_RRT* rrt_solver_ptr = nullptr;
//RRT_test::MP_RRT_Star* rrt_solver_ptr = nullptr;
//RRT_test::MP_RRT_Connect* rrt_solver_ptr = nullptr;
RRT_test::MP_RRT_Connect_Star* rrt_solver_ptr = nullptr;

int rrt_solver_state = RRT_SOLVER_STATE_OLD_PATH;	//规划器返回路径类型，应用于动态路径规划
#pragma endregion


std::vector<Eigen::VectorXd> path;

std::mutex rrt_solver_mutex;
std::mutex traj_mutex;


#pragma endregion

#pragma region 外部引用变量
extern bool open_can_success;
extern std::vector<double> joint_now, joint_tar;
extern arm_collision_model ACM;
extern arm_collision_model ACM_show;
extern std::mutex Lidar_Point;
extern std::vector<cv::Point2d> Lidar_point;
extern std::mutex QTM_mutex;
extern Q_Tree_Map QTM;
extern std::vector<cv::RotatedRect> obstacles;
extern std::vector<cv::RotatedRect> obstacles_new;
extern Eigen::Vector3d focus_point;
extern int VISUALSERVO_STATE;
extern int VISUALSERVO_TRAJ_STATE;
extern int OPTRAJFOLLOW;
#pragma endregion

extern std::map<std::string, std::string> parameters;
extern int autoState;

#pragma region 外部引用变量源自宏定义变量
//关节电机角度偏置，单位°
extern double FIRST_MOTOR_OFFSET;		//关节电机1角度偏置
extern double SECOND_MOTOR_OFFSET;	//关节电机2角度偏置
extern double THIRD_MOTOR_OFFSET;	//关节电机3角度偏置

//关节电机角度展开，单位°
extern double FIRST_MOTOR_OPEN;		//关节电机1角度展开
extern double SECOND_MOTOR_OPEN;	//关节电机2角度展开
extern double THIRD_MOTOR_OPEN;	//关节电机3角度展开

//关节电机角度收拢，单位°
extern double FIRST_MOTOR_CLOSE;		//关节电机1角度收拢
extern double SECOND_MOTOR_CLOSE;	//关节电机2角度收拢
extern double THIRD_MOTOR_CLOSE;	//关节电机3角度收拢
//轨迹插值速度限制
extern double SPEED_MAX;		//多关节运动速度 °/s
extern double SPEED_MAX_DEBUG_HIGH;	//单关节调试运动，高速情况 °/s，应用于机器人触摸屏控制
extern double SPEED_MAX_DEBUG_LOW;	//单关节调试运动，高速情况 °/s，应用于机器人触摸屏控制
#pragma endregion

#pragma region 机械臂顶层控制调用函数
//机械臂设置参数（初始化设置参数）
bool arm_set_param()
{
	FIRST_MOTOR_OFFSET = std::atof(parameters["FIRST_MOTOR_OFFSET"].c_str());
	SECOND_MOTOR_OFFSET = std::atof(parameters["SECOND_MOTOR_OFFSET"].c_str());
	THIRD_MOTOR_OFFSET = std::atof(parameters["THIRD_MOTOR_OFFSET"].c_str());
	FIRST_MOTOR_OPEN = std::atof(parameters["FIRST_MOTOR_OPEN"].c_str());
	SECOND_MOTOR_OPEN = std::atof(parameters["SECOND_MOTOR_OPEN"].c_str());
	THIRD_MOTOR_OPEN = std::atof(parameters["THIRD_MOTOR_OPEN"].c_str());
	FIRST_MOTOR_CLOSE = std::atof(parameters["FIRST_MOTOR_CLOSE"].c_str());
	SECOND_MOTOR_CLOSE = std::atof(parameters["SECOND_MOTOR_CLOSE"].c_str());
	THIRD_MOTOR_CLOSE = std::atof(parameters["THIRD_MOTOR_CLOSE"].c_str());

	SPEED_MAX = std::atof(parameters["SPEED_MAX"].c_str());
	SPEED_MAX_DEBUG_HIGH = std::atof(parameters["SPEED_MAX_DEBUG_HIGH"].c_str());
	SPEED_MAX_DEBUG_LOW = std::atof(parameters["SPEED_MAX_DEBUG_LOW"].c_str());

	speed_max_debug = SPEED_MAX_DEBUG_LOW;

	if (!M.set_param(FIRST_MOTOR_CAN, MODE_POSITION, 5))
	{
		return false;
	}
	if (!M.set_param(SECOND_MOTOR_CAN, MODE_POSITION, 5))
	{
		return false;
	}
	if (!M.set_param(THIRD_MOTOR_CAN, MODE_POSITION, 5))
	{
		return false;
	}

	return true;
}

//机械臂释放（设置为电流模式，电流为0，可手动调整机械臂）
//@return true if arm free success
bool arm_free()
{
	if (!M.setMode(MODE_CURRENT, FIRST_MOTOR_CAN))
	{
		return false;
	}
	else
	{
		cout << "Motor " << FIRST_MOTOR_CAN << " free successed" << endl;
	}

	if (!M.setMode(MODE_CURRENT, SECOND_MOTOR_CAN))
	{
		return false;
	}
	else
	{
		cout << "Motor " << SECOND_MOTOR_CAN << " free successed" << endl;
	}

	if (!M.setMode(MODE_CURRENT, THIRD_MOTOR_CAN))
	{
		return false;
	}
	else
	{
		cout << "Motor " << THIRD_MOTOR_CAN << " free successed" << endl;
	}

	return true;
}

//机械臂设置控制模式（设置为位置模式）
//@return true if arm init mode success
bool arm_init_mode()
{
	if (!M.setMode(MODE_POSITION, FIRST_MOTOR_CAN))
	{
		return false;
	}
	else
	{
		cout << "Motor " << FIRST_MOTOR_CAN << " init_mode successed" << endl;
	}

	if (!M.setMode(MODE_POSITION, SECOND_MOTOR_CAN))
	{
		return false;
	}
	else
	{
		cout << "Motor " << SECOND_MOTOR_CAN << " init_mode successed" << endl;
	}

	if (!M.setMode(MODE_POSITION, THIRD_MOTOR_CAN))
	{
		return false;
	}
	else
	{
		cout << "Motor " << THIRD_MOTOR_CAN << " init_mode successed" << endl;
	}

	return true;
}

//机械臂失能
void arm_disable()
{
	M.disable(FIRST_MOTOR_CAN);
	M.disable(SECOND_MOTOR_CAN);
	M.disable(THIRD_MOTOR_CAN);
}

//机械臂急停
void arm_stop()
{
	ARM_STATE = ARM_FREE;

	M.stop(FIRST_MOTOR_CAN);
	M.stop(SECOND_MOTOR_CAN);
	M.stop(THIRD_MOTOR_CAN);
}

//检查此刻机械臂位置是否在合法范围
//@return true if arm is at legal angles
bool arm_position_test()
{
	if (joint_now[0] > 91 || joint_now[0] < -181 ||
		joint_now[1] > 151 || joint_now[1] < -151 ||
		joint_now[2] > 151 || joint_now[2] < -151)
	{
		return false;
	}
	else
	{
		return true;
	}
}

//机械臂打开
void arm_open()
{
	ARM_STATE = ARM_GET_TRAJ;

	double angle_1_now = M.getAngle(FIRST_MOTOR_CAN);
	double angle_2_now = M.getAngle(SECOND_MOTOR_CAN);
	double angle_3_now = M.getAngle(THIRD_MOTOR_CAN);

	joint_now[0] = angle_1_now;
	joint_now[1] = angle_2_now;
	joint_now[2] = angle_3_now;

	TAR.lock();

	joint_tar = { FIRST_MOTOR_OPEN, SECOND_MOTOR_OPEN, THIRD_MOTOR_OPEN, 0, 0 };
	//joint_tar = { 0, 0, 0, 0, 0 };
	TAR.unlock();

	if (get_traj(joint_now, joint_tar, traj))
	{
		ARM_STATE = ARM_GET_TRAJ_SUCCESS;
	}
	else
	{
		ARM_STATE = ARM_FREE;
	}
}

//机械臂收拢
void arm_close()
{
	ARM_STATE = ARM_GET_TRAJ;

	double angle_1_now = M.getAngle(FIRST_MOTOR_CAN);
	double angle_2_now = M.getAngle(SECOND_MOTOR_CAN);
	double angle_3_now = M.getAngle(THIRD_MOTOR_CAN);

	joint_now[0] = angle_1_now;
	joint_now[1] = angle_2_now;
	joint_now[2] = angle_3_now;

	TAR.lock();

	joint_tar = { FIRST_MOTOR_CLOSE, SECOND_MOTOR_CLOSE, THIRD_MOTOR_CLOSE, 0, 0 };

	TAR.unlock();

	if (get_traj(joint_now, joint_tar, traj))
	{
		ARM_STATE = ARM_GET_TRAJ_SUCCESS;
	}
	else
	{
		ARM_STATE = ARM_FREE;
	}
}

//机械臂向前移动
void arm_up()
{
	ARM_STATE = ARM_GET_TRAJ;

	double angle_1_now = M.getAngle(FIRST_MOTOR_CAN);
	double angle_2_now = M.getAngle(SECOND_MOTOR_CAN);
	double angle_3_now = M.getAngle(THIRD_MOTOR_CAN);

	joint_now[0] = angle_1_now;
	joint_now[1] = angle_2_now;
	joint_now[2] = angle_3_now;

	MOVE_TYPE = MOVE_TYPE_JOYSTICK;

	if (get_traj_kdl(joint_now, traj, Eigen::Vector2d(0, 1), FOCUS_STATE, focus_point))
	{
		ARM_STATE = ARM_GET_TRAJ_SUCCESS;
	}
	else
	{
		ARM_STATE = ARM_FREE;
	}
}

//机械臂向后移动
void arm_down()
{
	ARM_STATE = ARM_GET_TRAJ;

	double angle_1_now = M.getAngle(FIRST_MOTOR_CAN);
	double angle_2_now = M.getAngle(SECOND_MOTOR_CAN);
	double angle_3_now = M.getAngle(THIRD_MOTOR_CAN);

	joint_now[0] = angle_1_now;
	joint_now[1] = angle_2_now;
	joint_now[2] = angle_3_now;

	MOVE_TYPE = MOVE_TYPE_JOYSTICK;

	if (get_traj_kdl(joint_now, traj, Eigen::Vector2d(0, -1), FOCUS_STATE, focus_point))
	{
		ARM_STATE = ARM_GET_TRAJ_SUCCESS;
	}
	else
	{
		ARM_STATE = ARM_FREE;
	}
}

//机械臂向右移动
void arm_right()
{
	ARM_STATE = ARM_GET_TRAJ;

	double angle_1_now = M.getAngle(FIRST_MOTOR_CAN);
	double angle_2_now = M.getAngle(SECOND_MOTOR_CAN);
	double angle_3_now = M.getAngle(THIRD_MOTOR_CAN);

	joint_now[0] = angle_1_now;
	joint_now[1] = angle_2_now;
	joint_now[2] = angle_3_now;

	MOVE_TYPE = MOVE_TYPE_JOYSTICK;

	if (get_traj_kdl(joint_now, traj, Eigen::Vector2d(1, 0), FOCUS_STATE, focus_point))
	{
		ARM_STATE = ARM_GET_TRAJ_SUCCESS;
	}
	else
	{
		ARM_STATE = ARM_FREE;
	}
}

//机械臂向左移动
void arm_left()
{
	ARM_STATE = ARM_GET_TRAJ;

	double angle_1_now = M.getAngle(FIRST_MOTOR_CAN);
	double angle_2_now = M.getAngle(SECOND_MOTOR_CAN);
	double angle_3_now = M.getAngle(THIRD_MOTOR_CAN);

	joint_now[0] = angle_1_now;
	joint_now[1] = angle_2_now;
	joint_now[2] = angle_3_now;

	MOVE_TYPE = MOVE_TYPE_JOYSTICK;

	if (get_traj_kdl(joint_now, traj, Eigen::Vector2d(-1, 0), FOCUS_STATE, focus_point))
	{
		ARM_STATE = ARM_GET_TRAJ_SUCCESS;
	}
	else
	{
		ARM_STATE = ARM_FREE;
	}
}

//机械臂向指定方向移动
void arm_direction(double direction_x, double direction_y)
{
	ARM_STATE = ARM_GET_TRAJ;

	double angle_1_now = M.getAngle(FIRST_MOTOR_CAN);
	double angle_2_now = M.getAngle(SECOND_MOTOR_CAN);
	double angle_3_now = M.getAngle(THIRD_MOTOR_CAN);

	joint_now[0] = angle_1_now;
	joint_now[1] = angle_2_now;
	joint_now[2] = angle_3_now;

	MOVE_TYPE = MOVE_TYPE_JOYSTICK;

	if (get_traj_kdl(joint_now, traj, Eigen::Vector2d(direction_x, direction_y), FOCUS_STATE, focus_point))
	{
		ARM_STATE = ARM_GET_TRAJ_SUCCESS;
	}
	else
	{
		ARM_STATE = ARM_FREE;
	}
}

//机械臂调试运动
//@param number 调试运动关节ID
//@param direction 调试运动方向
void arm_debug(int number, int direction)
{
	MOVE_TYPE = MOVE_TYPE_DEBUG;
	ARM_STATE = ARM_GET_TRAJ;

	double angle_1_now = M.getAngle(FIRST_MOTOR_CAN);
	double angle_2_now = M.getAngle(SECOND_MOTOR_CAN);
	double angle_3_now = M.getAngle(THIRD_MOTOR_CAN);

	joint_now[0] = angle_1_now;
	joint_now[1] = angle_2_now;
	joint_now[2] = angle_3_now;

	joint_tar[0] = angle_1_now;
	joint_tar[1] = angle_2_now;
	joint_tar[2] = angle_3_now;

	TAR.lock();

	if (number == FIRST_MOTOR_CAN)
	{
		if (direction == DEBUG_POSITIVE)
		{
			joint_tar[0] = FIRST_MOTOR_MAX;
		}
		else if (direction == DEBUG_NEGATIVE)
		{
			joint_tar[0] = FIRST_MOTOR_MIN;
		}
	}
	else if (number == SECOND_MOTOR_CAN)
	{
		if (direction == DEBUG_POSITIVE)
		{
			joint_tar[1] = SECOND_MOTOR_MAX;
		}
		else if (direction == DEBUG_NEGATIVE)
		{
			joint_tar[1] = SECOND_MOTOR_MIN;
		}
	}
	else if (number == THIRD_MOTOR_CAN)
	{
		if (direction == DEBUG_POSITIVE)
		{
			joint_tar[2] = THIRD_MOTOR_MAX;
		}
		else if (direction == DEBUG_NEGATIVE)
		{
			joint_tar[2] = THIRD_MOTOR_MIN;
		}
	}

	TAR.unlock();

	traj.resize(2);
	traj[0].positions[0] = joint_now[0];
	traj[0].positions[1] = joint_now[1];
	traj[0].positions[2] = joint_now[2];

	traj[1].positions[0] = joint_tar[0];
	traj[1].positions[1] = joint_tar[1];
	traj[1].positions[2] = joint_tar[2];

	ARM_STATE = ARM_GET_TRAJ_SUCCESS;
}
#pragma endregion

#pragma region 机械臂底层伺服及规划函数
//机械臂工作函数
void motor_work()
{
	thread t_follow_traj(follow_traj_thread_fun);	//创建轨迹跟随线程
	thread t_rrt_solver_work(rrt_solver_work_thread_fun);	//创建路径规划线程，仅应用于动态路径规划，使用静态路径规划器时应将其注释
	//thread t_motor_auto(motor_auto);

	t_rrt_solver_work.join();
	t_follow_traj.join();
	//t_motor_auto.join();
}

//机械臂演示自动运动函数
void motor_auto()
{
	int tum = 0;
	clock_t start, now;
	while (true)
	{
		while (AUTO_FLAG == 1)
		{
			if(tum==8)
				tum = 0;
			while (AUTO_FLAG == 1)
			{
				if (tum == 0)
				{
					arm_up();
				}
				else if (tum == 1)
				{
					arm_down();
				}
				else if (tum == 2)
				{
					arm_right();
				}
				else if (tum == 3)
				{
					arm_left();
				}
				else if (tum == 4)
				{
					arm_left();
				}
				else if (tum == 5)
				{
					arm_right();
				}
				else if (tum == 6)
				{
					arm_down();
				}
				else if (tum == 7)
				{
					arm_up();
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(10000));
				break;
			}
			tum++;
		}
	}
}

//机械臂轨迹跟随线程函数
void follow_traj_thread_fun()
{
	while (true)
	{
		//判断机械臂状态
		if (ARM_STATE == ARM_GET_TRAJ_SUCCESS)
		{
			follow_traj();	//若轨迹已规划成功，跟随轨迹（初始轨迹）
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

extern void udp_send_debug_info(int16_t error_index);	//UDP发送函数，向上位机发送机器人调试信息

//机械臂路径规划（RRT）线程函数，仅应用于动态路径规划，使用静态路径规划器时应将其注释
void rrt_solver_work_thread_fun()
{
	while (true)
	{
		//判断机械臂状态是否处于跟随轨迹状态
		//判断机械臂运动类型是否为避障轨迹
		//判断路径规划器是够已经初始化
		if (ARM_STATE == ARM_FOLLOW_TRAJ && MOVE_TYPE == MOVE_TYPE_TRAJ && rrt_solver_ptr)
		{
			//判断是否有新障碍物出现
			if (obstacles_new.size() > 0)
			{
				traj_mutex.lock();

				rrt_solver_mutex.lock();

				rrt_solver_ptr->set_obs_new(true);	//设置规划器 出现新障碍物 标志位为true

				//获取路径规划结果
				//int res = rrt_solver_ptr->d_rrt_search(path);
				//int res = rrt_solver_ptr->d_rrt_star_search(path);
				//int res = rrt_solver_ptr->d_rrt_connect_search(path);
				//int res = rrt_solver_ptr->d_rrt_connect_star_search(path);
				//int res = rrt_solver_ptr->mp_rrt_search(path);
				//int res = rrt_solver_ptr->mp_rrt_star_search(path);
				//int res = rrt_solver_ptr->mp_rrt_connect_search(path);
				int res = rrt_solver_ptr->mp_rrt_connect_star_search(path);

				rrt_solver_mutex.unlock();

				obstacles_new.clear();	//清空新增障碍物列表

				if (res == D_RRT_NEW_PATH)	//若规划器返回新路径
				{
					udp_send_debug_info(16);
					//根据路径更新初始轨迹（此时还未计算时间戳）
					traj.resize(path.size());

					for (int i = 0; i < traj.size(); i++)
					{
						traj[i].positions[0] = path[i][0];
						traj[i].positions[1] = path[i][1];
						traj[i].positions[2] = path[i][2];
					}

					//轨迹插值
					traj_to_interpolation_trapezoid();

					rrt_solver_state = RRT_SOLVER_STATE_NEW_PATH;	//将 规划器返回路径类型 全局变量 置为 新路径
				}
				else if (res == D_RRT_OVER_TIME)	//若路径规划超时
				{
					ARM_STATE = ARM_FREE;	//机械臂状态设为空闲（机械臂将停止跟随路径）
					cout << "search time over 2 seconds" << endl;
					udp_send_debug_info(14);

					traj_mutex.unlock();
					continue;
				}
				else if (res == D_RRT_ERROR)	//若路径规划出错（一般为机械臂现状态或目标状态将发生碰撞）
				{
					ARM_STATE = ARM_FREE;	//机械臂状态设为空闲（机械臂将停止跟随路径）
					cout << "error" << endl;
					udp_send_debug_info(15);

					traj_mutex.unlock();
					continue;
				}

				traj_mutex.unlock();

			}
			continue;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
}

//轨迹跟随函数
void follow_traj()
{
	ARM_STATE = ARM_FOLLOW_TRAJ;	//设置机械臂状态为跟随轨迹

	int sleep_time;	//轨迹点执行时间间隔

	if (MOVE_TYPE == MOVE_TYPE_JOYSTICK || MOVE_TYPE == MOVE_TYPE_FOCUS)	//若机械臂运动类型为手柄控制或目标锁定（笛卡尔轨迹）
	{
		traj_to_cartesian_line(50, 50, line_step);	//直线插值
	}
	else
	{
		traj_to_interpolation_trapezoid();	//梯形速度插值
		//traj_to_interpolation_S_acc();	//S加速度插值
	}

	sleep_time = (int)(TRAJ_TIME_STEP * 1000); //秒转换为毫秒

	int n = traj_interpolation.size();
	for (int i = 0; i < n; i++)
	{
		traj_mutex.lock();

		if (rrt_solver_state == RRT_SOLVER_STATE_NEW_PATH)	//若规划器返回新轨迹，轨迹已发生改变，需从第一轨迹点开始跟随
		{
			i = 0;	//更新循环i值
			n = traj_interpolation.size();	//更新循环n值
			rrt_solver_state = RRT_SOLVER_STATE_OLD_PATH;	//将 规划器返回路径类型 全局变量置为旧路径

			traj_mutex.unlock();
			continue;
		}

		if (VISUALSERVO_TRAJ_STATE == 2)	//视觉伺服更新机械臂末端运动方向
		{
			traj_to_interpolation_trapezoid();	//梯形速度插值
			i = 0;	//更新循环i值
			n = traj_interpolation.size();	//更新循环n值
			VISUALSERVO_TRAJ_STATE = 1;	//将 规划器返回路径类型 全局变量置为旧路径

			traj_mutex.unlock();
			continue;
		}

		if (ARM_STATE < ARM_FOLLOW_TRAJ)	//若机械臂状态被改变，则轨迹跟随被打断
		{
			cout << "following traj has been interuptted" << endl;

			traj_mutex.unlock();
			autoState = 0;
			return;
		}

		//发送关节电机控制指令
		M.setAngle(traj_interpolation[i].positions[0], FIRST_MOTOR_CAN);
		M.setAngle(traj_interpolation[i].positions[1], SECOND_MOTOR_CAN);
		M.setAngle(traj_interpolation[i].positions[2], THIRD_MOTOR_CAN);

		//更新机械臂各关节角度值
		joint_now[0] = traj_interpolation[i].positions[0];
		joint_now[1] = traj_interpolation[i].positions[1];
		joint_now[2] = traj_interpolation[i].positions[2];

		//更新机械臂末端云台各关节角度值
		joint_now[3] = traj_interpolation[i].positions[3];
		joint_now[4] = traj_interpolation[i].positions[4];

		//更新可视化模型关节角度值
		ACM_show.arm_set(joint_now[0], joint_now[1], joint_now[2]);

		if (MOVE_TYPE == MOVE_TYPE_TRAJ)	//若机械臂运动类型为避障轨迹
		{
			//更新规划器中机械臂当前位置
			rrt_solver_ptr->set_bot_state(Eigen::Vector3d(joint_now[0], joint_now[1], joint_now[2]));
		}

		traj_mutex.unlock();

		std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
	}

	ARM_STATE = ARM_FREE;	//将机械臂状态置为空闲
	if (VISUALSERVO_STATE == 1)
	{
		VISUALSERVO_STATE = 2;
	}
	if (VISUALSERVO_STATE == 3)
	{
		VISUALSERVO_STATE = 4;
	}
	if (OPTRAJFOLLOW == 1)
		OPTRAJFOLLOW = 0;
	cout << "finishing" << endl;
}

//计算机械臂各关节权重
//@param joint_now 机械臂当前各关节角度
//@param joint_tar 机械臂各关节目标角度
//@return w 权重
Eigen::VectorXd calculate_weight(std::vector<double> joint_now, std::vector<double> joint_tar)
{
	Eigen::VectorXd w(3);

	for (int i = 0; i < 3; i++)
	{
		w[i] = 1;
	}

	//权重与机械臂长度成反比
	w[2] *= 1.6;
	w[1] *= 0.9;
	w[0] *= 0.4;

	//权重与目标角度与当前角度之差成反比
	for (int i = 0; i < 3; i++)
	{
		double temp = abs(joint_now[i] - joint_tar[i]);
		w[i] = 1 / (temp < 1 ? 1 : temp);
	}

	w = w / w.norm() * sqrt(3);	//归一化

	return w;
}

//规划避障路径
//@param joint_now 机械臂当前各关节角度
//@param joint_tar 机械臂各关节目标角度
//@param traj 机械臂初始轨迹
//@return true if get traj success
bool get_traj(std::vector<double> joint_now, std::vector<double> joint_tar, std::vector<traj_point>& traj)
{
	MOVE_TYPE = MOVE_TYPE_TRAJ;	//设置机械臂运动类型为避障路径

	//QTM重新建图
	QTM_mutex.lock();

	obstacles.clear();	//清空障碍物列表
	obstacles_new.clear();	//清空新增障碍物列表

	QTM.decay_for_rebuild(QTM.root);
	QTM.tbd_for_rebuild(QTM.root);

	Lidar_Point.lock();
	QTM.build_map(Lidar_point);
	Lidar_Point.unlock();

	obstacles = QTM.obs_rects;
	obstacles_new = QTM.obs_rects_new;

	QTM_mutex.unlock();

	if (rrt_solver_ptr)	//若规划器指针已被初始化，释放其内存（因目标点改变，需重新初始化）
	{
		rrt_solver_mutex.lock();
		delete rrt_solver_ptr;
		rrt_solver_mutex.unlock();
	}

	//路径规划器初始化
	rrt_solver_mutex.lock();

	//静态路径规划器
	//rrt_solver_ptr = new RRT_test::RRT(space_move, Eigen::Vector3d(joint_now[0], joint_now[1], joint_now[2]), Eigen::Vector3d(joint_tar[0], joint_tar[1], joint_tar[2]));
	//rrt_solver_ptr = new RRT_test::RRT_Star(space_move, Eigen::Vector3d(joint_now[0], joint_now[1], joint_now[2]), Eigen::Vector3d(joint_tar[0], joint_tar[1], joint_tar[2]),5);
	//rrt_solver_ptr = new RRT_test::RRT_Connect(space_move, Eigen::Vector3d(joint_now[0], joint_now[1], joint_now[2]), Eigen::Vector3d(joint_tar[0], joint_tar[1], joint_tar[2]));
	//rrt_solver_ptr = new RRT_test::RRT_Connect_Star(space_move, Eigen::Vector3d(joint_now[0], joint_now[1], joint_now[2]), Eigen::Vector3d(joint_tar[0], joint_tar[1], joint_tar[2]),5);
	//rrt_solver_ptr = new RRT_test::Q_RRT_Star(space_move, Eigen::Vector3d(joint_now[0], joint_now[1], joint_now[2]), Eigen::Vector3d(joint_tar[0], joint_tar[1], joint_tar[2]), 5, 2);

	//动态路径规划器
	//rrt_solver_ptr = new RRT_test::D_RRT(space_move, Eigen::Vector3d(joint_now[0], joint_now[1], joint_now[2]), Eigen::Vector3d(joint_tar[0], joint_tar[1], joint_tar[2]));
	//rrt_solver_ptr = new RRT_test::D_RRT_Star(space_move, Eigen::Vector3d(joint_now[0], joint_now[1], joint_now[2]), Eigen::Vector3d(joint_tar[0], joint_tar[1], joint_tar[2]),5);
	//rrt_solver_ptr = new RRT_test::D_RRT_Connect(space_move, Eigen::Vector3d(joint_now[0], joint_now[1], joint_now[2]), Eigen::Vector3d(joint_tar[0], joint_tar[1], joint_tar[2]));
	//rrt_solver_ptr = new RRT_test::D_RRT_Connect_Star(space_move, Eigen::Vector3d(joint_now[0], joint_now[1], joint_now[2]), Eigen::Vector3d(joint_tar[0], joint_tar[1], joint_tar[2]), 5);
	//rrt_solver_ptr = new RRT_test::MP_RRT(space_move, Eigen::Vector3d(joint_now[0], joint_now[1], joint_now[2]), Eigen::Vector3d(joint_tar[0], joint_tar[1], joint_tar[2]));
	//rrt_solver_ptr = new RRT_test::MP_RRT_Star(space_move, Eigen::Vector3d(joint_now[0], joint_now[1], joint_now[2]), Eigen::Vector3d(joint_tar[0], joint_tar[1], joint_tar[2]), 5);
	//rrt_solver_ptr = new RRT_test::MP_RRT_Connect(space_move, Eigen::Vector3d(joint_now[0], joint_now[1], joint_now[2]), Eigen::Vector3d(joint_tar[0], joint_tar[1], joint_tar[2]));
	rrt_solver_ptr = new RRT_test::MP_RRT_Connect_Star(space_move, Eigen::Vector3d(joint_now[0], joint_now[1], joint_now[2]), Eigen::Vector3d(joint_tar[0], joint_tar[1], joint_tar[2]), 5);

	rrt_solver_ptr->set_obstacle_free_checker(isStateValid_obstacle);	//设置碰撞检测函数（针对所有障碍物）
	rrt_solver_ptr->set_obstacle_free_new_checker(isStateValid_obstacle_new);	//设置碰撞检测函数（针对新增障碍物）
	rrt_solver_ptr->set_limit_time(3);	//设置求解限制时间

	rrt_solver_ptr->set_lazy(true);	//设置是否采用lazy策略

	Eigen::VectorXd w = calculate_weight(joint_now, joint_tar);	//计算权重

	cout << "weight = [" << w[0] << ", " << w[1] << ", " << w[2] << "]" << endl;

	rrt_solver_ptr->set_weight(w);	//设置路径规划器权重

	//路径规划器求解
	// 静态路径规划器
	//int res = rrt_solver_ptr->rrt_search(path);
	//int res = rrt_solver_ptr->rrt_star_search(path);
	//int res = rrt_solver_ptr->rrt_connect_search(path);
	//int res = rrt_solver_ptr->rrt_connect_star_search(path);
	//int res = rrt_solver_ptr->q_rrt_star_search(path);

	//动态路径规划器
	//int res = rrt_solver_ptr->d_rrt_search(path);
	//int res = rrt_solver_ptr->d_rrt_star_search(path);
	//int res = rrt_solver_ptr->d_rrt_connect_search(path);
	//int res = rrt_solver_ptr->d_rrt_connect_star_search(path);
	//int res = rrt_solver_ptr->mp_rrt_search(path);
	//int res = rrt_solver_ptr->mp_rrt_star_search(path);
	//int res = rrt_solver_ptr->mp_rrt_connect_search(path);
	int res = rrt_solver_ptr->mp_rrt_connect_star_search(path);

	rrt_solver_mutex.unlock();

	if (res == 0)	//若求得路径，将路径赋予traj
	{
		traj.resize(path.size());

		for (int i = 0; i < traj.size(); i++)
		{
			traj[i].positions[0] = path[i][0];
			traj[i].positions[1] = path[i][1];
			traj[i].positions[2] = path[i][2];
		}

		udp_send_debug_info(16);
		return true;
	}
	else
	{
		if (res == 3)	//若求解超时
		{
			cout << "search time over 2 seconds" << endl;
			udp_send_debug_info(14);
		}
		else if (res == 2)
		{
			cout << "error" << endl;
			udp_send_debug_info(15);

		}
		return false;
	}
}
#pragma endregion

#pragma region 机械臂路径规划相关函数
//rrt路径规划器相关变量初始化
void rrt_init()
{
	cout << "rrt_init start" << endl;
	std::vector<double>space_lim_move(6);
	space_lim_move[0] = -181;
	space_lim_move[1] = 91;
	space_lim_move[2] = -155;
	space_lim_move[3] = 151;
	space_lim_move[4] = -155;
	space_lim_move[5] = 151;

	space_move = RRT_test::RRT_Space(space_lim_move, 3);	//初始化路径规划器规划空间

	cout << "rrt_init succesed" << endl;
}

//碰撞检测函数：针对全部障碍物
//@param v_x 关节角度vector
bool isStateValid_obstacle(Eigen::VectorXd v_x)
{
	QTM_mutex.lock();
	bool is_collision = ACM.collision(v_x[0], v_x[1], v_x[2], obstacles);
	QTM_mutex.unlock();
	return !is_collision;
}

//碰撞检测函数：针对新增障碍物
//@param v_x 关节角度vector
bool isStateValid_obstacle_new(Eigen::VectorXd v_x)
{
	QTM_mutex.lock();
	bool is_collision = ACM.collision(v_x[0], v_x[1], v_x[2], obstacles_new);
	QTM_mutex.unlock();
	return !is_collision;
}

//运动学逆解，应用于求解目标拍摄位置对应的机器人关节角度
//@param angle_1——angle_5 机械臂3关节、云台2关节角度
//@param position_tar	目标拍摄位置
//@param z_tar 目标拍摄角度
bool kdl_ik(double angle_1, double angle_2, double angle_3, double angle_4, double angle_5, Eigen::Vector3d position_tar, Eigen::Vector3d z_tar)
{
	//建立运动学模型，长度单位m
	KDL::Chain chain;
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.7, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.5, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.4, 0))));
	chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(0.0, 0.142, 0))));
	chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0.0, 0, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0, 0))));

	//建立正逆运动学解求解器
	ChainFkSolverPos_recursive fksolver(chain);	//正运动学求解器
	ChainIkSolverVel_pinv iksolver1v(chain);	//逆运动学求解器
	ChainIkSolverPos_NR_JL iksolver(chain, fksolver, iksolver1v, 100, 1e-6);	//最大迭代次数100, 精度达到1e-6时停止迭代


	//正运动学求解
	KDL::ChainFkSolverPos_recursive fwdkin(chain);
	int n = chain.getNrOfJoints();

	//设置运动副极限值
	KDL::JntArray q_min(n);
	q_min.data[0] = -150 * PI / 180;
	q_min.data[1] = -150 * PI / 180;
	q_min.data[2] = -150 * PI / 180;
	q_min.data[3] = -40 * PI / 180;
	q_min.data[4] = -60 * PI / 180;
	q_min.data[5] = -PI;

	KDL::JntArray q_max(n);
	q_max.data[0] = 90 * PI / 180;
	q_max.data[1] = 150 * PI / 180;
	q_max.data[2] = 150 * PI / 180;
	q_max.data[3] = 40 * PI / 180;
	q_max.data[4] = 60 * PI / 180;
	q_max.data[5] = PI;

	iksolver.setJointLimits(q_min, q_max);

	//创建运动副数组
	KDL::JntArray q_init(n);	//初始角度
	KDL::JntArray q_sol(n);		//解角度

	q_init.data[0] = angle_1 * PI / 180;
	q_init.data[1] = angle_2 * PI / 180;
	q_init.data[2] = angle_3 * PI / 180;
	q_init.data[3] = angle_4 * PI / 180;
	q_init.data[4] = angle_5 * PI / 180;
	q_init.data[5] = 0;

	KDL::Frame pos_goal;	//目标位姿
	fwdkin.JntToCart(q_init, pos_goal);	//暂时将现位姿赋予目标位姿

	cout << "---------------initial_pose---------------" << endl;
	cout << pos_goal.p.data << endl;
	cout << pos_goal.M.data << endl;

	//根据position_tar对pos_goal平移参数进行修改
	pos_goal.p[0] = position_tar(0);
	pos_goal.p[1] = position_tar(1);
	pos_goal.p[2] = position_tar(2);

	Eigen::Vector3d z_ori;	//位姿Z轴矢量
	z_ori(0) = pos_goal.M.data[2];
	z_ori(1) = pos_goal.M.data[5];
	z_ori(2) = pos_goal.M.data[8];

	double z_dot = z_ori.dot(z_tar);	//现位姿Z轴矢量与目标位姿Z轴矢量点乘
	Eigen::Vector3d z_cross = z_ori.cross(z_tar);	//现位姿Z轴矢量与目标位姿Z轴矢量叉乘
	z_cross /= z_cross.norm();	//归一化

	double angle = acos(z_dot);	//求矩阵绕z_cros旋转角度

	KDL::Vector rotvec;	//旋转轴矢量

	rotvec(0) = z_cross(0);
	rotvec(1) = z_cross(1);
	rotvec(2) = z_cross(2);


	pos_goal.M = KDL::Rotation::Rot(rotvec, angle) * pos_goal.M;	//将pos_goal旋转参数绕rotvec旋转angle

	cout << "---------------target_pose---------------" << endl;
	cout << pos_goal.p.data << endl;
	cout << pos_goal.M.data << endl;


	//Set destination frame

	int try_time = 0;

	while (try_time < 10)	//最多尝试10次
	{
		try_time += 1;

		int ret = iksolver.CartToJnt(q_init, pos_goal, q_sol);	//求逆解


		if (ret == 0)	//若求解成功
		{
			std::cout << "-------------- successed ---------------" << std::endl;

			//设置joint_tar
			TAR.lock();
			joint_tar[0] = q_sol.data(0) / PI * 180;
			joint_tar[1] = q_sol.data(1) / PI * 180;
			joint_tar[2] = q_sol.data(2) / PI * 180;
			joint_tar[3] = q_sol.data(3) / PI * 180;
			joint_tar[4] = q_sol.data(4) / PI * 180;
			TAR.unlock();

			std::cout << "-------------- joint_tar ---------------" << std::endl;

			cout << joint_tar[0] << endl;
			cout << joint_tar[1] << endl;
			cout << joint_tar[2] << endl;
			cout << joint_tar[3] << endl;
			cout << joint_tar[4] << endl;

			return true;
		}

		q_init.data.setRandom();
		q_init.data *= PI;
	}

	std::cout << "XXXXXXXXXXXXXX failed XXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
	return false;
}
//运动学逆解，应用于求解YOLO识别出的拍摄位置对应的机器人关节角度
//@param angle_1——angle_5 机械臂3关节、云台2关节角度
//@param position_tar	目标拍摄位置
//@param z_tar 目标拍摄角度
bool YOLO_kdl_ik(std::vector<double> joint_now, std::pair<double, double> position_tar, Eigen::Vector3d focus_point)
{
	//建立运动学模型，长度单位m
	KDL::Chain chain;
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.7, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.5, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.542, 0))));	//包含第三关节臂0.4，云台0.142
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0))));	//设置一冗余自由度，使得可仅修改矩阵平移参数求解

	//建立正逆运动学解求解器
	ChainFkSolverPos_recursive fksolver(chain);	//正运动学求解器
	ChainIkSolverVel_pinv iksolver1v(chain);	//逆运动学求解器
	ChainIkSolverPos_NR_JL iksolver(chain, fksolver, iksolver1v, 100, 1e-6);	//最大迭代次数100, 精度达到1e-6时停止迭代

	//正运动学求解
	KDL::ChainFkSolverPos_recursive fwdkin(chain);
	int n = chain.getNrOfJoints();

	//设置运动副极限值
	KDL::JntArray q_min(n);
	q_min.data[0] = -181 * PI / 180;
	q_min.data[1] = -151 * PI / 180;
	q_min.data[2] = -151 * PI / 180;
	q_min.data[3] = -180 * PI / 180;
	KDL::JntArray q_max(n);
	q_max.data[0] = 91 * PI / 180;
	q_max.data[1] = 151 * PI / 180;
	q_max.data[2] = 151 * PI / 180;
	q_max.data[3] = 180 * PI / 180;
	iksolver.setJointLimits(q_min, q_max);

	//创建运动副数组
	KDL::JntArray q_init(n);	//初始角度
	KDL::JntArray q_sol(n);		//解角度

	q_init.data[0] = joint_now[0] * PI / 180;
	q_init.data[1] = joint_now[1] * PI / 180;
	q_init.data[2] = joint_now[2] * PI / 180;
	q_init.data[3] = 0;

	KDL::Frame pos_goal;	//目标位姿
	fwdkin.JntToCart(q_init, pos_goal);	//暂时将现位姿赋予目标位姿

	//根据position修改pos_goal平移参数
	pos_goal.p[0] = position_tar.first;
	pos_goal.p[1] = position_tar.second;

	if (!iksolver.CartToJnt(q_init, pos_goal, q_sol))	//求运动学逆解
	{
		//将解得角度赋予theta
		joint_tar[0] = q_sol.data[0] * 180 / PI;
		joint_tar[1] = q_sol.data[1] * 180 / PI;
		joint_tar[2] = q_sol.data[2] * 180 / PI;


		//根据目标位置与focus_point空间位置关系，计算云台电机关节角度
		Eigen::Vector3d z_axis = Eigen::Vector3d(pos_goal.p[0], pos_goal.p[1], pos_goal.p[2]) - focus_point;
		z_axis /= z_axis.norm();

		double alpha = q_sol.data[0] + q_sol.data[1] + q_sol.data[2];
		Eigen::AngleAxisd rotation_vector(-alpha, Eigen::Vector3d(0, 0, 1));
		z_axis = rotation_vector.matrix() * z_axis;

		joint_tar[4] = asin(-z_axis[1]) * 180 / PI;
		joint_tar[3] = -atan(z_axis[0] / z_axis[2]) * 180 / PI;

		return true;
	}
	else
	{
		return false;
	}
}

//运动学逆解，应用于遮挡规避时求解机械臂末端移动时目标锁定所需关节角度
//@param joint_now 现关节角度
//@param position	相机位置
//@param focus_point 目标锁定点
//@param theta 解得关节角度（包括3关节电机，2云台电机），单位为rad
bool solve_focus_kdl(std::vector<double> joint_now, std::pair<double, double> position, Eigen::Vector3d focus_point,
	std::vector<double>& theta)
{
	//建立运动学模型，长度单位m
	KDL::Chain chain;
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.7, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.5, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.542, 0))));	//包含第三关节臂0.4，云台0.142
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0))));	//设置一冗余自由度，使得可仅修改矩阵平移参数求解

	//建立正逆运动学解求解器
	ChainFkSolverPos_recursive fksolver(chain);	//正运动学求解器
	ChainIkSolverVel_pinv iksolver1v(chain);	//逆运动学求解器
	ChainIkSolverPos_NR_JL iksolver(chain, fksolver, iksolver1v, 100, 1e-6);	//最大迭代次数100, 精度达到1e-6时停止迭代

	//正运动学求解
	KDL::ChainFkSolverPos_recursive fwdkin(chain);
	int n = chain.getNrOfJoints();

	//设置运动副极限值
	KDL::JntArray q_min(n);
	q_min.data[0] = -181 * PI / 180;
	q_min.data[1] = -151 * PI / 180;
	q_min.data[2] = -151 * PI / 180;
	q_min.data[3] = -180 * PI / 180;
	KDL::JntArray q_max(n);
	q_max.data[0] = 91 * PI / 180;
	q_max.data[1] = 151 * PI / 180;
	q_max.data[2] = 151 * PI / 180;
	q_max.data[3] = 180 * PI / 180;
	iksolver.setJointLimits(q_min, q_max);

	//创建运动副数组
	KDL::JntArray q_init(n);	//初始角度
	KDL::JntArray q_sol(n);		//解角度

	q_init.data[0] = joint_now[0] * PI / 180;
	q_init.data[1] = joint_now[1] * PI / 180;
	q_init.data[2] = joint_now[2] * PI / 180;
	q_init.data[3] = 0;

	KDL::Frame pos_goal;	//目标位姿
	fwdkin.JntToCart(q_init, pos_goal);	//暂时将现位姿赋予目标位姿

	//根据position修改pos_goal平移参数
	pos_goal.p[0] = position.first;
	pos_goal.p[1] = position.second;

	if (!iksolver.CartToJnt(q_init, pos_goal, q_sol))	//求运动学逆解
	{
		//将解得角度赋予theta
		theta[0] = q_sol.data[0];
		theta[1] = q_sol.data[1];
		theta[2] = q_sol.data[2];


		//根据目标位置与focus_point空间位置关系，计算云台电机关节角度
		Eigen::Vector3d z_axis = Eigen::Vector3d(pos_goal.p[0], pos_goal.p[1], pos_goal.p[2]) - focus_point;
		z_axis /= z_axis.norm();

		double alpha = q_sol.data[0] + q_sol.data[1] + q_sol.data[2];
		Eigen::AngleAxisd rotation_vector(-alpha, Eigen::Vector3d(0, 0, 1));
		z_axis = rotation_vector.matrix() * z_axis;

		theta[4] = asin(-z_axis[1]);
		theta[3] = atan(z_axis[0] / z_axis[2]);

		return true;
	}
	else
	{
		return false;
	}
}

//运动学正解，应用于视觉伺服求解机械臂末端位置
//@param joint_now 现关节角度
std::pair<double,double> solve_visualservo_kdl(std::vector<double> joint_now)
{
	//建立运动学模型，长度单位m
	KDL::Chain chain;
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.7, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.5, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.542, 0))));	//包含第三关节臂0.4，云台0.142
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0))));	//设置一冗余自由度，使得可仅修改矩阵平移参数求解

	//建立正逆运动学解求解器
	ChainFkSolverPos_recursive fksolver(chain);	//正运动学求解器
	ChainIkSolverVel_pinv iksolver1v(chain);	//逆运动学求解器
	ChainIkSolverPos_NR_JL iksolver(chain, fksolver, iksolver1v, 100, 1e-6);	//最大迭代次数100, 精度达到1e-6时停止迭代

	//正运动学求解
	KDL::ChainFkSolverPos_recursive fwdkin(chain);
	int n = chain.getNrOfJoints();

	//设置运动副极限值
	KDL::JntArray q_min(n);
	q_min.data[0] = -181 * PI / 180;
	q_min.data[1] = -151 * PI / 180;
	q_min.data[2] = -151 * PI / 180;
	q_min.data[3] = -180 * PI / 180;
	KDL::JntArray q_max(n);
	q_max.data[0] = 91 * PI / 180;
	q_max.data[1] = 151 * PI / 180;
	q_max.data[2] = 151 * PI / 180;
	q_max.data[3] = 180 * PI / 180;
	iksolver.setJointLimits(q_min, q_max);

	//创建运动副数组
	KDL::JntArray q_init(n);	//初始角度
	KDL::JntArray q_sol(n);		//解角度

	q_init.data[0] = joint_now[0] * PI / 180;
	q_init.data[1] = joint_now[1] * PI / 180;
	q_init.data[2] = joint_now[2] * PI / 180;
	q_init.data[3] = 0;

	KDL::Frame pos_now;	//目标位姿
	fwdkin.JntToCart(q_init, pos_now);	//暂时将现位姿赋予目标位姿

	std::pair<double, double> position_now;
	position_now.first = pos_now.p[0];
	position_now.second = pos_now.p[1];

	return position_now;
	
}

//运动学逆解，应用于视觉伺服过程中求解机械臂末端移动时目标锁定所需关节角度
//@param joint_now 现关节角度
//@param position	相机位置
//@param theta 解得关节角度（包括3关节电机，2云台电机），单位为rad
bool solve_visualservo_kdl(std::vector<double> joint_now, std::pair<double, double> position, std::vector<double>& theta)
{
	//建立运动学模型，长度单位m
	KDL::Chain chain;
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.7, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.5, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.542, 0))));	//包含第三关节臂0.4，云台0.142
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0))));	//设置一冗余自由度，使得可仅修改矩阵平移参数求解

	//建立正逆运动学解求解器
	ChainFkSolverPos_recursive fksolver(chain);	//正运动学求解器
	ChainIkSolverVel_pinv iksolver1v(chain);	//逆运动学求解器
	ChainIkSolverPos_NR_JL iksolver(chain, fksolver, iksolver1v, 100, 1e-6);	//最大迭代次数100, 精度达到1e-6时停止迭代

	//正运动学求解
	KDL::ChainFkSolverPos_recursive fwdkin(chain);
	int n = chain.getNrOfJoints();

	//设置运动副极限值
	KDL::JntArray q_min(n);
	q_min.data[0] = -181 * PI / 180;
	q_min.data[1] = -151 * PI / 180;
	q_min.data[2] = -151 * PI / 180;
	q_min.data[3] = -180 * PI / 180;
	KDL::JntArray q_max(n);
	q_max.data[0] = 91 * PI / 180;
	q_max.data[1] = 151 * PI / 180;
	q_max.data[2] = 151 * PI / 180;
	q_max.data[3] = 180 * PI / 180;
	iksolver.setJointLimits(q_min, q_max);

	//创建运动副数组
	KDL::JntArray q_init(n);	//初始角度
	KDL::JntArray q_sol(n);		//解角度

	q_init.data[0] = joint_now[0] * PI / 180;
	q_init.data[1] = joint_now[1] * PI / 180;
	q_init.data[2] = joint_now[2] * PI / 180;
	q_init.data[3] = 0;

	KDL::Frame pos_goal;	//目标位姿
	fwdkin.JntToCart(q_init, pos_goal);	//暂时将现位姿赋予目标位姿

	//根据position修改pos_goal平移参数
	pos_goal.p[0] = position.first;
	pos_goal.p[1] = position.second;

	if (!iksolver.CartToJnt(q_init, pos_goal, q_sol))	//求运动学逆解
	{
		//将解得角度赋予theta
		theta[0] = q_sol.data[0];
		theta[1] = q_sol.data[1];
		theta[2] = q_sol.data[2];
		theta[3] = joint_now[3] * PI / 180;
		theta[4] = joint_now[5] * PI / 180;
		theta[5] = joint_now[5] * PI / 180;

		return true;
	}
	else
	{
		return false;
	}
}

//路径规划，应用于手柄控制或目标锁定移动
//@param joint_now 现关节角度
//@param traj 初始轨迹（无时间戳）
//@param direction 机械臂末端移动方向
//@param is_focus 是否目标锁定
//@param focus_point 目标锁定点
//@param tar_pos 笛卡尔空间直线运动目标位置点
bool get_traj_kdl(std::vector<double> joint_now, std::vector<traj_point>& traj, Eigen::Vector2d direction, bool is_focus, Eigen::Vector3d focus_point, Eigen::Vector2d tar_pos)
{
	//建立运动学模型，长度单位m
	KDL::Chain chain;
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.7, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.5, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.542, 0))));	//包含第三关节臂0.4，云台0.142
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0))));	//设置一冗余自由度，使得可仅修改矩阵平移参数求解

	//建立正逆运动学解求解器
	ChainFkSolverPos_recursive fksolver(chain);	//正运动学求解器
	ChainIkSolverVel_pinv iksolver1v(chain);	//逆运动学求解器
	ChainIkSolverPos_NR_JL iksolver(chain, fksolver, iksolver1v, 100, 1e-6);	//最大迭代次数100, 精度达到1e-6时停止迭代

	//正运动学求解
	KDL::ChainFkSolverPos_recursive fwdkin(chain);
	int n = chain.getNrOfJoints();

	//设置运动副极限值
	KDL::JntArray q_min(n);
	q_min.data[0] = -181 * PI / 180;
	q_min.data[1] = -151 * PI / 180;
	q_min.data[2] = -151 * PI / 180;
	q_min.data[3] = -180 * PI / 180;
	KDL::JntArray q_max(n);
	q_max.data[0] = 91 * PI / 180;
	q_max.data[1] = 151 * PI / 180;
	q_max.data[2] = 151 * PI / 180;
	q_max.data[3] = 180 * PI / 180;
	iksolver.setJointLimits(q_min, q_max);

	//创建运动副数组
	KDL::JntArray q_init(n);	//初始角度
	KDL::JntArray q_sol(n);		//解角度

	q_init.data[0] = joint_now[0] * PI / 180;
	q_init.data[1] = joint_now[1] * PI / 180;
	q_init.data[2] = joint_now[2] * PI / 180;
	q_init.data[3] = 0;

	KDL::Frame pos_goal;	//目标位姿
	fwdkin.JntToCart(q_init, pos_goal);	//暂时将现位姿赋予目标位姿

	double step = line_step / 1000;	//平移步长5mm
	Eigen::Vector2d step_direc = Eigen::Vector2d(direction[0] * step, direction[1] * step);

	//根据direction修改pos_goal平移参数
	pos_goal.p[0] += step_direc[0];
	pos_goal.p[1] += step_direc[1];

	traj.clear();

	traj_point p;
	p.positions[0] = joint_now[0];
	p.positions[1] = joint_now[1];
	p.positions[2] = joint_now[2];
	p.positions[3] = joint_now[3];
	p.positions[4] = joint_now[4];

	traj.push_back(p);


	while (!iksolver.CartToJnt(q_init, pos_goal, q_sol))	//求运动学逆解
	{
		p.positions[0] = q_sol.data[0] * 180 / PI;
		p.positions[1] = q_sol.data[1] * 180 / PI;
		p.positions[2] = q_sol.data[2] * 180 / PI;

		if (is_focus)	//若处于目标锁定状态，则求解云台电机角度

		{
			Eigen::Vector3d z_axis = Eigen::Vector3d(pos_goal.p[0], pos_goal.p[1], pos_goal.p[2]) - focus_point;
			z_axis /= z_axis.norm();

			double alpha = q_sol.data[0] + q_sol.data[1] + q_sol.data[2];
			Eigen::AngleAxisd rotation_vector(-alpha, Eigen::Vector3d(0, 0, 1));
			z_axis = rotation_vector.matrix() * z_axis;

			p.positions[4] = asin(-z_axis[1]) * 180 / PI;
			p.positions[3] = atan(z_axis[0] / z_axis[2]) * 180 / PI;
		}

		q_init.data[0] = q_sol.data[0];
		q_init.data[1] = q_sol.data[1];
		q_init.data[2] = q_sol.data[2];
		q_init.data[3] = q_sol.data[3];

		if (ACM.collision(p.positions[0], p.positions[1], p.positions[2]))	//若发生自碰撞，break
		{
			break;
		}

		traj.push_back(p);

		fwdkin.JntToCart(q_sol, pos_goal);	//运动学正解，更新pos_goal（是否必要？）

		if (is_focus && MOVE_TYPE == MOVE_TYPE_FOCUS)
		{
			double temp_1 = pos_goal.p[0] - tar_pos[0];
			double temp_2 = pos_goal.p[1] - tar_pos[1];
			if (temp_1 * (temp_1 + step_direc[0]) < 0 || temp_2 * (temp_2 + step_direc[1]) < 0)
			{
				break;
			}
		}

		pos_goal.p[0] += step_direc[0];
		pos_goal.p[1] += step_direc[1];
	}

	if (traj.size() > 2)
	{
		return true;
	}
	else
	{
		return false;
	}
}
#pragma endregion

#pragma region 仿真验证函数
//机械臂打开——模拟
void arm_open_simulation()
{
	ARM_STATE = ARM_GET_TRAJ;


	joint_now[0] = -180;
	joint_now[1] = 150;
	joint_now[2] = -150;

	TAR.lock();

	joint_tar = { 0,0,0,0,0 };

	TAR.unlock();

	if (get_traj(joint_now, joint_tar, traj))
	{
		ARM_STATE = ARM_GET_TRAJ_SUCCESS;
	}
	else
	{
		ARM_STATE = ARM_FREE;
	}
}
//机械臂收拢——模拟
void arm_close_simulation()
{
	ARM_STATE = ARM_GET_TRAJ;


	joint_now[0] = 0;
	joint_now[1] = 0;
	joint_now[2] = 0;

	TAR.lock();

	joint_tar = { -180,150,-150,0,0 };

	TAR.unlock();

	if (get_traj(joint_now, joint_tar, traj))
	{
		ARM_STATE = ARM_GET_TRAJ_SUCCESS;
	}
	else
	{
		ARM_STATE = ARM_FREE;
	}
}
//轨迹跟随函数——模拟
void follow_traj_simulation(std::vector<traj_point> traj)
{
	ARM_STATE = ARM_FOLLOW_TRAJ;

	double angle_1_s;
	double angle_2_s;
	double angle_3_s;

	for (int i = 0; i < traj.size(); i++)
	{
		//cout << "the "<<i+1<<"th point"<< endl;
		if (i == 0)
		{
			angle_1_s = joint_now[0];
			angle_2_s = joint_now[1];
			angle_3_s = joint_now[2];
		}
		else
		{
			angle_1_s = traj[i - 1].positions[0];
			angle_2_s = traj[i - 1].positions[1];
			angle_3_s = traj[i - 1].positions[2];
		}

		double max_angle = max(abs(traj[i].positions[0] - angle_1_s), abs(traj[i].positions[1] - angle_2_s));
		max_angle = max(max_angle, abs(traj[i].positions[2] - angle_3_s));

		double time_from_pre = max_angle / 18;

		if (time_from_pre < 0.01)
		{
			if (ARM_STATE < ARM_FOLLOW_TRAJ)
			{
				cout << "following traj has been interuptted" << endl;
				return;
			}

			joint_now[0] = traj[i].positions[0];
			joint_now[1] = traj[i].positions[1];
			joint_now[2] = traj[i].positions[2];

			ACM_show.arm_set(traj[i].positions[0], traj[i].positions[1], traj[i].positions[2]);

			Sleep(10);
		}
		else
		{
			int step_num = int(time_from_pre / 0.01);
			double angle_1_step = (traj[i].positions[0] - angle_1_s) / step_num;
			double angle_2_step = (traj[i].positions[1] - angle_2_s) / step_num;
			double angle_3_step = (traj[i].positions[2] - angle_3_s) / step_num;

			double angle_1_tar = angle_1_s;
			double angle_2_tar = angle_2_s;
			double angle_3_tar = angle_3_s;

			for (int i = 0; i < step_num; i++)
			{
				angle_1_tar += angle_1_step;
				angle_2_tar += angle_2_step;
				angle_3_tar += angle_3_step;

				if (ARM_STATE < ARM_FOLLOW_TRAJ)
				{
					cout << "following traj has been interuptted" << endl;
					return;
				}

				joint_now[0] = angle_1_tar;
				joint_now[1] = angle_2_tar;
				joint_now[2] = angle_3_tar;

				ACM_show.arm_set(angle_1_tar, angle_2_tar, angle_3_tar);

				Sleep(10);
			}
		}
	}
	ARM_STATE = ARM_FREE;
	cout << "finishing" << endl;
}
//机械臂工作函数——模拟
void motor_work_simulation()
{
	while (true)
	{
		if (ARM_STATE == ARM_GET_TRAJ_SUCCESS)
		{
			follow_traj_simulation(traj);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}
#pragma endregion

#pragma region 轨迹插值函数
//直线插值
void traj_to_interpolation()
{
	std::ofstream f_log("traj.txt");

	double speed_max;

	//设置最大速度
	if (MOVE_TYPE == MOVE_TYPE_DEBUG)
	{
		speed_max = speed_max_debug;
	}
	else
	{
		speed_max = SPEED_MAX;
	}

	traj[0].time_from_start = 0;
	int traj_size = traj.size();

	//计算初始轨迹时间戳
	for (int i = 1; i < traj_size; i++)
	{
		double max_angle = max(max(abs(traj[i].positions[0] - traj[i - 1].positions[0]), abs(traj[i].positions[1] - traj[i - 1].positions[1])),
			abs(traj[i].positions[2] - traj[i - 1].positions[2]));

		if (MOVE_TYPE == MOVE_TYPE_JOYSTICK || MOVE_TYPE == MOVE_TYPE_FOCUS)
		{
			//机械臂运动类型为手柄控制或目标锁定时，轨迹为笛卡尔轨迹，个点间隔mm级，需进行限制时间
			traj[i].time_from_start = traj[i - 1].time_from_start + std::max(max_angle / speed_max, 0.1);
		}
		else
		{
			//关节空间轨迹可直接由各关节最大运动角度和最大速度计算得到
			traj[i].time_from_start = traj[i - 1].time_from_start + max_angle / speed_max;
		}
	}


	traj_interpolation.resize(0);

	traj_interpolation.push_back(traj[0]);

	double time_from_pre = 0;
	int step_num = 0;
	double angle_1_step = 0;
	double angle_2_step = 0;
	double angle_3_step = 0;
	double angle_4_step = 0;
	double angle_5_step = 0;
	traj_point point_temp;

	//直线插值
	for (int i = 1; i < traj.size(); i++)
	{
		double time_from_pre = traj[i].time_from_start - traj[i - 1].time_from_start;
		if (time_from_pre < TRAJ_TIME_STEP)
		{
			traj_interpolation.push_back(traj[i]);
			f_log << traj[i].positions[0] << " " << traj[i].positions[1] << " " << traj[i].positions[2] << "," << endl;
		}
		else
		{
			step_num = int(time_from_pre / TRAJ_TIME_STEP);
			angle_1_step = (traj[i].positions[0] - traj[i - 1].positions[0]) / step_num;
			angle_2_step = (traj[i].positions[1] - traj[i - 1].positions[1]) / step_num;
			angle_3_step = (traj[i].positions[2] - traj[i - 1].positions[2]) / step_num;
			angle_4_step = (traj[i].positions[3] - traj[i - 1].positions[3]) / step_num;
			angle_5_step = (traj[i].positions[4] - traj[i - 1].positions[4]) / step_num;

			point_temp.positions[0] = traj[i - 1].positions[0];
			point_temp.positions[1] = traj[i - 1].positions[1];
			point_temp.positions[2] = traj[i - 1].positions[2];
			point_temp.positions[3] = traj[i - 1].positions[3];
			point_temp.positions[4] = traj[i - 1].positions[4];

			for (int i = 0; i < step_num; i++)
			{
				point_temp.positions[0] += angle_1_step;
				point_temp.positions[1] += angle_2_step;
				point_temp.positions[2] += angle_3_step;
				point_temp.positions[3] += angle_4_step;
				point_temp.positions[4] += angle_5_step;

				traj_interpolation.push_back(point_temp);
				f_log << point_temp.positions[0] << " " << point_temp.positions[1] << " " << point_temp.positions[2] << "," << endl;
			}
		}
	}
}

//笛卡尔空间直线插值
//@param v_max 最大速度	mm/s
//@param a_max 最大加速度 mm/s2
//@param line_step 直线离散点步长 mm/s2
void traj_to_cartesian_line(double v_max, double a_max, double line_step)
{
	double t_step = TRAJ_TIME_STEP;
	//double t_step = 0.05;

	std::ofstream f_log("traj.txt");

	traj_interpolation.resize(0);

	double point_num = traj.size();	//插值前轨迹点数量，即直线被离散为多少个已line_step（全局变量）为间隔的点

	double step_num = pow(v_max, 2) / a_max / line_step / 2;	//若存在匀速段，则经过step_num个间隔进入匀速段，即加速段需step_num个间隔

	double t_acc;	//加速段时间，进入匀速段时间
	double t_dec;	//进入减速段时刻
	double t_all;	//总时间

	if (point_num - 1 > step_num * 2)	//若存在匀速段
	{
		t_acc = v_max / a_max;
		t_all = t_acc * 2 + (point_num - 1 - step_num * 2) * line_step / v_max;
		t_dec = t_all - t_acc;
	}
	else
	{
		t_acc = sqrt((point_num - 1) / 2 * line_step / a_max);
		t_dec = t_acc;
		t_all = t_acc * 2;
	}

	//创建 t = 0:t_acc 至 ind = 0->point_num 的映射
	double ind_acc = min(step_num, (point_num - 1) / 2);
	double a_ind = 2 * ind_acc / pow(t_acc, 2);	//索引加速度
	double v_ind = line_step / v_max;	//索引匀速段速度

	double t = 0;
	double ind_temp;	//临时索引
	traj_point point_temp;	//临时轨迹点

	while (t < t_all - t_step)	//时间未达到总运动时间
	{
		if (t < t_acc)	//加速段
		{
			ind_temp = a_ind * pow(t, 2) / 2;
		}
		else if (t < t_dec)	//匀速段
		{
			ind_temp = ind_acc + (t - t_acc) / v_ind;
		}
		else
		{
			//减速段
			ind_temp = point_num - 1 - 0.5 * a_ind * pow((t_all - t), 2);
		}

		int ind_temp_floor = int(ind_temp);	//左轨迹点索引
		double alpha = ind_temp - ind_temp_floor;	//两轨迹点间比例

		for (int i = 0; i < 5; i++)
		{
			point_temp.positions[i] = traj[ind_temp_floor].positions[i] * (1 - alpha) + traj[ind_temp_floor + 1].positions[i] * alpha;
		}

		t += t_step;

		traj_interpolation.push_back(point_temp);
	}

	traj_interpolation.push_back(traj.back());
}

//梯形速度轨迹插值辅助函数：获取中间数据
//@param traj 带时间戳的初始轨迹
//@param t 各轨迹点时间
//@param v_m 各轨迹段速度
//@param a_m 各轨迹段加速度
//@param v_max 最大速度
//@param a_max 最大加速度
void get_data_trapezoid(std::vector<traj_point>& traj, std::vector<double>& t, std::vector<std::vector<double>>& v_m,
	std::vector<std::vector<double>>& a_m, double v_max, double a_max, std::vector<int>& max_ind)
{
	int traj_size = traj.size();
	for (int i = 1; i < traj_size; i++)
	{
		//获取各关节最大运动角度与关节编号
		double angle_max = 0;
		int ind = -1;
		std::vector<double> angles_temp(3);
		for (int j = 0; j < 3; j++)
		{
			angles_temp[j] = traj[i].positions[j] - traj[i - 1].positions[j];
			double angle_temp = abs(angles_temp[j]);

			if (angle_temp > angle_max)
			{
				angle_max = angle_temp;
				ind = j;
			}
		}

		max_ind[i] = ind;

		//根据运动角度最大的关节，获取最短运动时间，速度，加速度
		double angle_tri = pow(v_max, 2) / a_max;

		if (angle_tri <= angle_max)	//若加至最大速度
		{
			v_m[i][ind] = v_max * (angles_temp[ind] < 0 ? -1 : 1);
			a_m[i][ind] = a_max * (angles_temp[ind] < 0 ? -1 : 1);

			t[i] = t[i - 1] + 2 * v_max / a_max + (angle_max - angle_tri) / v_max;
			traj[i].time_from_start = t[i];
		}
		else	//若未加至最大速度
		{
			v_m[i][ind] = sqrt(a_max * angle_max) * (angles_temp[ind] < 0 ? -1 : 1);
			a_m[i][ind] = a_max * (angles_temp[ind] < 0 ? -1 : 1);

			t[i] = t[i - 1] + 2 * abs(v_m[i][ind]) / a_max;
			traj[i].time_from_start = t[i];
		}

		//按比例计算其他关节速度与加速度
		for (int j = 0; j < 3; j++)
		{
			if (j == ind)
			{
				continue;
			}

			double ratio = angles_temp[j] / angles_temp[ind];
			v_m[i][j] = v_m[i][ind] * ratio;
			a_m[i][j] = a_m[i][ind] * ratio;
		}
	}
}

//梯形速度轨迹插值辅助函数，插值操作
//@param traj_interpolation 插值轨迹
//@param traj 带时间戳的初始轨迹
//@param t 各轨迹点时间
//@param v_m 各轨迹段速度
//@param a_m 各轨迹段加速度
//@param time_step 时间步长
//@param max_ind 运动角度最大的关节索引
void traj_to_interpolation_trapezoid_helper(std::vector<traj_point>& traj_interpolation, std::vector<traj_point>& traj,
	std::vector<double>& t, std::vector<std::vector<double>>& v_m, std::vector<std::vector<double>>& a_m, double time_step,
	std::vector<int>& max_ind)
{
	int traj_size = traj.size();

	for (int i = 1; i < traj_size; i++)
	{
		double t_1 = t[i - 1] + abs(v_m[i][max_ind[i]] / a_m[i][max_ind[i]]);	//由加速段进入匀速段时刻
		double t_2 = t[i] - abs(v_m[i][max_ind[i]] / a_m[i][max_ind[i]]);		//由匀速段进入减速段时刻

		traj_point mp_temp;

		std::vector<int> ind_list;

		for (int j = 0; j < 3; j++)
		{
			if (abs(traj[i - 1].positions[j] - traj[i].positions[j]) > 0.0001)
			{
				ind_list.push_back(j);
			}
		}

		std::vector<double> angle_sm(3);	//由加速段进入匀速段角度
		std::vector<double> angle_em(3);	//由匀速段进入减速段角度

		for (int j : ind_list)
		{
			angle_sm[j] = traj[i - 1].positions[j] + (pow(v_m[i][j], 2) / a_m[i][j] / 2);
			angle_em[j] = angle_sm[j] + (t_2 - t_1) * v_m[i][j];
		}

		double t_now = t[i - 1];

		while (t_now < t[i])
		{
			for (int j = 0; j < 5; j++)
			{
				mp_temp.positions[j] = traj[i - 1].positions[j];
			}

			if (t_now < t_1)	//加速段
			{
				for (int j : ind_list)
				{
					mp_temp.positions[j] = traj[i - 1].positions[j] + a_m[i][j] * pow(t_now - t[i - 1], 2) / 2;
				}
			}
			else if (t_now < t_2)
			{
				for (int j : ind_list)
				{
					mp_temp.positions[j] = angle_sm[j] + (t_now - t_1) * v_m[i][j];
				}
			}
			else
			{
				for (int j : ind_list)
				{
					mp_temp.positions[j] = traj[i].positions[j] - a_m[i][j] * pow(t_now - t[i], 2) / 2;
				}
			}
			traj_interpolation.push_back(mp_temp);

			t_now += time_step;
		}
	}
}

//梯形速度轨迹插值
void traj_to_interpolation_trapezoid()
{
	std::ofstream f_log("traj.txt");

	double speed_max;

	if (MOVE_TYPE == MOVE_TYPE_DEBUG)
	{
		speed_max = speed_max_debug;
	}
	else
	{
		speed_max = SPEED_MAX;
	}

	traj[0].time_from_start = 0;
	int traj_size = traj.size();

	std::vector<double>t(traj_size);	//各轨迹点时间
	std::vector<std::vector<double>>v_m(traj_size, std::vector<double>(3));	//各轨迹段速度
	std::vector<std::vector<double>>a_m(traj_size, std::vector<double>(3));	//各轨迹段加速度
	std::vector<int>max_ind(traj_size);

	//获取中间数据
	get_data_trapezoid(traj, t, v_m, a_m, speed_max, a_max, max_ind);
	traj_interpolation.resize(0);	//插值轨迹清零

	//插值操作
	traj_to_interpolation_trapezoid_helper(traj_interpolation, traj, t, v_m, a_m, TRAJ_TIME_STEP, max_ind);
}

//S加速度轨迹插值
void traj_to_interpolation_S_acc()
{
	std::ofstream f_log("traj.txt");

	double speed_max;

	if (MOVE_TYPE == MOVE_TYPE_DEBUG)
	{
		speed_max = speed_max_debug;
	}
	else
	{
		speed_max = SPEED_MAX;
	}

	traj_interpolation.clear();	//插值轨迹清零

	int traj_size = traj.size();

	double j_max = 2;

	traj_interpolation.push_back(traj[0]);	//插入起点

	for (int i = 0; i < traj_size - 1; i++)
	{
		traj_to_interpolation_S_acc_helper(traj_interpolation, traj[i], traj[i + 1], j_max, a_max, speed_max);	//中间各段路径轨迹插值
	}

	traj_interpolation.push_back(traj.back());	//插入终点
}

//S加速度轨迹插值辅助函数，插值操作
//@param traj_interpolation 插值轨迹
//@param start 起始轨迹点
//@param end 终止轨迹点
//@param J 加加速度限制值
//@param A 加速度限制值
//@param V 速度限制值
void traj_to_interpolation_S_acc_helper(std::vector<traj_point>& traj_interpolation, traj_point start, traj_point end, double J, double A, double V)
{
	std::ofstream f_log("traj.txt");

	//最大角度偏移绝对值
	double L = max(abs(start.positions[0] - end.positions[0]), max(abs(start.positions[1] - end.positions[1]), abs(start.positions[2] - end.positions[2])));

	if (L == 0)	//判断最大角度偏移值是否为零
	{
		traj_interpolation.push_back(end);	//插入终点并返回
		return;
	}

	std::vector<double> ratio(3);	//关节比例系数

	for (int i = 0; i < 3; i++)
	{
		ratio[i] = (end.positions[i] - start.positions[i]) / L;	//关节实际偏移角度 / 最大偏移角度绝对值
	}

	double Sref = 2 * pow(A, 3) / pow(J, 2);	//辅助，恰好达到最大加速度所需路程

	std::vector<double> T(7);	//各段时间
	std::vector<double> T_time(7);	//各段结束时刻

	if (L <= Sref)	//判断路程是否满足可达限制加速度，若否
	{
		T[0] = pow(L / 2 / J, 1.0 / 3);
		double v_max = J * pow(T[0], 2);	//计算可达最大速度

		if (v_max < V)	//若最大速度小于限制速度，三段
		{
			T[1] = 0;
			T[2] = T[0];
			T[3] = 0;
			T[4] = T[0];
			T[5] = 0;
			T[6] = T[0];
		}
		else
		{
			//若最大速度大于限制速度，五段，有匀速段
			T[0] = sqrt(V / J);
			double Sa = J * pow(T[0], 3);	//加速段路程

			T[1] = 0;
			T[2] = T[0];
			T[3] = (L - 2 * Sa) / V;
			T[4] = T[0];
			T[5] = 0;
			T[6] = T[0];
		}
	}
	else	//若路程满足可达限制加速度
	{
		T[0] = pow(L / 2 / J, 1.0 / 3);
		double v_max = J * pow(T[0], 2);	//可达最大速度

		if (v_max >= V)	//若最大速度大于限制速度，五段，有匀速段
		{
			T[0] = sqrt(V / J);
			double Sa = J * pow(T[0], 3);

			T[1] = 0;
			T[2] = T[0];
			T[3] = (L - 2 * Sa) / V;
			T[4] = T[0];
			T[5] = 0;
			T[6] = T[0];
		}
		else
		{
			//若最大速度小于限制速度，七段，有匀速段和匀加速段
			T[0] = A / J;
			T[1] = V / A - T[0];
			double S_ref2 = V * (2 * T[0] + T[1]);

			T[2] = T[0];
			T[4] = T[0];
			T[5] = T[1];
			T[6] = T[0];

			if (L < S_ref2)
			{
				v_max = -pow(A, 2) / 2 / J + sqrt(pow(A, 4) + 4 * A * pow(J, 2) * L) / (2 * J);
				T[1] = v_max / A - T[0];
				T[5] = v_max / A - T[4];
				T[3] = 0;
			}

			else
			{
				T[3] = (L - S_ref2) / V;
			}
		}
	}

	T_time[0] = T[0];
	//计算各段结束时刻
	for (int i = 1; i < 7; i++)
	{
		T_time[i] = T_time[i - 1] + T[i];
	}

	double	t = 0;

	double t_step = TRAJ_TIME_STEP;

	//traj_s = zeros(ceil(T(7) / t_step), 1);
	//traj_v = zeros(ceil(T(7) / t_step), 1);
	//traj_a = zeros(ceil(T(7) / t_step), 1);
	//traj_j = zeros(ceil(T(7) / t_step), 1);

	std::vector<double> v(7);
	std::vector<double> S(7);

	double a_max = J * T[0];

	//计算各段结束时刻速度
	v[0] = J * pow(T[0], 2) / 2;
	v[1] = v[0] + a_max * T[1];
	v[2] = v[1] + J * pow(T[0], 2) / 2;
	v[3] = v[2];
	v[4] = v[1];
	v[5] = v[0];
	v[6] = 0;

	//计算各段结束时刻位置
	S[0] = J * pow(T[0], 3) / 6;
	S[1] = S[0] + v[0] * T[1] + J * T[0] * pow(T[1], 2) / 2;
	S[2] = S[1] + v[1] * T[0] + J * pow(T[0], 3) / 3;
	S[3] = S[2] + v[2] * T[3];
	S[4] = S[3] + v[3] * T[4] - J * pow(T[4], 3) / 6;
	S[5] = S[4] + v[4] * T[5] - J * T[4] * pow(T[5], 2) / 2;
	S[6] = S[5] + v[5] * T[6] - J * pow(T[4], 3) / 3;

	double s_val;	//当前位置
	double v_val;	//当前速度
	double a_val;	//当前加速度
	double j_val;	//当前加加速度

	traj_point point_temp;

	while (t < T_time[6])
	{
		if (t < T_time[0])	//加加速度按
		{
			s_val = J * pow(t, 3) / 6;
			v_val = J * pow(t, 2) / 2;
			a_val = J * t;
			j_val = J;
		}
		else if (t < T_time[1])	//匀加速段
		{
			double t_temp = t - T_time[0];
			s_val = S[0] + v[0] * t_temp + J * T[0] * pow(t_temp, 2) / 2;
			v_val = v[0] + a_max * t_temp;
			a_val = a_max;
			j_val = 0;
		}
		else if (t < T_time[2])	//减加速段
		{
			double t_temp = t - T_time[1];
			s_val = S[1] + v[1] * t_temp + J * T[0] * pow(t_temp, 2) / 2 - J * pow(t_temp, 3) / 6;
			v_val = v[1] + a_max * t_temp - J * pow(t_temp, 2) / 2;
			a_val = a_max - J * t_temp;
			j_val = -J;
		}
		else if (t < T_time[3])	//匀速段
		{
			double t_temp = t - T_time[2];
			s_val = S[2] + v[2] * t_temp;
			v_val = v[2];
			a_val = 0;
			j_val = 0;
		}
		else if (t < T_time[4])	//加减速段
		{
			double  t_temp = t - T_time[3];
			s_val = S[3] + v[3] * t_temp - J * pow(t_temp, 3) / 6;
			v_val = v[3] - J * pow(t_temp, 2) / 2;
			a_val = -J * t_temp;
			j_val = -J;
		}
		else if (t < T_time[5])	//匀减速段
		{
			double t_temp = t - T_time[4];
			s_val = S[4] + v[4] * t_temp - J * T[4] * pow(t_temp, 2) / 2;
			v_val = v[4] - a_max * t_temp;
			a_val = -A;
			j_val = 0;
		}
		else
		{
			//减减速段
			double t_temp = t - T_time[5];
			s_val = S[5] + v[5] * t_temp - J * T[4] * pow(t_temp, 2) / 2 + J * pow(t_temp, 3) / 6;
			v_val = v[5] - a_max * t_temp + J * pow(t_temp, 2) / 2;
			a_val = -a_max + J * t_temp;
			j_val = J;
		}

		t += t_step;

		for (int i = 0; i < 3; i++)
		{
			point_temp.positions[i] = start.positions[i] + ratio[i] * s_val;	//根据位置与比例系数计算各关节角度
		}

		//f_log << s_val << endl;

		traj_interpolation.push_back(point_temp);	//插入轨迹点
	}
}
#pragma endregion