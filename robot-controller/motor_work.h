/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of motor_work

**************************************************************************/

#pragma once

#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <iostream>

#include "serial/serial.h"
#include "camera.h"
#include "motor.h"
#include "opencv2/opencv.hpp"


//静态RRT规划库
#include "rrt.h"
#include "rrt_star.h"
#include "rrt_connect.h"
#include "rrt_connect_star.h"
#include "Q_rrt_star.h"
//动态RRT规划库
#include "D_rrt.h"
#include "D_rrt_star.h"
#include "D_rrt_connect.h"
#include "D_rrt_connect_star.h"
#include "MP_rrt.h"
#include "MP_rrt_star.h"
#include "MP_rrt_connect.h"
#include "MP_rrt_connect_star.h"

#include "collision.h"

//KDL库
#include "chainiksolverpos_nr_jl.hpp"
#include "chainfksolverpos_recursive.hpp"
#include "chainiksolvervel_pinv.hpp"

#include "Eigen/Dense"

#include "ssbot_config.h"

#pragma region 宏定义变量
//机械臂状态
#define ARM_FREE 0	//机械臂空闲状态
#define ARM_GET_TRAJ 1	//机械臂正在规划路径
#define ARM_GET_TRAJ_SUCCESS 2	//机械臂路径规划成功
#define ARM_FOLLOW_TRAJ 3	//机械臂正在跟随路径

//机械臂单关节调试状态
#define DEBUG_POSITIVE 0	//机械臂关节正传
#define DEBUG_NEGATIVE 1	//机械臂关节反传

//机械臂运动类型
#define MOVE_TYPE_TRAJ 0	//机械臂避障运动
#define MOVE_TYPE_DEBUG 1	//机械臂调试运动
#define MOVE_TYPE_FOCUS 3	//机械臂目标锁定运动
#define MOVE_TYPE_JOYSTICK 2	//机械臂手柄控制运动

//现执行轨迹状态
#define RRT_SOLVER_STATE_OLD_PATH 0	//原有路径
#define RRT_SOLVER_STATE_NEW_PATH 1	//新路径
#pragma endregion

using std::cout;
using std::endl;
using std::thread;
using std::max;
using std::min;

#pragma region 机械臂顶层控制调用函数
bool arm_set_param();	//机械臂设置参数（初始化设置参数）
bool arm_free();	//机械臂释放（设置为电流模式，电流为0，可手动调整机械臂）
bool arm_init_mode();	//机械臂设置控制模式（设置为位置模式）
void arm_disable();	//机械臂失能
void arm_stop();	//机械臂急停
bool arm_position_test();	//检查此刻机械臂位置是否在合法范围
void arm_open();	//机械臂打开
void arm_close();	//机械臂收拢
void arm_up();		//机械臂向前移动
void arm_down();	//机械臂向后移动
void arm_right();	//机械臂向右移动
void arm_left();	//机械臂向左移动
void arm_direction(double direction_x, double direction_y); //机械臂向指定方向移动
void arm_debug(int number, int direction);	//机械臂调试运动
#pragma endregion

#pragma region 机械臂底层伺服及规划函数
void motor_work();	//机械臂工作函数
void motor_auto();  //机械臂演示自动运动函数
bool get_traj(std::vector<double> joint_now, std::vector<double> joint_tar, std::vector<traj_point>& traj);	//规划避障路径
void follow_traj();	//轨迹跟随函数
void follow_traj_thread_fun();	//机械臂轨迹跟随线程函数
void rrt_solver_work_thread_fun();	//机械臂路径规划（RRT）线程函数，仅应用于动态路径规划，使用静态路径规划器时应将其注释
Eigen::VectorXd calculate_weight(std::vector<double> joint_now, std::vector<double> joint_tar);	//计算机械臂各关节权重
#pragma endregion

#pragma region 机械臂轨迹插值函数
void traj_to_interpolation();	//直线插值
void traj_to_cartesian_line(double v_max, double a_max, double line_step);	//笛卡尔空间直线插值
void traj_to_interpolation_trapezoid();	//梯形速度轨迹插值
void get_data_trapezoid(std::vector<traj_point>& traj, std::vector<double>& t, std::vector<std::vector<double>>& v_m,
	std::vector<std::vector<double>>& a_m, double v_max, double a_max, std::vector<int>& max_ind);	//梯形速度轨迹插值辅助函数：获取中间数据
void traj_to_interpolation_trapezoid_helper(std::vector<traj_point>& traj_interpolation, std::vector<traj_point>& traj,
	std::vector<double>& t, std::vector<std::vector<double>>& v_m, std::vector<std::vector<double>>& a_m, double time_step,
	std::vector<int>& max_ind);	//梯形速度轨迹插值辅助函数，插值操作
void traj_to_interpolation_S_acc();	//S加速度轨迹插值
void traj_to_interpolation_S_acc_helper(std::vector<traj_point>& traj_interpolation, traj_point start, traj_point end, double J, double A, double V);	//S加速度轨迹插值辅助函数，插值操作
#pragma endregion

#pragma region 机械臂路径规划相关函数
void rrt_init();	//rrt路径规划器相关变量初始化
bool isStateValid_obstacle(Eigen::VectorXd v_x);	//碰撞检测函数：针对全部障碍物
bool isStateValid_obstacle_new(Eigen::VectorXd v_x);	//碰撞检测函数：针对新增障碍物

bool kdl_ik(double angle_1, double angle_2, double angle_3, double angle_4, double angle_5, Eigen::Vector3d position_tar, Eigen::Vector3d z_tar);//运动学逆解，应用于求解目标拍摄位置对应的机器人关节角度
bool YOLO_kdl_ik(std::vector<double> joint_now, std::pair<double, double> position_tar, Eigen::Vector3d focus_point);
bool get_traj_kdl(std::vector<double> joint_now, std::vector<traj_point>& traj, Eigen::Vector2d direction,
	bool is_focus, Eigen::Vector3d focus_point, Eigen::Vector2d tar_pos = Eigen::Vector2d(0, 0));//运动学逆解，应用于遮挡规避时求解机械臂末端移动时目标锁定所需关节角度
bool solve_focus_kdl(std::vector<double> joint_now, std::pair<double, double> position, Eigen::Vector3d center_img,
	std::vector<double>& theta);//路径规划，应用于手柄控制或目标锁定移动
std::pair<double, double> solve_visualservo_kdl(std::vector<double> joint_now); //视觉伺服获取机械臂末端当前位置
bool solve_visualservo_kdl(std::vector<double> joint_now, std::pair<double, double> position, std::vector<double>& theta);  //视觉伺服过程求解逆解
#pragma endregion

#pragma region 仿真验证函数
void follow_traj_simulation(std::vector<traj_point> traj);	//机械臂打开——模拟
void motor_work_simulation();	//机械臂收拢——模拟
void arm_open_simulation();		//轨迹跟随函数——模拟
void arm_close_simulation();	//机械臂工作函数——模拟
#pragma endregion