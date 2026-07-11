/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of lidar scan

**************************************************************************/

#include "lidar.h"
#include <time.h>

#pragma region 全局变量

extern int MOVE_TYPE;	//机械臂运动类型

int LIDAR_STATE = -1;	//全局激光雷达状态

int ARM_OBSTACLE_STOP = 0;  //机械臂遇障碍物终止运动标志

std::mutex Lidar_Point;	//全局互斥锁——激光雷达

vector<cv::Point2d> Lidar_point;	//激光雷达vector

std::vector<cv::RotatedRect> obstacles;	//QTM获得旋转矩形作为障碍物
std::vector<cv::RotatedRect> obstacles_new;	//QTM获得旋转矩形作为障碍物

std::mutex obstacles_new_mutex;


ILidarDriver* lidar1;	//激光雷达——上
ILidarDriver* lidar2;	//激光雷达——下

int max_fail_time = 10;
#pragma endregion

#pragma region 外部变量
extern std::map<std::string, std::string> parameters;

extern motor M;

extern std::mutex QTM_mutex;
extern Q_Tree_Map QTM;

extern int ARM_STATE;
extern arm_collision_model ACM_show;

extern int beepState;
#pragma endregion

extern void udp_send_debug_info(int16_t error_index);	//UDP发送函数，向上位机发送机器人调试信息

//激光雷达扫描程序
//void lidar_scan()
//{
//	//设置QTM参数
//	QTM.set_min_points_num(4);
//	QTM.set_decay_speed(1);
//	QTM.set_max_points_num(15);
//
//	rplidar_response_measurement_node_t* nodes_u;
//	rplidar_response_measurement_node_t* nodes_d;
//	nodes_u = (rplidar_response_measurement_node_t*)malloc(sizeof(rplidar_response_measurement_node_t) * 2000);
//	nodes_d = (rplidar_response_measurement_node_t*)malloc(sizeof(rplidar_response_measurement_node_t) * 2000);
//	size_t   count_u = 2000;
//	size_t   count_d = 2000;
//
//	while (true)
//	{
//		//连接激光雷达
//		bool connect_lidar_u = (lidar1.RP->connect(parameters["serial_lidar_u"].c_str(), 256000) == RESULT_OK);
//		int connect_lidar_u_time = 0;
//		while (!connect_lidar_u&& connect_lidar_u_time<10)
//		{
//			connect_lidar_u_time++;
//			connect_lidar_u = (lidar1.RP->connect(parameters["serial_lidar_u"].c_str(), 256000) == RESULT_OK);
//			cout << "上激光雷达重连" << connect_lidar_u_time <<"次" << endl;
//			std::this_thread::sleep_for(std::chrono::milliseconds(300));
//		}
//		if (!connect_lidar_u)
//		{
//			cout << "上激光雷达连接失败" << endl;
//		}
//		else
//		{
//			cout << "上激光雷达连接成功" << endl;
//		}
//		bool connect_lidar_d = (lidar2.RP->connect(parameters["serial_lidar_d"].c_str(), 256000) == RESULT_OK);
//		int connect_lidar_d_time = 0;
//		while (!connect_lidar_d && connect_lidar_d_time < 10)
//		{
//			connect_lidar_d_time++;
//			connect_lidar_d = (lidar2.RP->connect(parameters["serial_lidar_d"].c_str(), 256000) == RESULT_OK);
//			cout << "下激光雷达重连" << connect_lidar_d_time << "次" << endl;
//			std::this_thread::sleep_for(std::chrono::milliseconds(300));
//		}
//		if (!connect_lidar_d)
//		{
//			cout << "下激光雷达连接失败" << endl;
//		}
//		else
//		{
//			cout << "下激光雷达连接成功" << endl;
//		}
//
//		if (connect_lidar_u && connect_lidar_d)
//		{
//			clock_t start_u, end_u, start_d, end_d;
//
//			cout << "lidar success" << endl;
//			lidar1.RP->startMotor();
//			lidar2.RP->startMotor();
//
//			lidar1.RP->startScan(0, 1);
//			lidar2.RP->startScan(0, 1);
//
//			u_result op_result_u;
//			u_result op_result_d;
//
//			LIDAR_STATE = LIDAR_WORK;
//
//			while (true)
//			{
//				//判断雷达工作状态
//				if (LIDAR_STATE == LIDAR_WORK)
//				{
//					//获取关节1角度与激光雷达数据
//					double ANGLE_1_u = M.getAngle(FIRST_MOTOR_CAN);
//
//					start_u = clock();
//					op_result_u = lidar1.RP->grabScanData(nodes_u, count_u);//引用了nodes和count 所以其实已经改变了count的值
//					end_u = clock();
//					double ANGLE_1_d = M.getAngle(FIRST_MOTOR_CAN) + 3;
//					start_d = clock();
//					op_result_d = lidar2.RP->grabScanData(nodes_d, count_d);//引用了nodes和count 所以其实已经改变了count的值
//					end_d = clock();
//
//					if (end_u - start_u > 100)
//					{
//						max_fail_time--;
//						cout << "上雷达单次检测时间超100ms  时间为" << end_u - start_u << "ms" << endl;
//					}
//					if (end_d - start_d > 100)
//					{
//						max_fail_time--;
//						cout << "下雷达单次检测时间超100ms  时间为" << end_d - start_d << "ms" << endl;
//					}
//
//					if (max_fail_time < 0)
//					{
//						cout << "因数据读取卡顿，激光雷达重启" << endl;
//
//						QTM_mutex.lock();
//						Lidar_point.clear();
//						QTM.free();
//						obstacles.clear();
//						obstacles_new.clear();
//						QTM_mutex.unlock();
//
//						lidar1.RP->stopMotor();
//						lidar1.RP->stop();
//
//						lidar2.RP->stopMotor();
//						lidar2.RP->stop();
//
//						lidar1.RP->disconnect();
//						lidar2.RP->disconnect();
//
//						break;
//					}
//
//					if (IS_OK(op_result_u) && IS_OK(op_result_d))
//					{
//						//激光雷达数据排列
//						lidar1.RP->ascendScanData(nodes_u, count_u);
//						lidar2.RP->ascendScanData(nodes_d, count_d);
//
//						lidar_point point_temp;
//						double count_step_u = count_u / 360.0;
//						double count_step_d = count_d / 360.0;
//
//						Lidar_Point.lock();
//						Lidar_point.clear();
//
//						cv::Point2d cv_point_temp;
//
//						//激光雷达——上 采样点坐标解算
//						for (int i = 0; i < count_u; i = (int)(i + count_step_u))
//						{
//							point_temp.dis = nodes_u[i].distance_q2 / 4.0f;
//							point_temp.angle = (nodes_u[i].angle_q6_checkbit & 0xFFFF) / (1 << 7);
//
//							if (point_temp.dis > 50 && point_temp.dis < 2000)
//							{
//								double x, y;
//								x = point_temp.dis * cos((270.0 - point_temp.angle) / 57.29578);
//								y = point_temp.dis * sin((270.0 - point_temp.angle) / 57.29578) + 170;
//
//								cv_point_temp.x = x * cos(ANGLE_1_u / 57.29578) - y * sin(ANGLE_1_u / 57.29578);
//								cv_point_temp.y = x * sin(ANGLE_1_u / 57.29578) + y * cos(ANGLE_1_u / 57.29578);
//
//								if (abs(cv_point_temp.x + 95) < 100 && abs(cv_point_temp.y + 102) < 100)
//								{
//									continue;
//								}
//								Lidar_point.push_back(cv_point_temp);
//							}
//						}
//
//						//激光雷达——下 采样点坐标解算
//						for (int i = 0; i < count_d; i = (int)(i + count_step_d))
//						{
//							point_temp.dis = nodes_d[i].distance_q2 / 4.0f;
//							point_temp.angle = (nodes_d[i].angle_q6_checkbit & 0xFFFF) / (1 << 7);
//							if (point_temp.dis > 10 && point_temp.dis < 2000)
//							{
//								double x, y;
//								x = point_temp.dis * cos((-90 + point_temp.angle) / 57.29578);
//								y = point_temp.dis * sin((-90 + point_temp.angle) / 57.29578) + 170;
//								cv_point_temp.x = x * cos(ANGLE_1_d / 57.29578) - y * sin(ANGLE_1_d / 57.29578);
//								cv_point_temp.y = x * sin(ANGLE_1_d / 57.29578) + y * cos(ANGLE_1_d / 57.29578);
//								if (abs(cv_point_temp.x) < 100 && abs(cv_point_temp.y) < 100)
//								{
//									continue;
//								}
//								Lidar_point.push_back(cv_point_temp);
//							}
//						}
//
//						Lidar_Point.unlock();
//					}
//
//					//QTM建图、障碍物获取
//					QTM_mutex.lock();
//					obstacles.clear();
//					QTM.decay_for_rebuild(QTM.root);
//					QTM.tbd_for_rebuild(QTM.root);
//					Lidar_Point.lock();
//					QTM.build_map(Lidar_point);
//					Lidar_Point.unlock();
//					obstacles = QTM.obs_rects;
//
//					//obstacles_new仅在motor_work线程中clear()
//					if (ARM_STATE == ARM_FOLLOW_TRAJ)
//					{
//						obstacles_new.insert(obstacles_new.end(), QTM.obs_rects_new.begin(), QTM.obs_rects_new.end());
//					}
//
//					QTM_mutex.unlock();
//
//					//机械臂运动过程实时碰撞检测
//					if (ARM_STATE == ARM_FOLLOW_TRAJ && MOVE_TYPE != MOVE_TYPE_DEBUG && MOVE_TYPE != MOVE_TYPE_JOYSTICK)
//					{
//						if (ACM_show.obstacle_collision(obstacles))
//						{
//							//若检测到碰撞，立即停止
//							arm_stop();
//							udp_send_debug_info(17);
//							//beepState = BEEP_THREE;
//						}
//					}
//				}
//				else if (LIDAR_STATE == LIDAR_START)	//激光雷达开始工作
//				{
//					lidar1.RP->startMotor();
//					lidar1.RP->startScan(0, 1);
//					lidar2.RP->startMotor();
//					lidar2.RP->startScan(0, 1);
//
//					LIDAR_STATE = LIDAR_WORK;
//				}
//				else if (LIDAR_STATE == LIDAR_STOP)	//激光雷达停止工作
//				{
//					lidar1.RP->stopMotor();
//					lidar1.RP->stop();
//
//					lidar2.RP->stopMotor();
//					lidar2.RP->stop();
//
//					LIDAR_STATE = LIDAR_FREE;
//				}
//
//				std::this_thread::sleep_for(std::chrono::milliseconds(100));
//			}
//		}
//		else
//		{
//			break;
//		}
//	}
//}
void lidar_scan()
{
	//设置QTM参数
	QTM.set_min_points_num(2);
	QTM.set_decay_speed(1);
	QTM.set_max_points_num(15);

	IChannel* _channel_1;
	IChannel* _channel_2;/*
	sl_lidar_response_measurement_node_hq_t* nodes_u;
	sl_lidar_response_measurement_node_hq_t* nodes_d;*/


	while (true)
	{
		//连接激光雷达
		_channel_1 = (*createSerialPortChannel(parameters["serial_lidar_u"].c_str(), 256000));
		bool connect_lidar_u = SL_IS_OK((lidar1)->connect(_channel_1));

		if (!connect_lidar_u)
		{
			cout << "上激光雷达连接失败" << endl;
		}
		_channel_2 = (*createSerialPortChannel(parameters["serial_lidar_d"].c_str(), 256000));
		bool connect_lidar_d = SL_IS_OK((lidar2)->connect(_channel_2));

		if (!connect_lidar_d)
		{
			cout << "下激光雷达连接失败" << endl;
		}

		if (connect_lidar_u && connect_lidar_d)
		{
			clock_t start_u, end_u, start_d, end_d;


			lidar1->stop();
			lidar2->stop();
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			cout << "lidar success" << endl;
			lidar1->setMotorSpeed();
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			lidar2->setMotorSpeed();
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			//lidar2->startScan(0, 1);
			////std::this_thread::sleep_for(std::chrono::milliseconds(100));
			//lidar1->startScan(0, 1);
			if (SL_IS_FAIL(lidar1->startScan(0, 1))) // you can force slamtec lidar to perform scan operation regardless whether the motor is rotating
			{
				fprintf(stderr, "Error, cannot start the 1-scan operation.\n");

			}
			if (SL_IS_FAIL(lidar2->startScan(0, 1))) // you can force slamtec lidar to perform scan operation regardless whether the motor is rotating
			{
				fprintf(stderr, "Error, cannot start the 2-scan operation.\n");

			}
			/*std::this_thread::sleep_for(std::chrono::milliseconds(100));*/



			/*lidar1->startScan(0, 1);
			lidar2->startScan(0, 1);*/

			sl_result op_result_u;
			sl_result op_result_d;

			LIDAR_STATE = LIDAR_WORK;

			while (true)
			{
				//判断雷达工作状态
				if (LIDAR_STATE == LIDAR_WORK)
				{
					//获取关节1角度与激光雷达数据
					double ANGLE_1_u = M.getAngle(FIRST_MOTOR_CAN);

					/*start_u = clock();*/
					/*end_u = clock();*/
					double ANGLE_1_d = M.getAngle(FIRST_MOTOR_CAN) + 3;
					sl_lidar_response_measurement_node_hq_t nodes_u[2000];
					sl_lidar_response_measurement_node_hq_t nodes_d[2000];
					size_t   count_u = _countof(nodes_u);
					size_t   count_d = _countof(nodes_d);

					/*start_d = clock();*/


					op_result_u = lidar1->grabScanDataHq(nodes_u, count_u);//引用了nodes和count 所以其实已经改变了count的值
					//std::this_thread::sleep_for(std::chrono::milliseconds(10));
					op_result_d = lidar2->grabScanDataHq(nodes_d, count_d);//引用了nodes和count 所以其实已经改变了count的值
					//op_result_d = 0;
					if (op_result_u == SL_RESULT_OPERATION_TIMEOUT)
					{
						int i = 0;
					}
					if (op_result_d == SL_RESULT_OPERATION_TIMEOUT)
					{
						int i = 0;
					}
					if ((SL_IS_OK(op_result_u)) && (SL_IS_OK(op_result_d)))
					{
						//激光雷达数据排列
						lidar1->ascendScanData(nodes_u, count_u);
						lidar2->ascendScanData(nodes_d, count_d);

						lidar_point point_temp;
						double count_step_u = count_u / 360.0;
						double count_step_d = count_d / 360.0;

						Lidar_Point.lock();
						Lidar_point.clear();

						cv::Point2d cv_point_temp;

						//激光雷达——上 采样点坐标解算
						for (int i = 0; i < count_u; i = (int)(i + count_step_u))
						{
							point_temp.dis = nodes_u[i].dist_mm_q2 / 4.0f;
							point_temp.angle = (nodes_u[i].angle_z_q14 * 90.f) / 16384.f;

							if (point_temp.dis > 50 && point_temp.dis < 2000)
							{
								double x, y;
								x = point_temp.dis * cos((270.0 - point_temp.angle) / 57.29578);
								y = point_temp.dis * sin((270.0 - point_temp.angle) / 57.29578) + 170;

								cv_point_temp.x = x * cos(ANGLE_1_u / 57.29578) - y * sin(ANGLE_1_u / 57.29578);
								cv_point_temp.y = x * sin(ANGLE_1_u / 57.29578) + y * cos(ANGLE_1_u / 57.29578);

								if (abs(cv_point_temp.x + 95) < 100 && abs(cv_point_temp.y + 102) < 100)
								{
									continue;
								}
								Lidar_point.push_back(cv_point_temp);
							}
						}

						//激光雷达——下 采样点坐标解算
						for (int i = 0; i < count_d; i = (int)(i + count_step_d))
						{
							point_temp.dis = nodes_d[i].dist_mm_q2 / 4.0f;
							point_temp.angle = (nodes_d[i].angle_z_q14 * 90.f) / 16384.f;
							if (point_temp.dis > 10 && point_temp.dis < 2000)
							{
								double x, y;
								x = point_temp.dis * cos((-90 + point_temp.angle) / 57.29578);
								y = point_temp.dis * sin((-90 + point_temp.angle) / 57.29578) + 170;
								cv_point_temp.x = x * cos(ANGLE_1_d / 57.29578) - y * sin(ANGLE_1_d / 57.29578);
								cv_point_temp.y = x * sin(ANGLE_1_d / 57.29578) + y * cos(ANGLE_1_d / 57.29578);
								if (abs(cv_point_temp.x) < 100 && abs(cv_point_temp.y) < 100)
								{
									continue;
								}
								Lidar_point.push_back(cv_point_temp);
							}
						}

						Lidar_Point.unlock();
					}

					//QTM建图、障碍物获取
					QTM_mutex.lock();
					//obstacles.clear();
					QTM.decay_for_rebuild(QTM.root);
					QTM.tbd_for_rebuild(QTM.root);
					Lidar_Point.lock();
					QTM.build_map(Lidar_point);
					Lidar_Point.unlock();
					obstacles = QTM.obs_rects;

					//obstacles_new仅在motor_work线程中clear()
					if (ARM_STATE == ARM_FOLLOW_TRAJ)
					{
						obstacles_new.insert(obstacles_new.end(), QTM.obs_rects_new.begin(), QTM.obs_rects_new.end());
					}/*

					if (parameters["TOTP"] == "1")
					{
						obstacle_expand(obstacles, obstacles_expanded, atof(parameters["TOTP_expand_val"].c_str()));
						obstacle_expand(obstacles_new, obstacles_expanded_new, atof(parameters["TOTP_expand_val"].c_str()));
					}*/

					QTM_mutex.unlock();

					//机械臂运动过程实时碰撞检测
					if (ARM_STATE == ARM_FOLLOW_TRAJ && MOVE_TYPE != MOVE_TYPE_DEBUG && MOVE_TYPE != MOVE_TYPE_JOYSTICK)
					{
						if (ACM_show.obstacle_collision(obstacles))
						{
							//若检测到碰撞，立即停止
							arm_stop();
							udp_send_debug_info(17);
							ARM_OBSTACLE_STOP = 1;
							//beepState = BEEP_THREE;
						}
					}
				}
				else if (LIDAR_STATE == LIDAR_START)	//激光雷达开始工作
				{
					lidar1->setMotorSpeed();
					lidar2->setMotorSpeed();
					lidar1->startScan(0, 1);
					lidar2->startScan(0, 1);

					LIDAR_STATE = LIDAR_WORK;
				}
				else if (LIDAR_STATE == LIDAR_STOP)	//激光雷达停止工作
				{
					lidar1->stop();
					lidar2->stop();

					LIDAR_STATE = LIDAR_FREE;
				}

				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
		}
		else
		{
			break;
		}
	}
}