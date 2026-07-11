#include "rs_camera_work.h"

using std::cout;
using std::endl;
using std::thread;

#pragma region 全局变量
extern std::map<std::string, std::string> parameters;

double ff = 418.403, Cx = 427.106, Cy = 242.333;	//realsense外参
rs2::context ctx;	//realsense context

gloRS RS0(ctx, 0), RS1(ctx, 1), RS2(ctx, 2), RS3(ctx, 3); //0,末端左侧;1,上方左侧;2,末端右侧;3上方右侧
vector<gloRS*> RSs = { &RS0, &RS1, &RS2, &RS3 };
vector<rs2::pipeline_profile> profiles;

int rs_end_left, rs_end_right, rs_top_left, rs_top_right;

gloData stateCV;

int TARGET_STATE = TARGET_FREE;	//目标拍摄位置识别状态，初始为空闲
int DETECT_STATE = 0; //心脏识别与被动靶标识别状态，初始为空闲
double AprilTagLength = 0.065;
double camera_distance = 0;
int picture_svae = 0;
int cout_flag = 0;

int VISUALSERVO_STATE = 0;    //视觉伺服控制标志，初始为空闲
int VISUALSERVO_TRAJ_STATE = 0;    //视觉伺服轨迹更新标志，初始为空闲
Eigen::Vector3d direction; //视觉伺服运动方向
double u_visualservo_kp = 4.0;
double u_visualservo_ki = 0.05;
double u_visualservo_kd = 0.01;
double v_visualservo_kp = 12.0;
double v_visualservo_ki = 0.05;
double v_visualservo_kd = 0.01;

Eigen::Vector3d position_tar(0, 0, 0);	//目标拍摄位置
Eigen::Vector3d z_tar(0, 0, 0);			//目标拍摄方向

//被动靶标误差数据记录
string err_path = "C:\\Users\\Butel\\Desktop\\被动靶标数据测试\\err_data.txt";

//转换矩阵初始化
Eigen::Isometry3d T10 = Eigen::Isometry3d::Identity();
Eigen::Isometry3d T21 = Eigen::Isometry3d::Identity();
Eigen::Isometry3d T32 = Eigen::Isometry3d::Identity();
Eigen::Isometry3d T43 = Eigen::Isometry3d::Identity();
Eigen::Isometry3d T54 = Eigen::Isometry3d::Identity();
Eigen::Isometry3d T65 = Eigen::Isometry3d::Identity();
Eigen::Isometry3d T73 = Eigen::Isometry3d::Identity();
Eigen::Isometry3d T83 = Eigen::Isometry3d::Identity();
Eigen::Isometry3d T90 = Eigen::Isometry3d::Identity();
Eigen::Isometry3d TA0 = Eigen::Isometry3d::Identity();
Eigen::Isometry3d visualservo_T_cam = Eigen::Isometry3d::Identity();
#pragma endregion

#pragma region 外部引用变量
extern int ARM_STATE;
extern std::vector<traj_point>traj;
extern std::vector<double> joint_now, joint_tar;
extern camera Cam;
extern int beepState;
extern std::ofstream f_log;
extern bool FOCUS_STATE;
extern int lc_now;
extern int lc_zero;
extern int ptz_speed;
extern int ptz_speed_joy;
extern motor M;
extern Eigen::Vector3d focus_point;
extern arm_collision_model ACM;
extern std::vector<cv::RotatedRect> obstacles;
extern int OP_STATUS;
extern int OPTRAJFOLLOW;
extern int MOVE_TYPE;

#pragma endregion

//YOLO模型初始化
Net_config yolo_nets = { 0.3, 0.7, 0.3, "myonnx/heartmodel/wydheartmodel20250409.onnx" };
YOLO yolo_model(yolo_nets);

//陀螺仪数据
GYRO_DATA gyro_data;

//YOLOv5心脏识别数据
cv::Rect heartbox;

#pragma region realsense工作函数
//3D camera初始化函数
void rs_camera_init()
{
	rs_end_left = atoi(parameters["rs_end_left"].c_str());
	rs_end_right = atoi(parameters["rs_end_right"].c_str());
	rs_top_left = atoi(parameters["rs_top_left"].c_str());
	rs_top_right = atoi(parameters["rs_top_right"].c_str());

	RS0.init();
	RS1.init();
	RS2.init();
	RS3.init();

	udp_send_debug_info(DEBUG_3D_CAMERA_INIT_SUCCESS);
}
void visual_show()
{
	//cout << cv::getBuildInformation() << endl;
	gloRS* ptr_end_left = RSs[rs_end_left];
	gloRS* ptr_end_right = RSs[rs_end_right];
	gloRS* ptr_top_left = RSs[rs_top_left];
	gloRS* ptr_top_right = RSs[rs_top_right];
	if (ptr_end_left->initialized && ptr_end_right->initialized && ptr_top_left->initialized && ptr_top_right->initialized)
	{
		/*const auto window_name = "RGB-Camera-Image";
		cv::namedWindow(window_name, 0);
		cv::resizeWindow(window_name, 1701, 480);
		cv::Mat YOLOMat;*/

		Eigen::Vector4d position_left, position_right;
		Eigen::Vector4d position;
		int success_YOLO_num;
		std::pair<double, double> tmp;

		rs2::align align_to = rs2::align(RS2_STREAM_COLOR);
		ExposuerSwitch(1);

		cv::VideoWriter video_left("C:/Users/Butel/Desktop/SSBOT2/heartmodel/left.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(1920, 1080));
		cv::VideoWriter video_right("C:/Users/Butel/Desktop/SSBOT2/heartmodel/right.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(1920, 1080));
		//被动靶标及心脏识别测试
		while (true)
		{
			if (DETECT_STATE == 1) //HEART_DETECT
			{
				/*ExposuerSwitch(0);
				std::this_thread::sleep_for(std::chrono::milliseconds(100));*/
				ptr_end_left->waitForData();
				ptr_end_right->waitForData();

				success_YOLO_num = 0;
				clock_t begin = clock();
				position_left = ptr_end_left->getObjectDepth(yolo_model,rs_end_left);
				clock_t end = clock();
				cout << "左相机识别时间：" << (end - begin) / CLOCKS_PER_SEC * 1000 << "ms" << endl;
				begin = clock();
				position_right = ptr_end_right->getObjectDepth(yolo_model, rs_end_right);
				end = clock();
				cout << "右相机识别时间：" << (end - begin) / CLOCKS_PER_SEC * 1000 << "ms" << endl;
				if (judgeYOLO(position_left))
					success_YOLO_num++;
				if (judgeYOLO(position_right))
					success_YOLO_num++;
				if (success_YOLO_num)
				{
					position = (position_left + position_right) / success_YOLO_num;
					cout << "心坐识别位置：" << position(0) << " " << position(1) << " " << position(2) << endl;
					focus_point(0) = position(0);
					focus_point(1) = position(1);
					focus_point(2) = position(2);
					tmp.first = focus_point(0);
					tmp.second = focus_point(1);
					if (processYOLOResult(tmp, focus_point) == false)
					{
						OP_STATUS = 2;
						cout << "YOLO正视位置不可达, 寻优求取最佳位置后拍摄" << endl;
					}
				}
				else
					cout << "YOLO读取心脏位置失败" << endl;
				DETECT_STATE = 0;
			}

			if (DETECT_STATE == 2)
			{
				cv::Mat color_image_left;
				cv::Mat depth_image_left;
				cv::Mat color_image_right;
				ptr_end_left->waitForData();
				ptr_end_right->waitForData();

				rs2::frameset aligned_frames = align_to.process(ptr_end_left->data);

				ptr_end_left->getColorMat(color_image_left);
				ptr_end_left->getDepthColorMat(depth_image_left);
				ptr_end_right->getColorMat(color_image_right);
				cv::Size targetSize = depth_image_left.size();
				// 创建一个用于存储缩放后图像 A 的矩阵
				cv::Mat resizedcolor;
				cv::Mat resizedcolor2;
				// 将图像 A 缩放到和图像 B 一样的尺寸
				cv::resize(color_image_left, resizedcolor, targetSize);
				cv::resize(color_image_right, resizedcolor2, targetSize);

				cv::Mat mergedImage(resizedcolor.rows, 2 * resizedcolor.cols + 5, resizedcolor.type(), cv::Scalar(0, 0, 0)); // 初始化为黑色
				cv::Rect roi1(cv::Point(0, 0), resizedcolor.size());
				cv::Rect roi2(cv::Point(resizedcolor.cols + 5, 0), resizedcolor.size());
				resizedcolor.copyTo(mergedImage(roi1));
				resizedcolor2.copyTo(mergedImage(roi2));
				// 画一条竖直的黑色线，作为分割区域
				cv::line(mergedImage, cv::Point(resizedcolor.cols, 0), cv::Point(resizedcolor.cols, mergedImage.rows), cv::Scalar(0, 0, 0), 5);
				/*imshow(window_name_front, color_image_front);
				imshow(window_name_side, color_image_side);*/
				/*imshow(window_name, mergedImage);
				cv::waitKey(20);*/

				// 将当前帧写入视频文件
				video_left.write(color_image_left);
				video_right.write(color_image_right);
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
	}
	else
	{
		cout << "相机未初始化成功" << endl;
	}
}
//3D camera工作函数
void rs_camera_work()
{
	thread capture_thread(mainCapture);	//图像采集线程
	thread manager_thread(rs_manager);	//图像采集线程
	//thread recognition_thread(recognition);  //旧靶标识别线程
	thread AprilTag_recognition(AprilTag_recognition); //被动靶标识别线程
	thread visual_show_thread(visual_show);//视频可视化线程
	thread AprilTag_VisualSero(AprilTag_visualservo);
	capture_thread.join();
	manager_thread.join();
	//recognition_thread.join();
	visual_show_thread.join();
	AprilTag_recognition.join();
	AprilTag_VisualSero.join();
}
#pragma endregion

#pragma region 图像采集
//图像采集函数
void mainCapture()
{
	thread capture_t1(mainCapture0);	//3D camera 0图像采集线程
	thread capture_t2(mainCapture1);	//3D camera 1图像采集线程
	thread capture_t3(mainCapture2);	//3D camera 2图像采集线程
	thread capture_t4(mainCapture3);	//3D camera 3图像采集线程
	//thread show_pic_t(show_pic);	
	capture_t1.join();
	capture_t2.join();
	capture_t3.join();
	capture_t4.join();
	//show_pic_t.join();
}
void show_pic()
{
	cout << "各个相机红外图像显示" << endl;
	//cv::Mat m1(500, 500, CV_8UC1, cv::Scalar::all(255));

	//cv::imshow("测试", m1);
	//cv::waitKey(0);
	while (cv::waitKey(100))
	{
		RS0.waitForData();
		RS1.waitForData();
		RS2.waitForData();
		RS3.waitForData();
		RS0.showDepthIR(0, 0, 0);
		RS1.showDepthIR(0, 0, 0);
		RS2.showDepthIR(0, 0, 0);
		RS3.showDepthIR(0, 0, 0);
	}
}
//3D camera 0图像程序
void mainCapture0()
{
	if (RS0.initialized)
	{
		while (1)
		{
			if (RS0.work_state == 1)
			{
				RS0.waitForData();
				
				if (TARGET_STATE == 1)
				{
					RS0.recordIR(&stateCV);
					RS0.recordRGB(&stateCV);
				}
				else
				{
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				}
			}
		}
	}
}

//3D camera 1图像程序
void mainCapture1()
{
	if (RS1.initialized)
	{
		while (1)
		{
			if (RS1.work_state == 1)
			{
				RS1.waitForData();
				
				if (TARGET_STATE == 1)
				{

					RS1.recordIR(&stateCV);
					RS1.recordRGB(&stateCV);
				}
				else
				{
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				}
			}
		}
	}
}

//3D camera 2图像程序
void mainCapture2()
{
	if (RS2.initialized)
	{
		while (true)
		{		
			if (RS2.work_state == 1)
			{
				RS2.waitForData();			

				if (TARGET_STATE == 1)
				{
					RS2.recordIR(&stateCV);
					RS2.recordRGB(&stateCV);
				}
				else
				{
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				}
			}
		}
	}
}

//3D camera 3图像程序
void mainCapture3()
{
	if (RS3.initialized)
	{
		while (1)
		{
			if (RS3.work_state == 1)
			{
				RS3.waitForData();
				

				if (TARGET_STATE == 1)
				{
					RS3.recordIR(&stateCV);
					RS3.recordRGB(&stateCV);
				}
				else
				{
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				}
			}
		}
	}
}

//3D camera 管理函数
void rs_manager()
{
	while (1)
	{
		if (TARGET_STATE == 3)	//若目标拍摄位置识别状态为 准备获取图像
		{
			TARGET_STATE = TARGET_GET_IMAGE;	//目标拍摄位置识别状态置为 获取图像

			std::this_thread::sleep_for(std::chrono::milliseconds(700));	//延时1秒，供图像采集线程采集图像

			udp_send_debug_info(DEBUG_INFO_GET_PICTURE_FINISHED);  //发送图像采集完成信息

			TARGET_STATE = TARGET_GET_POSE;	//目标拍摄位置识别状态置为 获取位姿

			cout << "ok" << endl;
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
	}
}
#pragma endregion

#pragma region 位姿识别
//位姿识别函数
void recognition()
{
	while (1)
	{
		if (TARGET_STATE == 2)	//若目标拍摄位置识别状态为 获取位姿
		{
			f_log << "[" << get_time() << "] " << "图像采集完毕，开始识别" << endl;

			int success_index = -1;	//识别成功的3D camera索引

			//优先次序为0、2、1、3
			if(stateCV.rs_datas[rs_end_left].processIR(0, rs_end_left))
			{
				success_index = rs_end_left;
			}
			else if (stateCV.rs_datas[rs_end_right].processIR(0, rs_end_right))
			{
				success_index = rs_end_right;
			}
			else if (stateCV.rs_datas[rs_top_left].processIR(0, rs_top_left))
			{
				success_index = rs_top_left;
			}
			else if (stateCV.rs_datas[rs_top_right].processIR(0, rs_top_right))
			{
				success_index = rs_top_right;
			}

			for (int i = 0; i < 4; i++)
			{
				stateCV.rs_datas[i].left_pictures.clear();
				stateCV.rs_datas[i].right_pictures.clear();
			}

			if (success_index != -1)	//若识别成功
			{
				f_log << "[" << get_time() << "] " << "靶标图像识别成功" << endl;
				cout << "相机 " << success_index << " 识别到靶标" << endl;
				udp_send_debug_info(DEBUG_INFO_TARGET_CAMERA_SUCCESS);	//发送识别成功信息
				stateCV.rs_datas[success_index].get_T(success_index);	//获取识别成功的3D camera的变换矩阵
				stateCV.rs_datas[success_index].processPNP();	//求解位姿
			}
			else	//若识别失败
			{
				f_log << "[" << get_time() << "] " << "靶标图像识别失败" << endl;
				udp_send_debug_info(DEBUG_ERROR_TARGET_CAMERA_FAIL);	//发送识别失败信息
			}

			TARGET_STATE = TARGET_FREE;	//目标位姿识别状态置为 空闲
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}
}

//aprilTag位姿识别函数
void AprilTag_recognition()
{
	while (1)
	{
		if (TARGET_STATE == 2)	//若目标拍摄位置识别状态为 获取位姿
		{
			f_log << "[" << get_time() << "] " << "AprilTag二维码开始识别" << endl;

			int success_index = -1;	//识别成功的3D camera索引
			AprilTag_DATA AprilTag_pose;

			//优先次序为0、2、1、3
			clock_t begin = clock();
			AprilTag_pose = stateCV.rs_datas[rs_end_left].processAprilTag(1, rs_end_left, 7, 0);
			if (judge(AprilTag_pose) == 0)
			{
				AprilTag_pose = stateCV.rs_datas[rs_end_right].processAprilTag(1, rs_end_right, 7, 0);
			}

			clock_t end = clock();
			fstream fout(err_path, ios::out | ios::app);
			cout << "时间：" << (end - begin) / CLOCKS_PER_SEC * 1000 << "ms" << endl;
			cout << "AprilTag位置 无误差补偿:\n" << AprilTag_pose.x << "  " << AprilTag_pose.y << " " << AprilTag_pose.z << endl;
			//fout << "AprilTag位置 无误差补偿:\n" << AprilTag_pose_end_left.x << "  " << AprilTag_pose_end_left.y << " " << AprilTag_pose_end_left.z << endl;
			cout << "AprilTag角度 无误差补偿:\n" << AprilTag_pose.Rx / PI * 180 << "  " << AprilTag_pose.Ry / PI * 180 << " " << AprilTag_pose.Rz / PI * 180 << endl;
			//fout << "AprilTag角度 无误差补偿:\n" << AprilTag_pose_end_left.Rx / PI * 180 << "  " << AprilTag_pose_end_left.Ry / PI * 180 << " " << AprilTag_pose_end_left.Rz / PI * 180 << endl;
			fout.close();

			for (int i = 0; i < 4; i++)
			{
				stateCV.rs_datas[i].RGB_pictures.clear();
			}

			if (judge(AprilTag_pose)==true)
				success_index = 1;			
			
			if (success_index != -1)	//若识别成功
			{
				f_log << "[" << get_time() << "] " << "靶标图像识别成功" << endl;
				udp_send_debug_info(DEBUG_INFO_TARGET_CAMERA_SUCCESS);	//发送识别成功信息
				processAprilTagResult(AprilTag_pose);
			}
			else	//若识别失败
			{
				f_log << "[" << get_time() << "] " << "靶标图像识别失败" << endl;
				udp_send_debug_info(DEBUG_ERROR_TARGET_CAMERA_FAIL);	//发送识别失败信息
			}

			TARGET_STATE = TARGET_FREE;	//目标位姿识别状态置为 空闲
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
	}
}

//AprilTag位姿识别结果处理函数
void processAprilTagResult(AprilTag_DATA AprilTag_pose)
{
	Eigen::Vector4d  point_rec_mat;

	point_rec_mat(0) = AprilTag_pose.x;
	point_rec_mat(1) = AprilTag_pose.y;
	point_rec_mat(2) = AprilTag_pose.z;
	point_rec_mat(3) = 1;

	cout << "------------------机器人坐标系下point_rec_mat------------------" << endl;
	cout << point_rec_mat << endl;

	//目标拍摄位置初始化为圆心坐标（后会更改）
	for (int i = 0; i < 3; i++)
	{
		position_tar(i) = point_rec_mat(i);
	}

	//被动靶标姿态获取
	{
		Eigen::Matrix3d R_z = Eigen::AngleAxisd(AprilTag_pose.Rz, Eigen::Vector3d(0, 0, 1)).matrix();
		Eigen::Matrix3d R_y = Eigen::AngleAxisd(AprilTag_pose.Ry, Eigen::Vector3d(0, 1, 0)).matrix();
		Eigen::Matrix3d R_x = Eigen::AngleAxisd(AprilTag_pose.Rx, Eigen::Vector3d(1, 0, 0)).matrix();

		Eigen::Matrix3d R = R_z * R_y * R_x;



		for (int i = 0; i < 3; i++)
		{
			z_tar(i) = R(i, 2);
		}
	}

	//圆心坐标延z轴延伸投影至相机原点所在平面
	position_tar -= position_tar(2) / z_tar(2) * z_tar;

	cout << "------------------position_tar------------------" << endl;
	cout << position_tar << endl;

	if (z_tar(2) < 0)
	{
		z_tar *= -1;
	}

	cout << "------------------z_tar------------------" << endl;
	cout << z_tar << endl;

	f_log << "[" << get_time() << "] " << "相机位姿:" << "position_tar = [" << position_tar[0] << ", " << position_tar[1] << ", " << position_tar[2] << "] "
		<< "z_tar = [" << z_tar[0] << ", " << z_tar[1] << ", " << z_tar[2] << "] " << endl;

	if (kdl_ik(joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4],
		position_tar, z_tar.head(3)))	//若成功
	{
		ARM_STATE = ARM_GET_TRAJ;
		if (get_traj(joint_now, joint_tar, traj))	//路径规划，机械臂及云台运动至目标位置
		{
			f_log << "[" << get_time() << "] " << "机器人目标拍摄姿态求解成功" << " joint_tar = [" << joint_tar[0] << ", "
				<< joint_tar[1] << ", " << joint_tar[2] << ", " << joint_tar[3] << ", " << joint_tar[4] << "]" << endl;
			udp_send_debug_info(DEBUG_INFO_TARGET_IK_SUCCESS);
			ARM_STATE = ARM_GET_TRAJ_SUCCESS;
			VISUALSERVO_STATE = VISUAL_SERVO_WAIT;
			Cam.setTilt_with_speed(joint_tar[3], ptz_speed);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			Cam.setPan_with_speed(joint_tar[4], ptz_speed);
			joint_now[3] = joint_tar[3];
			joint_now[4] = joint_tar[4];
		}
		//若失败
		else
		{
			ARM_STATE = ARM_FREE;
			f_log << "[" << get_time() << "] " << "机器人目标拍摄姿态求解失败" << endl;
			udp_send_debug_info(DEBUG_ERROR_TARGET_IK_FAIL);
		}
	}
	//若失败
	else
	{
		ARM_STATE = ARM_FREE;
		f_log << "[" << get_time() << "] " << "机器人目标拍摄姿态求解失败" << endl;
		udp_send_debug_info(DEBUG_ERROR_TARGET_IK_FAIL);
	}
}

//心脏位置YOLO识别结果处理函数
bool processYOLOResult(std::pair<double, double> position_tar, Eigen::Vector3d focus_point)
{
	if (YOLO_kdl_ik(joint_now, position_tar, focus_point))	//若成功
	{
		ARM_STATE = ARM_GET_TRAJ;
		if (get_traj(joint_now, joint_tar, traj))	//路径规划，机械臂及云台运动至目标位置
		{
			f_log << "[" << get_time() << "] " << "YOLO识别机器人目标拍摄姿态求解成功" << " joint_tar = [" << joint_tar[0] << ", "
				<< joint_tar[1] << ", " << joint_tar[2] << ", " << joint_tar[3] << ", " << joint_tar[4] << "]" << endl;
			udp_send_debug_info(DEBUG_INFO_TARGET_IK_SUCCESS);
			ARM_STATE = ARM_GET_TRAJ_SUCCESS;
			//VISUALSERVO_STATE = VISUAL_SERVO_YOLO_WAIT;

			Cam.setTilt_with_speed(joint_tar[3], 12);
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			Cam.setPan_with_speed(joint_tar[4], 12);
			joint_now[3] = joint_tar[3];
			joint_now[4] = joint_tar[4];

			return true;
		}
		//若失败
		else
		{
			ARM_STATE = ARM_FREE;
			f_log << "[" << get_time() << "] " << "YOLO识别机器人目标拍摄姿态求解失败" << endl;
			udp_send_debug_info(DEBUG_ERROR_TARGET_IK_FAIL);
			return false;
		}
	}
	//若失败
	else
	{
		ARM_STATE = ARM_FREE;
		f_log << "[" << get_time() << "] " << "YOLO识别机器人目标拍摄姿态求解失败" << endl;
		udp_send_debug_info(DEBUG_ERROR_TARGET_IK_FAIL);
		return false;
	}
}

//循环心脏位置YOLO识别结果处理函数
bool OPprocessYOLOResult(std::pair<double, double> position_tar, std::pair<double, double> position_now, Eigen::Vector3d focus_point)
{
	std::vector<double> theta_solv(5);
	if (solve_focus_kdl(joint_now, position_tar, focus_point, theta_solv) && !ACM.collision(theta_solv[0] / PI * 180, theta_solv[1] / PI * 180, theta_solv[2] / PI * 180, obstacles))	//若成功
	{

		MOVE_TYPE = MOVE_TYPE_FOCUS;
		Eigen::Vector2d oa_direction;   //相机运动方向
		oa_direction[0] = position_tar.first - position_now.first;
		oa_direction[1] = position_tar.second - position_now.second;
		oa_direction /= oa_direction.norm();    //归一化

		Eigen::Vector2d occlusion_target;   //相机运动目标位置
		occlusion_target[0] = position_tar.first;
		occlusion_target[1] = position_tar.second;

		if (get_traj_kdl(joint_now, traj, oa_direction, true, focus_point, occlusion_target))
		{
			ARM_STATE = ARM_GET_TRAJ_SUCCESS;   //若求解笛卡尔直线路径成功，将机械臂状态置为ARM_GET_TRAJ_SUCCESS
			OPTRAJFOLLOW = 1;
			cout << "空间避障开始运动" << endl;
			return true;
		}
		else
		{
			cout << "空间避障规划路径失败" << endl;
			return false;
		}
	}
	//若失败
	else
	{
		ARM_STATE = ARM_FREE;
		cout << "空间避障目标位置不可达" << endl;
		return false;
	}
}

//视觉伺服追踪aprilTag二维码函数
void AprilTag_visualservo()
{
	cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
	while (1)
	{
		Eigen::Vector3d direction_cam;
		if (VISUALSERVO_STATE == 2)	//若目标拍摄位置识别状态为 获取位
		{
			cv::VideoCapture cap;
			cap.open(0);
			cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);//可以
			cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
			if (!cap.isOpened())
				cout << "高清相机读取失败" << endl;
			cv::Mat frame;
			double u_err = 100, v_err = 100;
			double u_err_last = 0, v_err_last = 0;
			double u_err_sum = 0, v_err_sum = 0;
			double direction_x = 0, direction_y, x_err, y_err, linestep_err;
			clock_t begin = clock();
			int first_flag = 1;
			while (sqrt(u_err * u_err + v_err * v_err) > 80)
			{
				cap >> frame;
				if (frame.empty())
					continue;
				cv::Mat resized_frame;
				cv::resize(frame, resized_frame, cv::Size(960, 540), 0, 0, cv::INTER_LINEAR);
				cv::imshow("高清相机画面", resized_frame);
				cv::waitKey(20);

				vector<double> tmp;
				tmp = CentreGet(frame, 0);
				u_err = tmp[0] - 960;
				v_err = tmp[1] - 540;
				u_err_sum = u_err_sum + u_err;
				v_err_sum = v_err_sum + v_err;

				x_err = u_visualservo_kp * u_err + u_visualservo_ki * u_err_sum + u_visualservo_kd * (u_err - u_err_last);
				y_err = v_visualservo_kp * v_err + v_visualservo_ki * v_err_sum + v_visualservo_kd * (v_err - v_err_last);
				linestep_err = sqrt(x_err * x_err + y_err * y_err);

				direction_x = u_err / (sqrt(u_err * u_err + v_err * v_err));
				direction_y = v_err / (sqrt(u_err * u_err + v_err * v_err));

				u_err_last = u_err;
				v_err_last = v_err;
				if (linestep_err > 2)
					linestep_err = 2;

				direction_cam(0) = direction_x;
				direction_cam(1) = direction_y;
				direction_cam(2) = 0;
				visualservo_get_T();

				direction = visualservo_T_cam.rotation() * direction_cam;
				if (first_flag == 1)
				{
					arm_direction(direction(0) * linestep_err, direction(1) * linestep_err);
					first_flag = 0;
				}
				else
				{
					VISUALSERVO_TRAJ_STATE = VISUAL_SERVO_TRAJ_UPDATE;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
			arm_stop();
			VISUALSERVO_STATE = VISUAL_SERVO_FREE;
			VISUALSERVO_TRAJ_STATE = VISUAL_SERVO_TRAJ_FREE;
			clock_t end = clock();
			cout << "时间：" << (end - begin) / CLOCKS_PER_SEC * 1000 << "ms" << endl;
			std::cout << "视觉伺服结束" << endl;
		}
		else if (VISUALSERVO_STATE == 4)	//若目标拍摄位置识别状态为 获取位
		{
			double u_err = 100, v_err = 100;
			cv::VideoCapture cap;
			cap.open(0);
			cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);//可以
			cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
			if (!cap.isOpened())
			{
				cout << "高清相机读取失败" << endl;
				u_err = 0;
				v_err = 0;
			}
			cv::Mat frame;
			double u_err_last = 0, v_err_last = 0;
			double u_err_sum = 0, v_err_sum = 0;
			double direction_x = 0, direction_y = 0, x_err = 0, y_err = 0, linestep_err;
			clock_t begin = clock();
			int first_flag = 1;
			while (sqrt(u_err * u_err + v_err * v_err) > 80)
			{
				cap >> frame;
				if (frame.empty())
					continue;
				cv::Mat resized_frame;
				cv::resize(frame, resized_frame, cv::Size(960, 540), 0, 0, cv::INTER_LINEAR);
				cv::imshow("高清相机画面", resized_frame);
				cv::waitKey(20);

				vector<double> tmp;
				tmp = YOLOCentreGet(frame, yolo_model);
				u_err = tmp[0] - 960;
				v_err = tmp[1] - 540;
				cout << "u_err:" << u_err << " v_err:" << v_err << endl;
				u_err_sum = u_err_sum + u_err;
				v_err_sum = v_err_sum + v_err;

				x_err = u_visualservo_kp * u_err + u_visualservo_ki * u_err_sum + u_visualservo_kd * (u_err - u_err_last);
				y_err = v_visualservo_kp * v_err + v_visualservo_ki * v_err_sum + v_visualservo_kd * (v_err - v_err_last);
				linestep_err = sqrt(x_err * x_err + y_err * y_err);

				direction_x = u_err / (sqrt(u_err * u_err + v_err * v_err));
				direction_y = v_err / (sqrt(u_err * u_err + v_err * v_err));

				u_err_last = u_err;
				v_err_last = v_err;
				if (linestep_err > 0.5)
					linestep_err = 0.5;

				direction_cam(0) = direction_x;
				direction_cam(1) = direction_y;
				direction_cam(2) = 0;
				visualservo_get_T();

				direction = visualservo_T_cam.rotation() * direction_cam;
				if (first_flag == 1)
				{
					arm_direction(direction(0) * linestep_err, direction(1) * linestep_err);
					first_flag = 0;
				}
				else
				{
					VISUALSERVO_TRAJ_STATE = VISUAL_SERVO_TRAJ_UPDATE;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
			arm_stop();
			VISUALSERVO_STATE = VISUAL_SERVO_FREE;
			VISUALSERVO_TRAJ_STATE = VISUAL_SERVO_TRAJ_FREE;
			clock_t end = clock();
			cout << "时间：" << (end - begin) / CLOCKS_PER_SEC * 1000 << "ms" << endl;
			std::cout << "视觉伺服结束" << endl;
		}

		else if (VISUALSERVO_STATE == 8)	//若目标拍摄位置识别状态为 获取位
		{
		double u_err = 100, v_err = 100;
		cv::VideoCapture cap;
		cap.open(8);
		cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);//可以
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
		if (!cap.isOpened())
		{
			cout << "高清相机读取失败" << endl;
			u_err = 0;
			v_err = 0;
		}
		cv::Mat frame;
		cap >> frame;
		double u_err_last = 0, v_err_last = 0;
		double u_err_sum = 0, v_err_sum = 0;
		double direction_wx = 0, direction_wy = 0, wx = 0, wy = 0, wx_norm = 0, wy_norm = 0;
		double thetax = 0.0, thetay = 0.0, newthetax = 0.0, newthetay = 0.0;
		clock_t begin = clock();
		int first_flag = 1;
		int err_flag = 0;
		cv::Mat resized_frame;
		while (sqrt(u_err * u_err + v_err * v_err) > 110)
		{
			cap >> frame;
			if (frame.empty())
				continue;

			vector<double> tmp;
			clock_t begin = clock();
			tmp = YOLOCentreGet(frame, yolo_model);
			clock_t end = clock();
			cout << "识别时间：" << (end - begin) << "ms" << endl;
			cv::resize(frame, resized_frame, cv::Size(960, 540), 0, 0, cv::INTER_LINEAR);
			cv::imshow("高清相机画面", resized_frame);
			cv::waitKey(1);
			if (abs(tmp[0]) < 1e-4)
			{
				err_flag++;
				if (err_flag == 10)
					break;
				else
					continue;
			}
			else
				err_flag == 0;
			u_err = tmp[0] - 960;
			v_err = tmp[1] - 540;
			cout << "u_err:" << u_err << " v_err:" << v_err << endl;
			if (sqrt(u_err * u_err + v_err * v_err) < 110)
				break;
			u_err_sum = u_err_sum + u_err;
			v_err_sum = v_err_sum + v_err;

			wx_norm = abs(u_visualservo_kp * u_err + u_visualservo_ki * u_err_sum);
			wy_norm = abs(v_visualservo_kp * v_err + v_visualservo_ki * v_err_sum);

			if (u_err >= 0)
				direction_wy = -1.0;
			else
				direction_wy = 1.0;
			if (v_err >= 0)
				direction_wx = -1.0;
			else
				direction_wx = 1.0;

			wx_norm = max(min(50.0, wx_norm), 26.0);
			wy_norm = max(min(50.0, wy_norm), 26.0);
			wx = direction_wx * wx_norm;
			wy = direction_wy * wy_norm;

			Cam.getPan(thetax);
			Cam.getTilt(thetay);
			newthetax = min(max((thetax + wx * 0.1), -40.0), 40.0);
			newthetay = min(max((thetay + wy * 0.1), -40.0), 40.0);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
			Cam.setPan_with_speed(newthetax, wx_norm);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
			Cam.setTilt_with_speed(newthetay, wy_norm);
			std::this_thread::sleep_for(std::chrono::milliseconds(80));
			Cam.stopPan();
			Cam.stopTilt();

			/*std::this_thread::sleep_for(std::chrono::milliseconds(20));
			Cam.setPanMotor_with_speed(wx);
			std::this_thread::sleep_for(std::chrono::milliseconds(40));
			Cam.setPanMotor_with_speed(0.0);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
			Cam.setTilMotor_with_speed(wy);
			std::this_thread::sleep_for(std::chrono::milliseconds(40));
			Cam.setTilMotor_with_speed(0.0);*/
		}
		if(err_flag > 0)
			cout << "多次检测目标失败" << endl;
		else
		{
			clock_t end = clock();
			cout << "时间：" << (end - begin) << "ms" << endl;
			cout << "视觉伺服结束" << endl;
		}
		Cam.setPanMotor_with_speed(0.0);
		Cam.setTilMotor_with_speed(0.0);
		Cam.setPanMotor2Servo();
		Cam.setTilMotor2Servo();
		VISUALSERVO_STATE = VISUAL_SERVO_FREE;
		cap.release();
		heartbox = cv::Rect();
		}

		//if (VISUALSERVO == 2)	//若目标拍摄位置识别状态为 获取位
		//{
		//	cv::VideoCapture cap;
		//	cap.open(0);
		//	cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);//可以
		//	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
		//	if (!cap.isOpened())
		//		cout << "高清相机读取失败" << endl;
		//	cv::Mat frame;
		//	double u_err = 100, v_err = 100;
		//	double u_err_last = 0, v_err_last = 0;
		//	double u_err_sum = 0, v_err_sum = 0;
		//	double u_speed = 0, v_speed = 0;
		//	double L1 = 700, L2 = 500, L3 = 510;
		//	double theta1 = 0, theta2 = 0, theta3 = 0;
		//	double x1 = 0, y1 = 0, x2 = 0, y2 = 0, x_tar, y_tar;
		//	double new_theta2 = 0, new_theta3 = 0;
		//	Eigen::Vector2d xh(-1.0, -1.0);
		//	Eigen::Vector2d v_xy(0.0, 0.0);
		//	Eigen::Vector2d v_theta(0.0, 0.0);
		//	Eigen::Vector2d uv_observe;
		//	Eigen::Matrix2d Angle2Translate;
		//	Eigen::Matrix2d Q, P, R, C, K, Pfe, I;
		//	Q << 0.3, 0,
		//		0, 0.3;
		//	P << 0.3, 0,
		//		0, 0.3;
		//	R << 0.1, 0,
		//		0, 0.1;
		//	I << 1, 0,
		//		0, 1;
		//	clock_t begin = clock();

		//	cap >> frame;
		//	vector<double> centre, centre_new;
		//	centre = CentreGet(frame, 0);
		//	cv::Mat resized_frame;
		//	double w1 = 0.0, w2 = 0.0;
		//	int frame_MAX = 10, flag = 0;
		//	while (sqrt(u_err * u_err + v_err * v_err) > 110)
		//	{
		//		u_err = 960 - centre[0];
		//		v_err = 540 - centre[1];
		//		u_err_sum = u_err_sum + u_err;
		//		v_err_sum = v_err_sum + v_err;

		//		u_speed = u_visualservo_kp * u_err + u_visualservo_ki * u_err_sum + u_visualservo_kd * (u_err - u_err_last)/0.1;
		//		v_speed = v_visualservo_kp * v_err + v_visualservo_ki * v_err_sum + v_visualservo_kd * (v_err - v_err_last)/0.1;
		//		u_err_last = u_err;
		//		v_err_last = v_err;
		//		/*if (u_speed > 600)
		//			u_speed = 600;
		//		else if (u_speed < -600)
		//			u_speed = -600;
		//		if (v_speed > 600)
		//			v_speed = 600;
		//		else if (v_speed < -600)
		//			v_speed = -600;*/

		//		v_xy(0) = u_speed / xh(0);
		//		v_xy(1) = v_speed / xh(1);
		//		theta1 = M.getAngle(FIRST_MOTOR_CAN) / 180 * PI;
		//		theta2 = M.getAngle(SECOND_MOTOR_CAN) / 180 * PI;
		//		theta3 = M.getAngle(THIRD_MOTOR_CAN) / 180 * PI;
		//		x1 = -L1 * sin(theta1) - L2 * sin(theta1 + theta2) - L3 * sin(theta1 + theta2 + theta3);
		//		y1 = L1 * cos(theta1) + L2 * cos(theta1 + theta2) + L3 * cos(theta1 + theta2 + theta3);
		//		Angle2Translate << -L2 * cos(theta1+theta2), -L3 * cos(theta1 + theta2 +theta3),
		//						   -L2* sin(theta1 + theta2), -L3* sin(theta1 + theta2 + theta3);
		//		v_theta = Angle2Translate.inverse() * v_xy;
		//		w1 = v_theta(0);
		//		w2 = (v_theta(1) - v_theta(0));
		//		if (w1 > 1.5/180*PI)
		//			w1 = 1.5 / 180 * PI;
		//		else if (w1 < -1.5 / 180 * PI)
		//			w1 = -1.5 / 180 * PI;
		//		if (w2 > 1.5 / 180 * PI)
		//			w2 = 1.5 / 180 * PI;
		//		else if (w2 < -1.5 / 180 * PI)
		//			w2 = -1.5 / 180 * PI;
		//		new_theta2 = (theta2 + w1 * 0.05) / PI * 180;
		//		new_theta3 = (theta3 + w2 * 0.05) / PI * 180;
		//		M.setAngle(new_theta2, SECOND_MOTOR_CAN);
		//		M.setAngle(new_theta3, THIRD_MOTOR_CAN);
		//		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		//		theta1 = M.getAngle(FIRST_MOTOR_CAN) / 180 * PI;
		//		theta2 = M.getAngle(SECOND_MOTOR_CAN) / 180 * PI;
		//		theta3 = M.getAngle(THIRD_MOTOR_CAN) / 180 * PI;
		//		x2 = -L1 * sin(theta1) - L2 * sin(theta1 + theta2) - L3 * sin(theta1 + theta2 + theta3);
		//		y2 = L1 * cos(theta1) + L2 * cos(theta1 + theta2) + L3 * cos(theta1 + theta2 + theta3);

		//		/*position2 = solve_visualservo_kdl(joint_now);
		//		x2 = position2.first;
		//		y2 = position2.second;*/

		//		C << x2 - x1, 0,
		//			0, y2 - y1;
		//		xh = xh;
		//		Pfe = P + Q;
		//		K = Pfe * C.transpose() * ((C * Pfe * C.transpose() + R)).inverse();
		//		while (flag< frame_MAX)
		//		{
		//			cap >> frame;
		//			centre_new = CentreGet(frame, 0);
		//			if (abs(centre_new[0]) > 1e-2)
		//				break;
		//			else
		//				flag++;
		//		}
		//		if (abs(centre_new[0]) < 1e-2 && abs(centre_new[1]) < 1e-2)
		//			break;
		//		uv_observe(0) = centre_new[0] - centre[0];
		//		uv_observe(1) = centre_new[1] - centre[1];
		//		centre[0] = centre_new[0];
		//		centre[1] = centre_new[1];
		//		xh = xh + K * (uv_observe - C * xh);
		//		P = (I - K * C) * Pfe;

		//		cv::resize(frame, resized_frame, cv::Size(960, 540), 0, 0, cv::INTER_LINEAR);
		//		cv::imshow("高清相机画面", resized_frame);
		//		cv::waitKey(20);
		//		//cout << xh(0) << ' ' << xh(1) << endl;
		//		cout << centre[0] << ' ' << centre[1] << endl;
		//		flag = 0;
		//	}
		//	arm_stop();
		//	VISUALSERVO = VISUAL_SERVO_FREE;
		//	clock_t end = clock();
		//	if (abs(centre_new[0]) < 1e-2 && abs(centre_new[1]) < 1e-2)
		//		cout << "无法识别到二维码图像，视觉伺服失败" << endl;
		//	else
		//	{
		//		cout << "时间：" << (end - begin) / CLOCKS_PER_SEC * 1000 << "ms" << endl;
		//		cout << "视觉伺服结束" << endl;
		//	}
		//}

	}
}
#pragma endregion

#pragma region gloRS类
//gloRS类默认构造函数
//@param num 3d camera 索引
gloRS::gloRS(rs2::context& ctx, int num)
{
	number = num;
	devices = ctx.query_devices();
}

//gloRS类成员函数：初始化
void gloRS::init()
{
	int d = devices.size();
	if (number < d)
	{
		D435 = devices[number];
		IR = D435.query_sensors()[0];

		//IR.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);	//设置自动曝光开启
		IR.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);	//设置自动曝光关闭
		IR.set_option(RS2_OPTION_EXPOSURE, 150);	//设置曝光参数150

		cfg.enable_device(D435.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));	//使能设备
		cfg.enable_stream(RS2_STREAM_INFRARED, 1, 848, 480, RS2_FORMAT_ANY, 90);	//设置图片大小，帧率
		cfg.enable_stream(RS2_STREAM_INFRARED, 1);	//红外相机1
		cfg.enable_stream(RS2_STREAM_INFRARED, 2);	//红外相机2
		cfg.enable_stream(RS2_STREAM_DEPTH);	//深度图像
		//cfg.enable_stream(RS2_STREAM_COLOR);	//rgb图像
		cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 15);	//rgb图像 目标检测使用设置大小帧率

		/*align_to = rs2::align(RS2_STREAM_COLOR);*/
		//pipe.start(cfg);	//设置参数
		profiles.push_back(pipe.start(cfg));
		initialized = true;

		cout << "rs-camera-" << number << " init successed" << endl;
	}
}

//gloRS类成员函数：设置工作状态，切换参数
//@param state 工作状态
void gloRS::set_work_state(int state)	//遮挡规避与靶标识别需切换相机参数
{
	if (work_state != state)
	{
		if (state)	//靶标识别状态
		{
			IR.set_option(RS2_OPTION_EMITTER_ENABLED, 0);	//关闭结构光发射器
			IR.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);	//设置自动曝光关闭
			IR.set_option(RS2_OPTION_EXPOSURE, 150);	//设置曝光参数150
		}
		else	//遮挡规避状态
		{
			IR.set_option(RS2_OPTION_EMITTER_ENABLED, 1);	//开启结构光发射器
			auto range = IR.get_option_range(RS2_OPTION_LASER_POWER);
			IR.set_option(RS2_OPTION_LASER_POWER, range.max); //设置结构光发射器最大功率
			IR.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);	//设置自动曝光开启
		}

		work_state = state;
	}
}

//gloRS类成员函数：等待图像数据
void gloRS::waitForData()
{
	rs_data_mutex.lock();
	data = pipe.wait_for_frames();
	rs_data_mutex.unlock();
}

void gloRS::getColorMat(cv::Mat& a)
{
	rs2::frame color = data.get_color_frame();
	const int w = color.as<rs2::video_frame>().get_width();
	const int h = color.as<rs2::video_frame>().get_height();
	a = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)color.get_data());
	cvtColor(a, a, CV_BGR2RGB);//转换
}
//gloRS类成员函数：获取红外图像
//@param a 图片
//@param LR 红外图像索引（12 左右）
void gloRS::getIrMat(cv::Mat& a, int LR)
{
	rs2::frame IR1 = data.get_infrared_frame(LR);
	const int w = IR1.as<rs2::video_frame>().get_width();
	const int h = IR1.as<rs2::video_frame>().get_height();
	a = cv::Mat(cv::Size(w, h), CV_8UC1, (void*)IR1.get_data());
}

//gloRS类成员函数：获取深度图像
rs2::frame gloRS::getDepthImage()
{
	return data.get_depth_frame();
}

//gloRS类成员函数：获取颜色渲染后的深度图像
//@param a 图片
//@param num 相机索引（01 顶部左右）
void gloRS::getDepthColorMat(cv::Mat& a, int num)
{
	rs2::frame Db = data.get_depth_frame();
	const int dw = Db.as<rs2::video_frame>().get_width();
	const int dh = Db.as<rs2::video_frame>().get_height();
	rs2::frame color_depth_frames = color_map.colorize(Db);
	a = cv::Mat(cv::Size(dw, dh), CV_8UC3, (void*)color_depth_frames.get_data());
	if (!num)
	{
		flip(a, a, -1);
	}
}

//gloRS类成员函数：红外图像曝光闪烁
//@param IO 切换选择
void gloRS::toggleIR(int IO)
{
	if (IO == 0)
	{
		IR.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
		IR.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
		IR.set_option(RS2_OPTION_EXPOSURE, 2);
	}
	else
	{
		IR.set_option(RS2_OPTION_EMITTER_ENABLED, 1);
		IR.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
	}
}

//gloRS类成员函数：显示深度图像与红外图像
//@param showD 是否显示深度图像
//@param showI 是否显示红外图像
void gloRS::showDepthIR(int showD, int showI,int showC)
{
	if (showI)
	{
		cv::Mat IR;
		getIrMat(IR, 1);
		std::stringstream ss;
		ss << number;
		std::string Snum = ss.str();
		std::string IRname = "camera_" + std::to_string(number) + "IR-left" + Snum;
		cv::imshow(IRname, IR);
	}
	if (showD)
	{
		cv::Mat depthImage;
		getDepthColorMat(depthImage, number);
		std::stringstream ss;
		ss << number;
		std::string Snum = ss.str();
		std::string Dname = "Colored-Processed-Depth" + Snum;
		cv::imshow(Dname, depthImage);
	}
	if (showC)
	{
		cv::Mat colorImage;
		getColorMat(colorImage);
		std::stringstream ss;
		ss << number;
		std::string Snum = ss.str();
		std::string Colorname = "camera_" + std::to_string(number) + "Color" + Snum;
		cv::imshow(Colorname, colorImage);
	}
	cv::waitKey(1);
}

//gloRS类成员函数：记录红外图像
//@param state gloData变量
void gloRS::recordIR(struct gloData* state)
{
	cv::Mat L, R;
	getIrMat(L, 1);	//获取左红外图像
	getIrMat(R, 2);	//获取右红外图像
	if (state->rs_datas[number].left_pictures.empty())
	{
		cv::waitKey(1);
	}
	state->rs_datas[number].left_pictures.push_back(L.clone());
	state->rs_datas[number].right_pictures.push_back(R.clone());
};

//gloRS类成员函数：记录RGB图像
//@param state gloData变量
void gloRS::recordRGB(struct gloData* state)
{
	cv::Mat RGBpicture;
	getColorMat(RGBpicture);
	if (state->rs_datas[number].RGB_pictures.empty())
	{
		cv::waitKey(1);
	}
	state->rs_datas[number].RGB_pictures.push_back(RGBpicture.clone());
};

//gloRS类成员函数：frame转为mat
//@param f frame
cv::Mat gloRS::frame_to_mat(rs2::frame& f)
{
	using namespace cv;
	using namespace rs2;

	auto vf = f.as<video_frame>();
	const int w = vf.get_width();
	const int h = vf.get_height();

	if (f.get_profile().format() == RS2_FORMAT_BGR8)
	{
		return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_RGB8)
	{
		auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
		Mat r_bgr;
		cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
		return r_bgr;
	}
	else if (f.get_profile().format() == RS2_FORMAT_Z16)
	{
		return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_Y8)
	{
		return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
	{
		return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
	}

	throw std::runtime_error("Frame format is not supported yet!");
}

//gloRS类成员函数：frame转为深度mat
//@param f frame
cv::Mat gloRS::depth_frame_to_meters(rs2::depth_frame& f)
{
	cv::Mat dm = frame_to_mat(f);
	dm.convertTo(dm, CV_64F);
	dm = dm * f.get_units();
	return dm;
}

//gloRS类成员函数：神经网络放大图像
//@param f frame
//cv::Mat gloRS::img_zoomz_in(cv::Mat img, int i)
//{
//	string algorithm;
//	string path;
//	cv::Mat img_new;
//	switch (i)
//	{
//	case 0:
//		algorithm = string("espcn");
//		path = "D:\\opencv-contrib4.5.1\\超分测试\\ESPCN\\ESPCN_x2.pb";
//		break;
//	case 1:
//		algorithm = string("edsr");
//		path = "D:\\opencv-contrib4.5.1\\超分测试\\EDSR\\EDSR_x2.pb"; 
//		break;
//	case 2:
//		algorithm = string("fsrcnn");
//		path = "D:\\opencv-contrib4.5.1\\超分测试\\FSRCNN\\FSRCNN_x2.pb"; 
//		break;
//	}
//	int scale = 2;
//
//	cv::dnn_superres::DnnSuperResImpl sr;
//	// 读取模型
//	sr.readModel(path);
//	// 设定算法和放大比例
//	sr.setModel(algorithm, scale);
//	// 放大图像
//	sr.upsample(img, img_new);
//	return img_new;
//}

//gloRS类成员函数：配合YOLO目标检测结果获取相机坐标系下位置
//@param box 目标检测识别框
Eigen::Vector4d gloRS::getObjectDepth(YOLO yolo_model, int cameraID)
{
	rs2::align align_to = rs2::align(RS2_STREAM_COLOR);
	rs2::frameset aligned_frames = align_to.process(data);

	rs2::depth_frame depth_frame = data.get_depth_frame();
	rs2::frame color_frame = data.get_color_frame();

	cv::Mat color_mat = frame_to_mat(color_frame);
	cv::Mat depth_mat = depth_frame_to_meters(depth_frame);

	yolo_model.detect(color_mat);
	Eigen::Vector4d  heart_centre_point;
	Eigen::Vector4d  heart_centre_point_in_camera;

	static const string window_name = "3D-camera" + to_string(cameraID);
	cv::namedWindow(window_name, 0);
	cv::resizeWindow(window_name, 640, 480);
	cv::imshow(window_name,color_mat);
	cv::waitKey(1);
	string path = "C:/Users/Butel/Desktop/SSBOT2/3D-camera" + to_string(cameraID) + ".jpg";
	cv::imwrite(path, color_mat);
	//std::this_thread::sleep_for(std::chrono::milliseconds(5000));


	double rgb_ff, rgb_Cx, rgb_Cy;
	switch (cameraID)
	{
	case 0:
		rgb_ff = 1363.09;  rgb_Cx = 940.855;  rgb_Cy = 561.777;  break;
		//info.fx = 1365.08*2;   info.fy = 1365.1 * 2;  info.cx = 988.475 * 2;  info.cy = 559.585 * 2;  break;
	case 1:
		rgb_ff = 606.613;  rgb_Cx = 324.474;  rgb_Cy = 245.668;  break;
	case 2:
		rgb_ff = 606.613;  rgb_Cx = 324.474;  rgb_Cy = 245.668;  break;
	case 3:
		rgb_ff = 1366.1;  rgb_Cx = 951.424;  rgb_Cy = 576.123;  break;
	}

	if (!(heartbox.x==0 && heartbox.y==0 && heartbox.width==0 && heartbox.height==0))
	{
		cv::Point centre;
		centre.x = heartbox.x + 0.5 * heartbox.width;
		centre.y = heartbox.y + 0.5 * heartbox.height;
		heartbox.x = (centre.x - 30) / 1920.0 * 848;
		heartbox.y = (centre.y - 30) / 1080.0 * 480;
		heartbox.width = 60 / 1920.0 * 848;
		heartbox.height = 60 / 1080.0 * 480;
		cv::Scalar m = mean(depth_mat(heartbox));

		heart_centre_point(0) = centre.x;
		heart_centre_point(1) = centre.y;
		heart_centre_point(2) = m[0];
		heart_centre_point(3) = 1;

		cout << "心脏距相机距离" << std::setprecision(5) << m[0] << " m" << endl;

		heart_centre_point_in_camera(0) = (heart_centre_point(0) - rgb_Cx) / rgb_ff * heart_centre_point(2) - 0.0325;
		heart_centre_point_in_camera(1) = (heart_centre_point(1) - rgb_Cy) / rgb_ff * heart_centre_point(2);
		heart_centre_point_in_camera(2) = heart_centre_point(2);
		heart_centre_point_in_camera(3) = 1;

		cout << "------------------心脏在相机坐标系下位置------------------" << endl;
		cout << heart_centre_point_in_camera << endl;

		stateCV.rs_datas[cameraID].get_T(cameraID);//测试

		heart_centre_point_in_camera = stateCV.rs_datas[cameraID].T_3d_cam * heart_centre_point_in_camera;
		heart_centre_point_in_camera[2] = (lc_now+ lc_zero) / 1000.0 + heart_centre_point_in_camera[2];

		cout << "------------------心脏在机器人坐标系下位置------------------" << endl;
		cout << heart_centre_point_in_camera << endl;
	}

	heartbox = cv::Rect();
	if (judgeYOLO(heart_centre_point_in_camera))
		return heart_centre_point_in_camera;
	else
	{
		heart_centre_point_in_camera(0) = 0;
		heart_centre_point_in_camera(1) = 0;
		heart_centre_point_in_camera(2) = 0;
		heart_centre_point_in_camera(3) = 0;
		return heart_centre_point_in_camera;
	}
}

#pragma endregion

#pragma region gloData类
gloData::gloData()
{
	rs_datas.resize(4);
}

gloData::~gloData()
{

}
#pragma endregion

#pragma region rsData类
rsData::rsData()
{
}

rsData::~rsData()
{

}

//rsData类成员函数：获取转换矩阵
//@param number 3D camera索引
void rsData::get_T(int number)
{
	//变换矩阵初始化
	{
		T10 = Eigen::Isometry3d::Identity();
		T21 = Eigen::Isometry3d::Identity();
		T32 = Eigen::Isometry3d::Identity();
		T43 = Eigen::Isometry3d::Identity();
		T54 = Eigen::Isometry3d::Identity();
		T65 = Eigen::Isometry3d::Identity();
		T73 = Eigen::Isometry3d::Identity();
		T83 = Eigen::Isometry3d::Identity();
		T90 = Eigen::Isometry3d::Identity();
		TA0 = Eigen::Isometry3d::Identity();
	}

	//以关节1圆心为原点（高度方向与关节4轴心平齐） T10
	{
		double theta1 = joint_now[0] / 180 * PI;

		T10.rotate(Eigen::AngleAxisd(theta1, Eigen::Vector3d(0, 0, 1)));
	}

	//关节2->关节1 T21
	{
		double theta2 = joint_now[1] / 180 * PI;

		T21.translate(Eigen::Vector3d(0, 0.7, 0));
		T21.rotate(Eigen::AngleAxisd(theta2, Eigen::Vector3d(0, 0, 1)));
	}

	//关节3->关节2 T32
	{
		double theta3 = joint_now[2] / 180 * PI;

		T32.translate(Eigen::Vector3d(0, 0.5, 0));
		T32.rotate(Eigen::AngleAxisd(theta3, Eigen::Vector3d(0, 0, 1)));
	}

	//关节4->关节3 T43
	{
		double theta4 = joint_now[3] / 180 * PI;

		T43.translate(Eigen::Vector3d(0, 0.4, 0));
		T43.rotate(Eigen::AngleAxisd(theta4, Eigen::Vector3d(0, 1, 0)));
	}

	//关节5->关节4 T54
	{
		double theta5 = joint_now[4] / 180 * PI;

		T54.translate(Eigen::Vector3d(0, 0.1118, 0));
		T54.rotate(Eigen::AngleAxisd(theta5, Eigen::Vector3d(1, 0, 0)));
	}

	//相机->关节5 T65
	{
		double theta6 = joint_now[5] / 180 * PI;
		T65.rotate(Eigen::AngleAxisd(PI, Eigen::Vector3d(1, 0, 0)));
		T65.rotate(Eigen::AngleAxisd(theta6, Eigen::Vector3d(0, 0, 1)));
	}

	if (atoi(parameters["rs_tilt"].c_str()))//相机是否倾斜安装
	{
		//3D相机0（末端左侧）->相机 T73
		{
			T73.translate(Eigen::Vector3d(-0.01023, 0.35197, -0.05226));
			T73.rotate(Eigen::AngleAxisd(-PI / 2, Eigen::Vector3d(0, 0, 1)));
			T73.rotate(Eigen::AngleAxisd(PI / 180 * 160.32, Eigen::Vector3d(1, 0, 0)));
			T73.rotate(Eigen::AngleAxisd(-PI / 180 * 9.39, Eigen::Vector3d(0, 1, 0)));
		}

		//3D相机1（末端右侧）->3D相机0（末端左侧） T83
		{
			T83.translate(Eigen::Vector3d(0.01023, 0.35197, -0.05226));
			T83.rotate(Eigen::AngleAxisd(-PI / 2, Eigen::Vector3d(0, 0, 1)));
			T83.rotate(Eigen::AngleAxisd(-PI / 180 * 160.32, Eigen::Vector3d(1, 0, 0)));
			T83.rotate(Eigen::AngleAxisd(-PI / 180 * 9.39, Eigen::Vector3d(0, 1, 0)));
		}
	}
	else
	{
		//3D相机0（末端左侧）->相机 T73
		{
			T73.translate(Eigen::Vector3d(-0.01023, 0.35316, -0.06088));
			T73.rotate(Eigen::AngleAxisd(-PI / 2, Eigen::Vector3d(0, 0, 1)));
			T73.rotate(Eigen::AngleAxisd(PI / 180 * 160, Eigen::Vector3d(1, 0, 0)));
		}

		//3D相机2（末端右侧）->3D相机0（末端左侧） T83
		{
			T83.translate(Eigen::Vector3d(0.01023, 0.35316, -0.06088));
			T83.rotate(Eigen::AngleAxisd(-PI / 2, Eigen::Vector3d(0, 0, 1)));
			T83.rotate(Eigen::AngleAxisd(-PI / 180 * 160, Eigen::Vector3d(1, 0, 0)));
		}
	}

	//3D相机3(上方右侧)->原点 T90
	{
		double theta_1 = -30.0 / 180 * PI;
		double theta_2 = -115.0 / 180 * PI;
		T90.translate(Eigen::Vector3d(0.03823, -0.01579, 0.19016));
		T90.rotate(Eigen::AngleAxisd(theta_1, Eigen::Vector3d(0, 0, 1)));
		T90.rotate(Eigen::AngleAxisd(theta_2, Eigen::Vector3d(1, 0, 0)));
	}

	//3D相机1(上方左侧)->原点 TA0
	{
		double theta_1 = 30.0 / 180 * PI;
		double theta_2 = -115.0 / 180 * PI;
		double theta_3 = PI;

		TA0.translate(Eigen::Vector3d(-0.03823, -0.01579, 0.19016));
		TA0.rotate(Eigen::AngleAxisd(theta_1, Eigen::Vector3d(0, 0, 1)));
		TA0.rotate(Eigen::AngleAxisd(theta_2, Eigen::Vector3d(1, 0, 0)));
		TA0.rotate(Eigen::AngleAxisd(theta_3, Eigen::Vector3d(0, 0, 1)));
	}


	if (number == atoi(parameters["rs_end_left"].c_str()))
	{
		T_3d_cam = T10 * T21 * T32 * T73;
	}
	else if (number == atoi(parameters["rs_end_right"].c_str()))
	{
		T_3d_cam = T10 * T21 * T32 * T83;
	}
	else if (number == atoi(parameters["rs_top_left"].c_str()))
	{
		T_3d_cam = TA0;
	}
	else if (number == atoi(parameters["rs_top_right"].c_str()))
	{
		T_3d_cam = T90;
	}

	T_cam = T10 * T21 * T32 * T43 * T54 * T65;
}

//rsData类成员函数：红外图像识别
//@param show 是否显示
//@param index 3D camera索引
//@return true if 图像识别成功
bool rsData::processIR(int show, int index)
{
	ff = atof(parameters["ff_" + std::to_string(index)].c_str());
	Cx = atof(parameters["Cx_" + std::to_string(index)].c_str());
	Cy = atof(parameters["Cy_" + std::to_string(index)].c_str());

	std::vector<cv::Mat> LL, RR;
	for (int i = 1; i < left_pictures.size(); i++)
	{
		//图像相邻帧差分
		cv::Mat R1 = right_pictures.back();
		right_pictures.pop_back();
		cv::Mat R2 = right_pictures.back();
		right_pictures.pop_back();

		cv::Mat L1 = left_pictures.back();
		left_pictures.pop_back();
		cv::Mat L2 = left_pictures.back();
		left_pictures.pop_back();

		cv::Mat R, resultR, L, resultL;

		cv::absdiff(L1, L2, L);
		cv::absdiff(R1, R2, R);

		//用于核对过程
		//imshow("1", L1);
		//imshow("2", L2);
		//imshow("3", L);
		//cv::waitKey(100);

		L.convertTo(resultL, CV_8UC1, 0.7);
		R.convertTo(resultR, CV_8UC1, 0.7);
		LL.push_back(resultL);
		RR.push_back(resultR);
	}

	//cv::destroyWindow("1");
	//cv::destroyWindow("2");
	//cv::destroyWindow("3");

	left_pictures.clear();
	right_pictures.clear();

	if (LL.empty())
	{
		return false;
	}

	cv::Mat result1 = cv::Mat::zeros(LL[0].size(), CV_8UC1), result2 = cv::Mat::zeros(LL[0].size(), CV_8UC1);

	//差分图像叠加
	while (!LL.empty())
	{
		add(RR.back(), result2, result2);
		add(LL.back(), result1, result1);
		LL.pop_back();
		RR.pop_back();
	}

	//叠加图像模糊
	blur(result1, result1, cv::Size(3, 3));
	blur(result2, result2, cv::Size(3, 3));

	//imshow("result1", result1);
	//imshow("result2", result2);
	//cv::waitKey(0);
	//cv::destroyWindow("result1");
	//cv::destroyWindow("result2");

	//叠加图像阈值处理
	cv::Mat  img_edge1, labels1, centroids1, img_color1, stats1;
	cv::Mat  img_edge2, labels2, centroids2, img_color2, stats2;
	threshold(result1, img_edge1, 150, 255, cv::THRESH_BINARY);
	threshold(result2, img_edge2, 150, 255, cv::THRESH_BINARY);

	//imshow("thresh_result1", img_edge1);
	//imshow("thresh_result2", img_edge2);
	//cv::waitKey(0);
	//cv::destroyWindow("thresh_result1");
	//cv::destroyWindow("thresh_result2");

	//阈值处理图像高斯模糊
	GaussianBlur(img_edge1, img_edge1, cv::Size(9, 9), 2, 2);
	GaussianBlur(img_edge2, img_edge2, cv::Size(9, 9), 2, 2);
	//cv::imshow("img_edge1_gaussian", img_edge1);
	//cv::imshow("img_edge2_gaussian", img_edge2);
	//cv::waitKey(10);
	//cv::destroyWindow("img_edge1_gaussian");
	//cv::destroyWindow("img_edge2_gaussian");

	std::vector<cv::Vec3f> circles_1, circles_2;


	//高斯模糊后图像Hough圆检测
	//HoughCircles(srcGray, circles, cv::HOUGH_GRADIENT, 1, 10, 200, 60);

	HoughCircles(img_edge1, circles_1, cv::HOUGH_GRADIENT_ALT, 1.5, 10, 300, .9);
	HoughCircles(img_edge2, circles_2, cv::HOUGH_GRADIENT_ALT, 1.5, 10, 300, .9);
	//cv::imshow("HoughResult_1", img_edge1);
	//cv::imshow("HoughResult_2", img_edge2);

	/*cv::waitKey(2000);*/

	cv::destroyWindow("HoughResult_1");
	cv::destroyWindow("HoughResult_2");
	//判断是否左右图像均仅检测到一个圆
	if (circles_1.size() != 1 || circles_2.size() != 1)
	{
		return false;
	}

	//将得到的结果绘图
	//for (size_t i = 0; i < circles_1.size(); i++)
	//{
	//	cv::Point center(cvRound(circles_1[i][0]), cvRound(circles_1[i][1]));
	//	int radius = cvRound(circles_1[i][2]);
	//	//检测圆中心
	//	circle(img_edge1, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
	//	//检测圆轮廓
	//	circle(img_edge1, center, radius, cv::Scalar(120, 120, 120), 3, 8, 0);
	//}

	//for (size_t i = 0; i < circles_2.size(); i++)
	//{
	//	cv::Point center(cvRound(circles_2[i][0]), cvRound(circles_2[i][1]));
	//	int radius = cvRound(circles_2[i][2]);
	//	//检测圆中心
	//	circle(img_edge2, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
	//	//检测圆轮廓
	//	circle(img_edge2, center, radius, cv::Scalar(120, 120, 120), 3, 8, 0);
	//}

	//cv::imshow("HoughResult_1", img_edge1);
	//cv::imshow("HoughResult_2", img_edge2);

	//cv::waitKey(2000);

	cv::destroyWindow("HoughResult_1");
	cv::destroyWindow("HoughResult_2");

	//圆心在相机坐标系中坐标（x,y此时暂未计算，z已计算）
	point_rec_in_3d_cam(0) = circles_1[0][0];
	point_rec_in_3d_cam(1) = circles_1[0][1];
	point_rec_in_3d_cam(2) = 0.05026 * ff / (circles_1[0][0] - circles_2[0][0]);
	point_rec_in_3d_cam(3) = 1;

	beepState = BEEP_TWO;

	left_pictures.clear();
	right_pictures.clear();

	return true;	//图像识别成功
}

//rsData类成员函数：目标位姿获取
void rsData::processPNP()
{
	Eigen::Vector4d  point_rec_mat;
	//std::cout << "数据"<< state->point_rec_in_3d_cam[0][2]<<' '<< state->point_rec_in_3d_cam[1][2] << ' ' << state->point_rec_in_3d_cam[2][2] << std::endl;

	//计算相机坐标系下圆心（此处计算x,y）
	//point_rec_mat(0) = (point_rec_in_3d_cam(0) - Cx) / ff * point_rec_in_3d_cam(2) - 0.0175;
	point_rec_mat(0) = (point_rec_in_3d_cam(0) - Cx) / ff * point_rec_in_3d_cam(2);
	point_rec_mat(1) = (point_rec_in_3d_cam(1) - Cy) / ff * point_rec_in_3d_cam(2);
	point_rec_mat(2) = point_rec_in_3d_cam(2);
	point_rec_mat(3) = 1;

	//将相机坐标系下圆心转换至机器人坐标系下
	cout << "------------------转换前point_rec_mat------------------" << endl;
	cout << point_rec_mat << endl;

	cout << "------------------T_3d_cam------------------" << endl;
	cout << T_3d_cam.matrix() << endl;

	point_rec_mat = T_3d_cam * point_rec_mat;

	cout << "------------------转换后point_rec_mat------------------" << endl;
	cout << point_rec_mat << endl;

	int time = 0;
	while (!gyro_data.ready && time >= 20)	//等待陀螺仪姿态准备
	{
		time++;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	//若陀螺仪超时未准备好，返回
	if (time >= 20)
	{
		TARGET_STATE = TARGET_FREE;
		return;
	}

	//目标拍摄位置初始化为圆心坐标（后会更改）
	for (int i = 0; i < 3; i++)
	{
		position_tar(i) = point_rec_mat(i);
	}

	//陀螺仪姿态获取
	{
		Eigen::Matrix3d R_z = Eigen::AngleAxisd(gyro_data.z / 180 * PI, Eigen::Vector3d(0, 0, 1)).matrix();
		Eigen::Matrix3d R_y = Eigen::AngleAxisd(gyro_data.y / 180 * PI, Eigen::Vector3d(0, 1, 0)).matrix();
		Eigen::Matrix3d R_x = Eigen::AngleAxisd(gyro_data.x / 180 * PI, Eigen::Vector3d(1, 0, 0)).matrix();

		Eigen::Matrix3d R = R_z * R_y * R_x;



		for (int i = 0; i < 3; i++)
		{
			z_tar(i) = R(i, 2);
		}
	}

	//圆心坐标延z轴延伸投影至相机原点所在平面
	position_tar -= position_tar(2) / z_tar(2) * z_tar;

	cout << "------------------position_tar------------------" << endl;
	cout << position_tar << endl;

	if (z_tar(2) < 0)
	{
		z_tar *= -1;
	}

	cout << "------------------z_tar------------------" << endl;
	cout << z_tar << endl;

	f_log << "[" << get_time() << "] " << "相机位姿:" << "position_tar = [" << position_tar[0] << ", " << position_tar[1] << ", " << position_tar[2] << "] "
		<< "z_tar = [" << z_tar[0] << ", " << z_tar[1] << ", " << z_tar[2] << "] " << endl;

	//求运动学逆解
	if (kdl_ik(joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4],
		position_tar, z_tar.head(3)))	//若成功
	{
		ARM_STATE = ARM_GET_TRAJ;
		if (get_traj(joint_now, joint_tar, traj))	//路径规划，机械臂及云台运动至目标位置
		{
			f_log << "[" << get_time() << "] " << "机器人目标拍摄姿态求解成功" << " joint_tar = [" << joint_tar[0] << ", "
				<< joint_tar[1] << ", " << joint_tar[2] << ", " << joint_tar[3] << ", " << joint_tar[4] << "]" << endl;
			udp_send_debug_info(DEBUG_INFO_TARGET_IK_SUCCESS);
			ARM_STATE = ARM_GET_TRAJ_SUCCESS;
			Cam.setTilt_with_speed(joint_tar[3], ptz_speed);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			Cam.setPan_with_speed(joint_tar[4], ptz_speed);
			joint_now[3] = joint_tar[3];
			joint_now[4] = joint_tar[4];
		}
		//若失败
		else
		{
			ARM_STATE = ARM_FREE;
			f_log << "[" << get_time() << "] " << "机器人目标拍摄姿态求解失败" << endl;
			udp_send_debug_info(DEBUG_ERROR_TARGET_IK_FAIL);
		}
	}
	//若失败
	else
	{
		ARM_STATE = ARM_FREE;
		f_log << "[" << get_time() << "] " << "机器人目标拍摄姿态求解失败" << endl;
		udp_send_debug_info(DEBUG_ERROR_TARGET_IK_FAIL);
	}
}

//rsData类成员函数：AprilTag的RGB像识别
//@param show 是否显示
//@param index 3D camera索引
//@param num 使用图片的数量
//@parm err_compensate 是否进行角度误差补偿
//@return true if 图像识别成功
AprilTag_DATA rsData::processAprilTag(int show, int index, int num, int err_compensate)
{
	AprilTag_DATA AprilTag_pose;
	AprilTag_pose = AprikTagMean(index, num, show, err_compensate);
	return AprilTag_pose;
}

//gloRS类成员函数：检测相机拍摄画面下的AprilTag标识
//@param num 相机索引序号
//@param t 是否绘图 1：绘制 0：不绘制
AprilTag_DATA rsData::getAprilTag(cv::Mat AprilTag_picture, int index, int show, int errcompensate)
{
	getopt_t* getopt = getopt_create();

	// 添加一个布尔型选项 '-h' 或者 "--help"，用于显示帮助信息
	getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
	// 添加一个布尔型选项 '-d' 或者 "--debug"，启用调试输出（速度较慢
	getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");//关闭debug模式可以加快运行速度
	// 添加一个布尔型选项 '-q' 或者 "--quiet"，减少输出
	getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
	// 添加一个字符串选项 '-f' 或者 "--family"，指定要使用的标签族，默认为 "tag36h11"
	getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
	// 添加一个整数选项 '-t' 或者 "--threads"，指定要使用的 CPU 线程数，默认为 1
	getopt_add_int(getopt, 't', "threads", "16", "Use this many CPU threads");
	// 添加一个双精度浮点数选项 '-x' 或者 "--decimate"，指定输入图像的降采样因子，默认为 2.0
	getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
	// 添加一个双精度浮点数选项 '-b' 或者 "--blur"，对输入图像应用低通滤波，默认为 0.0
	getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
	// 添加一个布尔型选项 '-0' 或者 "--refine-edges"，花费更多时间尝试对齐标签的边缘
	getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

	/***********输入标定的相机参数*************/
	apriltag_detection_info_t info; // parameters of the camera calibrations 在这里把标定得到的四元参数输入到程序里
	info.tagsize = AprilTagLength; //标识的实际尺寸，单位m
	//f是相机的焦距 [cx,cy]是相机的光学中心（主点，principal point）
	switch (index)
	{
	case 0:
		info.fx = 1363.09;   info.fy = 1359.99;  info.cx = 940.855;  info.cy = 561.777;  break;
		//info.fx = 1365.08*2;   info.fy = 1365.1 * 2;  info.cx = 988.475 * 2;  info.cy = 559.585 * 2;  break;
	case 1:
		info.fx = 606.613;   info.fy = 605.097;  info.cx = 324.474;  info.cy = 245.668;  break;
	case 2:
		info.fx = 606.613;   info.fy = 605.097;  info.cx = 324.474;  info.cy = 245.668;  break;
	case 3:
		info.fx = 1366.1;   info.fy = 1363.16;  info.cx = 951.424;  info.cy = 576.123;  break;
	}
	//这行代码声明了一个AprilTag家族的指针，并将其初始化为空
	apriltag_family_t* tf = NULL;
	//这段代码根据用户选择的AprilTag家族名称，动态创建相应的AprilTag家族对象
	const char* famname = getopt_get_string(getopt, "family");
	if (!strcmp(famname, "tag36h11")) {
		tf = tag36h11_create();
	}
	else {
		cout << "Unrecognized tag family name." << endl;
	}

	//这行代码创建了一个AprilTag检测器对象
	apriltag_detector_t* td = apriltag_detector_create();//检测器
	 //这行代码将选定的AprilTag家族添加到AprilTag检测器中
	apriltag_detector_add_family(td, tf);
	//这行代码设置AprilTag检测器的四边形减采样参数，即图像下采样的因子
	td->quad_decimate = getopt_get_double(getopt, "decimate");
	//这行代码设置AprilTag检测器的四边形模糊参数，即图像模糊程度
	td->quad_sigma = getopt_get_double(getopt, "blur");
	//这行代码设置AprilTag检测器的线程数目，这里固定为1
	td->nthreads = getopt_get_int(getopt, "threads"); //td->nthreads = 8;
	//这行代码设置AprilTag检测器是否输出调试信息
	td->debug = getopt_get_bool(getopt, "debug");
	//这行代码设置AprilTag检测器是否尝试对边缘进行更精细的调整
	td->refine_edges = getopt_get_bool(getopt, "refine-edges");

	cv::Mat gray;
	cv::cvtColor(AprilTag_picture, gray, cv::COLOR_BGR2GRAY);

	image_u8_t im =
	{ gray.cols,  //width
		gray.rows,  //height
		gray.cols,  //stride 对齐内存，提高处理速度
		gray.data  //buffer
	};

	//利用AprilTag检测器 td 对灰度图像进行AprilTag检测，返回一个指向 zarray_t 结构体的指针，其中包含检测到的AprilTag信息
	zarray_t* detections = apriltag_detector_detect(td, &im);
	//cout << "相机" << num << "识别到" << zarray_size(detections) << " tags" << endl;

	AprilTag_DATA AprilTag_pose;
	AprilTag_DATA average_AprilTag_pose;
	//遍历检测到的AprilTag，获取每个检测到的标记的信息
	for (int i = 0; i < zarray_size(detections); i++)
	{
		fstream fout(err_path, ios::out | ios::app);
		apriltag_detection_t* det;  //识别到的单个apriltag对象
		zarray_get(detections, i, &det); //单个apriltag对象赋值

		/*********加入位姿估计函数**********/
		info.det = det;  //det-->info赋值
		apriltag_pose_t pose; //位姿对象
		Eigen::Vector3d transform_T;
		Eigen::Matrix3d transform_R;
		Eigen::RowVector4d tmp(0.0, 0.0, 0.0, 1.0);
		estimate_pose_for_tag_homography(&info, &pose);
		/************输出三维位置坐标信息***************/
		memcpy(&transform_T, pose.t->data, sizeof(Eigen::Vector3d));
		memcpy(&transform_R, pose.R->data, sizeof(Eigen::Matrix3d));

		double distance, x, y;
		x = det->c[0];
		y = det->c[1];
		distance = transform_T.norm();

		//cout << "相对相机位置" << transform_T << " 直线距离" << transform_T.norm() << endl;
		camera_distance = camera_distance + transform_T.norm();
		Eigen::MatrixXd upper(transform_R.rows(), transform_R.cols() + transform_T.cols());
		upper << transform_R.transpose(), transform_T;
		Eigen::MatrixXd transform(upper.rows() + tmp.rows(), upper.cols());
		transform << upper,
			tmp;

		Eigen::Isometry3d transformCam2t = Eigen::Isometry3d::Identity();
		Eigen::Isometry3d transformB2t = Eigen::Isometry3d::Identity();
		Eigen::Isometry3d transformB2AprilTag = Eigen::Isometry3d::Identity();
		// 提取旋转矩阵（左上角的 3x3 部分）和平移向量（右上角的 3 个元素）
		transformCam2t.linear() = transform.block<3, 3>(0, 0);  // 提取旋转
		transformCam2t.translation() = transform.block<3, 1>(0, 3);  // 提取平移

		Eigen::Isometry3d RGB2MIDDLE = Eigen::Isometry3d::Identity();
		transformCam2t = transformCam2t * RGB2MIDDLE.translate(Eigen::Vector3d(-0.0325, 0, 0));

		transformCam2t = AprilTag2Target(transformCam2t, det->id);
		if (cout_flag == 0)
		{
			cout << "Apriltag中心坐标 " << det->c[0] << " " << det->c[1] << " " << distance << endl;
			cout_flag = 1;
		}
		stateCV.rs_datas[index].get_T(index);
		transformB2t = stateCV.rs_datas[index].T_3d_cam * transformCam2t;

		if (errcompensate == 0)
		{
			AprilTag_pose = R2RPY(transformB2t.matrix().block(0, 0, 3, 3), transformB2t.matrix().block<3, 1>(0, 3));
			AprilTag_pose.distance = distance;
			cout << "AprilTag ID" << det->id << "角度：" << AprilTag_pose.Rx / PI * 180 << " " << AprilTag_pose.Ry / PI * 180 << " " << AprilTag_pose.Rz / PI * 180 << " " << distance << endl;
			//fout << "AprilTag ID" << det->id << "角度：" << AprilTag_pose.Rx / PI * 180 << " " << AprilTag_pose.Ry / PI * 180 << " " << AprilTag_pose.Rz / PI * 180 << " " << distance << endl;
			fout.close();
		}
		else
		{
			AprilTag_pose = R2RPY(transformB2t.matrix().block(0, 0, 3, 3), transformB2t.matrix().block<3, 1>(0, 3));
			AprilTag_pose.distance = distance;
			cout << "未补偿AprilTag ID" << det->id << "角度：" << AprilTag_pose.Rx / PI * 180 << " " << AprilTag_pose.Ry / PI * 180 << " " << AprilTag_pose.Rz / PI * 180 << " " << distance << endl;
			//误差补足
			AprilTag_pose = ErrCompensate(AprilTag_pose, det->id);
			cout << "补偿后AprilTag ID" << det->id << "角度：" << AprilTag_pose.Rx / PI * 180 << " " << AprilTag_pose.Ry / PI * 180 << " " << AprilTag_pose.Rz / PI * 180 << " " << distance << endl;
		}

		average_AprilTag_pose.x = average_AprilTag_pose.x + AprilTag_pose.x;
		average_AprilTag_pose.y = average_AprilTag_pose.y + AprilTag_pose.y;
		average_AprilTag_pose.z = average_AprilTag_pose.z + AprilTag_pose.z;
		average_AprilTag_pose.Rx = average_AprilTag_pose.Rx + AprilTag_pose.Rx;
		average_AprilTag_pose.Ry = average_AprilTag_pose.Ry + AprilTag_pose.Ry;
		average_AprilTag_pose.Rz = average_AprilTag_pose.Rz + AprilTag_pose.Rz;
		average_AprilTag_pose.distance = average_AprilTag_pose.distance + AprilTag_pose.distance;

		if (show == 1)
		{
			cv::line(AprilTag_picture, cv::Point(det->p[0][0], det->p[0][1]),
				cv::Point(det->p[1][0], det->p[1][1]),
				cv::Scalar(0, 0xff, 0), 1);
			cv::line(AprilTag_picture, cv::Point(det->p[0][0], det->p[0][1]),
				cv::Point(det->p[3][0], det->p[3][1]),
				cv::Scalar(0, 0, 0xff), 1);
			cv::line(AprilTag_picture, cv::Point(det->p[1][0], det->p[1][1]),
				cv::Point(det->p[2][0], det->p[2][1]),
				cv::Scalar(0xff, 0, 0), 1);
			cv::line(AprilTag_picture, cv::Point(det->p[2][0], det->p[2][1]),
				cv::Point(det->p[3][0], det->p[3][1]),
				cv::Scalar(0xff, 0, 0), 1);
			std::stringstream ss;
			ss << det->id;
			std::string text = ss.str();
			int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
			double fontscale = 1.0;
			int baseline;
			cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
				&baseline);
			cv::putText(AprilTag_picture, text, cv::Point(det->c[0] - textsize.width / 2,
				det->c[1] + textsize.height / 2),
				fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
		}
	}
	if (show == 1)
	{
		cv::Mat resized_frame;
		switch (index)
		{
		case 0:
			cv::resize(AprilTag_picture, resized_frame, cv::Size(960, 540), 0, 0, cv::INTER_LINEAR);
			cv::imshow("相机 0 拍摄画面", resized_frame);
			cv::waitKey(20);
			break;
		case 1:
			cv::resize(AprilTag_picture, resized_frame, cv::Size(960, 540), 0, 0, cv::INTER_LINEAR);
			cv::imshow("相机 1 拍摄画面", resized_frame);
			cv::moveWindow("相机 1 拍摄画面", 1000, 0);
			cv::waitKey(20);
			break;
		case 2:
			cv::resize(AprilTag_picture, resized_frame, cv::Size(960, 540), 0, 0, cv::INTER_LINEAR);
			cv::imshow("相机 2 拍摄画面", resized_frame);
			cv::waitKey(20);
			break;
		case 3:
			cv::resize(AprilTag_picture, resized_frame, cv::Size(960, 540), 0, 0, cv::INTER_LINEAR);
			cv::imshow("相机 3 拍摄画面", resized_frame);
			cv::waitKey(20);
			break;
		}
		//std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	}

	if (zarray_size(detections) != 0)
	{
		average_AprilTag_pose.x = average_AprilTag_pose.x / zarray_size(detections);;
		average_AprilTag_pose.y = average_AprilTag_pose.y / zarray_size(detections);;
		average_AprilTag_pose.z = average_AprilTag_pose.z / zarray_size(detections);
		average_AprilTag_pose.Rx = average_AprilTag_pose.Rx / zarray_size(detections);
		average_AprilTag_pose.Ry = average_AprilTag_pose.Ry / zarray_size(detections);
		average_AprilTag_pose.Rz = average_AprilTag_pose.Rz / zarray_size(detections);
		average_AprilTag_pose.distance = average_AprilTag_pose.distance / zarray_size(detections);
	}

	apriltag_detections_destroy(detections);
	apriltag_detector_destroy(td);

	if (!strcmp(famname, "tag36h11")) {
		tag36h11_destroy(tf);
	}

	getopt_destroy(getopt);

	return average_AprilTag_pose;
}

//同一个相机AprilTag识别结果求平均
AprilTag_DATA rsData::AprikTagMean(int index, int num, int show, int err_compensate)
{
	AprilTag_DATA AprilTag_pose, Tmp_pose;
	cv::Mat AprilTag_picture;
	cout_flag = 0;
	deque<double> x, y, z, Rx, Ry, Rz, distance;
	int noAprilTag = 0;

	if (num > stateCV.rs_datas[index].RGB_pictures.size())
	{
		if (stateCV.rs_datas[index].RGB_pictures.size() % 2 == 0)
			num = stateCV.rs_datas[index].RGB_pictures.size() - 1;
		else
			num = stateCV.rs_datas[index].RGB_pictures.size();
	}

	for (int i = 0; i < num; i++)
	{
		AprilTag_picture = stateCV.rs_datas[index].RGB_pictures[i];
		Tmp_pose = getAprilTag(AprilTag_picture, index, show, err_compensate);
		if (judge(Tmp_pose) == 0)
		{
			noAprilTag++;
			if(noAprilTag>=2)
				return AprilTag_pose;
		}

		if (abs(Tmp_pose.x - 0) > 1e-5)
			x.push_back(Tmp_pose.x);
		if (abs(Tmp_pose.y - 0) > 1e-5)
			y.push_back(Tmp_pose.y);
		if (abs(Tmp_pose.z - 0) > 1e-5)
			z.push_back(Tmp_pose.z);
		if (abs(Tmp_pose.Rx - 0) > 1e-5)
			Rx.push_back(Tmp_pose.Rx);
		if (abs(Tmp_pose.Ry - 0) > 1e-5)
			Ry.push_back(Tmp_pose.Ry);
		if (abs(Tmp_pose.Rz - 0) > 1e-5)
			Rz.push_back(Tmp_pose.Rz);
		if (abs(Tmp_pose.distance - 0) > 1e-5)
			distance.push_back(Tmp_pose.distance);
	}

	AprilTag_pose.x = AbnormalRemove(x);
	AprilTag_pose.y = AbnormalRemove(y);
	AprilTag_pose.z = AbnormalRemove(z);
	AprilTag_pose.Rx = AbnormalRemove(Rx);
	AprilTag_pose.Ry = AbnormalRemove(Ry);
	AprilTag_pose.Rz = AbnormalRemove(Rz);
	AprilTag_pose.distance = AbnormalRemove(distance);


	return AprilTag_pose;
}
#pragma endregion

#pragma region 辅助函数
//获取时间
std::string get_time()
{
	std::time_t now = time(0);

	std::tm* ltm = localtime(&now);

	// 输出 tm 结构的各个组成部分

	std::string s;

	s += std::to_string(1900 + ltm->tm_year);
	s += "-";
	s += std::to_string(1 + ltm->tm_mon);
	s += "-";
	s += std::to_string(ltm->tm_mday);
	s += "-";
	s += std::to_string(ltm->tm_hour);
	s += "-";
	s += std::to_string(ltm->tm_min);
	s += "-";
	s += std::to_string(ltm->tm_sec);

	return s;
}

//旋转矩阵转欧拉角
AprilTag_DATA R2RPY(Eigen::Matrix3d R, Eigen::Vector3d T)
{
	double e = 1 - 3;
	AprilTag_DATA data;
	if (abs(R(2, 0) - 1) < e || abs(R(2, 0) + 1) < e)
	{
		return data;
	}
	else
	{
		data.x = T[0];
		data.y = T[1];
		data.z = T[2];

		data.Rx = atan2(R(2, 1), R(2, 2));
		data.Rz = atan2(R(1, 0), R(0, 0));

		if (abs(cos(data.Rz)) > abs(sin(data.Rz)))
			data.Ry = atan2(-1 * R(2, 0), R(0, 0) / cos(data.Rz));
		else
			data.Ry = atan2(-1 * R(2, 0), R(1, 0) / sin(data.Rz));
	}
	return data;
}

//AprilTag坐标系与靶标坐标系变换
Eigen::Isometry3d AprilTag2Target(Eigen::Isometry3d transform, int ID)
{
	Eigen::Isometry3d TA2t = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d Cam2t = Eigen::Isometry3d::Identity();

	switch (ID)
	{
	case 0://AprilTag0->target
		TA2t.translate(Eigen::Vector3d(0, 0, 0.04009));
		TA2t.rotate(Eigen::AngleAxisd(PI / 2, Eigen::Vector3d(1, 0, 0)));
		break;
	case 1://AprilTag0->target
		TA2t.translate(Eigen::Vector3d(0, 0, 0.036));
		TA2t.rotate(Eigen::AngleAxisd(PI, Eigen::Vector3d(1, 0, 0)));
		break;
	case 2://AprilTag0->target
		TA2t.translate(Eigen::Vector3d(0, 0, 0.04009));
		TA2t.rotate(Eigen::AngleAxisd(PI / 18, Eigen::Vector3d(1, 0, 0)));
		TA2t.rotate(Eigen::AngleAxisd(PI / 2, Eigen::Vector3d(0, 1, 0)));
		TA2t.rotate(Eigen::AngleAxisd(PI / 2, Eigen::Vector3d(1, 0, 0)));
		break;
	case 3://AprilTag0->target
		TA2t.translate(Eigen::Vector3d(0, 0, 0.04009));
		TA2t.rotate(Eigen::AngleAxisd(PI / 18, Eigen::Vector3d(1, 0, 0)));
		TA2t.rotate(Eigen::AngleAxisd(PI / 2, Eigen::Vector3d(1, 0, 0)));
		TA2t.rotate(Eigen::AngleAxisd(PI, Eigen::Vector3d(0, 0, 1)));
		break;
	case 4://AprilTag0->target
		TA2t.translate(Eigen::Vector3d(0, 0, 0.04009));
		TA2t.rotate(Eigen::AngleAxisd(PI / 18, Eigen::Vector3d(1, 0, 0)));
		TA2t.rotate(Eigen::AngleAxisd(-PI / 2, Eigen::Vector3d(0, 1, 0)));
		TA2t.rotate(Eigen::AngleAxisd(PI / 2, Eigen::Vector3d(1, 0, 0)));
		break;
	}
	Cam2t = transform * TA2t;
	return Cam2t;
}

//判断AprilTag识别结果
bool judge(AprilTag_DATA AprilTag_pose)
{
	if (abs(AprilTag_pose.x - 0) < 1e-1 && abs(AprilTag_pose.y - 0) < 1e-1 && abs(AprilTag_pose.z - 0) < 1e-1
		&& abs(AprilTag_pose.Rx - 0) < 1e-1 && abs(AprilTag_pose.Ry - 0) < 1e-1 && abs(AprilTag_pose.Rz - 0) < 1e-1)
	{
		return false;
	}
	else
		return true;
}

//求取数组固定分位数
double calculatePercentile(deque<double> sortedArr, double percentile)
{
	// Step 1: Calculate the index
	double index = (sortedArr.size() - 1) * percentile;
	int lowerIndex = std::floor(index);
	int upperIndex = std::ceil(index);

	// Step 2: Check if the index is an integer
	if (lowerIndex == upperIndex) {
		return sortedArr[lowerIndex];
	}

	// Step 3: Linear interpolation
	double lowerValue = sortedArr[lowerIndex];
	double upperValue = sortedArr[upperIndex];
	return lowerValue + (upperValue - lowerValue) * (index - lowerIndex);
}

double predict(int tt_flag, vector<double>& datav)
{
	int num = datav.size();
	double err = 0;
	std::string ModelFileName;
	switch (tt_flag)
	{
	case 0:
		ModelFileName = "C:\\software\\SSBOT2-数据测试\\误差补偿模型\\SVR_6_model_rx";
		break;
	case 1:
		ModelFileName = "C:\\software\\SSBOT2-数据测试\\误差补偿模型\\SVR_6_model_ry";
		break;
	case 2:
		ModelFileName = "C:\\software\\SSBOT2-数据测试\\误差补偿模型\\SVR_6_model_rz";
		break;
	}
	svm_node* sample = vector2svmnode(datav);
	svm_model* SVMmodel = svm_load_model(ModelFileName.c_str());
	double predictvalue = svm_predict(SVMmodel, sample);		//返回值为1或0
	return predictvalue;
}

svm_node* vector2svmnode(const vector<double>& v)
{
	int l = v.size();
	svm_node* sn = new svm_node[l + 1];
	int i = 0;
	for (auto d : v)
	{
		sn[i].index = i + 1;	//index从1开始
		sn[i].value = d;
		i++;
	}
	sn[l].index = -1;			//标志结束
	return sn;
}

AprilTag_DATA ErrCompensate(AprilTag_DATA AprilTag_pose, int ID)
{
	torch::jit::script::Module Rx_model, Ry_model, Rz_model;
	double Rx_input_mean, Rx_input_standard, Ry_input_mean, Ry_input_standard, Rz_input_mean, Rz_input_standard, length_input_mean, length_input_standard;
	double Rx_mean, Rx_standard, Ry_mean, Ry_standard, Rz_mean, Rz_standard;
	switch (ID)
	{
		case 0:
			Rx_model = torch::jit::load("./libtorchmodel/0_Rx_model.pt");
			Ry_model = torch::jit::load("./libtorchmodel/0_Ry_model.pt");
			Rz_model = torch::jit::load("./libtorchmodel/0_Rz_model.pt");
			Rx_input_mean = -15.36620663;
			Rx_input_standard = 13.42856038;
			Ry_input_mean = -0.61479048;
			Ry_input_standard = 14.38646277;
			Rz_input_mean = 46.31651479;
			Rz_input_standard = 68.59573661;
			length_input_mean = 0.85537362;
			length_input_standard = 0.11266972;
			Rx_mean = 7.52126576;
			Rx_standard = 2.10253295;
			Ry_mean = 1.92885093;
			Ry_standard = 1.85027208;
			Rz_mean = 4.59675722;
			Rz_standard = 3.86464869;
			break;
		case 1:
			Rx_model = torch::jit::load("./libtorchmodel/1_Rx_model.pt");
			Ry_model = torch::jit::load("./libtorchmodel/1_Ry_model.pt");
			Rz_model = torch::jit::load("./libtorchmodel/1_Rz_model.pt");
			Rx_input_mean = 1.88057843;
			Rx_input_standard = 15.53915072;
			Ry_input_mean = -6.7759786;
			Ry_input_standard = 14.94767113;
			Rz_input_mean = 20.27337381;
			Rz_input_standard = 99.07124397;
			length_input_mean = 0.84836735;
			length_input_standard = 0.11702528;
			Rx_mean = -1.90993838;
			Rx_standard = 2.14764745;
			Ry_mean = 7.38666563;
			Ry_standard = 1.81581006;
			Rz_mean = 3.81646764;
			Rz_standard = 4.30402222;
			break;
		case 2:
			Rx_model = torch::jit::load("./libtorchmodel/2_Rx_model.pt");
			Ry_model = torch::jit::load("./libtorchmodel/2_Ry_model.pt");
			Rz_model = torch::jit::load("./libtorchmodel/2_Rz_model.pt");
			Rx_input_mean = 0.6695419;
			Rx_input_standard = 15.61981004;
			Ry_input_mean = -7.7587127;
			Ry_input_standard = 13.69952986;
			Rz_input_mean = -20.30174977;
			Rz_input_standard = 76.19546089;
			length_input_mean = 0.87609822;
			length_input_standard = 0.10788736;
			Rx_mean = -0.81965738;
			Rx_standard = 1.7084081;
			Ry_mean = 1.21136859;
			Ry_standard = 1.70158602;
			Rz_mean = 2.89412852;
			Rz_standard = 4.22318838;
			break;
		case 3:
			Rx_model = torch::jit::load("./libtorchmodel/3_Rx_model.pt");
			Ry_model = torch::jit::load("./libtorchmodel/3_Ry_model.pt");
			Rz_model = torch::jit::load("./libtorchmodel/3_Rz_model.pt");
			Rx_input_mean = 8.6408113;
			Rx_input_standard = 14.2452327;
			Ry_input_mean = -4.03359049;
			Ry_input_standard = 14.8421818;
			Rz_input_mean = -5.66952624;
			Rz_input_standard = 119.224204;
			length_input_mean = 0.87757999;
			length_input_standard = 0.104793381;
			Rx_mean = -1.89305202;
			Rx_standard = 2.0793236;
			Ry_mean = 3.16291187;
			Ry_standard = 1.63794111;
			Rz_mean = 3.53764403;
			Rz_standard = 4.15465165;
			break;
		case 4:
			Rx_model = torch::jit::load("./libtorchmodel/4_Rx_model.pt");
			Ry_model = torch::jit::load("./libtorchmodel/4_Ry_model.pt");
			Rz_model = torch::jit::load("./libtorchmodel/4_Rz_model.pt");
			Rx_input_mean = 2.54698345;
			Rx_input_standard = 15.70095869;
			Ry_input_mean = 4.68838295;
			Ry_input_standard = 13.32555067;
			Rz_input_mean = 70.88915633;
			Rz_input_standard = 99.67705869;
			length_input_mean = 0.86989781;
			length_input_standard = 0.10654397;
			Rx_mean = -3.66126916;
			Rx_standard = 1.78891359;
			Ry_mean = 2.41161705;
			Ry_standard = 1.50987743;
			Rz_mean = 2.46084367;
			Rz_standard = 3.95077962;
			break;
	}

	vector<double> data;
	double Rx_output, Ry_output, Rz_output;
	data.push_back((AprilTag_pose.Rx / PI * 180 - Rx_input_mean) / Rx_input_standard);
	data.push_back((AprilTag_pose.Ry / PI * 180 - Ry_input_mean) / Ry_input_standard);
	data.push_back((AprilTag_pose.Rz / PI * 180 - Rz_input_mean) / Rz_input_standard);
	data.push_back((AprilTag_pose.distance - length_input_mean) / length_input_standard);
	
	auto tensor = torch::tensor({
	  {data[0], data[1], data[2], data[3]} });

	// 构建示例输入
	std::vector<torch::jit::IValue> inputs;
	// 将张量添加到输入向量中
	inputs.push_back(tensor);
	inputs.push_back(tensor);

	// 执行模型推理并输出tensor
	at::Tensor Rx_tnesor_output = Rx_model.forward(inputs).toTensor();
	at::Tensor Ry_tnesor_output = Ry_model.forward(inputs).toTensor();
	at::Tensor Rz_tnesor_output = Rz_model.forward(inputs).toTensor();

	Rx_output = (Rx_tnesor_output[0][0].item<double>() * Rx_standard) + Rx_mean;
	Ry_output = (Ry_tnesor_output[0][0].item<double>() * Ry_standard) + Ry_mean;
	Rz_output = (Rz_tnesor_output[0][0].item<double>() * Rz_standard) + Rz_mean;

	AprilTag_pose.Rx = AprilTag_pose.Rx + Rx_output / 180 * PI;
	AprilTag_pose.Ry = AprilTag_pose.Ry + Ry_output / 180 * PI;
	AprilTag_pose.Rz = AprilTag_pose.Rz + Rz_output / 180 * PI;

	return AprilTag_pose;
}

double AbnormalRemove(deque<double> data)
{
	int num = data.size();
	double Q1, Q2, Q3;
	double IQR, MAX_Q, MIN_Q;
	double k_order;
	double k_left, k_right;
	int flag = 0;
	double sum = 0;

	int positivenum = 0, negativenum = 0;
	for (int ii = 0; ii < data.size(); ii++)
	{
		if (data[ii] > 0)
			positivenum++;
		if (data[ii] < 0)
			negativenum++;
	}
	if (positivenum >= negativenum)
	{
		for (int ii = 0; ii < data.size(); ii++)
		{
			if (data[ii] < 0)
				data[ii] = -1.0 * data[ii];
		}
	}
	else
	{
		for (int ii = 0; ii < data.size(); ii++)
		{
			if (data[ii] > 0)
				data[ii] = -1.0 * data[ii];
		}
	}

	sort(data.begin(), data.end());

	if (num <= 3)
	{
		for (int i = 0; i < data.size(); i++)
		{
			sum = sum + data[i];
		}
		return sum / data.size();
	}


	/*k_order = (double)num * 0.25;
	k_left = ceil(k_order) - k_order;
	k_right = k_order - floor(k_order);
	if (abs(k_left) < 1e-4 && abs(k_right) < 1e-4)
		Q1 = data[(int)floor(k_order)];
	else
	{
		if((int)floor(k_order) != 0)
			Q1 = k_left * data[(int)floor(k_order) - 1] + k_right * data[(int)ceil(k_order) - 1];
		else
			Q1 = k_right * data[(int)ceil(k_order) - 1];
	}*/
	Q1 = calculatePercentile(data, 0.25);

	/*k_order = (double)num * 0.5;
	k_left = ceil(k_order) - k_order;
	k_right = k_order - floor(k_order);
	if (abs(k_left) < 1e-4 && abs(k_right) < 1e-4)
		Q2 = data[(int)floor(k_order)];
	else
	{
		if ((int)floor(k_order) != 0)
			Q2 = k_left * data[(int)floor(k_order) - 1] + k_right * data[(int)ceil(k_order) - 1];
		else
			Q2 = k_right * data[(int)ceil(k_order) - 1];
	}*/
	Q2 = calculatePercentile(data, 0.5);

	/*k_order = (double)num * 0.75;
	k_left = ceil(k_order) - k_order;
	k_right = k_order - floor(k_order);
	if (abs(k_left) < 1e-4 && abs(k_right) < 1e-4)
		Q3 = data[(int)floor(k_order)];
	else
	{
		if ((int)floor(k_order) != 0)
			Q3 = k_left * data[(int)floor(k_order) - 1] + k_right * data[(int)ceil(k_order) - 1];
		else
			Q3 = k_right * data[(int)ceil(k_order) - 1];
	}*/
	Q3 = calculatePercentile(data, 0.75);

	IQR = Q3 - Q1;
	MAX_Q = Q3 + 1.5 * IQR;
	MIN_Q = Q1 - 1.5 * IQR;

	while (flag==0)
	{
		if (data[0] < MIN_Q)
			data.pop_front();
		if (data[data.size()-1] > MAX_Q)
			data.pop_back();
		if (data[0] >= MIN_Q && data[data.size() - 1] <= MAX_Q)
			flag = 1;
	}

	sum = 0;
	for (int i = 0; i < data.size(); i++)
	{
		sum = sum + data[i];
	}
	if (data.size() > 0)
		return sum / data.size();
	else
		return 0;
}

//获取术野相机二维码中心坐标
vector<double> CentreGet(cv::Mat HDframe, int t)
{
	getopt_t* getopt = getopt_create();

	// 添加一个布尔型选项 '-h' 或者 "--help"，用于显示帮助信息
	getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
	// 添加一个布尔型选项 '-d' 或者 "--debug"，启用调试输出（速度较慢
	getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");//关闭debug模式可以加快运行速度
	// 添加一个布尔型选项 '-q' 或者 "--quiet"，减少输出
	getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
	// 添加一个字符串选项 '-f' 或者 "--family"，指定要使用的标签族，默认为 "tag36h11"
	getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
	// 添加一个整数选项 '-t' 或者 "--threads"，指定要使用的 CPU 线程数，默认为 1
	getopt_add_int(getopt, 't', "threads", "16", "Use this many CPU threads");
	// 添加一个双精度浮点数选项 '-x' 或者 "--decimate"，指定输入图像的降采样因子，默认为 2.0
	getopt_add_double(getopt, 'x', "decimate", "3.0", "Decimate input image by this factor");
	// 添加一个双精度浮点数选项 '-b' 或者 "--blur"，对输入图像应用低通滤波，默认为 0.0
	getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
	// 添加一个布尔型选项 '-0' 或者 "--refine-edges"，花费更多时间尝试对齐标签的边缘
	getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

	//这行代码声明了一个AprilTag家族的指针，并将其初始化为空
	apriltag_family_t* tf = NULL;
	//这段代码根据用户选择的AprilTag家族名称，动态创建相应的AprilTag家族对象
	const char* famname = getopt_get_string(getopt, "family");
	if (!strcmp(famname, "tag36h11")) {
		tf = tag36h11_create();
	}
	else {
		cout << "Unrecognized tag family name." << endl;
	}

	//这行代码创建了一个AprilTag检测器对象
	apriltag_detector_t* td = apriltag_detector_create();//检测器
	 //这行代码将选定的AprilTag家族添加到AprilTag检测器中
	apriltag_detector_add_family(td, tf);
	//这行代码设置AprilTag检测器的四边形减采样参数，即图像下采样的因子
	td->quad_decimate = getopt_get_double(getopt, "decimate");
	//这行代码设置AprilTag检测器的四边形模糊参数，即图像模糊程度
	td->quad_sigma = getopt_get_double(getopt, "blur");
	//这行代码设置AprilTag检测器的线程数目，这里固定为1
	td->nthreads = getopt_get_int(getopt, "threads"); //td->nthreads = 8;
	//这行代码设置AprilTag检测器是否输出调试信息
	td->debug = getopt_get_bool(getopt, "debug");
	//这行代码设置AprilTag检测器是否尝试对边缘进行更精细的调整
	td->refine_edges = getopt_get_bool(getopt, "refine-edges");

	cv::Mat gray;
	cv::cvtColor(HDframe, gray, cv::COLOR_BGR2GRAY);

	image_u8_t im =
	{ gray.cols,  //width
		gray.rows,  //height
		gray.cols,  //stride 对齐内存，提高处理速度
		gray.data  //buffer
	};

	//利用AprilTag检测器 td 对灰度图像进行AprilTag检测，返回一个指向 zarray_t 结构体的指针，其中包含检测到的AprilTag信息
	zarray_t* detections = apriltag_detector_detect(td, &im);
	//cout << "相机" << num << "识别到" << zarray_size(detections) << " tags" << endl;

	vector<double> centrepose;
	//遍历检测到的AprilTag，获取每个检测到的标记的信息
	for (int i = 0; i < zarray_size(detections); i++)
	{
		apriltag_detection_t* det;  //识别到的单个apriltag对象
		zarray_get(detections, i, &det); //单个apriltag对象赋值

		if (det->id == 1)
		{
			centrepose.push_back(det->c[0]);
			centrepose.push_back(det->c[1]);
		}

		if (t == 1)
		{
			cv::line(HDframe, cv::Point(det->p[0][0], det->p[0][1]),
				cv::Point(det->p[1][0], det->p[1][1]),
				cv::Scalar(0, 0xff, 0), 1);
			cv::line(HDframe, cv::Point(det->p[0][0], det->p[0][1]),
				cv::Point(det->p[3][0], det->p[3][1]),
				cv::Scalar(0, 0, 0xff), 1);
			cv::line(HDframe, cv::Point(det->p[1][0], det->p[1][1]),
				cv::Point(det->p[2][0], det->p[2][1]),
				cv::Scalar(0xff, 0, 0), 1);
			cv::line(HDframe, cv::Point(det->p[2][0], det->p[2][1]),
				cv::Point(det->p[3][0], det->p[3][1]),
				cv::Scalar(0xff, 0, 0), 1);
			std::stringstream ss;
			ss << det->id;
			std::string text = ss.str();
			int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
			double fontscale = 1.0;
			int baseline;
			//在矩形框中间加上id号
			//getTextSize 是一个函数调用，它用来计算给定文本在指定字体、字号和厚度下的大小
			cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
				&baseline);
			//在原始帧上绘制AprilTag的ID号
			cv::putText(HDframe, text, cv::Point(det->c[0] - textsize.width / 2,
				det->c[1] + textsize.height / 2),
				fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);


			/*cv::imshow("相机 0 拍摄画面", HDframe);
			cv::imwrite("C:\\Users\\Butel\\Desktop\\测试图片\\" + to_string(picture_svae) + ".jpg", HDframe);
			cv::waitKey(20);*/
				
		}
	}

	apriltag_detections_destroy(detections);
	apriltag_detector_destroy(td);

	if (!strcmp(famname, "tag36h11")) {
		tag36h11_destroy(tf);
	}

	getopt_destroy(getopt);

	if(centrepose.size()==0)
	{
		centrepose.push_back(0.0);
		centrepose.push_back(0.0);
	}
	return centrepose;
}

//获取术野相机YOLO识别中心坐标
vector<double> YOLOCentreGet(cv::Mat& HDframe, YOLO yolo_model)
{
	yolo_model.detect(HDframe);
	vector<double>  YOLO_centre_point;
	if (!(heartbox.x == 0 && heartbox.y == 0 && heartbox.width == 0 && heartbox.height == 0))
	{
		cv::Point centre;
		centre.x = heartbox.x + 0.5 * heartbox.width;
		centre.y = heartbox.y + 0.5 * heartbox.height;

		YOLO_centre_point.push_back(centre.x);
		YOLO_centre_point.push_back(centre.y);
	}
	else
	{
		YOLO_centre_point.push_back(0.0);
		YOLO_centre_point.push_back(0.0);
	}
	return YOLO_centre_point;
}

//求取视觉伺服的运动方向变换矩阵
void visualservo_get_T()
{
	//变换矩阵初始化
	{
		T10 = Eigen::Isometry3d::Identity();
		T21 = Eigen::Isometry3d::Identity();
		T32 = Eigen::Isometry3d::Identity();
		T43 = Eigen::Isometry3d::Identity();
		T54 = Eigen::Isometry3d::Identity();
		T65 = Eigen::Isometry3d::Identity();
		T73 = Eigen::Isometry3d::Identity();
		T83 = Eigen::Isometry3d::Identity();
		T90 = Eigen::Isometry3d::Identity();
		TA0 = Eigen::Isometry3d::Identity();
	}

	//以关节1圆心为原点（高度方向与关节4轴心平齐） T10
	{
		double theta1 = joint_now[0] / 180 * PI;

		T10.rotate(Eigen::AngleAxisd(theta1, Eigen::Vector3d(0, 0, 1)));
	}

	//关节2->关节1 T21
	{
		double theta2 = joint_now[1] / 180 * PI;

		T21.translate(Eigen::Vector3d(0, 0.7, 0));
		T21.rotate(Eigen::AngleAxisd(theta2, Eigen::Vector3d(0, 0, 1)));
	}

	//关节3->关节2 T32
	{
		double theta3 = joint_now[2] / 180 * PI;

		T32.translate(Eigen::Vector3d(0, 0.5, 0));
		T32.rotate(Eigen::AngleAxisd(theta3, Eigen::Vector3d(0, 0, 1)));
	}

	//关节4->关节3 T43
	{
		double theta4 = joint_now[3] / 180 * PI;

		T43.translate(Eigen::Vector3d(0, 0.4, 0));
		T43.rotate(Eigen::AngleAxisd(theta4, Eigen::Vector3d(0, 1, 0)));
	}

	//关节5->关节4 T54
	{
		double theta5 = joint_now[4] / 180 * PI;

		T54.translate(Eigen::Vector3d(0, 0.1118, 0));
		T54.rotate(Eigen::AngleAxisd(theta5, Eigen::Vector3d(1, 0, 0)));
	}

	//相机->关节5 T65
	{
		double theta6 = joint_now[5] / 180 * PI;
		T65.rotate(Eigen::AngleAxisd(PI, Eigen::Vector3d(1, 0, 0)));
		T65.rotate(Eigen::AngleAxisd(theta6, Eigen::Vector3d(0, 0, 1)));
	}

	visualservo_T_cam = T10 * T21 * T32 * T43 * T54 * T65;
}

//判断YOLO识别数值是否正常
bool judgeYOLO(Eigen::Vector4d point)
{
	if ((point[0] > -2.0 && point[0] < 2.0) && (point[1] > 0.2 && point[1] < 2.0) && (point[2] > 0.2 && point[2] < 1.8))
		return true;
	else
		return false;
}

//机械臂末端相机曝光度切换
void ExposuerSwitch(int flag)
{
	//曝光测试
	auto color_sensor_left = profiles[rs_end_left].get_device().first<rs2::color_sensor>();
	auto depth_sensor_left = profiles[rs_end_left].get_device().first<rs2::depth_sensor>();
	auto color_sensor_right = profiles[rs_end_right].get_device().first<rs2::color_sensor>();
	auto depth_sensor_right = profiles[rs_end_right].get_device().first<rs2::depth_sensor>();
	if (flag == 1)
	{
		if (color_sensor_left.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
			// 禁用自动曝光
			color_sensor_left.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.f);
		}

		if (depth_sensor_left.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
			// 禁用自动曝光
			depth_sensor_left.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.f);
		}

		if (depth_sensor_left.supports(RS2_OPTION_EXPOSURE)) {
			depth_sensor_left.set_option(RS2_OPTION_EXPOSURE, 860.f);//正常156附近
			depth_sensor_left.set_option(RS2_OPTION_GAIN, 16.f);
		}

		if (color_sensor_left.supports(RS2_OPTION_EXPOSURE)) {
			color_sensor_left.set_option(RS2_OPTION_EXPOSURE, 19.f);//正常156附近
			color_sensor_left.set_option(RS2_OPTION_GAIN, 0.f);
		}

		if (color_sensor_right.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
			// 禁用自动曝光
			color_sensor_right.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.f);
		}

		if (depth_sensor_right.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
			// 禁用自动曝光
			depth_sensor_right.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.f);
		}

		if (color_sensor_right.supports(RS2_OPTION_EXPOSURE)) {
			color_sensor_right.set_option(RS2_OPTION_EXPOSURE, 19.f);//正常156附近
			color_sensor_right.set_option(RS2_OPTION_GAIN, 0.f);
		}

		if (depth_sensor_right.supports(RS2_OPTION_EXPOSURE)) {
			depth_sensor_right.set_option(RS2_OPTION_EXPOSURE, 1200.f);//正常156附近
		}
	}
	else
	{
		if (!color_sensor_left.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
			// 禁用自动曝光
			color_sensor_left.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.f);
		}

		if (color_sensor_left.supports(RS2_OPTION_EXPOSURE)) {
			color_sensor_left.set_option(RS2_OPTION_EXPOSURE, 156.f);//正常156附近
			color_sensor_left.set_option(RS2_OPTION_GAIN, 16.f);
		}

		if (!color_sensor_right.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
			// 禁用自动曝光
			color_sensor_right.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.f);
		}

		if (color_sensor_right.supports(RS2_OPTION_EXPOSURE)) {
			color_sensor_right.set_option(RS2_OPTION_EXPOSURE, 156.f);//正常156附近
			color_sensor_right.set_option(RS2_OPTION_GAIN, 16.f);
		}
	}
}
#pragma endregion

//void main()
//{
//	rsData rd_test;
//
//	joint_now[0] = -179.916;
//	joint_now[1] = 150.001;
//	joint_now[2] = -149.986;
//	joint_now[3] = 0;
//	joint_now[4] = 0;
//
//	rd_test.get_T(0);
//
//	rs_camera_work();
//}

#pragma region YOLOv5代码
YOLO::YOLO(Net_config config)
{
	this->confThreshold = config.confThreshold;
	this->nmsThreshold = config.nmsThreshold;
	this->objThreshold = config.objThreshold;

	this->net = cv::dnn::readNet(config.modelpath);
	//使用GPU加速
	this->net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
	this->net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
	//只使用CPU
	/*this->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
	this->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);*/
	ifstream ifs("class.names");
	std::string line;
	while (getline(ifs, line)) this->class_names.push_back(line);
	this->num_class = class_names.size();

	//if (endsWith(config.modelpath, "6.onnx"))
	//{
	//	anchors = (float*)anchors_1280;
	//	this->num_stride = 4;
	//	this->inpHeight = 1280;
	//	this->inpWidth = 1280;
	//}
	//else
	//{
	//	anchors = (float*)anchors_640;
	//	this->num_stride = 3;
	//	this->inpHeight = 640;
	//	this->inpWidth = 640;
	//}

	anchors = (float*)anchors_640;
	this->num_stride = 3;
	this->inpHeight = 640;
	this->inpWidth = 640;
}

cv::Mat YOLO::resize_image(cv::Mat srcimg, int* newh, int* neww, int* top, int* left)
{
	int srch = srcimg.rows, srcw = srcimg.cols;
	*newh = this->inpHeight;
	*neww = this->inpWidth;
	cv::Mat dstimg;
	if (this->keep_ratio && srch != srcw) {
		float hw_scale = (float)srch / srcw;
		if (hw_scale > 1) {
			*newh = this->inpHeight;
			*neww = int(this->inpWidth / hw_scale);
			resize(srcimg, dstimg, cv::Size(*neww, *newh), cv::INTER_AREA);
			*left = int((this->inpWidth - *neww) * 0.5);
			copyMakeBorder(dstimg, dstimg, 0, 0, *left, this->inpWidth - *neww - *left, cv::BORDER_CONSTANT, 114);
		}
		else {
			*newh = (int)this->inpHeight * hw_scale;
			*neww = this->inpWidth;
			resize(srcimg, dstimg, cv::Size(*neww, *newh), cv::INTER_AREA);
			*top = (int)(this->inpHeight - *newh) * 0.5;
			copyMakeBorder(dstimg, dstimg, *top, this->inpHeight - *newh - *top, 0, 0, cv::BORDER_CONSTANT, 114);
		}
	}
	else {
		resize(srcimg, dstimg, cv::Size(*neww, *newh), cv::INTER_AREA);
	}
	return dstimg;
}

void YOLO::drawPred(float conf, int left, int top, int right, int bottom, cv::Mat& frame, int classid)   // Draw the predicted bounding box
{
	//Draw a rectangle displaying the bounding box
	rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 255), 2);

	//Get the label for the class name and its confidence
	std::string label = cv::format("%.2f", conf);
	label = this->class_names[classid] + ":" + label;

	//Display the label at the top of the bounding box
	int baseLine;
	cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	top = max(top, labelSize.height);
	//rectangle(frame, Point(left, top - int(1.5 * labelSize.height)), Point(left + int(1.5 * labelSize.width), top + baseLine), Scalar(0, 255, 0), FILLED);
	putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 1);
}

void YOLO::detect(cv::Mat& frame)
{
	int newh = 0, neww = 0, padh = 0, padw = 0;
	cv::Mat dstimg = this->resize_image(frame, &newh, &neww, &padh, &padw);
	cv::Mat blob = cv::dnn::blobFromImage(dstimg, 1 / 255.0, cv::Size(this->inpWidth, this->inpHeight), cv::Scalar(0, 0, 0), true, false);
	this->net.setInput(blob);
	vector<cv::Mat> outs;
	this->net.forward(outs, this->net.getUnconnectedOutLayersNames());

	int num_proposal = outs[0].size[1];
	int nout = outs[0].size[2];
	if (outs[0].dims > 2)
	{
		outs[0] = outs[0].reshape(0, num_proposal);
	}
	/////generate proposals
	vector<float> confidences;
	vector<cv::Rect> boxes;
	vector<int> classIds;
	float ratioh = (float)frame.rows / newh, ratiow = (float)frame.cols / neww;
	int n = 0, q = 0, i = 0, j = 0, row_ind = 0; ///xmin,ymin,xamx,ymax,box_score,class_score
	float* pdata = (float*)outs[0].data;
	float data1 = pdata[0], data2 = pdata[1], data3 = pdata[2], data4 = pdata[3], data5 = pdata[4], data6 = pdata[5];
	for (n = 0; n < this->num_stride; n++)   ///����ͼ�߶�
	{
		const float stride = pow(2, n + 3);
		int num_grid_x = (int)ceil((this->inpWidth / stride));
		int num_grid_y = (int)ceil((this->inpHeight / stride));
		for (q = 0; q < 3; q++)    ///anchor
		{
			const float anchor_w = this->anchors[n * 6 + q * 2];
			const float anchor_h = this->anchors[n * 6 + q * 2 + 1];
			for (i = 0; i < num_grid_y; i++)
			{
				for (j = 0; j < num_grid_x; j++)
				{
					float box_score = pdata[4];
					data5 = pdata[4];
					if (box_score > this->objThreshold)
					{
						cv::Mat scores = outs[0].row(row_ind).colRange(5, nout);
						cv::Point classIdPoint;
						double max_class_socre;
						// Get the value and location of the maximum score
						minMaxLoc(scores, 0, &max_class_socre, 0, &classIdPoint);
						max_class_socre *= box_score;
						if (max_class_socre > this->confThreshold)
						{
							const int class_idx = classIdPoint.x;
							//float cx = (pdata[0] * 2.f - 0.5f + j) * stride;  ///cx
							//float cy = (pdata[1] * 2.f - 0.5f + i) * stride;   ///cy
							//float w = powf(pdata[2] * 2.f, 2.f) * anchor_w;   ///w
							//float h = powf(pdata[3] * 2.f, 2.f) * anchor_h;  ///h

							float cx = pdata[0];  ///cx
							float cy = pdata[1];   ///cy
							float w = pdata[2];   ///w
							float h = pdata[3];  ///h

							data1 = pdata[0];
							data2 = pdata[1];
							data3 = pdata[2];
							data4 = pdata[3];

							int left = int((cx - padw - 0.5 * w) * ratiow);
							int top = int((cy - padh - 0.5 * h) * ratioh);

							confidences.push_back((float)max_class_socre);
							boxes.push_back(cv::Rect(left, top, (int)(w * ratiow), (int)(h * ratioh)));
							classIds.push_back(class_idx);
						}
					}
					row_ind++;
					pdata += nout;
				}
			}
		}
	}

	// Perform non maximum suppression to eliminate redundant overlapping boxes with
	// lower confidences
	vector<int> indices;
	cv::dnn::NMSBoxes(boxes, confidences, this->confThreshold, this->nmsThreshold, indices);//过滤低置信度并做非最大值抑制NMS
	for (size_t i = 0; i < indices.size(); ++i)
	{
		int idx = indices[i];
		cv::Rect box = boxes[idx];
		this->drawPred(confidences[idx], box.x, box.y, //用于在图像 frame 上绘制边界框，并显示检测的类别和置信度
			box.x + box.width, box.y + box.height, frame, classIds[idx]);
		if (i == 0)
			heartbox = box;
	}
}
#pragma endregion