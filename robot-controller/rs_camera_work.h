#pragma once
#include <iostream>
#include <thread>
#include <vector>
#include <fstream>

#include "opencv2/opencv.hpp"
#include "librealsense2/rs.hpp"
#include "camera.h"
#include "Eigen/Dense"
#include "Eigen/SVD"
#include "motor_work.h"
#include "button.h"
#include "udp_control.h"
#include "motor_work.h"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/dnn.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "torch/script.h"
#include <torch/torch.h>
//#include <opencv2/dnn_superres.hpp>
#include <opencv2/core/utils/logger.hpp>
#include "svm.h"

extern "C" {
#include "apriltag_pose.h"
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
}

#include "ssbot_config.h"
#include "opencv2/imgproc/types_c.h"
#pragma region 宏定义变量
//目标拍摄位置获取状态
#define TARGET_GET_IMAGE_PRE 3;	//准备获取图像
#define TARGET_GET_IMAGE 1;	//获取图像
#define TARGET_GET_POSE 2;	//获取位姿
#define TARGET_FREE 0	//空闲

#define WORK_STATE_TARGET 1;	//目标拍摄位置识别模式
#define WORK_STATE_OCCLUSION_AVOIDANCE 0;	//遮挡规避模式

//视觉伺服控制状态
#define VISUAL_SERVO_WAIT 1; //准备视觉伺服
#define VISUAL_SERVO_WORK 2; //视觉伺服中
#define VISUAL_SERVO_FREE 0; //视觉伺服空闲状态
#define VISUAL_SERVO_YOLO_WAIT 3;//YOLO识别时视觉伺服准备
#define VISUAL_SERVO_YOLO_WORK 4;//YOLO识别视觉伺服中
//视觉伺服轨迹更新状态
#define VISUAL_SERVO_TRAJ_FREE 0; //视觉伺服更新空闲
#define VISUAL_SERVO_TRAJ_UPDATE_FINISHED 2; //视觉伺服更新轨迹完成
#define VISUAL_SERVO_TRAJ_UPDATE 1; //视觉伺服等待更新轨迹状态

//心脏识别与被动靶标识别状态
#define HEART_DETECT 1;
#define PICTURE_DETECT 2;
#define FREE_DETECT 0;
#pragma endregion


#pragma region YOLOv5目标检测
struct Net_config
{
	float confThreshold; // 置信度阈值
	float nmsThreshold;  // 非最大值抑制阈值
	float objThreshold;  // 物体置信度阈值
	std::string modelpath;
};

//int endsWith(std::string s, std::string sub) {
//	return s.rfind(sub) == (s.length() - sub.length()) ? 1 : 0;
//}

const float anchors_640[3][6] = { {10.0,  13.0, 16.0,  30.0,  33.0,  23.0},
								 {30.0,  61.0, 62.0,  45.0,  59.0,  119.0},
								 {116.0, 90.0, 156.0, 198.0, 373.0, 326.0} };

const float anchors_1280[4][6] = { {19, 27, 44, 40, 38, 94},{96, 68, 86, 152, 180, 137},{140, 301, 303, 264, 238, 542},
					   {436, 615, 739, 380, 925, 792} };

class YOLO
{
public:
	YOLO(Net_config config);
	void detect(cv::Mat& frame);
private:
	float* anchors;
	int num_stride;
	int inpWidth;
	int inpHeight;
	vector<std::string> class_names;
	int num_class;

	float confThreshold;
	float nmsThreshold;
	float objThreshold;
	const bool keep_ratio = true;
	cv::dnn::Net net;
	void drawPred(float conf, int left, int top, int right, int bottom, cv::Mat& frame, int classid);
	cv::Mat resize_image(cv::Mat srcimg, int* newh, int* neww, int* top, int* left);
};
#pragma endregion


//陀螺仪数据结构体
struct GYRO_DATA
{
	float x = 0;
	float y = 0;
	float z = 0;
	bool ready = false;
};

//AprilTag识别位姿结构体
struct AprilTag_DATA
{
	double x = 0;
	double y = 0;
	double z = 0;
	double Rx = 0;
	double Ry = 0;
	double Rz = 0;

	double distance = 0;
};

#pragma region rsData类
//rsData 结构体存储单个rs相机图像
class rsData
{
public:
	std::vector<cv::Mat> left_pictures, right_pictures;	//左右图像集
	std::vector<cv::Mat> RGB_pictures;  //RGB图
	Eigen::Vector4d  point_rec_in_3d_cam;	//靶标圆心在3D相机坐标系下坐标

	Eigen::Isometry3d T_3d_cam = Eigen::Isometry3d::Identity();	//3D camera变换矩阵
	Eigen::Isometry3d T_cam = Eigen::Isometry3d::Identity();	//相机变换矩阵

	bool processIR(int show, int index);	//rsData类成员函数：红外图像识别
	void processPNP();	//rsData类成员函数：目标位姿获取
	AprilTag_DATA rsData::processAprilTag(int show, int index, int num, int err_compensate); //rsData类成员函数：AprilTag的RGB图像识别
	AprilTag_DATA rsData::AprikTagMean(int index, int num, int show, int err_compensate);
	AprilTag_DATA rsData::getAprilTag(cv::Mat AprilTag_picture, int id, int t, int errcompensate);


	void get_T(int number);	//rsData类成员函数：获取转换矩阵

	rsData();
	~rsData();
};
#pragma endregion

#pragma region gloData类
class  gloData
{
public:
	std::vector<rsData> rs_datas;	//rs_data数组

	gloData();
	~gloData();
};
#pragma endregion

#pragma region gloRS类
class gloRS
{
public:
	gloRS(rs2::context& ctx, int num = 1);	//gloRS类默认构造函数
	void init();	//gloRS类成员函数：初始化
	void waitForData();	//gloRS类成员函数：等待图像数据
	void gloRS::getColorMat(cv::Mat& a);
	void getIrMat(cv::Mat& a, int LR = 1);	//gloRS类成员函数：获取红外图像
	void getDepthColorMat(cv::Mat& a, int num = 1);	//gloRS类成员函数：获取颜色渲染后的深度图像
	rs2::frame getDepthImage();	//gloRS类成员函数：获取深度图像
	void toggleIR(int IO = 0);	//gloRS类成员函数：红外图像曝光闪烁
	void showDepthIR(int showD = 1, int showI = 1, int showC = 0);	//gloRS类成员函数：显示深度图像与红外图像
	void recordIR(struct gloData* state);	//gloRS类成员函数：记录红外图像
	void gloRS::recordRGB(struct gloData* state); //gloRS类成员函数：记录RGB图像
	void set_work_state(int state);	//遮挡规避与靶标识别需切换相机参数
	Eigen::Vector4d getObjectDepth(YOLO yolo_modle, int cameraID); //配合YOLO目标检测结果获取相机坐标系下位置
	cv::Mat frame_to_mat(rs2::frame& f);
	cv::Mat depth_frame_to_meters(rs2::depth_frame& f);
	//cv::Mat img_zoomz_in(cv::Mat img, int i);

	bool initialized = false;
	rs2::frameset data;

	rs2::device D435;// = devices[1];
	rs2::sensor IR;//= D435.query_sensors()[0];

	int work_state = WORK_STATE_TARGET;	//1，靶标识别状态；0，遮挡规避状态

private:

	int number;	//3d camera索引
	rs2::device_list devices;//= ctx.query_devices();
	rs2::colorizer color_map;
	rs2::config cfg;
	rs2::pipeline pipe;
	std::mutex rs_data_mutex;
};
#pragma endregion

#pragma region realsense工作函数
void rs_camera_init();	//3D camera初始化函数
void rs_camera_work();	//3D camera工作函数
#pragma endregion

#pragma region 图像采集
void mainCapture();		//图像采集函数
void mainCapture0();	//3D camera 0图像程序
void mainCapture1();	//3D camera 1图像程序
void mainCapture2();	//3D camera 2图像程序
void mainCapture3();	//3D camera 3图像程序
void show_pic();
void rs_manager();	//3D camera 管理函数
#pragma endregion

#pragma region 位姿识别
void recognition();
void processAprilTagResult(AprilTag_DATA AprilTag_pose);
bool processYOLOResult(std::pair<double, double> position_tar, Eigen::Vector3d focuspoint);
bool OPprocessYOLOResult(std::pair<double, double> position_tar, std::pair<double, double> position_now, Eigen::Vector3d focus_point);
void AprilTag_recognition();
void AprilTag_visualservo();
#pragma endregion

#pragma region 辅助函数
std::string get_time();
AprilTag_DATA R2RPY(Eigen::Matrix3d R, Eigen::Vector3d T);
Eigen::Isometry3d AprilTag2Target(Eigen::Isometry3d transform, int ID);
bool judge(AprilTag_DATA AprilTag_pose);
double calculatePercentile(deque<double> sortedArr, double percentile);
double predict(int tt_flag, vector<double>& datav);
svm_node* vector2svmnode(const vector<double>& v);
AprilTag_DATA ErrCompensate(AprilTag_DATA AprilTag_pose, int ID);
double AbnormalRemove(deque<double> data);
vector<double> CentreGet(cv::Mat HDframe, int t);
vector<double> YOLOCentreGet(cv::Mat& HDframe, YOLO yolo_model);
void visualservo_get_T();
bool judgeYOLO(Eigen::Vector4d point);
void ExposuerSwitch(int flag);
#pragma endregion