#pragma once
#include <iostream>
#include <sstream>
#include <fstream>
#include <thread>
#include <vector>
#include <set>
#include <deque>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/dnn.hpp"
#include "Eigen/Dense"

#include "q_tree.h"
#include "rs_camera_work.h"
#include "motor_work.h"

//宏变量
#pragma region
//障碍寻优算法状态
#define OP_FREE 0;
#define OP_MAP_UPDATE 1;
#define OP_WORK 2;

//是否启用障碍寻优标志
#define OP_ON 1;
#define OP_OFF 0;
#define OP_ONCE 2;
#pragma endregion


#pragma region 障碍物寻优类
class Obstacle_Optimization
{
public:
	std::pair<double, double> OP_point;  //最优点
	std::pair<double, double> target_point;  //目标点 单位:m
	std::pair<double, double> endposition;  //目标点 单位:m
	int OP_resolution = 0;            //地图比例尺 单位:mm
	Eigen::MatrixXi OP_obs_mat;   //栅格地图矩阵
	std::vector<std::pair<int, int>> target_obstacle; //目标障碍物数组  矩阵中的位置点 单位:矩阵单位长度
	std::vector<std::pair<int, int>> target_obstacle_expansion; //膨胀后的目标障碍物数组  矩阵中的位置点 单位:矩阵单位长度
	std::vector<std::pair<int, int>> boundary_point;    //膨胀后的中心点  机器人坐标系中的位置点 单位:mm
	std::vector<std::pair<int, int>> convex_hull_point; //凸包边界点   机器人坐标系中的位置点 单位:mm

	std::vector<std::pair<int, int>> except_target_obstacle;       //除了目标点附近障碍物外的其他障碍物  矩阵中的位置点 单位:矩阵单位长度
	std::vector<std::pair<int, int>> except_target_obstacle_point; //除了目标点附近障碍物外的其他障碍物  机器人坐标系中的位置点  单位:mm

	double origin_step = 0;                       //搜索原始步长 单位:m
	double step_add = 0;                          //步长变化量 单位:m
	double step_max = 0;                          //步长最大值 单位:m
	double step_min = 0;						  //步长最小值 单位:m
	int expansion_distance = 0;				      //障碍物膨胀长度 单位:矩阵单位长度

	cv::Point2d optimization_position;        //优化求出的最佳位置

	int r = 8;                                //初始目标点附近障碍物检测距离 单位:矩阵单位长度

	double k_obstacle = 1.0;        //障碍物权重系数
	double k_target = 1.0;			//目标点权重系数
	double efficiency_distance = 0.5; //损失函数计算的障碍物有效距离 单位:m

public:
	Obstacle_Optimization();
	~Obstacle_Optimization();
	std::pair<double, double> GetTargetPoint(Eigen::Vector3d focus_point);
	void SetExpansionDitance(int distance);
	void SetStep(Eigen::Vector4d stepdata);
	void SetResolution(int map_resolution);
	std::vector<std::pair<int, int>> findNearOne(Eigen::MatrixXi& matrix, int x, int y, int radius);
	std::vector<std::pair<int, int>> findConnectedPoints(Eigen::MatrixXi& matrix, std::vector<std::pair<int, int>> points);
	void GetObsMat(Eigen::MatrixXi obs_mat);
	bool GetTargetObstacle(int type);
	void TargetObstacleExpansion();
	void GetBoundaryPoint();
	void GetConvexHull();
	double GetLoss(std::pair<double, double> OP_point, std::vector<std::pair<int, int>> selected_obstacle_point);
	void GetExceptTargetObstacle();
	std::vector<std::pair<int, int>> HullFilterObstacle(std::pair<double, double> start_point, std::pair<double, double> end_point);
	std::pair<double, double> GetOPPosition();
};
#pragma endregion

#pragma region
int orientation(const std::pair<int, int>& p, const std::pair<int, int>& q, const std::pair<int, int>& r);
bool compare(const std::pair<int, int>& p1, const std::pair<int, int>& p2, const std::pair<int, int>& p0);
int findBottomLeft(const std::vector<std::pair<int, int>>& points);
int countLargeDifferencePositions(const Eigen::MatrixXi& mat1, const Eigen::MatrixXi& mat2, int threshold);
#pragma endregion

void OP_test_thread();

