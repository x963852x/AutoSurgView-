/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of collision test

**************************************************************************/

#pragma once
#include <vector>

#include "opencv2/opencv.hpp"
#include "q_tree.h"

//机械臂节点、障碍物节点 类型
#define TYPE_CIRCLE 0
#define TYPE_RECTANGLE 1

#define PI 3.1415926

//机械臂结构体，存储机械臂各段长度，单位/m
struct ARM
{
	double L1 = 0.7;
	double L2 = 0.5;
	double L3 = 0.4;
};

#pragma region 机械臂节点类
//机械臂节点类
class arm_node
{
public:
	arm_node();	//机械臂节点类默认构造函数
	arm_node(double x, double y, double r);	//机械臂节点类构造函数——对象类型为 圆
	arm_node(double x, double y, double a, double b);	//机械臂节点类构造函数——对象类型为 矩形
	~arm_node();
	int type;		//节点类型
	cv::Point2d center;	//节点中心
	double radius;		//节点半径
	double length_a, length_b;	//节点长、宽
	double angle;		//节点旋转角度
};
#pragma endregion

#pragma region 机械臂碰撞模型类
//机械臂碰撞模型类
class arm_collision_model
{
private:
	//初始机械臂各段节点化表示
	std::vector<arm_node> arm_1{ arm_node(0,0,70), arm_node(0,350,110,630), arm_node(0,700,70) };
	std::vector<arm_node> arm_2{ arm_node(0,250,85,410) };
	std::vector<arm_node> arm_3{ arm_node(0,0,68),arm_node(0,240,85,380) };
	std::vector<arm_node> arm_4{ arm_node(0,85,168,160) };

	//机械臂节点 vector,保存原始节点数据
	std::vector<std::vector<arm_node> > arm_origin{ arm_1 ,arm_2, arm_3, arm_4 };

	//机械臂自碰撞检测矩阵 
	bool self_collision_array[4][4];
public:
	//机械臂节点 vector,计算在相应角度下节点数据
	std::vector<std::vector<arm_node> > arm{ arm_1 ,arm_2, arm_3, arm_4 };

	arm_collision_model();	//机械臂碰撞模型类默认构造函数
	~arm_collision_model();
	void arm_set(double angle_1, double angle_2, double angle_3);	//机械臂碰撞模型类成员函数：设置机械臂状态
	bool self_collision();	//机械臂碰撞模型类成员函数：检测是否发生自碰撞
	bool obstacle_collision(std::vector<cv::RotatedRect>obstacles);	//机械臂碰撞模型类成员函数：检测是否与障碍物发生碰撞
	bool collision(double angle_1, double angle_2, double angle_3, std::vector<cv::RotatedRect>obstacles);	//机械臂碰撞模型类成员函数：检测是否发生碰撞——包含自碰撞与障碍物碰撞
	bool collision(double angle_1, double angle_2, double angle_3);	//机械臂碰撞模型类成员函数：检测是否发生碰撞——仅包含自碰撞
};
#pragma endregion

#pragma region 碰撞检测功能函数
bool rotated_rectangle_circle_collision(cv::Point2d center1, double length_a, double length_b, double angle,
	cv::Point2d center2, double radius);	//碰撞检测函数——针对矩形与圆形
bool rotated_rectangle_collision(cv::Point2d center1, double length_a1, double length_b1, double angle1,
	cv::Point2d center2, double length_a2, double length_b2, double angle2);	//碰撞检测函数——针对两圆形
bool circle_collision(cv::Point2d center1, double radius1, cv::Point2d center2, double radius2);	//碰撞检测函数——针对两圆形
bool node_obstacle_collision(arm_node& node, cv::RotatedRect ob);	//碰撞检测函数——针对 机械臂节点 与 旋转矩形
#pragma endregion