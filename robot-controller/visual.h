#pragma once

#include "opencv2/opencv.hpp"
#include "collision.h"
#include "motor_work.h"
#include "obstacle_optimization.h"
#include "occlusion_avoidance.h"

void visualization();	//可视化程序
void draw_arm(cv::Mat& src, std::vector<double> joint_now);	//绘制机械臂
void draw_lp(cv::Mat& src, const std::vector<cv::Point2d> lp);	//绘制激光雷达点
void draw_arm(cv::Mat& src, arm_collision_model& ACM);	//绘制机械臂
void draw_rotated_rectangle(cv::Mat& src, cv::Point2d center, double length_a, double length_b, double angle);	//绘制旋转矩形
void draw_qtm(cv::Mat& src, Node* ptr);	//递归绘制四叉树地图
void draw_qtm_obs_rects(cv::Mat& src, Q_Tree_Map& q);	//绘制四叉树地图障碍物