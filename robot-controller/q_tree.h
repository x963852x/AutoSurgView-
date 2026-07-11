#pragma once
#include <queue>
#include <unordered_set>
#include <stack>
#include "opencv2/opencv.hpp"
#include "Eigen/Dense"

#pragma region 宏定义变量
//四叉树地图节点状态
#define EMPTY 0	//空
#define TBD 1	//未知
#define HALF 2	//半满
#define SELFFULL 3	//满
#pragma endregion

#pragma region 四叉树地图节点类
class Node
{
public:
	Node()	//默认构造函数
	{
		R[0] = NULL;
		R[1] = NULL;
		R[2] = NULL;
		R[3] = NULL;
	}
	Node* R[4];	//节点指针数组
	cv::Point2d center;	//节点中心坐标
	int length;	//节点边长
	int val = EMPTY;	//节点值，初始为空
	int depth;	//节点深度
	int points_num = 0; //仅最高分辨率节点此值有效
};
#pragma endregion

#pragma region 四叉树地图类
class Q_Tree_Map
{
public:
	Q_Tree_Map(int length, int Resolution);	//Q_Tree_Map类构造函数
	~Q_Tree_Map();	//Q_Tree_Map类析构函数
	void clear(Node*& p);	//Q_Tree_Map类成员函数：递归删除节点p及其子节点
	void decay_for_rebuild(Node* p);	//Q_Tree_Map类成员函数：将节点中激光雷达点数量衰减
	void tbd_for_rebuild(Node* ptr);	//Q_Tree_Map类成员函数：将节点状态设置为未知
	void build_map(const std::vector<cv::Point2d> lp);	//Q_Tree_Map类成员函数：建图
	void insert(const cv::Point2d p);	//Q_Tree_Map类成员函数：将雷达点插入地图
	void full_node(Node* ptr);	//Q_Tree_Map类成员函数：确认节点状态
	void set_resolution(int Resolution);	//Q_Tree_Map类成员函数：设置四叉树地图分辨率

	void obs_mat_to_rec();	//Q_Tree_Map类成员函数：矩阵化地图获取矩形障碍物

	bool rects_in_obs_rects(cv::RotatedRect rec);	//Q_Tree_Map类成员函数：判断障碍物矩形是否在障碍物矩形数组中
	std::vector<cv::RotatedRect> Points_to_rects(std::vector<cv::Point2f>points);	//Q_Tree_Map类成员函数：栅格地图点数组转化为障碍物矩形数组

	void set_min_points_num(int v);	//Q_Tree_Map类成员函数：设置判满最小点数量
	void set_decay_speed(int v);	//Q_Tree_Map类成员函数：设置衰减速度
	void set_max_points_num(int v);	//Q_Tree_Map类成员函数：设置最大点数量

	void free();

	Node* root;	//根节点指针

	Eigen::MatrixXi obs_mat;
	Eigen::MatrixXi OBOP_obs_mat;
	std::vector<cv::RotatedRect> obs_rects;
	std::vector<cv::RotatedRect> obs_rects_new;

private:
	int min_points_num = 2;	//节点中激光雷达点判满最小数量
	int decay_speed = 3;	//节点中激光雷达点衰减速度
	int max_points_num = 10;	//节点中激光雷达点最大数量
	int resolution;	//地图分辨率
	int map_length;	//地图边长
	int obs_mat_full_val = 20;	//地图矩阵化表达，满值
	int obs_mat_empty_val = 255;	//地图矩阵化表达，空值
};
#pragma endregion