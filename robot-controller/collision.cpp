/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of collision test

**************************************************************************/

#include "collision.h"

using std::cout;
using std::endl;

#pragma region 全局变量
//全局机械臂结构体变量
struct ARM arm_collision;
arm_collision_model ACM;		//全局机械臂碰撞模型对象——用于路径规划时碰撞检测
arm_collision_model ACM_show;	//全局机械臂碰撞模型对象——用于可视化与实时碰撞检测，与机械臂真实情况绑定
#pragma endregion

#pragma region 机械臂节点类
//机械臂节点类默认构造函数
arm_node::arm_node()
{
	type = -1;
	radius = -1;
	length_a = -1;
	length_b = -1;
	angle = 0;
}

//机械臂节点类构造函数——对象类型为 圆
//@param x,y 节点中心坐标 单位mm
//@param r 节点半径 单位mm
arm_node::arm_node(double x, double y, double r)
{
	type = TYPE_CIRCLE;
	center.x = x;
	center.y = y;
	radius = r;
	length_a = -1;
	length_b = -1;
	angle = 0;
}

//机械臂节点类构造函数——对象类型为 矩形
//@param x,y 节点中心坐标 单位mm
//@param a,b 节点长、宽 单位mm
arm_node::arm_node(double x, double y, double a, double b)
{
	type = TYPE_RECTANGLE;
	center.x = x;
	center.y = y;
	radius = -1;
	length_a = a;
	length_b = b;
	angle = 0;
}

arm_node::~arm_node()
{
}
#pragma endregion

#pragma region 机械臂碰撞模型类
//机械臂碰撞模型类默认构造函数
arm_collision_model::arm_collision_model()
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			self_collision_array[i][j] = false;
		}
	}
	self_collision_array[0][2] = true;
	self_collision_array[0][3] = true;
	self_collision_array[1][3] = true;
}

arm_collision_model::~arm_collision_model()
{
}

//机械臂碰撞模型类成员函数：设置机械臂状态
//@param angle_1 关节1角度 单位 °
//@param angle_2 关节2角度 单位 °
//@param angle_3 关节3角度 单位 °
void arm_collision_model::arm_set(double angle_1, double angle_2, double angle_3)
{
	double x, y;
	cv::Point2d point_temp(0, 0);

	for (int i = 0; i < arm_origin[0].size(); i++)
	{
		arm_node node_a = arm_origin[0][i];

		x = node_a.center.x * cos(angle_1 / 57.29578) - node_a.center.y * sin(angle_1 / 57.29578);
		y = node_a.center.x * sin(angle_1 / 57.29578) + node_a.center.y * cos(angle_1 / 57.29578);

		arm[0][i].type = node_a.type;

		if (node_a.type == TYPE_CIRCLE)
		{
			arm[0][i].center.x = x;
			arm[0][i].center.y = y;
			arm[0][i].radius = node_a.radius;
		}
		else
		{
			arm[0][i].center.x = x;
			arm[0][i].center.y = y;
			arm[0][i].length_a = node_a.length_a;
			arm[0][i].length_b = node_a.length_b;
			arm[0][i].angle = angle_1;
		}
	}

	point_temp.x = -arm_collision.L1 * 1000 * sin(angle_1 / 57.29578);
	point_temp.y = arm_collision.L1 * 1000 * cos(angle_1 / 57.29578);

	for (int i = 0; i < arm_origin[1].size(); i++)
	{
		arm_node node_a = arm_origin[1][i];

		x = node_a.center.x * cos((angle_1 + angle_2) / 57.29578) - node_a.center.y * sin((angle_1 + angle_2) / 57.29578) + point_temp.x;
		y = node_a.center.x * sin((angle_1 + angle_2) / 57.29578) + node_a.center.y * cos((angle_1 + angle_2) / 57.29578) + point_temp.y;

		arm[1][i].type = node_a.type;

		if (node_a.type == TYPE_CIRCLE)
		{
			arm[1][i].center.x = x;
			arm[1][i].center.y = y;
			arm[1][i].radius = node_a.radius;
		}
		else
		{
			arm[1][i].center.x = x;
			arm[1][i].center.y = y;
			arm[1][i].length_a = node_a.length_a;
			arm[1][i].length_b = node_a.length_b;
			arm[1][i].angle = angle_1 + angle_2;
		}
	}

	point_temp.x -= arm_collision.L2 * 1000 * sin((angle_1 + angle_2) / 57.29578);
	point_temp.y += arm_collision.L2 * 1000 * cos((angle_1 + angle_2) / 57.29578);

	for (int i = 0; i < arm_origin[2].size(); i++)
	{
		arm_node node_a = arm_origin[2][i];

		x = node_a.center.x * cos((angle_1 + angle_2 + angle_3) / 57.29578) - node_a.center.y * sin((angle_1 + angle_2 + angle_3) / 57.29578) + point_temp.x;
		y = node_a.center.x * sin((angle_1 + angle_2 + angle_3) / 57.29578) + node_a.center.y * cos((angle_1 + angle_2 + angle_3) / 57.29578) + point_temp.y;

		arm[2][i].type = node_a.type;

		if (node_a.type == TYPE_CIRCLE)
		{
			arm[2][i].center.x = x;
			arm[2][i].center.y = y;
			arm[2][i].radius = node_a.radius;
		}
		else
		{
			arm[2][i].center.x = x;
			arm[2][i].center.y = y;
			arm[2][i].length_a = node_a.length_a;
			arm[2][i].length_b = node_a.length_b;
			arm[2][i].angle = angle_1 + angle_2 + angle_3;
		}
	}

	point_temp.x -= arm_collision.L3 * 1000 * sin((angle_1 + angle_2 + angle_3) / 57.29578);
	point_temp.y += arm_collision.L3 * 1000 * cos((angle_1 + angle_2 + angle_3) / 57.29578);

	for (int i = 0; i < arm_origin[3].size(); i++)
	{
		arm_node node_a = arm_origin[3][i];

		x = node_a.center.x * cos((angle_1 + angle_2 + angle_3) / 57.29578) - node_a.center.y * sin((angle_1 + angle_2 + angle_3) / 57.29578) + point_temp.x;
		y = node_a.center.x * sin((angle_1 + angle_2 + angle_3) / 57.29578) + node_a.center.y * cos((angle_1 + angle_2 + angle_3) / 57.29578) + point_temp.y;

		arm[3][i].type = node_a.type;

		if (node_a.type == TYPE_CIRCLE)
		{
			arm[3][i].center.x = x;
			arm[3][i].center.y = y;
			arm[3][i].radius = node_a.radius;
		}
		else
		{
			arm[3][i].center.x = x;
			arm[3][i].center.y = y;
			arm[3][i].length_a = node_a.length_a;
			arm[3][i].length_b = node_a.length_b;
			arm[3][i].angle = angle_1 + angle_2 + angle_3;
		}
	}
}

//机械臂碰撞模型类成员函数：检测是否发生自碰撞
//@return true if collide
bool arm_collision_model::self_collision()
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = i + 1; j < 4; j++)
		{
			if (self_collision_array[i][j])
			{
				for (auto& node_a : arm[i])
				{
					for (auto& node_b : arm[j])
					{
						//根据两节点类型选择节点碰撞检测函数
						if (node_a.type == TYPE_CIRCLE && node_b.type == TYPE_CIRCLE)
						{
							if (circle_collision(node_a.center, node_a.radius, node_b.center, node_b.radius))
							{
								return true;
							}
						}
						else if (node_a.type == TYPE_CIRCLE && node_b.type == TYPE_RECTANGLE)
						{
							if (rotated_rectangle_circle_collision(node_b.center, node_b.length_a, node_b.length_b, node_b.angle,
								node_a.center, node_a.radius))
							{
								return true;
							}
						}
						else if (node_a.type == TYPE_RECTANGLE && node_b.type == TYPE_CIRCLE)
						{
							if (rotated_rectangle_circle_collision(node_a.center, node_a.length_a, node_a.length_b, node_a.angle,
								node_b.center, node_b.radius))
							{
								return true;
							}
						}
						else
						{
							if (rotated_rectangle_collision(node_a.center, node_a.length_a, node_a.length_b, node_a.angle,
								node_b.center, node_b.length_a, node_b.length_b, node_b.angle))
							{
								return true;
							}
						}
					}
				}
			}
		}
	}
	return false;
}

//机械臂碰撞模型类成员函数：检测是否与障碍物发生碰撞
//@param QTM 障碍物四叉树地图
//@return true if collide
bool arm_collision_model::obstacle_collision(std::vector<cv::RotatedRect>obstacles)
{
	for (int i = 0; i < 4; i++)
	{
		for (auto& node : arm[i])
		{
			for (auto& ob : obstacles)
			{
				if (node_obstacle_collision(node, ob))
				{
					return true;
				}
			}
		}
	}
	return false;
}

//机械臂碰撞模型类成员函数：检测是否发生碰撞——包含自碰撞与障碍物碰撞
//@param angle_1 关节1角度  单位 °
//@param angle_2 关节2角度  单位 °
//@param angle_3 关节3角度  单位 °
//@param QTM 障碍物四叉树地图
//@return true if collide
bool arm_collision_model::collision(double angle_1, double angle_2, double angle_3, std::vector<cv::RotatedRect>obstacles)
{
	arm_set(angle_1, angle_2, angle_3);	//根据关节角度设置机械臂状态
	return (self_collision() || obstacle_collision(obstacles));
}

//机械臂碰撞模型类成员函数：检测是否发生碰撞——仅包含自碰撞
//@param angle_1 关节1角度 单位 °
//@param angle_2 关节2角度 单位 °
//@param angle_3 关节3角度 单位 °
//@return true if collide
bool arm_collision_model::collision(double angle_1, double angle_2, double angle_3)
{
	arm_set(angle_1, angle_2, angle_3);
	return self_collision();
}
#pragma endregion

#pragma region 碰撞检测功能函数
//检测是否发生碰撞——针对两矩形
//@param center1 矩形1中心  单位 mm
//@param length_a1,length_b1 矩形1长、宽 单位 mm
//@param angle1 矩形1角度 单位 °
//@param center2 矩形2中心 单位 mm
//@param length_a2,length_b2 矩形2长、宽 单位 mm
//@param angle2 矩形2角度 单位 °
//@return true if collide
bool rotated_rectangle_collision(cv::Point2d center1, double length_a1, double length_b1, double angle1,
	cv::Point2d center2, double length_a2, double length_b2, double angle2)
{
	cv::Vec2d Ax = cv::Vec2d(cos(angle1 / 180 * PI), sin(angle1 / 180 * PI));
	cv::Vec2d Ay = cv::Vec2d(-sin(angle1 / 180 * PI), cos(angle1 / 180 * PI));
	double WA = length_a1 / 2;
	double HA = length_b1 / 2;

	cv::Vec2d Bx = cv::Vec2d(cos(angle2 / 180 * PI), sin(angle2 / 180 * PI));
	cv::Vec2d By = cv::Vec2d(-sin(angle2 / 180 * PI), cos(angle2 / 180 * PI));
	double WB = length_a2 / 2;
	double HB = length_b2 / 2;

	cv::Vec2d T = cv::Vec2d(center2.x - center1.x, center2.y - center1.y);

	if (abs(T(0) * Ax(0) + T(1) * Ax(1)) > WA + WB * abs(Bx(0) * Ax(0) + Bx(1) * Ax(1)) + HB * abs(By(0) * Ax(0) + By(1) * Ax(1)) ||
		abs(T(0) * Ay(0) + T(1) * Ay(1)) > HA + WB * abs(Bx(0) * Ay(0) + Bx(1) * Ay(1)) + HB * abs(By(0) * Ay(0) + By(1) * Ay(1)) ||
		abs(T(0) * Bx(0) + T(1) * Bx(1)) > WB + WA * abs(Ax(0) * Bx(0) + Ax(1) * Bx(1)) + HA * abs(Ay(0) * Bx(0) + Ay(1) * Bx(1)) ||
		abs(T(0) * By(0) + T(1) * By(1)) > HB + WA * abs(Ax(0) * By(0) + Ax(1) * By(1)) + HA * abs(Ay(0) * By(0) + Ay(1) * By(1))
		)
	{
		return false;
	}
	else
	{
		return true;
	}
}

//碰撞检测函数——针对矩形与圆形
//@param center1 矩形中心 单位 mm
//@param length_a,length_b 矩形长、宽 单位 mm
//@param angle1 矩形角度 单位 °
//@param center2 圆形中心 单位 mm
//@param radius 圆形半径 单位 mm
//@return true if collide
bool rotated_rectangle_circle_collision(cv::Point2d center1, double length_a, double length_b, double angle, cv::Point2d center2, double radius)
{
	cv::Vec2d Ax = cv::Vec2d(cos(angle / 180 * PI), sin(angle / 180 * PI));
	cv::Vec2d Ay = cv::Vec2d(-sin(angle / 180 * PI), cos(angle / 180 * PI));
	double WA = length_a / 2;
	double HA = length_b / 2;


	cv::Vec2d T = cv::Vec2d(center2.x - center1.x, center2.y - center1.y);
	cv::Vec2d T_2 = cv::Vec2d(T(0) * Ax(0) + T(1) * Ax(1), T(0) * Ay(0) + T(1) * Ay(1));

	if ((abs(T_2(0)) < WA + radius && abs(T_2(1)) < HA) ||
		(abs(T_2(0)) < WA && abs(T_2(1)) < HA + radius))
	{
		return true;
	}
	else
	{
		if ((abs(T_2(0)) - WA) * (abs(T_2(0)) - WA) + (abs(T_2(1)) - HA) * (abs(T_2(1)) - HA) < radius * radius)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}

//碰撞检测函数——针对两圆形
//@param center1 圆形1中心 单位 mm
//@param radius1 圆形1半径 单位 mm
//@param center2 圆形2中心 单位 mm
//@param radius2 圆形2半径 单位 mm
//@return true if collide
bool circle_collision(cv::Point2d center1, double radius1, cv::Point2d center2, double radius2)
{
	if ((center1.x - center2.x) * (center1.x - center2.x) + (center1.y - center2.y) * (center1.y - center2.y) <= (radius1 + radius2) * (radius1 + radius2))
	{
		return true;
	}
	else
	{
		return false;
	}
}

//碰撞检测函数——针对 机械臂节点 与 旋转矩形
//@param node 机械臂节点
//@param ptr 四叉树地图根节点指针
//@return true if collide
bool node_obstacle_collision(arm_node& node, cv::RotatedRect ob)
{
	if (node.type == TYPE_CIRCLE)
	{
		if (rotated_rectangle_circle_collision(ob.center, ob.size.width, ob.size.height, ob.angle,
			node.center, node.radius))
		{
			return true;
		}
	}
	if (node.type == TYPE_RECTANGLE)
	{
		if (rotated_rectangle_collision(ob.center, ob.size.width, ob.size.height, ob.angle,
			node.center, node.length_a, node.length_b, node.angle))
		{
			return true;
		}
	}
	return false;
}
#pragma endregion


//void main()
//{
//	/*if (rotated_rectangle_collision(cv::Point2d(0, 0), 10, 10, -45,
//		cv::Point2d(0, 10.1), 10, 10, 0))*/
//
//	/*if (rotated_rectangle_circle_collision(cv::Point2d(10, 10), 10, 10, 0,
//		cv::Point2d(10, 20.1), 5))*/
//	if (circle_collision(cv::Point2d(10, 10), 5.2, cv::Point2d(10, 20.1), 5))
//	{
//		cout << "collision!" << endl;
//	}
//	else
//	{
//		cout << "clear" << endl;
//	}
//}