#include "visual.h"

#pragma region 外部引用变量
extern motor M;
extern std::vector<cv::Point2d> Lidar_point;

extern std::vector<double> joint_now;

extern arm_collision_model ACM,ACM_show;

extern std::mutex QTM_mutex;
extern Q_Tree_Map QTM;

extern Eigen::Vector2d oa_direction;

extern Obstacle_Optimization OB_OP;
extern std::mutex OP_mutex;
#pragma endregion

#pragma region 全局变量
ARM arm;	//机械臂结构体，存储机械臂各段长度，单位/m

cv::Mat bg = cv::Mat::zeros(500, 500, CV_8UC3);	//绘制图片

cv::Mat oa_bg = cv::Mat::zeros(180, 320, CV_8UC3);	//绘制图片——模拟遮挡
#pragma endregion

//可视化程序
void visualization()
{
	ACM_show.arm_set(joint_now[0], joint_now[1], joint_now[2]);

	while (cv::waitKey(100))
	{
		bg = 0;	//图片清空
		

		draw_arm(bg, ACM_show);
		draw_lp(bg, Lidar_point);
		QTM_mutex.lock();
		draw_qtm_obs_rects(bg, QTM);
		draw_qtm(bg, QTM.root);
		QTM_mutex.unlock();

		//优化测试
		OP_mutex.lock();
		for (const auto& point_expansion : OB_OP.boundary_point)
		{
			cv::circle(bg, cv::Point2d(500 / 2 + point_expansion.first / 10, 500 / 2 - point_expansion.second / 10), 1, cv::Scalar(0, 255, 255), 1);
		}
		cv::circle(bg, cv::Point2d(500 / 2 + OB_OP.target_point.first * 100, 500 / 2 - OB_OP.target_point.second * 100), 1, cv::Scalar(0, 0, 255), 2);
		cv::circle(bg, cv::Point2d(500 / 2 + OB_OP.OP_point.first * 100, 500 / 2 - OB_OP.OP_point.second * 100), 1, cv::Scalar(203, 192, 255), 2);
		int x1, y1, x2, y2;
		for (int i = 0; i < OB_OP.convex_hull_point.size(); i++)
		{
			x1 = 500 / 2 + OB_OP.convex_hull_point[i].first / 10;
			y1 = 500 / 2 - OB_OP.convex_hull_point[i].second / 10;
			if (i == OB_OP.convex_hull_point.size() - 1)
			{
				x2 = 500 / 2 + OB_OP.convex_hull_point[0].first / 10;
				y2 = 500 / 2 - OB_OP.convex_hull_point[0].second / 10;
			}
			else
			{
				x2 = 500 / 2 + OB_OP.convex_hull_point[i + 1].first / 10;
				y2 = 500 / 2 - OB_OP.convex_hull_point[i + 1].second / 10;
			}
			cv::Point pt1(x1, y1);
			cv::Point pt2(x2, y2);
			cv::line(bg, pt1, pt2, cv::Vec3b(0, 255, 0), 1, cv::LINE_AA);
		}
		OP_mutex.unlock();

		cv::imshow("kdl_test_4", bg);

		//cv::imshow("oa_test", oa_bg);
	}
}

//绘制机械臂
//@param src 图片
//@param joint_now 机械臂关节角度
void draw_arm(cv::Mat &src,std::vector<double> joint_now)
{
	cv::Point2d joint_1, joint_2, joint_3, joint_4;
	int width = src.cols;
	int height = src.rows;

	joint_1.x = width / 2;
	joint_1.y = height / 2;

	joint_2.x = joint_1.x + arm.L1 * 100 * std::cos((90 + joint_now[0]) / 180 * PI);
	joint_2.y = joint_1.y - arm.L1 * 100 * std::sin((90 + joint_now[0]) / 180 * PI);

	joint_3.x = joint_2.x + arm.L2 * 100 * std::cos((90 + joint_now[0] + joint_now[1]) / 180 * PI);
	joint_3.y = joint_2.y - arm.L2 * 100 * std::sin((90 + joint_now[0] + joint_now[1]) / 180 * PI);

	joint_4.x = joint_3.x + arm.L3 * 100 * std::cos((90 + joint_now[0] + joint_now[1] + joint_now[2]) / 180 * PI);
	joint_4.y = joint_3.y - arm.L3 * 100 * std::sin((90 + joint_now[0] + joint_now[1] + joint_now[2]) / 180 * PI);

	cv::circle(src, joint_1, 20 / 5, (255, 255, 255), 1);
	cv::circle(src, joint_2, 20 / 5, (255, 255, 255), 1);
	cv::circle(src, joint_3, 20 / 5, (255, 255, 255), 1);
	cv::circle(src, joint_4, 20 / 5, (255, 255, 255), 1);

	cv::line(src, joint_1, joint_2, (255, 255, 255));
	cv::line(src, joint_2, joint_3, (255, 255, 255));
	cv::line(src, joint_3, joint_4, (255, 255, 255));
}

//绘制机械臂
//@param src 图片
//@param ACM 机械臂碰撞模型
void draw_arm(cv::Mat& src,arm_collision_model &ACM)
{
	int width = src.cols;
	int height = src.rows;

	for (auto& nodes : ACM.arm)
	{
		for (auto& node : nodes)
		{
			if (node.type == TYPE_CIRCLE)
			{
				cv::circle(src, cv::Point2d(node.center.x / 10 + width / 2, -node.center.y / 10 + height / 2), node.radius/10, cv::Scalar(255, 255, 255), 1);
			}
			else
			{
				draw_rotated_rectangle(src, cv::Point2d(node.center.x / 10 + width / 2, -node.center.y / 10 + height / 2), node.length_a / 10, node.length_b / 10, node.angle);
			}
		}
	}
}

//绘制旋转矩形
//@param src 图片
//@param center 矩形中心
//@param length_a 长
//@param length_b 宽
//@param angle 角度
void draw_rotated_rectangle(cv::Mat& src, cv::Point2d center, double length_a, double length_b,double angle)
{
	cv::Point2d points[4];
	points[0].x = (center.x + length_a / 2 * cos(angle / 57.29578) - length_b / 2 * sin(angle / 57.29578));
	points[0].y = (center.y - length_a / 2 * sin(angle / 57.29578) - length_b / 2 * cos(angle / 57.29578)) ;

	points[1].x = (center.x + (-length_a / 2 * cos(angle / 57.29578) - length_b / 2 * sin(angle / 57.29578))) ;
	points[1].y = (center.y - (-length_a / 2 * sin(angle / 57.29578) + length_b / 2 * cos(angle / 57.29578))) ;

	points[2].x = (center.x + (-length_a / 2 * cos(angle / 57.29578) + length_b / 2 * sin(angle / 57.29578))) ;
	points[2].y = (center.y - (-length_a / 2 * sin(angle / 57.29578) - length_b / 2 * cos(angle / 57.29578)));

	points[3].x = (center.x + (length_a / 2 * cos(angle / 57.29578) + length_b / 2 * sin(angle / 57.29578))) ;
	points[3].y = (center.y - (length_a / 2 * sin(angle / 57.29578) - length_b / 2 * cos(angle / 57.29578))) ;

	for (size_t i = 0; i < 4; i++)
	{
		cv::line(src, points[i], points[(i + 1) % 4], cv::Scalar(255, 255, 255), 1);
	}
}

//绘制激光雷达点
//@param src 图片
//@param lp 激光雷达点数组
void draw_lp(cv::Mat& src, const std::vector<cv::Point2d> lp)
{
	int width = src.cols;
	int height = src.rows;

	for (auto& point : lp)
	{
		cv::circle(src, cv::Point2d(width / 2 + point.x / 10, height / 2 - point.y / 10), 1, cv::Scalar(255, 255, 255), 1);
	}
}

//递归绘制四叉树地图
//@param src 图片
//@param ptr 四叉树地图根节点
void draw_qtm(cv::Mat& src, Node* ptr)
{
	int width = src.cols;
	int height = src.rows;

	if (ptr == NULL)
	{
		return;
	}
	else
	{
		if (ptr->val == SELFFULL)
		{
			draw_rotated_rectangle(src, cv::Point2d(ptr->center.x / 10 + width / 2, -ptr->center.y / 10 + height / 2), ptr->length/10, ptr->length/10, 0);
		}
		else if (ptr->val == HALF)
		{
			draw_qtm(src, ptr->R[0]);
			draw_qtm(src, ptr->R[1]);
			draw_qtm(src, ptr->R[2]);
			draw_qtm(src, ptr->R[3]);
		}
		else
		{
			return;
		}
	}
}

//绘制四叉树地图障碍物
//@param src 图片
//@param q 四叉树地图
void draw_qtm_obs_rects(cv::Mat& src, Q_Tree_Map& q)
{
	int width = src.cols;
	int height = src.rows;

	cv::Point2f vertex[4];

	for (auto& r : q.obs_rects_new)
	{
		r.points(vertex);

		for (int i = 0; i < 4; i++)
		{
			vertex[i].x = width / 2 + vertex[i].x / 10;
			vertex[i].y = height / 2 - vertex[i].y / 10;
		}

		for (int i = 0; i < 4; i++)
		{
			cv::line(src, vertex[i], vertex[(i + 1) % 4], cv::Scalar(0, 255, 0), 10);
		}
	}

	for (auto& r : q.obs_rects)
	{
		r.points(vertex);

		for (int i = 0; i < 4; i++)
		{
			vertex[i].x = width / 2 + vertex[i].x / 10;
			vertex[i].y = height / 2 - vertex[i].y / 10;
		}

		for (int i = 0; i < 4; i++)
		{
			cv::line(src, vertex[i], vertex[(i + 1) % 4], cv::Scalar(255, 100, 200));
		}
	}
}
