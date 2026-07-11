#include "q_tree.h"

#pragma region 全局变量
Q_Tree_Map QTM(4096, 64);	//全局四叉树地图对象
std::mutex QTM_mutex;	//全局互斥锁——四叉树地图
#pragma endregion

#pragma region 四叉树地图类
//Q_Tree_Map类构造函数
//@param length 地图边长，单位mm
//@param Resolution 地图分辨率，单位mm
Q_Tree_Map::Q_Tree_Map(int length, int Resolution) :map_length(length), resolution(Resolution)
{
	root = new Node();	//根节点开辟内存
	root->center.x = 0;	//设置根节点参数
	root->center.y = 0;
	root->depth = 0;	//根节点深度为0
	root->length = length;	//根节点边长为地图边长

	//参数设置
	obs_mat_full_val = 20;
	obs_mat_empty_val = 255;

	//地图矩阵化表达，矩阵中各个元素初始值为obs_mat_empty_val
	obs_mat = Eigen::MatrixXi::Ones(length / resolution, length / resolution);
	obs_mat *= obs_mat_empty_val;
}

//Q_Tree_Map类析构函数
Q_Tree_Map::~Q_Tree_Map()
{
	clear(root);	//递归删除所有节点
}

//Q_Tree_Map类成员函数：递归删除节点p及其子节点
//@param p 根节点
void Q_Tree_Map::clear(Node*& p)
{
	if (p == NULL) return;	//若空，返回
	//递归删除4各子节点
	for (int i = 0; i < 4; i++)
	{
		if (p->R[i]) clear(p->R[i]);
	}
	delete(p);
	p = NULL;
}

void Q_Tree_Map::free()
{
	root->val = EMPTY;

	for (int i = 0; i < 4; i++)
	{
		if (root->R[i]) clear(root->R[i]);
	}

	return;
}

//Q_Tree_Map类成员函数：建图
//@param lp 激光雷达点数组
void Q_Tree_Map::build_map(std::vector<cv::Point2d> lp)
{
	for (auto& p : lp)
	{
		insert(p);	//将各个雷达点插入地图
	}

	full_node(root);
	OBOP_obs_mat = obs_mat;
	obs_mat_to_rec();
}


//Q_Tree_Map类成员函数：将雷达点插入地图
//@param p 激光雷达点
void Q_Tree_Map::insert(const cv::Point2d p)
{
	Node* node_ptr = root;

	double l = root->length;	//边长

	int d = 0;	//深度

	std::vector<std::vector<int>> diretion = {
		{-1, 1},
		{1, 1},
		{-1, -1},
		{1, -1},
	};	//方向数组

	while (l > resolution)	//逐层深入
	{
		d += 1;	//深度+1

		int r_ind;

		//根据坐标与节点中心关系，判断坐标在节点哪部分
		if (p.x <= node_ptr->center.x)
		{
			if (p.y > node_ptr->center.y)
			{
				r_ind = 0;
			}
			else
			{
				r_ind = 2;
			}
		}
		else
		{
			if (p.y > node_ptr->center.y)
			{
				r_ind = 1;
			}
			else
			{
				r_ind = 3;
			}
		}

		if (node_ptr->R[r_ind] != NULL)	//若节点相应子节点不为空
		{
			node_ptr = node_ptr->R[r_ind];
			if (l / 2 <= resolution)	//若子节点为最深层节点
			{
				node_ptr->points_num += 1;	//子节点内部激光雷达点数量增加
				node_ptr->points_num = std::min(node_ptr->points_num, max_points_num);	//限幅
			}
		}
		else
		{
			Node* node_temp = new Node();	//创建子节点
			
			//设置相关参数
			node_temp->center.x = node_ptr->center.x + l / 4 * diretion[r_ind][0];
			node_temp->center.y = node_ptr->center.y + l / 4 * diretion[r_ind][1];
			node_temp->length = l / 2;
			node_temp->depth = d;

			node_ptr->R[r_ind] = node_temp;
			node_ptr->val = TBD;	//改变当前节点状态
			node_ptr = node_ptr->R[r_ind];

			if (l / 2 <= resolution)	//若子节点为最深层节点
			{
				node_ptr->val = TBD;	//设置子节点状态
				node_ptr->points_num += 1;	//子节点内部激光雷达点数量增加
				node_ptr->points_num = std::min(node_ptr->points_num, max_points_num);	//限幅
			}
		}

		l /= 2;
	}
}

//Q_Tree_Map类成员函数：确认节点状态
//@param ptr 节点指针
void Q_Tree_Map::full_node(Node* ptr)
{
	if (ptr->length == resolution)	//若节点为最深层节点
	{
		//矩阵化地图中，节点对应元素位置
		int temp_mat_x = ((int)(ptr->center.x) + map_length / 2) / resolution;
		int temp_mat_y = ((int)(ptr->center.y) + map_length / 2) / resolution;

		if (ptr->points_num >= min_points_num)	//若节点中点数量大于判满最小数量
		{
			ptr->val = SELFFULL;	//节点值状态置为满
			obs_mat(temp_mat_x, temp_mat_y) = obs_mat_full_val;	//矩阵化地图中，对应元素置为满
		}
		else
		{
			ptr->val = EMPTY;	//节点值状态置为空
			obs_mat(temp_mat_x, temp_mat_y) = obs_mat_empty_val;	//矩阵化地图中，对应元素置为空
		}
		return;
	}
	//非最深层节点
	else
	{
		if (ptr->val == TBD)	//若节点状态为未知
		{
			int full_num = 0;	//状态为满的子节点数量
			int half_num = 0;	//状态为半满的子节点数量
			int sons_num = 0;
			for (int i = 0; i < 4; i++)
			{
				if (ptr->R[i] != NULL)	//若子节点非空
				{
					full_node(ptr->R[i]);	//递归确定子节点状态
					if (ptr->R[i]->val == SELFFULL)	//若子节点状态为满
					{
						full_num++;
					}
					else if (ptr->R[i]->val == HALF)	//若子节点状态为半满
					{
						half_num += 1;
					}

					sons_num++;
				}
			}

			if (full_num == 4)
			{
				ptr->val = SELFFULL;
			}
			else if (full_num > 0 || half_num > 0)
			{
				ptr->val = HALF;
			}
			else
			{
				ptr->val = EMPTY;
			}
		}
	}
}

//Q_Tree_Map类成员函数：将节点状态设置为未知
//@param ptr 节点指针
void Q_Tree_Map::tbd_for_rebuild(Node* ptr)
{
	ptr->val = TBD;

	for (int i = 0; i < 4; i++)
	{
		if (ptr->R[i] != NULL)
		{
			tbd_for_rebuild(ptr->R[i]);	//递归设置子节点为未知
		}
	}
}

//Q_Tree_Map类成员函数：将节点中激光雷达点数量衰减
//@param ptr 节点指针
void Q_Tree_Map::decay_for_rebuild(Node* ptr)
{
	if (ptr->length == resolution)	//若节点为最深层节点
	{
		if (ptr->val != EMPTY)	//若状态非空
		{
			ptr->points_num -= decay_speed;	//衰减
			ptr->points_num = std::max(ptr->points_num, 0);	//限幅
		}
		else
		{
			ptr->points_num = 0;
		}
	}
	//递归对子节点执行衰减操作
	else
	{
		for (int i = 0; i < 4; i++)
		{
			if (ptr->R[i] != NULL)
			{
				decay_for_rebuild(ptr->R[i]);
			}
		}
	}
}

//Q_Tree_Map类成员函数：设置四叉树地图分辨率
//@param Resolution 分辨率
void Q_Tree_Map::set_resolution(int Resolution)
{
	resolution = Resolution;
}

//Q_Tree_Map类成员函数：矩阵化地图获取矩形障碍物
void Q_Tree_Map::obs_mat_to_rec()
{
	//std::cout << "obs_mat" << std::endl;
	//std::cout << obs_mat << std::endl;

	int rows = obs_mat.rows();
	int cols = obs_mat.cols();

	int index_max = 0;

	std::vector<int> min_neib;	//以index为序号的节点，其相邻节点中的最小序号
	std::vector<std::vector<cv::Point2f>> points_sets;
	std::vector< std::vector<std::pair<int, int>>> ind_sets;

	//TwoPass算法
	//第一次扫描
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			if (obs_mat(i, j) < obs_mat_empty_val)	//判断是否有障碍物
			{
				std::vector<int> index_neibs;	//相邻接点序号列表，用于更新其最小相邻接点序号

				int index_temp = obs_mat_full_val;	//求最小相邻接点序号
				int index_neib;	//相邻接点序号

				//八连通图
				for (int m = -1; m < 2; m++)
				{
					for (int n = -1; n < 2; n++)
					{
						int row_new = i + m;
						int col_new = j + n;

						if (row_new >= 0 && row_new < rows && col_new >= 0 && col_new < cols)	//判断序号合法性
						{
							index_neib = obs_mat(row_new, col_new);	//当前访问相邻节点值
							index_temp = std::min(index_temp, index_neib);	//取较小值
							if (index_neib < obs_mat_full_val)
							{
								index_neibs.push_back(index_neib);	//将相邻节点序号加入相邻节点序号列表
							}
						}
					}
				}

				if (index_temp == obs_mat_full_val)	//判断相邻节点中是否有已标记的
				{
					index_temp = index_max;	//相邻节点均未标记，获得新序号
					index_max++;	//下一序号加一

					min_neib.push_back(index_temp);	//设置序号index_temp与其自身等价
				}
				else
				{
					for (auto& ind : index_neibs)
					{
						min_neib[index_temp] = std::min(min_neib[index_temp], ind);	//更新与index_temp等价的序号值
						min_neib[ind] = std::min(min_neib[ind], index_temp);	//更新与ind等价的序号值
					}
				}
				obs_mat(i, j) = index_temp;
			}
		}
	}

	/*std::cout << "obs_mat" << std::endl;
	std::cout << obs_mat << std::endl;*/

	for (int i = 0; i < min_neib.size(); i++)
	{
		while (min_neib[i] != min_neib[min_neib[i]])
		{
			min_neib[i] = min_neib[min_neib[i]];	//将所有序号等价值更新至最小
		}
	}

	//将所有序号等价值更新至连续
	{
		int index_max_temp = 0;
		for (int i = 0; i < min_neib.size(); i++)
		{
			if (min_neib[i] == index_max_temp)
			{
				index_max_temp++;
			}
			else
			{
				if (min_neib[i] > index_max_temp)
				{

					int index_to_change = min_neib[i];

					for (int j = 0; i + j < min_neib.size(); j++)
					{
						if (min_neib[i + j] == index_to_change)
						{
							min_neib[i + j] = index_max_temp;
						}
					}

					index_max_temp++;

				}
			}
		}
	}

	//第二次扫描（省略更新obs_mat中值，改为查找序号等价最小值）
	std::unordered_set<int> visited_index;	//哈希表，记录探索过的序号值
	std::stack<std::pair<int, int>> stk;	//探索栈
	cv::Point2f center_temp;

	//深度优先搜索探索连通域
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			if (obs_mat(i, j) < obs_mat_full_val && !visited_index.count(min_neib[obs_mat(i, j)]))
			{
				int ind_temp = min_neib[obs_mat(i, j)];
				obs_mat(i, j) = obs_mat_empty_val;

				stk.push({ i,j });
				points_sets.push_back(std::vector<cv::Point2f>());
				ind_sets.push_back({});
				visited_index.insert(ind_temp);

				while (!stk.empty())
				{
					std::pair<int, int> p = stk.top();
					stk.pop();

					center_temp.x = p.first * resolution - map_length / 2;
					center_temp.y = p.second * resolution - map_length / 2;

					ind_sets.back().push_back({ p.first, p.second });

					points_sets.back().push_back(cv::Point2f(center_temp.x + resolution, center_temp.y + resolution));
					points_sets.back().push_back(cv::Point2f(center_temp.x, center_temp.y + resolution));
					points_sets.back().push_back(cv::Point2f(center_temp.x + resolution, center_temp.y));
					points_sets.back().push_back(cv::Point2f(center_temp.x, center_temp.y));

					for (int m = -1; m < 2; m++)
					{
						for (int n = -1; n < 2; n++)
						{
							int row_new = p.first + m;
							int col_new = p.second + n;

							if ((m != 0 || n != 0) && row_new >= 0 && row_new < rows && col_new >= 0 && col_new < cols
								&& obs_mat(row_new, col_new) < obs_mat_empty_val && min_neib[obs_mat(row_new, col_new)] == ind_temp)	//判断序号合法性
							{
								stk.push({ row_new, col_new });
								obs_mat(row_new, col_new) = obs_mat_empty_val;
							}
						}
					}
				}
			}
		}
	}

	obs_rects_new.clear();	//清空新增障碍物矩形
	std::vector<cv::RotatedRect> obs_rects_temp;	//障碍物矩形临时数组

	for (auto& points : points_sets)	//对点集合数组
	{
		std::vector<cv::RotatedRect> temp = Points_to_rects(points);	//将点集合转换为矩形

		for (auto& rec_temp : temp)
		{
			if (!rects_in_obs_rects(rec_temp))	//判断矩形是否已存在于障碍物矩形数组中
			{
				obs_rects_new.push_back(rec_temp);	//将矩形加入新增障碍物矩形数组
			}
			obs_rects_temp.push_back(rec_temp);	//将矩形加入临时障碍物矩形数组
		}
	}

	//cout << "obs_rects_new.size = " << obs_rects_new.size() << endl;
	obs_rects = obs_rects_temp;	//使用临时障碍物矩形数组替换障碍物矩形数组
}

//Q_Tree_Map类成员函数：栅格地图点数组转化为障碍物矩形数组
//@param points 栅格地图点数组
//@return ans RotatedRect障碍物矩形数组
std::vector<cv::RotatedRect> Q_Tree_Map::Points_to_rects(std::vector<cv::Point2f>points)
{
	std::vector<cv::RotatedRect> ans;

	cv::RotatedRect rec_temp = cv::minAreaRect(points);
	if (rec_temp.size.area() > 3 * resolution * resolution * points.size() / 4)	//如果解得障碍物矩形面积大于栅格面积三倍
	{
		//二分递归获得障碍物矩形数组
		std::vector<cv::RotatedRect> temp1 = Points_to_rects(std::vector<cv::Point2f>(points.begin(), points.begin() + points.size() / 8 * 4));
		std::vector<cv::RotatedRect> temp2 = Points_to_rects(std::vector<cv::Point2f>(points.begin() + points.size() / 8 * 4, points.end()));

		ans = temp1;
		ans.insert(ans.end(), temp2.begin(), temp2.end());
	}
	else
	{
		//障碍物矩形满足面积要求，为保证机械臂安全，长和宽均扩充，增加二倍栅格地图分辨率
		rec_temp.size.height += 2 * resolution;
		rec_temp.size.width += 2 * resolution;
		ans.push_back(rec_temp);
	}

	return ans;
}

//Q_Tree_Map类成员函数：判断障碍物矩形是否在障碍物矩形数组中
//@param rec 障碍物矩形
//@return true is rects_in_obs_rects
bool Q_Tree_Map::rects_in_obs_rects(cv::RotatedRect rec)
{
	for (auto& r : obs_rects)
	{
		if (r.center.x == rec.center.x && r.center.y == rec.center.y &&
			r.angle == rec.angle &&
			r.size.width == rec.size.width && r.size.height == rec.size.height)
		{
			return true;
		}
	}

	return false;
}

//Q_Tree_Map类成员函数：设置判满最小点数量
//@param v 判满最小点数量
void Q_Tree_Map::set_min_points_num(int v)
{
	min_points_num = v;
}

//Q_Tree_Map类成员函数：设置衰减速度
//@param v 衰减速度
void Q_Tree_Map::set_decay_speed(int v)
{
	decay_speed = v;
}

//Q_Tree_Map类成员函数：设置最大点数量
//@param v 最大点数量
void Q_Tree_Map::set_max_points_num(int v)
{
	max_points_num = v;
}
#pragma endregion