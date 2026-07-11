#include "obstacle_optimization.h"

#pragma region 全局变量
int OP_STATUS = 0;
int OPTRAJFOLLOW = 0;
Obstacle_Optimization OB_OP;
std::mutex OP_mutex;
#pragma endregion

#pragma region 外部变量
extern Q_Tree_Map QTM;
extern std::mutex Lidar_Point;	//全局互斥锁——激光雷达
extern std::mutex QTM_mutex;
extern Eigen::Vector3d focus_point;
extern std::vector<double> joint_now;
extern int ARM_STATE;
#pragma endregion

void OP_test_thread()
{
    Eigen::Vector4d stepdata(0.02, 0.002, 0.01, 0.03);
    OB_OP.SetExpansionDitance(10);
    OB_OP.SetResolution(64);
    OB_OP.SetStep(stepdata);

    //cv::namedWindow("Image from Eigen Matrix", cv::WINDOW_AUTOSIZE);

    Eigen::Vector2d v_xy(0.0, 0.0);
    Eigen::Vector2d v_theta(0.0, 0.0);
    while (true)
    {
        if (OP_STATUS == 0)
        {

        }
        else if (OP_STATUS == 1)//循环执行
        {
            Eigen::MatrixXi OP_obs_mat_last;
            OP_obs_mat_last = OB_OP.OP_obs_mat;
            QTM_mutex.lock();
            OB_OP.GetObsMat(QTM.OBOP_obs_mat);
            QTM_mutex.unlock();
            OB_OP.r = 8;
            if (OPTRAJFOLLOW == 0)
            {
                //代码测试
                //Eigen::Vector3d focuspoint(0.6, -0.6, 0.0);
                OB_OP.GetTargetPoint(focus_point);

                OB_OP.target_obstacle.clear();
                OB_OP.target_obstacle_expansion.clear();
                OB_OP.boundary_point.clear();
                OB_OP.convex_hull_point.clear();
                OB_OP.except_target_obstacle.clear();
                OB_OP.except_target_obstacle_point.clear();
                OB_OP.endposition = solve_visualservo_kdl(joint_now);
                OB_OP.OP_point.first = 0.0;
                OB_OP.OP_point.second = 0.0;
                if (OB_OP.GetTargetObstacle(0) == true)
                {
                    OP_mutex.lock();
                    OB_OP.TargetObstacleExpansion();
                    OB_OP.GetBoundaryPoint();
                    OB_OP.GetConvexHull();
                    OB_OP.GetExceptTargetObstacle();
                    OB_OP.OP_point = OB_OP.GetOPPosition();
                    //&& (abs(OB_OP.endposition.first - OB_OP.OP_point.first) > 1e-3 && abs(OB_OP.endposition.first - OB_OP.OP_point.first) > 1e-3)
                    if (abs(OB_OP.OP_point.first) > 1e-1 && abs(OB_OP.OP_point.second) > 1e-1)
                        OPprocessYOLOResult(OB_OP.OP_point, OB_OP.endposition, focus_point);
                    OP_mutex.unlock();
                }
            }
            else
            {
                OB_OP.endposition = solve_visualservo_kdl(joint_now);
                if (OB_OP.GetTargetObstacle(0) == false)
                {
                    ARM_STATE = 0;
                    OPTRAJFOLLOW = 0;
                }
                if (countLargeDifferencePositions(OB_OP.OP_obs_mat, OP_obs_mat_last, 1) > 4)
                {
                    OPTRAJFOLLOW = 0;
                    ARM_STATE = 0;
                    continue;
                }
            }
        }
        else if (OP_STATUS == 2) //单次执行
        {
            QTM_mutex.lock();
            OB_OP.GetObsMat(QTM.OBOP_obs_mat);
            QTM_mutex.unlock();

            //代码测试
                //Eigen::Vector3d focuspoint(0.6, -0.6, 0.0);
            OB_OP.GetTargetPoint(focus_point);
            OB_OP.r = 4;
            OB_OP.SetExpansionDitance(7);

            OB_OP.target_obstacle.clear();
            OB_OP.target_obstacle_expansion.clear();
            OB_OP.boundary_point.clear();
            OB_OP.convex_hull_point.clear();
            OB_OP.except_target_obstacle.clear();
            OB_OP.except_target_obstacle_point.clear();
            OB_OP.endposition = solve_visualservo_kdl(joint_now);
            OB_OP.OP_point.first = 0.0;
            OB_OP.OP_point.second = 0.0;
            if (OB_OP.GetTargetObstacle(1) == true)
            {
                OP_mutex.lock();
                OB_OP.TargetObstacleExpansion();
                OB_OP.GetBoundaryPoint();
                OB_OP.GetConvexHull();
                OB_OP.GetExceptTargetObstacle();
                OB_OP.OP_point = OB_OP.GetOPPosition();
                if (abs(OB_OP.OP_point.first) > 1e-1 && abs(OB_OP.OP_point.second) > 1e-1)
                    processYOLOResult(OB_OP.OP_point, focus_point);
                OP_mutex.unlock();
            }

            OP_STATUS = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //OP_STATUS = 0;
    }
}

#pragma region Obstacle_Optimization
//Obstacle_Optimization类默认构造函数
//@param 
Obstacle_Optimization::Obstacle_Optimization()
{

}

//Obstacle_Optimization类默认析构函数
//@param 
Obstacle_Optimization::~Obstacle_Optimization()
{

}

//Obstacle_Optimization类获取目标点函数
//@param focus_point 拍摄点的三维空间位置
//@return 拍摄点在平面栅格地图中的对应位置
std::pair<double, double> Obstacle_Optimization::GetTargetPoint(Eigen::Vector3d focus_point)
{
    std::pair<double, double> tmp_point;
    tmp_point.first = focus_point(0);
    tmp_point.second = focus_point(1);

    target_point = tmp_point;

    return tmp_point;
}

//Obstacle_Optimization类获设定膨胀距离函数
//@param distance 障碍物栅格的膨胀距离
void Obstacle_Optimization::SetExpansionDitance(int distance)
{
    expansion_distance = distance;
}

//Obstacle_Optimization类获设定搜索步长相关参数函数
//@param stepdata 搜索步长数据
void Obstacle_Optimization::SetStep(Eigen::Vector4d stepdata)
{
    origin_step = stepdata(0);
    step_add = stepdata(1);
    step_max = stepdata(2);
    step_min = stepdata(3);
}

//Obstacle_Optimization类获设定地图比例尺函数
//@param map_resolution 栅格地图与实际空间的比例尺
void Obstacle_Optimization::SetResolution(int map_resolution)
{
    OP_resolution = map_resolution;
}

//Obstacle_Optimization类获取当前空间栅格地图函数
//@param obs_mat 当前栅格地图矩阵
void Obstacle_Optimization::GetObsMat(Eigen::MatrixXi obs_mat)
{
    OP_obs_mat = obs_mat;
}

//Obstacle_Optimization类获目标位置最近的障碍物位置
//@param matrix 当前栅格地图矩阵 x,y 目标位置 radius检测距离半径
std::vector<std::pair<int, int>> Obstacle_Optimization::findNearOne(Eigen::MatrixXi& matrix, int x, int y, int radius)
{
    // 初始化最小距离为整型的最大值，用于后续比较
    int minDistance = std::numeric_limits<int>::max();
    // 初始化最近位置为 (-1, -1)，表示尚未找到符合条件的位置
    std::vector<std::pair<int, int>> nearPosition;

    // 遍历半径范围内的所有行
    // std::max(0, x - radius) 确保起始行索引不小于 0
    // std::min(matrix.rows() - 1, x + radius) 确保结束行索引不超过矩阵的行数
    for (int i = std::max(0, x - radius); i <= std::min(int(matrix.rows() - 1), x + radius); ++i)
    {
        // 遍历半径范围内的所有列
        // std::max(0, y - radius) 确保起始列索引不小于 0
        // std::min(matrix.cols() - 1, y + radius) 确保结束列索引不超过矩阵的列数
        for (int j = std::max(0, y - radius); j <= std::min(int(matrix.cols() - 1), y + radius); ++j)
        {
            if (matrix(i, j) == 20)
                nearPosition.push_back(std::make_pair(i, j));
            //// 计算当前点 (i, j) 到指定点 (x, y) 的欧几里得距离
            //int distance = std::sqrt((i - x) * (i - x) + (j - y) * (j - y));
            //// 检查当前点是否在半径范围内，并且该点的值为 1
            //if (distance <= radius && matrix(i, j) == 20) {
            //    // 如果当前距离小于最小距离
            //    if (distance < minDistance) {
            //        // 更新最小距离
            //        minDistance = distance;
            //        // 更新最近位置为当前点
            //        nearestPosition = std::make_pair(i, j);
            //    }
            //}
        }
    }

    // 返回最近位置
    return nearPosition;
}


std::vector<std::pair<int, int>> Obstacle_Optimization::findConnectedPoints(Eigen::MatrixXi& matrix, std::vector<std::pair<int, int>> points)
{
    int rows = matrix.rows();
    int cols = matrix.cols();

    // 8 邻域偏移量，用于遍历相邻的点
    const int dx[8] = { -1, -1, -1, 0, 0, 1, 1, 1 };
    const int dy[8] = { -1, 0, 1, -1, 1, -1, 0, 1 };

    // 初始化访问标记数组
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::vector<std::pair<int, int>> connectedPoints;

    // 辅助栈用于深度优先搜索
    std::vector<std::pair<int, int>> stack;

    // 将输入的 points 都加入栈中
    for (const auto& point : points) {
        stack.emplace_back(point);
    }

    std::pair<int, int> tmp;
    while (!stack.empty())
    {
        tmp = stack.back();
        auto x = tmp.first;
        auto y = tmp.second;
        stack.pop_back();

        // 检查当前点是否越界、是否已经访问过或者值为 255
        if (x < 0 || x >= rows || y < 0 || y >= cols || visited[x][y] || matrix(x, y) == 255) {
            continue;
        }

        // 标记当前点为已访问
        visited[x][y] = true;
        // 将当前点的坐标添加到连接点数组中
        connectedPoints.emplace_back(x, y);

        // 遍历 8 邻域的点
        for (int i = 0; i < 8; ++i) {
            int newX = x + dx[i];
            int newY = y + dy[i];
            stack.emplace_back(newX, newY);
        }
    }

    // 去重
    std::set<std::pair<int, int>> uniquePoints(connectedPoints.begin(), connectedPoints.end());
    connectedPoints.assign(uniquePoints.begin(), uniquePoints.end());

    return connectedPoints;
}

//Obstacle_Optimization类获取当前空间栅格地图函数
//@param obs_mat 当前栅格地图矩阵
//@return 目标点附近是否存在障碍物
bool Obstacle_Optimization::GetTargetObstacle(int type)
{
    int target_point_matx, target_point_maty;

    if (type == 1)
    {
        target_point_matx = int(target_point.first * 1000.0 / (OP_resolution * 1.0)) + 4096 / OP_resolution / 2;//机器人坐标系变矩阵
        target_point_maty = int(target_point.second * 1000.0 / (OP_resolution * 1.0)) + 4096 / OP_resolution / 2;
    }
    else
    {
        target_point_matx = int(endposition.first * 1000.0 / (OP_resolution * 1.0)) + 4096 / OP_resolution / 2;//机器人坐标系变矩阵
        target_point_maty = int(endposition.second * 1000.0 / (OP_resolution * 1.0)) + 4096 / OP_resolution / 2;
    }
    
    std::vector<std::pair<int, int>> target_obstacle_point;
    target_obstacle_point = findNearOne(OP_obs_mat, target_point_matx, target_point_maty, r);

    if (target_obstacle_point.size() == 0)
        return false;
    else
    {
        target_obstacle = findConnectedPoints(OP_obs_mat, target_obstacle_point);
        return true;
    }
}

//Obstacle_Optimization类对目标点附近的障碍物进行膨胀
void Obstacle_Optimization::TargetObstacleExpansion()
{
    std::set<std::pair<int, int>> dilatedSet;
    // 遍历每个障碍物
    for (const auto& obs : target_obstacle) {
        int x = obs.first;
        int y = obs.second;

        // 遍历以障碍物为中心的一个正方形区域
        for (int i = -expansion_distance; i <= expansion_distance; ++i) {
            for (int j = -expansion_distance; j <= expansion_distance; ++j) {
                // 计算当前点到障碍物中心的距离
                double distance = std::sqrt(i * i + j * j);
                if (distance <= expansion_distance) {
                    // 如果距离小于等于膨胀长度，则将该点加入膨胀后的集合
                    dilatedSet.insert({ x + i, y + j });
                }
            }
        }
    }

    target_obstacle_expansion.assign(dilatedSet.begin(), dilatedSet.end());
}

//Obstacle_Optimization类计算膨胀后障碍物的边界点
void Obstacle_Optimization::GetBoundaryPoint()
{
    std::pair<int, int> tmp;

    for (const auto& tt : target_obstacle_expansion)
    {
        tmp.first = tt.first * OP_resolution - OP_resolution / 2 - OP_obs_mat.rows() / 2 * OP_resolution;//矩阵转机器人坐标系
        tmp.second = tt.second * OP_resolution - OP_resolution / 2 - OP_obs_mat.cols() / 2 * OP_resolution;

        boundary_point.push_back(tmp);
    }

}

//Obstacle_Optimization类计算凸包函数
void Obstacle_Optimization::GetConvexHull()
{
    int n = boundary_point.size();
    if (n < 3)
        convex_hull_point = boundary_point;  // 点数少于 3 时，凸包就是这些点本身

    // 找到最左下角的点
    int pivot = findBottomLeft(boundary_point);

    // 将最左下角的点放到第一个位置
    std::vector<std::pair<int, int>> sortedPoints = boundary_point;
    std::swap(sortedPoints[0], sortedPoints[pivot]);

    // 对剩余的点按照极角排序
    std::pair<int, int> p0 = sortedPoints[0];
    std::sort(sortedPoints.begin() + 1, sortedPoints.end(), [p0](const std::pair<int, int>& p1, const std::pair<int, int>& p2) {
        return compare(p1, p2, p0);
        });

    // 移除共线且距离 p0 较近的点
    int m = 1;
    for (int i = 1; i < n; i++) {
        while (i < n - 1 && orientation(p0, sortedPoints[i], sortedPoints[i + 1]) == 0) {
            i++;
        }
        sortedPoints[m] = sortedPoints[i];
        m++;
    }

    //if (m < 3) return {};  // 不足以形成凸包

    // 初始化栈
    std::vector<std::pair<int, int>> hull;
    hull.push_back(sortedPoints[0]);
    hull.push_back(sortedPoints[1]);
    hull.push_back(sortedPoints[2]);

    // 进行 Graham 扫描
    for (int i = 3; i < m; i++) {
        while (hull.size() > 1 && orientation(hull[hull.size() - 2], hull[hull.size() - 1], sortedPoints[i]) != 2) {
            hull.pop_back();
        }
        hull.push_back(sortedPoints[i]);
    }

    convex_hull_point = hull;
}

//Obstacle_Optimization类损失计算函数
//@param point 函数输入点
//@return 损失函数值
double Obstacle_Optimization::GetLoss(std::pair<double, double> OP_point, std::vector<std::pair<int, int>> selected_obstacle_point)
{
    double value = 0.0;
    double distance = 0;
    double x1, y1, x2, y2;
    double rr = efficiency_distance * efficiency_distance;
    x1 = OP_point.first;
    y1 = OP_point.second;
    for (std::pair<int, int>& obstacle_point : selected_obstacle_point)
    {
        x2 = obstacle_point.first / 1000.0;
        y2 = obstacle_point.second / 1000.0;
        if (distance <= rr)
        {
            distance = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
            value = value + k_obstacle * 1 / distance;
        }
    }
    value = value + k_target * (x1 - target_point.first) * (x1 - target_point.first) + (y1 - target_point.second) * (y1 - target_point.second);
    value = value + k_target * (x1 - endposition.first) * (x1 - endposition.first) + (y1 - endposition.second) * (y1 - endposition.second);
    //value = value + k_target/50 * (x1) * (x1) + (y1) * (y1);
    return value;
}

//Obstacle_Optimization类目标障碍物外的障碍物位置提取
void Obstacle_Optimization::GetExceptTargetObstacle()
{
    std::pair<int, int> tmp;
    std::pair<double, double> tmp2;
    int rows = OP_obs_mat.rows();
    int cols = OP_obs_mat.cols();

    // 遍历矩阵的每个元素
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            // 检查当前位置是否在排除列表中
            bool isExcluded = false;
            for (const auto& pos : target_obstacle) {
                if (pos.first == i && pos.second == j) {
                    isExcluded = true;
                    break;
                }
            }

            // 如果当前位置不在排除列表中且值为 20，则添加到结果中
            if (!isExcluded && OP_obs_mat(i, j) == 20) {
                tmp.first = i;
                tmp.second = j;
                except_target_obstacle.push_back(tmp);
                tmp2.first = double(i * OP_resolution - OP_resolution / 2 - rows / 2 * OP_resolution) / 1000.0;
                tmp2.second = double(j * OP_resolution - OP_resolution / 2 - cols / 2 * OP_resolution) / 1000.0;
                except_target_obstacle_point.push_back(tmp2);
            }
        }
    }
}

//Obstacle_Optimization类过滤凸包边线一侧的障碍物函数
//@param start_point  end_point 正在搜索的凸包边线端点
//@return 符合条件可以传入损失函数计算的障碍物位置点
std::vector<std::pair<int, int>> Obstacle_Optimization::HullFilterObstacle(std::pair<double, double> start_point, std::pair<double, double> end_point)
{
    double k = 0.0, b = 0.0;
    double target_x = 0.0, target_y = 0.0;
    double judge_value = 0;
    std::vector<std::pair<int, int>> filtered_obstacle;
    target_x = target_point.first;
    target_y = target_point.second;
    if (abs(end_point.first - start_point.first) < 1e-1)
    {
        judge_value = target_x - start_point.first;
        for (std::pair<int, int>& obstacle_tmp : except_target_obstacle_point)
        {
            if (judge_value >= 0)
            {
                if (obstacle_tmp.first - start_point.first <= 0)
                    filtered_obstacle.push_back(obstacle_tmp);
            }
            else
            {
                if (obstacle_tmp.first - start_point.first >= 0)
                    filtered_obstacle.push_back(obstacle_tmp);
            }
        }
    }
    else
    {
        k = (end_point.second - start_point.second) / (end_point.first - start_point.first);
        b = end_point.second - k * end_point.first;

        judge_value = k * target_x + b - target_y;

        for (std::pair<int, int>& obstacle_tmp : except_target_obstacle_point)
        {
            if (judge_value >= 0)
            {
                if (k * obstacle_tmp.first + b - obstacle_tmp.second <= 0)
                    filtered_obstacle.push_back(obstacle_tmp);
            }
            else
            {
                if (k * obstacle_tmp.first + b - obstacle_tmp.second >= 0)
                    filtered_obstacle.push_back(obstacle_tmp);
            }
        }
    }
    return filtered_obstacle;
}

std::pair<double, double> Obstacle_Optimization::GetOPPosition()
{
    std::pair<double, double> start, end, OP_point, direction;
    double length = 1.0;
    double left, right, up, down;
    std::pair<double, double> point_best(0, 0);
    double loss = 1e5;
    double step = origin_step;
    double tmp = 0.0;
    std::vector<std::pair<int, int>> filtered_obstacle;
    for (int i = 0; i < convex_hull_point.size(); i++)
    {
        start.first = convex_hull_point[i].first / 1000.0;
        start.second = convex_hull_point[i].second / 1000.0;
        if (i == convex_hull_point.size() - 1)
        {
            end.first = convex_hull_point[0].first / 1000.0;
            end.second = convex_hull_point[0].second / 1000.0;
        }
        else
        {
            end.first = convex_hull_point[i + 1].first / 1000.0;
            end.second = convex_hull_point[i + 1].second / 1000.0;
        }
        if (abs(start.first - end.first) < 1e-1)
        {
            if (start.second < end.second)
            {
                direction.first = 0;
                direction.second = 1;
            }
            else
            {
                direction.first = 0;
                direction.second = -1;
            }

            down = std::min(start.second, end.second);
            up = std::max(start.second, end.second);
            OP_point.first = start.first + step * direction.first;
            OP_point.second = start.second + step * direction.second;
            filtered_obstacle = HullFilterObstacle(start, end);
            while (OP_point.second >= down && OP_point.second <= up)
            {
                tmp = GetLoss(OP_point, filtered_obstacle);
                if (tmp < loss)
                {
                    loss = tmp;
                    point_best.first = OP_point.first;
                    point_best.second = OP_point.second;
                }
                OP_point.first = OP_point.first + step * direction.first;
                OP_point.second = OP_point.second + step * direction.second;
            }
        }
        else
        {
            length = sqrt((start.first - end.first) * (start.first - end.first) + (start.second - end.second) * (start.second - end.second));
            direction.first = (end.first - start.first) / length;
            direction.second = (end.second - start.second) / length;

            left = std::min(start.first, end.first);
            right = std::max(start.first, end.first);
            OP_point.first = start.first + step * direction.first;
            OP_point.second = start.second + step * direction.second;
            filtered_obstacle = HullFilterObstacle(start, end);
            while (OP_point.first >= left && OP_point.first <= right)
            {
                tmp = GetLoss(OP_point, filtered_obstacle);
                if (tmp < loss)
                {
                    loss = tmp;
                    point_best.first = OP_point.first;
                    point_best.second = OP_point.second;
                }
                OP_point.first = OP_point.first + step * direction.first;
                OP_point.second = OP_point.second + step * direction.second;
            }
        }
    }
    return point_best;
}
#pragma endregion

//辅助函数
#pragma region
//Obstacle_Optimization类凸包计算的辅助函数，用来计算向量叉积，用于判断点的相对位置
int orientation(const std::pair<int, int>& p, const std::pair<int, int>& q, const std::pair<int, int>& r)
{
    int val = (q.second - p.second) * (r.first - q.first) - (q.first - p.first) * (r.second - q.second);
    if (val == 0) return 0;  // 共线
    return (val > 0) ? 1 : 2; // 顺时针或逆时针
}

//Obstacle_Optimization类凸包计算的辅助函数，用来比较函数，用于排序点
bool compare(const std::pair<int, int>& p1, const std::pair<int, int>& p2, const std::pair<int, int>& p0)
{
    int o = orientation(p0, p1, p2);
    if (o == 0) {
        return ((p1.first - p0.first) * (p1.first - p0.first) + (p1.second - p0.second) * (p1.second - p0.second))
            < ((p2.first - p0.first) * (p2.first - p0.first) + (p2.second - p0.second) * (p2.second - p0.second));
    }
    return o == 2;
}

//Obstacle_Optimization类凸包计算的辅助函数，用来找到最左下角的点
int findBottomLeft(const std::vector<std::pair<int, int>>& points)
{
    int ymin = points[0].second;
    int min = 0;
    for (size_t i = 1; i < points.size(); i++) {
        int y = points[i].second;
        if (y < ymin || (y == ymin && points[i].first < points[min].first)) {
            ymin = y;
            min = i;
        }
    }
    return min;
}

// 函数用于计算两个矩阵中差别很大的元素位置数量
int countLargeDifferencePositions(const Eigen::MatrixXi& mat1, const Eigen::MatrixXi& mat2, int threshold)
{
    // 确保两个矩阵具有相同的行数和列数
    if (mat1.rows() != mat2.rows() || mat1.cols() != mat2.cols())
    {
        std::cerr << "矩阵的大小不一致，无法进行比较。" << std::endl;
        return -1;
    }

    int count = 0;
    // 遍历矩阵的每一行
    for (int i = 0; i < mat1.rows(); ++i)
    {
        // 遍历矩阵的每一列
        for (int j = 0; j < mat1.cols(); ++j)
        {
            // 计算对应位置元素差值的绝对值
            int diff = std::abs(mat1(i, j) - mat2(i, j));
            // 如果差值的绝对值大于阈值，则认为该位置差别很大
            if (diff > threshold)
            {
                ++count;
            }
        }
    }
    return count;
}

#pragma endregion