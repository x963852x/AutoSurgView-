/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of occlusion avoidance

**************************************************************************/

#include "occlusion_avoidance.h"
#include <filesystem>
#include "pcl/io/pcd_io.h"

using std::string;

extern std::map<std::string, std::string> parameters;

#pragma region 外部引用变量
//机器人坐标变换矩阵
extern Eigen::Isometry3d T10;
extern Eigen::Isometry3d T21;
extern Eigen::Isometry3d T32;
extern Eigen::Isometry3d T43;
extern Eigen::Isometry3d T54;
extern Eigen::Isometry3d T65;
extern Eigen::Isometry3d T73;
extern Eigen::Isometry3d T83;
extern Eigen::Isometry3d T90;
extern Eigen::Isometry3d TA0;

double zoom_ratio = 4;	//相机视野缩放比例,因2.5代机器人相机不支持查询缩放倍数，故将其固定为 2（暂定）
extern std::vector<double> joint_now;
extern std::vector<traj_point>traj;

extern int lc_now;
extern int lc_zero;

extern int ARM_STATE;
extern bool FOCUS_STATE;
extern int MOVE_TYPE;

extern cv::Mat oa_bg;

extern arm_collision_model ACM;
extern std::vector<cv::RotatedRect> obstacles;

extern int rs_end_left, rs_end_right, rs_top_left, rs_top_right;

extern vector<gloRS*> RSs;
#pragma endregion

#pragma region 全局变量
double oa_height = 1.25;    //目标拍摄高度
double d_c2t = -(lc_now + lc_zero) / 1000.0 + oa_height;    //拍摄平面z坐标
double oa_val = 0;  //遮挡值
double oa_threshold = 25;    //遮挡阈值

Eigen::Isometry3d T_3d_cam_left = Eigen::Isometry3d::Identity();   //3D相机0（末端左侧）变换矩阵
Eigen::Isometry3d T_3d_cam_right = Eigen::Isometry3d::Identity();   //3D相机2（末端右侧）变换矩阵
Eigen::Isometry3d T_cam_main = Eigen::Isometry3d::Identity();    //主相机变换矩阵

Eigen::Vector3d focus_point;    //目标锁定点

int data_num = 0;
int pcd_num = 1;

//强化学习相关
int RL_point_num = 1;  //点云保存
int RL_occlusion_avoid_flag = 1;
static std::wstring policy_model_path()
{
    if (const char* path = std::getenv("AUTOSURGVIEW_MODEL"))
        return std::filesystem::path(path).wstring();
    return std::filesystem::path("robot-controller/models/ppo_AnzhenRobotContinue_new9.onnx").wstring();
}
//const ORTCHAR_T* modelPath = L"C:/software/SSBOT2-数据测试/myonnx/RL/ppo_AnzhenRobot_pointcloud.onnx";
// 初始化 ONNX Runtime
// 初始化ONNX Runtime环境
Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
// 创建会话配置选项对象
// 用于配置会话的各种参数：
Ort::SessionOptions session_options;
#pragma endregion

#pragma region  MIDACO优化算法相关
/***********************************************************************/
extern"C" { int midaco(long int*, long int*, long int*, long int*, long int*,
    long int*, double*, double*, double*, double*, double*,
    long int*, long int*, double*, double*, long int*,
    long int*, long int*, double*, long int*, char*); }
/***********************************************************************/
extern"C" { int midaco_print(int, long int, long int, long int*, long int*, double*,
    double*, double*, double*, double*, long int, long int,
    long int, long int, long int, double*, double*,
    long int, long int, double*, long int, char*); }
/***********************************************************************/
#pragma endregion

#pragma region Pyramid类函数
//Pyramid类默认构造函数
Pyramid::Pyramid()
{
    color = Eigen::Vector3d(255, 255, 255);

    center = Eigen::Vector4d(0, 0, 0, 1);
    center_img = Eigen::Vector4d(0, 0, 1, 1);
    
    points.resize(4);

    points[0] = Eigen::Vector4d(0.621, 0.349, 1, 1);
    points[1] = Eigen::Vector4d(-0.621, 0.349, 1, 1);
    points[2] = Eigen::Vector4d(-0.621, -0.349, 1, 1);
    points[3] = Eigen::Vector4d(0.621, -0.349, 1, 1);
}

//Pyramid类构造函数
//@param Ratio 相机缩放比例
//@param Height 目标拍摄高度（机器人坐标系下）
//@param Name 锥体名称
//@param Color 锥体绘制颜色
//@param T 锥体至机器人坐标系变换矩阵
Pyramid::Pyramid(double Ratio, double Height, std::string Name, Eigen::Vector3d Color, Eigen::Isometry3d T)
{
    color = Color;

    center = Eigen::Vector4d(0, 0, 0, 1);
    center_img = Eigen::Vector4d(0, 0, 1, 1);
    
    //根据变换矩阵，将锥体坐标转换至机器人坐标系
    center = T * center;
    center_img = T * center_img;

    points.resize(4);

    T_cam = T;
    ratio = Ratio;
    height = Height;

    name = Name;

    //根据缩放比例，计算角点在锥体坐标系内坐标
    points[0] = Eigen::Vector4d(0.621 / ratio, 0.349 / ratio, 1, 1);
    points[1] = Eigen::Vector4d(-0.621 / ratio, 0.349 / ratio, 1, 1);
    points[2] = Eigen::Vector4d(-0.621 / ratio, -0.349 / ratio, 1, 1);
    points[3] = Eigen::Vector4d(0.621 / ratio, -0.349 / ratio, 1, 1);

    //坐标变换，并将锥体四棱边延长至目标拍摄高度平面
    Eigen::Vector4d point_temp;
    for (auto& point : points)
    {
        point_temp = T * point;
        point = center + (point_temp - center) / (point_temp[2] - center[2]) * (height - center[2]);
        point[3] = 1;
    }

    //计算锥体坐标轴矢量在机器人坐标系下数据
    cal_vec(T);
    //计算用于绘制锥体坐标轴所需点
    axis_points.resize(4);
    axis_points[0] = point_o;
    axis_points[1] = point_o + vec_x;
    axis_points[2] = point_o + vec_y;
    axis_points[3] = point_o + vec_n;
}

Pyramid::~Pyramid()
{

}

//Pyramid类成员函数：设置锥体缩放比例、目标拍摄高度、变换矩阵
//@param Ratio 相机缩放比例
//@param Height 目标拍摄高度（机器人坐标系下）
//@param T 锥体至机器人坐标系变换矩阵
void Pyramid::set_ratio_height(double Ratio, double Height, Eigen::Isometry3d T)
{
    //判断上述参数与锥体现有参数是否相同（避免无用计算）
    if (ratio != Ratio || height != Height || T_cam.data() != T.data())
    {
        //更新参数
        T_cam = T;
        ratio = Ratio;
        height = Height;

        //根据变换矩阵，将锥体坐标转换至机器人坐标系
        center = Eigen::Vector4d(0, 0, 0, 1);
        center_img = Eigen::Vector4d(0, 0, 1, 1);

        center = T * center;
        center_img = T * center_img;

        //根据缩放比例，计算角点在锥体坐标系内坐标
        points[0] = Eigen::Vector4d(0.621 / ratio, 0.349 / ratio, 1, 1);
        points[1] = Eigen::Vector4d(-0.621 / ratio, 0.349 / ratio, 1, 1);
        points[2] = Eigen::Vector4d(-0.621 / ratio, -0.349 / ratio, 1, 1);
        points[3] = Eigen::Vector4d(0.621 / ratio, -0.349 / ratio, 1, 1);

        //坐标变换，并将锥体四棱边延长至目标拍摄高度平面
        Eigen::Vector4d point_temp;
        for (auto& point : points)
        {
            point_temp = T * point;
            point = center + (point_temp - center) / (point_temp[2] - center[2]) * (height - center[2]);
            point[3] = 1;
        }

        //计算锥体坐标轴矢量在机器人坐标系下数据
        cal_vec(T);
        //计算用于绘制锥体坐标轴所需点
        axis_points.resize(4);
        axis_points[0] = point_o;
        axis_points[1] = point_o + vec_x;
        axis_points[2] = point_o + vec_y;
        axis_points[3] = point_o + vec_n;
    }
}

//Pyramid类成员函数：PCL窗口中绘制锥体
//@param viewer 绘制窗口
void Pyramid::draw_pyramid(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    //绘制四条棱边
    for (int i = 0; i < 4; i++)
    {
        viewer->addLine(pcl::PointXYZ(center[0], center[1], center[2]), pcl::PointXYZ(points[i][0], points[i][1], points[i][2]), 
            color[0], color[1], color[2], name + "_cp" + std::to_string(i));
    }

    //绘制四条底边
    for (int i = 0; i < 4; i++)
    {
        int j = (i + 1) % 4;
        viewer->addLine(pcl::PointXYZ(points[i][0], points[i][1], points[i][2]), pcl::PointXYZ(points[j][0], points[j][1], points[j][2]),
            color[0], color[1], color[2], name + "_p"+ std::to_string(i) +"p" + std::to_string(j));
    }

    //绘制三条坐标轴
    for (int i = 1; i < 4; i++)
    {
        viewer->addLine(pcl::PointXYZ(axis_points[0][0], axis_points[0][1], axis_points[0][2]), pcl::PointXYZ(axis_points[i][0], axis_points[i][1], axis_points[i][2]),
            color[0], color[1], color[2], name + "_axis" + std::to_string(i));
    }
}

//Pyramid类成员函数：PCL窗口中删除锥体
//@param viewer 绘制窗口
void Pyramid::remove_pyramid(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    //删除四条棱边
    for (int i = 0; i < 4; i++)
    {
        viewer->removeShape(name + "_cp" + std::to_string(i));
    }

    //删除四条底边
    for (int i = 0; i < 4; i++)
    {
        int j = (i + 1) % 4;
        viewer->removeShape(name + "_p" + std::to_string(i) + "p" + std::to_string(j));
    }

    //删除三条坐标轴
    for (int i = 1; i < 4; i++)
    {
        viewer->removeShape(name + "_axis" + std::to_string(i));
    }
}

//Pyramid类成员函数：判断一点是否在锥体内部
//@param v 坐标点
//@return true if v is in the pyramid
bool Pyramid::is_point_in(Eigen::Vector4d v)
{
    //将坐标点v与锥体中心连线，延伸射线至目标拍摄平面（即投影至目标拍摄平面）
    v = center + (points[0][2] - center[2]) / (v[2] - center[2]) * (v - center);

    //判断投影点是否在锥体底面四边形内部
    //采用射线延伸法判断
    int num = 0;
    for (int i = 0; i < 4; i++)
    {
        int ind1 = i;
        int ind2 = (i + 1) % 4;

        if ((v[1] - points[ind2][1]) * (v[1] - points[ind1][1]) <= 0 && v[1] != points[ind2][1])
        {
            double x_hat = points[ind1][0] + (points[ind2][0] - points[ind1][0]) * (v[1] - points[ind1][1]) / (points[ind2][1] - points[ind1][1]);
            if (x_hat >= v[0])
            {
                num++;
            }
        }
    }

    if (num % 2 != 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//Pyramid类成员函数：计算锥体坐标轴矢量
//@param T 变换矩阵
void Pyramid::cal_vec(Eigen::Isometry3d T)
{
    point_o = T * Eigen::Vector4d(0, 0, 0, 1);
    vec_n = T * Eigen::Vector4d(0, 0, 1, 0);
    vec_x = T * Eigen::Vector4d(1, 0, 0, 0);
    vec_y = T * Eigen::Vector4d(0, 1, 0, 0);
}
#pragma endregion


#pragma region 辅助函数
//设置目标拍摄点
void set_focus_point()
{
    get_T(T_3d_cam_left, T_3d_cam_right, T_cam_main);  //更新矩阵
    focus_point = Eigen::Vector3d(T_cam_main.data()[12], T_cam_main.data()[13], T_cam_main.data()[14]); //相机原点
    
    Eigen::Vector3d z_axis; //相机Z轴矢量
    z_axis[0] = T_cam_main.data()[8];
    z_axis[1] = T_cam_main.data()[9];
    z_axis[2] = T_cam_main.data()[10];

    d_c2t = -(lc_now + lc_zero) * 0.001 + oa_height;

    focus_point += (d_c2t - focus_point[2]) / (z_axis[2]) * z_axis; //相机原点延Z轴投影至目标拍摄平面
    
    cout << "set focus_point: [" << focus_point[0] << ", " << focus_point[1] << ", " << focus_point[2] << "]" << endl;

    //data_num++;
    //std::string filePath = "C:/Users/Butel/Desktop/SSBOT2/pointcloud/" + std::to_string(data_num) + ".txt";
    //// 打开文件
    //std::ofstream outFile(filePath);
    //if (outFile.is_open()) {
    //    // 将向量的数据写入文件
    //    outFile << focus_point[0] << " " << focus_point[1] << " " << focus_point[2];
    //    // 关闭文件
    //    outFile.close();
    //    std::cout << "数据已成功保存到 " << filePath << std::endl;
    //}
    //else {
    //    std::cerr << "无法打开文件 " << filePath << std::endl;
    //}
    //pcd_num = 1;
}


//将3D camera点云转化为pcl点云
//@param cloud_in   //锥体内部点云
//@param cloud_out  //锥体外部点云
//@param points     //realsense点云数据
//@param T_3d_cam   //3D camera变换矩阵
//@param pyramid    //视场锥体
void PCL_Conversion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, const rs2::points& points, Eigen::Isometry3d& T_3d_cam, Pyramid& pyramid)  //点云分割为锥形内外
{
    double z_min = T_cam_main.data()[14] + d_c2t + 0.2; //最低高度，高于目标拍摄平面0.2m

    double z_max = T_cam_main.data()[14] - 0.05;    //最高高度，低于相机0.05m
   
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    auto Vertex = points.get_vertices();    //获取realsense点云

    //realsense点云转换至PCL
    Eigen::Vector4d temp;
    Eigen::Vector4d temp_after;
    for (int i = 0; i < points.size(); i += 50)    //每100个点采1个
    {
        //realsense坐标系下
        temp[0] = Vertex[i].x;
        temp[1] = Vertex[i].y;
        temp[2] = Vertex[i].z;
        temp[3] = 1;

        //转换至机器人坐标系下
        temp_after = T_3d_cam * temp;

        if (temp_after[2] >= z_min && temp_after[2] <= z_max
            && abs(temp_after[1]) <= 2.5 && abs(temp_after[0]) <= 2.5)  //判断点云是否在可能发生遮挡的区间内
        {
            if (pyramid.is_point_in(temp_after))    //若点在锥体内
            {
                cloud_in->points.push_back(pcl::PointXYZ(temp_after[0], temp_after[1], temp_after[2])); //加入cloud_in
            }
            else
            {
                cloud_out->points.push_back(pcl::PointXYZ(temp_after[0], temp_after[1], temp_after[2])); //加入cloud_out
            }
        }
    }
}

//将pcl点云保存用于强化学习使用
//@param cloud_RL    //强化学习保存点云
//@param points     //realsense点云数据
//@param T_3d_cam   //3D camera变换矩阵
void PCL_RLSave(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_RL, const rs2::points& points, Eigen::Isometry3d& T_3d_cam)
{
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    auto Vertex = points.get_vertices();    //获取realsense点云

    //realsense点云转换至PCL
    Eigen::Vector4d temp;
    Eigen::Vector4d temp_after;
    for (int i = 0; i < points.size(); i += 50)    //每80个点采1个
    {
        //realsense坐标系下
        temp[0] = Vertex[i].x;
        temp[1] = Vertex[i].y;
        temp[2] = Vertex[i].z;
        temp[3] = 1;

        //转换至机器人坐标系下
        temp_after = T_3d_cam * temp;

        cloud_RL->points.push_back(pcl::PointXYZ(temp_after[0], temp_after[1], temp_after[2]));
    }
}

//根据实际关节角度更新矩阵
//@param T_3d_cam 3D camera变换矩阵
//@param T_cam_main 相机矩阵
void get_T(Eigen::Isometry3d& T_3d_cam_left, Eigen::Isometry3d& T_3d_cam_right, Eigen::Isometry3d& T_cam_main)
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

    T_3d_cam_left = T10 * T21 * T32 * T73;
    T_3d_cam_right = T10 * T21 * T32 * T83;

    T_cam_main = T10 * T21 * T32 * T43 * T54 * T65;
}

//根据给定关节角度更新矩阵
//@param T_cam_main 相机矩阵
//@param theta 给定关节角度数组
void get_T(Eigen::Isometry3d& T_cam_main, std::vector<double> theta)
{
    //变换矩阵初始化
    {
        T10 = Eigen::Isometry3d::Identity();
        T21 = Eigen::Isometry3d::Identity();
        T32 = Eigen::Isometry3d::Identity();
        T43 = Eigen::Isometry3d::Identity();
        T54 = Eigen::Isometry3d::Identity();
        T65 = Eigen::Isometry3d::Identity();
    }

    //以关节1圆心为原点（高度方向与关节4轴心平齐） T10
    {
        T10.rotate(Eigen::AngleAxisd(theta[0], Eigen::Vector3d(0, 0, 1)));
    }

    //关节2->关节1 T21
    {
        T21.translate(Eigen::Vector3d(0, 0.7, 0));
        T21.rotate(Eigen::AngleAxisd(theta[1], Eigen::Vector3d(0, 0, 1)));
    }

    //关节3->关节2 T32
    {
        T32.translate(Eigen::Vector3d(0, 0.5, 0));
        T32.rotate(Eigen::AngleAxisd(theta[2], Eigen::Vector3d(0, 0, 1)));
    }

    //关节4->关节3 T43
    {
        T43.translate(Eigen::Vector3d(0, 0.4, 0));
        T43.rotate(Eigen::AngleAxisd(theta[3], Eigen::Vector3d(0, 1, 0)));
    }

    //关节5->关节4 T54
    {
        T54.translate(Eigen::Vector3d(0, 0.1118, 0));
        T54.rotate(Eigen::AngleAxisd(theta[4], Eigen::Vector3d(1, 0, 0)));
    }

    //相机->关节5 T65
    {
        T65.rotate(Eigen::AngleAxisd(PI + theta[5], Eigen::Vector3d(1, 0, 0)));
    }


    T_cam_main = T10 * T21 * T32 * T43 * T54 * T65;
}

//获得点云遮挡值,应用于实时遮挡检测
//@param pyramid 视场锥体
//@param cloud_in 锥体内部点云
//@param src 背景图
double get_occlusion_val(Pyramid& pyramid, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
    cv::Mat& src)
{
    if (cloud_in->empty())  //若锥体内无点云，直接返回0
    {
        return 0;
    }

    double res = 0;

    Eigen::Vector2d vec_temp;
    double v_val = 0;
    

    src *= 0;

    
    for (auto point : cloud_in->points)
    {
        Eigen::Vector4d vec_point;  //由锥体中心指向点point
        vec_point[0] = point.x - pyramid.center[0];
        vec_point[1] = point.y - pyramid.center[1];
        vec_point[2] = point.z - pyramid.center[2];
        vec_point[3] = 0;

        double k = 1 / (vec_point.dot(pyramid.vec_n));
        Eigen::Vector4d vec_project = k * vec_point - pyramid.vec_n;    //将点投影至图像平面

        double x = vec_project.dot(pyramid.vec_x);  //投影点在图像平面的X方向分量
        double y = vec_project.dot(pyramid.vec_y);  //投影点在图像平面的Y方向分量

        //模拟图像平面遮挡情况
        cv::circle(src, cv::Point2d(160 + x * pyramid.ratio * 386.47, 90 + y * pyramid.ratio * 257.88), 1, cv::Scalar(255, 255, 255));

        res += 1 / vec_project.norm();  //遮挡值累加
    }

    return res;
}

//获得点云遮挡值（无绘制功能），应用于遮挡规避搜索
//@param pyramid 视场锥体
//@param cloud_in 锥体内部点云
void get_occlusion_val(Pyramid& pyramid, pcl::PointCloud<pcl::PointXYZ>& cloud_in, pos_with_oa_info& pwoi)
{
    if (cloud_in.empty())  //若锥体内无点云，直接返回0
    {
        pwoi.oa_val = 0;
        return;
    }

    double res = 0;
    Eigen::Vector4d vec = Eigen::Vector4d(0, 0, 0, 0);

    Eigen::Vector2d vec_temp;
    double v_val = 0;


    for (auto point : cloud_in.points)
    {
        Eigen::Vector4d vec_point;  //由锥体中心指向点point
        vec_point[0] = point.x - pyramid.center[0];
        vec_point[1] = point.y - pyramid.center[1];
        vec_point[2] = point.z - pyramid.center[2];
        vec_point[3] = 0;

        double k = 1 / (vec_point.dot(pyramid.vec_n));

        Eigen::Vector4d vec_project = k * vec_point - pyramid.vec_n;    //由图像中心 指向 点point投影至图片点

        //double x = vec_project.dot(pyramid.vec_x);  //投影点在图像平面的X方向分量
        //double y = vec_project.dot(pyramid.vec_y);  //投影点在图像平面的Y方向分量

        double vec_project_norm = vec_project.norm();
        vec_project /= vec_project_norm;
        vec.x() -= vec_project.x() / vec_project_norm;
        vec.y() -= vec_project.y() / vec_project_norm;

        res += 1 / vec_project_norm;  //遮挡值累加
    }

    pwoi.oa_val = res;

    vec /= vec.norm();
    pwoi.oa_vec.x() = vec.x();
    pwoi.oa_vec.y() = vec.y();
}

//获得强化信息点云遮挡值，应用于遮挡规避搜索
//@param pyramid 视场锥体
//@param cloud_in 锥体内部点云
//@param cloud_out 锥体外部点云
//@param pwoi 术野相机位置
void get_occlusion_info_RL(Pyramid& pyramid, pcl::PointCloud<pcl::PointXYZ>& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>& cloud_out, pos_with_oa_info& pwoi)
{
    if (cloud_in.empty())  //若锥体内无点云，直接返回0
    {
        pwoi.oa_val = 0;
        return;
    }

    double res = 0;
    Eigen::Vector2d vec = Eigen::Vector2d(0.0, 0.0);
    Eigen::Vector2d global_vec = Eigen::Vector2d(0.0, 0.0);

    Eigen::Vector2d vec_temp(0.0, 0.0);
    double vec_sum = 0;

    double centroid_x = 0.0;
    double centroid_y = 0.0;
    double centroid_z = 0.0;
    int cloud_in_points_num = 0;

    for (auto point : cloud_in.points)
    {
        Eigen::Vector4d vec_point;  //由锥体中心指向点point
        vec_point[0] = point.x - pyramid.center[0];
        vec_point[1] = point.y - pyramid.center[1];
        vec_point[2] = point.z - pyramid.center[2];
        vec_point[3] = 0;

        centroid_x += point.x;
        centroid_y += point.y;
        centroid_z += point.z;
        cloud_in_points_num++;

        double k = 1 / (vec_point.dot(pyramid.vec_n));

        Eigen::Vector4d vec_project = k * vec_point - pyramid.vec_n;    //由图像中心 指向 点point投影至图片点

        vec_temp.x() = vec_project.x();
        vec_temp.y() = vec_project.y();

        double vec_project_norm = vec_project.norm();
        double vec_temp_norm = vec_temp.norm();

        vec.x() -= vec_temp.x() / vec_temp_norm * vec_project_norm;
        vec.y() -= vec_temp.y() / vec_temp_norm * vec_project_norm;

        res += 1 / vec_project_norm;  //遮挡值累加
        vec_sum += vec_project_norm;
    }

    pwoi.oa_val = res;
    global_vec = vec;
    vec /= vec_sum;
    pwoi.oa_RL_vec = vec / vec.norm();

    centroid_x /= cloud_in_points_num;
    centroid_y /= cloud_in_points_num;
    centroid_z /= cloud_in_points_num;
    vec_temp.x() = centroid_x - focus_point[0];
    vec_temp.y() = centroid_y - focus_point[1];
    pwoi.global_oa_vec = vec_temp;

    for (auto point : cloud_out.points)
    {
        Eigen::Vector4d vec_point;  //由锥体中心指向点point
        vec_point[0] = point.x - pyramid.center[0];
        vec_point[1] = point.y - pyramid.center[1];
        vec_point[2] = point.z - pyramid.center[2];
        vec_point[3] = 0;

        double k = 1 / (vec_point.dot(pyramid.vec_n));

        Eigen::Vector4d global_vec_project = k * vec_point - pyramid.vec_n;    //由图像中心 指向 点point投影至图片点

        vec_temp.x() = global_vec_project.x();
        vec_temp.y() = global_vec_project.y();

        double  global_vec_project_norm = global_vec_project.norm();
        double vec_temp_norm = vec_temp.norm();

        global_vec.x() -= vec_temp.x() / vec_temp_norm * global_vec_project_norm;
        global_vec.y() -= vec_temp.y() / vec_temp_norm * global_vec_project_norm;

        vec_sum += global_vec_project_norm;
    }

    global_vec /= vec_sum;
    pwoi.global_oa_vec = global_vec / global_vec.norm();
}

//调用强化学习模型计算新的术野相机位置，应用于遮挡规避搜索
//@param pwoi 术野相机位置
Eigen::Vector2d get_delta_pos_RL(pos_with_oa_info& pwoi, double unreach_num)
{
    Eigen::Vector2d delta_pos(0.0, 0.0);
    double max_position_x = 1.4, min_position_x = -1.4, max_position_y = 1.75, min_position_y = 0.6;
    double max_occlusion_val = 500.0, max_unreach_num = 10.0;

    // 创建ONNX推理会话
    // 参数1: 之前创建的环境对象(env)
    // 参数2: ONNX模型文件路径
    // 参数3: 会话配置选项
    // 说明: 此操作会加载模型到内存并准备执行环境
    const std::wstring modelPath = policy_model_path();
    Ort::Session session(env, modelPath.c_str(), session_options);

    // 使用 Allocator 获取输入/输出名称（1.14.0 新API）
    Ort::AllocatorWithDefaultOptions allocator;

    // 获取输入名称
    Ort::AllocatedStringPtr input_name_ptr = session.GetInputNameAllocated(0, allocator);
    const char* input_name = input_name_ptr.get();

    // 获取输出名称
    Ort::AllocatedStringPtr output_name_ptr = session.GetOutputNameAllocated(0, allocator);
    const char* output_name = output_name_ptr.get();

    // 定义输入形状
    std::vector<int64_t> input_shape = { 1, 8 };

    // 创建输入张量
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtDeviceAllocator,
        OrtMemType::OrtMemTypeCPU
    );

    // 准备输入数据
    float x_positoin_range = max_position_x - min_position_x;
    float y_positoin_range = max_position_y - min_position_y;
    float RL_input[9];
    RL_input[0] = (pwoi.position.first - min_position_x) / x_positoin_range;
    RL_input[1] = (pwoi.position.second - min_position_y) / y_positoin_range;
    RL_input[2] = std::min(pwoi.oa_val / max_occlusion_val, 1.0);
    RL_input[3] = pwoi.oa_RL_vec[0];
    RL_input[4] = pwoi.oa_RL_vec[1];
    RL_input[5] = pwoi.global_oa_vec[0];
    RL_input[6] = pwoi.global_oa_vec[1];
    RL_input[7] = pwoi.oa_centroid_vec[0];
    RL_input[8] = pwoi.oa_centroid_vec[1];
    std::vector<float> input_data = { RL_input[0], RL_input[1],RL_input[2], RL_input[3],RL_input[4], RL_input[5],
        RL_input[6], (float)std::min(unreach_num / max_unreach_num, 1.0) };
    std::vector<const char*> input_names = { input_name };
    std::vector<const char*> output_names = { output_name };

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        input_data.data(),
        input_data.size(),
        input_shape.data(),
        input_shape.size()
        );

    // 运行推理（使用新API）
    auto output_tensors = session.Run(
        Ort::RunOptions{ nullptr },
        input_names.data(),      // 输入名称数组
        &input_tensor,           // 输入张量数组
        1,                       // 输入数量
        output_names.data(),    // 输出名称数组
        1                        // 输出数量
    );

    // 获取输出
    float* output_data = output_tensors.front().GetTensorMutableData<float>(); //front获取张量中的第一个元素

    delta_pos[0] = std::max(std::min(output_data[0],1.0f), -1.0f);
    delta_pos[1] = std::max(std::min(output_data[1], 1.0f), -1.0f);

    return delta_pos;
}

//强化学习点云处理函数，将PCL点云处理为强化学习模型的输入格式
// 辅助函数：处理单个 PCL 点云
std::vector<float> ProcessSingleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = *cloud_1 + *cloud_2;
    int target_point_num = 512;
    std::vector<float> processed_points(target_point_num * 3);

    // PCL点云判空与获取大小
    if (!cloud || cloud->empty()) {
        std::fill(processed_points.begin(), processed_points.end(), 0.0f);
        return processed_points;
    }

    int num_raw_points = cloud->points.size();

    // 创建索引列表 [0, 1, 2, ..., N-1]
    std::vector<int> indices(num_raw_points);
    std::iota(indices.begin(), indices.end(), 0);

    // 随机数生成器
    std::random_device rd;
    std::mt19937 g(rd());

    if (num_raw_points >= target_point_num) {
        // === 情况A：点数过多，随机采样 ===
        std::shuffle(indices.begin(), indices.end(), g);
        for (int i = 0; i < target_point_num; ++i) {
            int original_idx = indices[i];
            // 从 PCL 数据结构中获取 x, y, z
            processed_points[i * 3 + 0] = cloud->points[original_idx].x - focus_point[0];
            processed_points[i * 3 + 1] = cloud->points[original_idx].y - focus_point[1];
            processed_points[i * 3 + 2] = cloud->points[original_idx].z - focus_point[2];
        }
    }
    else {
        // === 情况B：点数不足，重复填充 ===
        // 先拷贝所有原始点
        for (int i = 0; i < num_raw_points; ++i) {
            processed_points[i * 3 + 0] = cloud->points[i].x - focus_point[0];
            processed_points[i * 3 + 1] = cloud->points[i].y - focus_point[1];
            processed_points[i * 3 + 2] = cloud->points[i].z - focus_point[2];
        }
        // 剩余位置随机采样填充
        for (int i = num_raw_points; i < target_point_num; ++i) {
            // 随机选一个已有的点的索引
            int rand_idx = std::uniform_int_distribution<>(0, num_raw_points - 1)(g);
            processed_points[i * 3 + 0] = cloud->points[rand_idx].x - focus_point[0];
            processed_points[i * 3 + 1] = cloud->points[rand_idx].y - focus_point[1];
            processed_points[i * 3 + 2] = cloud->points[rand_idx].z - focus_point[2];
        }
    }

    return processed_points;
}

//调用强化学习模型计算新的术野相机位置，应用于遮挡规避搜索
//@param pwoi 术野相机位置
Eigen::Vector2d get_delta_pos_RL_POINT(pos_with_oa_info& pwoi, double unreach_num, std::vector<float> point_cloud_data)
{
    Eigen::Vector2d delta_pos(0.0, 0.0);
    double max_position_x = 1.4, min_position_x = -1.4, max_position_y = 1.75, min_position_y = 0.6;
    double max_occlusion_val = 500.0, max_unreach_num = 5.0;
    int target_point_num = 512;

    // 创建ONNX推理会话
    // 参数1: 之前创建的环境对象(env)
    // 参数2: ONNX模型文件路径
    // 参数3: 会话配置选项
    // 说明: 此操作会加载模型到内存并准备执行环境
    const std::wstring modelPath = policy_model_path();
    Ort::Session session(env, modelPath.c_str(), session_options);

    // 使用 Allocator 获取输入/输出名称（1.14.0 新API）
    Ort::AllocatorWithDefaultOptions allocator;

    // 获取输入名称
    size_t num_input_nodes = session.GetInputCount();
    std::vector<const char*> input_node_names;
    std::vector<Ort::AllocatedStringPtr> input_names_ptr; // 必须保持指针存活

    // 遍历获取所有输入名称 ["point_cloud", "robot_state"]
    for (size_t i = 0; i < num_input_nodes; i++) {
        auto input_name = session.GetInputNameAllocated(i, allocator);
        input_node_names.push_back(input_name.get());
        input_names_ptr.push_back(std::move(input_name));
    }

    // 获取输出名称
    Ort::AllocatedStringPtr output_name_ptr = session.GetOutputNameAllocated(0, allocator);
    const char* output_node_names[] = { output_name_ptr.get() };

    // 定义输入形状
    std::vector<int64_t> point_cloud_shape = { 1, target_point_num, 3 };
    std::vector<int64_t> robot_state_shape = { 1, 6 }; 

    // 准备机器人状态输入数据
    float x_positoin_range = max_position_x - min_position_x;
    float y_positoin_range = max_position_y - min_position_y;
    float RL_input[9];
    RL_input[0] = (pwoi.position.first - min_position_x) / x_positoin_range;
    RL_input[1] = (pwoi.position.second - min_position_y) / y_positoin_range;
    RL_input[2] = std::min(pwoi.oa_val / max_occlusion_val, 1.0);
    RL_input[3] = pwoi.oa_RL_vec[0];
    RL_input[4] = pwoi.oa_RL_vec[1];
    std::vector<float> robot_state_data = { RL_input[0], RL_input[1],RL_input[2], RL_input[3],RL_input[4],
        (float)std::min(unreach_num / max_unreach_num, 1.0) };

    //创建输入张量
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    std::vector<Ort::Value> input_tensors;
    // 创建 PointCloud Tensor
    input_tensors.push_back(Ort::Value::CreateTensor<float>(
        memory_info,
        point_cloud_data.data(), point_cloud_data.size(),
        point_cloud_shape.data(), point_cloud_shape.size()
        ));
    // 创建 RobotState Tensor
    input_tensors.push_back(Ort::Value::CreateTensor<float>(
        memory_info,
        robot_state_data.data(), robot_state_data.size(),
        robot_state_shape.data(), robot_state_shape.size()
        ));

    float* output_data;
    try {
        // 注意：session.Run 的输入顺序必须与 input_node_names 里的顺序一致
        auto output_tensors = session.Run(
            Ort::RunOptions{ nullptr },
            input_node_names.data(), // 输入名称数组
            input_tensors.data(),    // 输入张量数组
            2,                       // 输入数量 (现在是 2 个)
            output_node_names,       // 输出名称数组
            1                        // 输出数量
        );
        //获取输出结果
        output_data = output_tensors[0].GetTensorMutableData<float>();
    }
    catch (const Ort::Exception& e) {
        std::cerr << "ONNX Inference Error: " << e.what() << std::endl;
    }

    delta_pos[0] = std::max(std::min(output_data[0], 1.0f), -1.0f);
    delta_pos[1] = std::max(std::min(output_data[1], 1.0f), -1.0f);

    return delta_pos;
}

//判断函数，对正视目标拍摄点的位置进行遮挡检测
bool judge_init_pos(Pyramid& pyramid, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
    pos_with_oa_info pwoi_find;
    Eigen::Vector3d color(0, 255, 0);   //设置探索视场锥体颜色为绿色
    std::pair<double, double> pi = { focus_point(0), focus_point(1) };
    pwoi_find.position = pi;
    pwoi_find.oa_val = 1.0;
    pwoi_find.pos_val = 1.0;
    Eigen::Isometry3d T_cam_main;
    std::vector<double> theta_solv(6);
    pcl::PointCloud<pcl::PointXYZ> cloud_in_temp;

    if (!(solve_focus_kdl(joint_now, pwoi_find.position, focus_point, theta_solv) && !ACM.collision(theta_solv[0] / PI * 180, theta_solv[1] / PI * 180, theta_solv[2] / PI * 180, obstacles)))
        return false;
    get_T(T_cam_main, theta_solv);  //获得theta_solv情况下的相机变换矩阵
    Pyramid pyramid_temp(pyramid.ratio, pyramid.height, "pyramid_temp", color, T_cam_main); //创建临时视场锥体

    for (auto& point : cloud_in->points)
    {
        Eigen::Vector4d vec_point;
        vec_point[0] = point.x;
        vec_point[1] = point.y;
        vec_point[2] = point.z;
        vec_point[3] = 1;

        if (pyramid_temp.is_point_in(vec_point))
        {
            cloud_in_temp.push_back(point);
        }
    }

    for (auto& point : cloud_out->points)
    {
        Eigen::Vector4d vec_point;
        vec_point[0] = point.x;
        vec_point[1] = point.y;
        vec_point[2] = point.z;
        vec_point[3] = 1;

        if (pyramid_temp.is_point_in(vec_point))
        {
            cloud_in_temp.push_back(point);
        }
    }

    get_occlusion_val(pyramid_temp, cloud_in_temp, pwoi_find);
    if (pwoi_find.oa_val < oa_threshold)
        return true;
    else
        return false;
}

//比较函数，对带遮挡值的相机位置点进行比较
//@param a,b 带遮挡值的相机位置点
bool cmp_pos_with_oa_info(pos_with_oa_info& a, pos_with_oa_info& b)
{
    if (a.oa_val > b.oa_val)
    {
        return true;
    }

    if (a.oa_val == b.oa_val)
    {
        return a.pos_val < b.pos_val;
    }

    return false;

    //return a.oa_val > b.oa_val;
}

//获得无遮挡目标点
//@param pyramid    相机视场锥体
//@param pwoi_ori   带遮挡值的相机位置点，初始点
//@param cloud_in   锥体内部点云
//@param cloud_out  锥体外部点云
//@param step       迭代步长
//@param joint_now  机械臂当前各关节角度
//@param viewer     点云绘制窗口
//@return position  解得无遮挡拍摄位置（若未解得，返回初始点）
std::pair<double, double> get_occlusion_target(Pyramid& pyramid, pos_with_oa_info& pwoi_ori, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double step, std::vector<double> joint_now,
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = nullptr)
{
    std::vector<double> direction = { step, 0, -step, 0, step };    //相机位置迭代方向数组
    std::unordered_set<int> visited;    //哈希表，存储已探索过的位置

    visited.insert(pyramid.center[0] * 20000 + pyramid.center[1] * 100);
    std::priority_queue<pos_with_oa_info, std::vector<pos_with_oa_info>, decltype(&cmp_pos_with_oa_info)> q(cmp_pos_with_oa_info);  //优先队列，优先探索遮挡值较低的点

    get_occlusion_val(pyramid, *cloud_in, pwoi_ori);   //计算临时视场锥体遮挡值

    //将初始点周边四个点加入队列，其遮挡值初步置为初始点的遮挡值
    for (int i = 0; i < 4; i++)
    {
        std::pair<double, double> pi = { pyramid.center[0] + direction[i],pyramid.center[1] + direction[i + 1] };
        pos_with_oa_info pwoi_temp;
        pwoi_temp.position = pi;
        pwoi_temp.oa_val = pwoi_ori.oa_val;
        pwoi_temp.pos_val = direction[i] * pwoi_ori.oa_vec.x() + direction[i + 1] * pwoi_ori.oa_vec.y();

        q.push(pwoi_temp);  //加入队列

        visited.insert(pi.first * 20000 + pi.second * 100); //加入哈希表
    }
    
    Eigen::Vector3d color(0, 255, 0);   //设置探索视场锥体颜色为绿色

    while (!q.empty())  //队列非空，则继续迭代
    {
        if (!FOCUS_STATE)   //若目标锁定停止，退出
        {
            return { pyramid.center[0],pyramid.center[1] };
        }

        auto pwoi_top = q.top();    //取队首元素
        q.pop();    //弹出队首元素

        double x = pwoi_top.position.first;
        double y = pwoi_top.position.second;

        std::vector<double> theta_solv(6);

        //判断队首位置机械臂是否可达（逆运动学是否有解）
        //判断逆运动学解是否满足无碰撞
        //判断云台电机位置是否在限制范围之内
        if (solve_focus_kdl(joint_now, pwoi_top.position, focus_point, theta_solv) && !ACM.collision(theta_solv[0] / PI * 180, theta_solv[1] / PI * 180, theta_solv[2] / PI * 180, obstacles)
            && abs(theta_solv[4]) / PI * 180 < 40 && abs(theta_solv[3]) / PI * 180 < 40)
        {
            Eigen::Isometry3d T_cam_main;
            get_T(T_cam_main, theta_solv);  //获得theta_solv情况下的相机变换矩阵
            Pyramid pyramid_temp(pyramid.ratio, pyramid.height, "pyramid_temp", color, T_cam_main); //创建临时视场锥体

            pcl::PointCloud<pcl::PointXYZ> cloud_in_temp;   //临时视场锥体内部点云

            for (auto& point : cloud_in->points)
            {
                Eigen::Vector4d vec_point;
                vec_point[0] = point.x;
                vec_point[1] = point.y;
                vec_point[2] = point.z;
                vec_point[3] = 1;

                if (pyramid_temp.is_point_in(vec_point))
                {
                    cloud_in_temp.push_back(point);
                }
            }

            for (auto& point : cloud_out->points)
            {
                Eigen::Vector4d vec_point;
                vec_point[0] = point.x;
                vec_point[1] = point.y;
                vec_point[2] = point.z;
                vec_point[3] = 1;

                if (pyramid_temp.is_point_in(vec_point))
                {
                    cloud_in_temp.push_back(point);
                }
            }

            if (viewer) //若函数传入PCL绘制窗口，绘制临时视场锥体
            {
                pyramid_temp.remove_pyramid(viewer);
                pyramid_temp.draw_pyramid(viewer);
                viewer->spinOnce(1);
            }
            
            get_occlusion_val(pyramid_temp, cloud_in_temp, pwoi_top);   //计算临时视场锥体遮挡值
            if (pwoi_top.oa_val < oa_threshold / 2) //若遮挡值小于阈值，求解成功，返回解得相机位置
            {
                return pwoi_top.position;
            }

            for (int i = 0; i < 4; i++) //将队首元素的相邻元素入队
            {
                pos_with_oa_info pwoi_temp;
                pwoi_temp.position = { pwoi_top.position.first + direction[i],pwoi_top.position.second + direction[i + 1] };
                pwoi_temp.oa_val = pwoi_top.oa_val;
                pwoi_temp.pos_val = direction[i] * pwoi_top.oa_vec.x() + direction[i + 1] * pwoi_top.oa_vec.y();

                int visited_val = (int)(pwoi_temp.position.first * 20000 + pwoi_temp.position.second * 100);

                //判断该位置是否已被探索过
                //判断该位置是否在合理范围内（机器人坐标系下2m）
                //判断该位置是否与初始点足够近（机器人坐标系下0.5m）
                if (!visited.count(visited_val) && abs(pwoi_top.position.first)<2 && abs(pwoi_top.position.second) < 2 &&
                    abs(pwoi_top.position.first - pyramid.center[0]) < 0.4 && abs(pwoi_top.position.second - pyramid.center[1]) < 0.4)
                {
                    q.push(pwoi_temp);  //入队
                    visited.insert(visited_val);    //哈希表插入
                }
            }

        }
    }
    return { pyramid.center[0],pyramid.center[1] }; //未求得解，返回初始点
}
#pragma endregion


#pragma region 遮挡规避功能函数
//遮挡规避功能函数 
void occlusion_avoidance()
{
    //点云可视化窗口
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("pt Viewer"));
   
    rs2::colorizer color_map;   //realsense深度图渲染色彩

    rs2::pointcloud pc; //realsense点云

    //设置pcl窗口参数
    viewer2->addCoordinateSystem(0.1);  //绘制坐标系
    viewer2->setCameraPosition(0, -1, 1, 0, 1, -1, 0, 0, 1);    //设置观测点，观测方向
    viewer2->setBackgroundColor(0, 0, 0);   //设置背景色

    std::string cloud_RL_filepath;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_RL(new pcl::PointCloud<pcl::PointXYZ>);     //点云文件保存，强化学习使用
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);                   //在相机视场内点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_in(new pcl::PointCloud<pcl::PointXYZ>);          //在相机视场内点云——滤波后
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);                  //在相机视场外点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_out(new pcl::PointCloud<pcl::PointXYZ>);         //在相机视场外点云——滤波后

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> yellow(cloud_filtered_in, 0, 255, 0); // cloud_in配套颜色，黄色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(cloud_filtered_out, 255, 255, 255); //  cloud_out配套颜色，白色

    get_T(T_3d_cam_left, T_3d_cam_right, T_cam_main);    //获取3D camera变换矩阵，相机变换矩阵
    Pyramid rt_pyramid(zoom_ratio, -(lc_now + lc_zero) * 0.001 + oa_height, "rt_pyramid", Eigen::Vector3d(255, 0, 0), T_cam_main); //创建实时相机视场锥体

    string RL_experiment_path = "logs/RL_experiment.txt";
    ofstream fout(RL_experiment_path, ios::out | ios::app);
    int RL_experiment_save = 0;

    Eigen::Isometry3d T_cam_main_confront = Eigen::Isometry3d::Identity();    //主相机变换矩阵
    int init_pos_count = 0;
    double unreach_num = 0.0;
    clock_t init_pos_time = clock();
    clock_t RL_start_ov = clock();

    while (true)
    {
        if (FOCUS_STATE)    //若处于目标锁定状态
        {
            d_c2t = -(lc_now + lc_zero) * 0.001 + oa_height;   //实时更新目标拍摄平面高度

            get_T(T_3d_cam_left, T_3d_cam_right, T_cam_main);    //获取实时3D camera变换矩阵，相机变换矩阵

            //清空点云数据
            cloud_in->clear();
            cloud_out->clear();
            cloud_RL->clear();

            rt_pyramid.set_ratio_height(zoom_ratio, d_c2t, T_cam_main); //更新实时视场锥体参数

            // 0号相机，末端左侧
            RSs[rs_end_left]->waitForData();
            auto depth_left = RSs[rs_end_left]->getDepthImage();
            auto depth_left_color = color_map.colorize(depth_left);
            //cv::imshow("cam_0_末端左侧", cv::Mat(cv::Size(depth_left_color.get_width(), depth_left_color.get_height()), CV_8UC3, (void*)depth_left_color.get_data()));

            // 2号相机，末端右侧
            RSs[rs_end_right]->waitForData();
            auto depth_right = RSs[rs_end_right]->getDepthImage();
            auto depth_right_color = color_map.colorize(depth_right);
            //cv::imshow("cam_2_末端右侧", cv::Mat(cv::Size(depth_right_color.get_width(), depth_right_color.get_height()), CV_8UC3, (void*)depth_right_color.get_data()));

            //将realsense深度图转化为realsense点云
            auto points_left = pc.calculate(depth_left);
            auto points_right = pc.calculate(depth_right);

            //将realsense点云转化为pcl点云
            PCL_Conversion(cloud_in, cloud_out, points_right, T_3d_cam_right, rt_pyramid);
            PCL_Conversion(cloud_in, cloud_out, points_left, T_3d_cam_left, rt_pyramid);

            //强化学习点云录入
            /*PCL_RLSave(cloud_RL, points_right, T_3d_cam_right);
            PCL_RLSave(cloud_RL, points_left, T_3d_cam_left);
            cloud_RL->height = 1;
            cloud_RL->width = cloud_RL->size();
            cloud_RL_filepath = "C:\\Users\\Butel\\Desktop\\SSBOT2\\pointcloud\\" + std::to_string(data_num) + "_" + std::to_string(pcd_num) + ".pcd";
            pcl::io::savePCDFileASCII(cloud_RL_filepath, *cloud_RL);
            pcd_num++;*/

            //std::cout << "滤波前点云数量: " << cloud_in->size()+ cloud_out->size() << std::endl;

            //体素滤波
            pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
            approximate_voxel_filter.setLeafSize(0.05, 0.05, 0.05);
            approximate_voxel_filter.setInputCloud(cloud_in);
            approximate_voxel_filter.filter(*cloud_filtered_in);

            approximate_voxel_filter.setInputCloud(cloud_out);
            approximate_voxel_filter.filter(*cloud_filtered_out);

            //直通滤波
            pcl::PassThrough<pcl::PointXYZ> Cloud_Filter;
            //x方向
            Cloud_Filter.setInputCloud(cloud_filtered_in);
            Cloud_Filter.setFilterFieldName("x");
            Cloud_Filter.setFilterLimits(-2.0, 2.0);
            Cloud_Filter.filter(*cloud_filtered_in);
            Cloud_Filter.setInputCloud(cloud_filtered_out);
            Cloud_Filter.setFilterFieldName("x");
            Cloud_Filter.setFilterLimits(-2.0, 2.0);
            Cloud_Filter.filter(*cloud_filtered_out);


            //y方向
            Cloud_Filter.setInputCloud(cloud_filtered_in);
            Cloud_Filter.setFilterFieldName("y");
            Cloud_Filter.setFilterLimits(-2.0, 2.0);
            Cloud_Filter.filter(*cloud_filtered_in);
            Cloud_Filter.setInputCloud(cloud_filtered_out);
            Cloud_Filter.setFilterFieldName("y");
            Cloud_Filter.setFilterLimits(-2.0, 2.0);
            Cloud_Filter.filter(*cloud_filtered_out);

            /*std::cout << "滤波后点云数量: " << cloud_filtered_in->size() + cloud_filtered_out->size() << std::endl;
            if (!cloud_filtered_in->empty())
            {
                std::cout << "第一个点的 X 坐标: " << cloud_filtered_in->points[0].x << std::endl;
                std::cout << "第一个点的 Y 坐标: " << cloud_filtered_in->points[0].y << std::endl;
                std::cout << "第一个点的 Z 坐标: " << cloud_filtered_in->points[0].z << std::endl;
            }*/

            ////可视化
            viewer2->removeAllPointClouds();

            viewer2->addPointCloud<pcl::PointXYZ>(cloud_filtered_in, yellow, "cloud_filtered_in");
            viewer2->addPointCloud<pcl::PointXYZ>(cloud_filtered_out, white, "cloud_filtered_out");

            viewer2->removeAllShapes();

            rt_pyramid.draw_pyramid(viewer2);

            viewer2->spinOnce(1);

            //无遮挡回原位置
            //if ((clock() - init_pos_time) / CLOCKS_PER_SEC  > 8 && ARM_STATE == ARM_FREE)
            //{
            //    if (judge_init_pos(rt_pyramid, cloud_filtered_in, cloud_filtered_out) && abs(focus_point(0) - rt_pyramid.center[0])>1e-2 && abs(focus_point(1) - rt_pyramid.center[1]) > 1e-2)
            //    {
            //        Eigen::Vector2d oa_direction;   //相机运动方向
            //        oa_direction[0] = focus_point(0) - rt_pyramid.center[0];
            //        oa_direction[1] = focus_point(1) - rt_pyramid.center[1];
            //        oa_direction /= oa_direction.norm();    //归一化

            //        Eigen::Vector2d occlusion_target;   //相机运动目标位置
            //        occlusion_target[0] = focus_point(0);
            //        occlusion_target[1] = focus_point(1);

            //        if (get_traj_kdl(joint_now, traj, oa_direction, true, focus_point, occlusion_target))
            //        {
            //            ARM_STATE = ARM_GET_TRAJ_SUCCESS;   //若求解笛卡尔直线路径成功，将机械臂状态置为ARM_GET_TRAJ_SUCCESS
            //        }

            //        init_pos_count = 0;
            //        continue;
            //    }
            //    init_pos_time = clock();
            //}

            //获得遮挡值
            oa_val = get_occlusion_val(rt_pyramid, cloud_filtered_in, oa_bg);
            unreach_num = 0.0;

            if (oa_val < oa_threshold && ARM_STATE == ARM_FOLLOW_TRAJ && MOVE_TYPE == MOVE_TYPE_FOCUS)
            /*if (oa_val < oa_threshold && RL_experiment_save == 1)*/
            {
                //若遮挡值小于阈值，且机械臂在遮挡规避运动过程中，机械臂运动停止，针对遮挡消失情况
                arm_stop();
                if (RL_experiment_save == 1)
                {
                    fout << "遮挡值:" << oa_val << endl;
                    fout << "相机位置:" << rt_pyramid.center[0] << rt_pyramid.center[1] << endl;
                    fout << "遮挡规避时间" << (clock() - RL_start_ov) / (double)CLOCKS_PER_SEC * 1000 << endl;
                    fout << "遮挡规避结束\n" << endl;
                    RL_experiment_save = 0;
                }
                //udp_send_debug_info(DEBUG_RL_OCCLUSION_AVOIDANCE_END);
            }
            else if (oa_val >= oa_threshold && RL_occlusion_avoid_flag == 1 && ARM_STATE == ARM_FREE)
            {
                /*udp_send_debug_info(DEBUG_RL_OCCLUSION_AVOIDANCE_START);
                std::this_thread::sleep_for(std::chrono::milliseconds(600)); */

                cout << "oa_val = " << oa_val << endl;
                if (RL_experiment_save == 0)
                {
                    fout << "遮挡规避开始" << endl;
                    RL_start_ov = clock();
                    RL_experiment_save = 1;
                }
                fout << "遮挡值:" << oa_val << endl;

                pos_with_oa_info pwoi_ori;  //创建带有遮挡值的相机位置
                pwoi_ori.position = { rt_pyramid.center[0],rt_pyramid.center[1] };  //确定相机初始位置
                get_occlusion_info_RL(rt_pyramid, *cloud_filtered_in, *cloud_filtered_out, pwoi_ori);

                fout << "相机位置:" << pwoi_ori.position << endl;

                Eigen::Vector2d delta_pos = Eigen::Vector2d(0, 0);
                //无点云强化学习处理输出
                delta_pos = get_delta_pos_RL(pwoi_ori, unreach_num);

                //点云输入强化学习处理
                /*vector<float> point_input;
                point_input = ProcessSingleCloud(cloud_filtered_in, cloud_filtered_out);
                delta_pos = get_delta_pos_RL_POINT(pwoi_ori, unreach_num, point_input);*/

                double step = 0.08;
                if (abs(delta_pos.norm()) < 1e-5)
                {
                    cout << "强化学习模型输出错误" << endl;
                }
                else
                {
                    delta_pos = delta_pos / delta_pos.norm() * step;

                    cout << "before:[" << rt_pyramid.center[0] << "," << rt_pyramid.center[1]
                        << "] after:[" << rt_pyramid.center[0] + delta_pos[0] << "," << rt_pyramid.center[1] + delta_pos[1] << "]" << endl;

                    MOVE_TYPE = MOVE_TYPE_FOCUS;    //将机械臂运动类型置为目标锁定

                    Eigen::Vector2d oa_direction;   //相机运动方向
                    oa_direction[0] = delta_pos[0];
                    oa_direction[1] = delta_pos[1];
                    oa_direction /= oa_direction.norm();    //归一化

                    Eigen::Vector2d occlusion_target;   //相机运动目标位置
                    occlusion_target[0] = pwoi_ori.position.first + delta_pos[0];
                    occlusion_target[1] = pwoi_ori.position.second + delta_pos[1];

                    if (get_traj_kdl(joint_now, traj, oa_direction, true, focus_point, occlusion_target))
                    {
                        unreach_num = 0.0;
                        ARM_STATE = ARM_GET_TRAJ_SUCCESS;   //若求解笛卡尔直线路径成功，将机械臂状态置为ARM_GET_TRAJ_SUCCESS
                    }
                    else
                        unreach_num += 1.0;
                }
            }
            else if (oa_val >= oa_threshold && ARM_STATE == ARM_FREE)   //若遮挡值大于阈值，且机械臂在空闲状态
            {
                cout << "oa_val = " << oa_val << endl;
                
                pos_with_oa_info pwoi_ori;  //创建带有遮挡值的相机位置
                pwoi_ori.position = { rt_pyramid.center[0],rt_pyramid.center[1] };  //确定相机初始位置
                pwoi_ori.oa_val = oa_val;   //设置遮挡值

                //auto res = get_occlusion_target(rt_pyramid, pwoi_ori, cloud_in, cloud_out, 0.05, joint_now, viewer2);   //获取无遮挡位置
                auto res = get_MIDACO_target(rt_pyramid, pwoi_ori, cloud_in, cloud_out, 0.05, joint_now, viewer2);   //获取无遮挡位置
                //auto res = get_occlusion_target(rt_pyramid, pwoi_ori, cloud_in, cloud_out, 0.05, joint_now);   //获取无遮挡位置

                if (pwoi_ori.position != res)   //判断解得位置是否与初始位置相同（若相同，则为未求得解）
                {
                    cout << "before:[" << rt_pyramid.center[0] << "," << rt_pyramid.center[1]
                        << "] after:[" << res.first << "," << res.second << "]" << endl;

                    MOVE_TYPE = MOVE_TYPE_FOCUS;    //将机械臂运动类型置为目标锁定

                    Eigen::Vector2d oa_direction;   //相机运动方向
                    oa_direction[0] = res.first - pwoi_ori.position.first;
                    oa_direction[1] = res.second - pwoi_ori.position.second;
                    oa_direction /= oa_direction.norm();    //归一化

                    Eigen::Vector2d occlusion_target;   //相机运动目标位置
                    occlusion_target[0] = res.first;
                    occlusion_target[1] = res.second;

                    if (get_traj_kdl(joint_now, traj, oa_direction, true, focus_point, occlusion_target))
                    {
                        ARM_STATE = ARM_GET_TRAJ_SUCCESS;   //若求解笛卡尔直线路径成功，将机械臂状态置为ARM_GET_TRAJ_SUCCESS
                    }
                }
                else
                {
                    cout << "遮挡规避求解失败" << endl;
                }
            }
        }
    }
}
#pragma endregion

#pragma region MIDACO寻优算法
std::pair<double, double> get_MIDACO_target(Pyramid& pyramid, pos_with_oa_info& pwoi_ori, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double step, std::vector<double> joint_now,
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    Eigen::Vector3d point;

    point[0] = pwoi_ori.position.first;
    point[1] = pwoi_ori.position.second;
    point[2] = pyramid.center[2];

    //MIDACO优化算法相关参数
    long int o, n, ni, m, me, maxeval, maxtime, printeval, save2file, iflag, istop;
    long int liw, lrw, lpf, i, iw[5000], p = 1; double rw[20000], pf[20000];
    double   f[10], g[1000], x[1000], xl[1000], xu[1000], param[13];
    char key[] = "MIDACO_LIMITED_VERSION___[CREATIVE_COMMONS_BY-NC-ND_LICENSE]";

    /* STEP 1.A: Problem dimensions
    ******************************/
    o = 1; //目标值个数
    n = 2; //设计变量个数
    ni = 0; //整数优化设计变量个数
    m = 5; //约束总数
    me = 1; //等式约束个数

    /* STEP 1.B: Lower and upper bounds 'xl' & 'xu'
    **********************************************/
    for (i = 0; i < n; i++)
    {
        xl[i] = -2.0;
        xu[i] = 2.0;
    }

    /* STEP 1.C: Starting point 'x'
    ******************************/
    for (i = 0; i < n; i++)
    {
        x[i] = point[i]; /* Here for example: starting point = lower bounds */
    }

    /* STEP 2.A: Stopping criteria
    *****************************/
    maxeval = 10000;    //最大迭代次数
    maxtime = 5; //最长计算时间 单位：秒

    /* STEP 2.B: Printing options
    ****************************/
    printeval = 100; /* Print-Frequency for current best solution (e.g. 1000) */
    save2file = 1;    /* Save SCREEN and SOLUTION to TXT-files [ 0=NO/ 1=YES]  */

    //MIDACO优化算法相关参数
    param[0] = 0.0;  /* ACCURACY  */  //等式约束精度
    param[1] = 10.0;  /* SEED      */  //种子数
    param[2] = 0.0;  /* FSTOP     */  //最小停止参数
    param[3] = 0.0;  /* ALGOSTOP  */
    param[4] = 0.0;  /* EVALSTOP  */
    param[5] = 0.0;  /* FOCUS     */
    param[6] = 30;  /* ANTS      */  //蚂蚁数量
    param[7] = 5;  /* KERNEL    */   //内核数
    param[8] = 0.0;  /* ORACLE    */
    param[9] = 400;  /* PARETOMAX */  //非支配解数量
    param[10] = 0.1;  /* EPSILON   */
    param[11] = 0.0;  /* BALANCE   */
    param[12] = 0.0;  /* CHARACTER */

    //工作空间长度
    lrw = sizeof(rw) / sizeof(double);
    lpf = sizeof(pf) / sizeof(double);
    liw = sizeof(iw) / sizeof(long int);
    //打印MIDACO算法标题和基本信息
    midaco_print(1, printeval, save2file, &iflag, &istop, &*f, &*g, &*x, &*xl, &*xu,
        o, n, ni, m, me, &*rw, &*pf, maxeval, maxtime, &*param, p, &*key);

    pos_with_oa_info pwoi_find;

    cv::namedWindow("robot occlusion avoidence", 0);
    cv::resizeWindow("robot occlusion avoidence", 180, 320);

    while (istop == 0)  //队列非空，则继续迭代
    {
        if (!FOCUS_STATE)   //若目标锁定停止，退出
        {
            return { x[0],x[1] };
        }

        /* Evaluate objective function */
        problem_function(&*f, &*g, &*x, pyramid, viewer, cloud_in, cloud_out);

        /* Call MIDACO */
        midaco(&p, &o, &n, &ni, &m, &me, &*x, &*f, &*g, &*xl, &*xu, &iflag,
            &istop, &*param, &*rw, &lrw, &*iw, &liw, &*pf, &lpf, &*key);
        /* Call MIDACO printing routine */
        midaco_print(2, printeval, save2file, &iflag, &istop, &*f, &*g, &*x, &*xl, &*xu,
            o, n, ni, m, me, &*rw, &*pf, maxeval, maxtime, &*param, p, &*key);
    }

    std::vector<double> theta_solv(6);
    std::pair<double, double> pi = { x[0], x[1] };
    pwoi_find.position = pi;
    pwoi_find.oa_val = 1.0;
    pwoi_find.pos_val = 1.0;
    solve_focus_kdl(joint_now, pwoi_find.position, focus_point, theta_solv);
    Eigen::Isometry3d T_cam_main;
    get_T(T_cam_main, theta_solv);  //获得theta_solv情况下的相机变换矩阵
    Eigen::Vector3d color(0, 255, 0);   //设置探索视场锥体颜色为绿色
    Pyramid pyramid_temp(pyramid.ratio, pyramid.height, "pyramid_temp", color, T_cam_main); //创建临时视场锥体

    if (viewer) //若函数传入PCL绘制窗口，绘制临时视场锥体
    {
        pyramid_temp.remove_pyramid(viewer);
        pyramid_temp.draw_pyramid(viewer);
        viewer->spinOnce(1);
    }
    return { x[0],x[1] };
}
#pragma endregion

#pragma region MIDACO寻优目标函数
void problem_function(double* f, double* g, double* x, Pyramid& pyramid,
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
    Eigen::Vector3d point;
    point[0] = x[0];
    point[1] = x[1];
    point[2] = pyramid.center[2];

    pos_with_oa_info pwoi_find;
    Eigen::Vector3d color(0, 255, 0);   //设置探索视场锥体颜色为绿色
    std::pair<double, double> pi = { x[0], x[1] };
    pwoi_find.position = pi;
    pwoi_find.oa_val = 1.0;
    pwoi_find.pos_val = 1.0;
    Eigen::Isometry3d T_cam_main;
    std::vector<double> theta_solv(6);
    pcl::PointCloud<pcl::PointXYZ> cloud_in_temp;
    int flag = 1;

    if (solve_focus_kdl(joint_now, pwoi_find.position, focus_point, theta_solv) && !ACM.collision(theta_solv[0] / PI * 180, theta_solv[1] / PI * 180, theta_solv[2] / PI * 180, obstacles))
        flag = 0;
    get_T(T_cam_main, theta_solv);  //获得theta_solv情况下的相机变换矩阵
    Pyramid pyramid_temp(pyramid.ratio, pyramid.height, "pyramid_temp", color, T_cam_main); //创建临时视场锥体

    for (auto& point : cloud_in->points)
    {
        Eigen::Vector4d vec_point;
        vec_point[0] = point.x;
        vec_point[1] = point.y;
        vec_point[2] = point.z;
        vec_point[3] = 1;

        if (pyramid_temp.is_point_in(vec_point))
        {
            cloud_in_temp.push_back(point);
        }
    }

    for (auto& point : cloud_out->points)
    {
        Eigen::Vector4d vec_point;
        vec_point[0] = point.x;
        vec_point[1] = point.y;
        vec_point[2] = point.z;
        vec_point[3] = 1;

        if (pyramid_temp.is_point_in(vec_point))
        {
            cloud_in_temp.push_back(point);
        }
    }

    get_occlusion_val(pyramid_temp, cloud_in_temp, pwoi_find);
    f[0] = pwoi_find.oa_val;

    g[0] = flag;
    g[1] = -(abs(theta_solv[4]) / PI * 180 - 40);
    g[2] = -(abs(theta_solv[3]) / PI * 180 - 40);
    g[3] = -(abs(x[0] - pyramid.center[0]) - 0.5);
    g[4] = -(abs(x[1] - pyramid.center[1]) - 0.5);

}
#pragma endregion
