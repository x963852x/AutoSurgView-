/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of occlusion avoidance

**************************************************************************/

#pragma once
#include <unordered_set>
#include <queue>
#include <math.h>
#include <string>

#include "librealsense2/rs.hpp"
#include "glut.h"
#include "pcl/point_types.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/approximate_voxel_grid.h"
#include "pcl/octree/octree.h"
#include "opencv2/opencv.hpp"
#include "boost/thread/thread.hpp"
#include "rs_camera_work.h"
#include "motor_work.h"
#include "camera.h"
#include "LiftingColumn.h"
#include "motor_work.h"

#include <onnxruntime_cxx_api.h>

#include "ssbot_config.h"

//带遮挡值的相机位置点
struct pos_with_oa_info
{
    std::pair<double, double> position = { 0,0 }; //相机位置
    double oa_val = 0;  //遮挡值
    double pos_val = 0; //较父节点位移矢量 与 父节点遮挡矢量 点乘模值
    Eigen::Vector2d oa_vec = Eigen::Vector2d(0, 0);   //遮挡矢量
    
    //强化学习相关
    Eigen::Vector2d oa_RL_vec = Eigen::Vector2d(0, 0);   //遮挡矢量（新计算公式）
    Eigen::Vector2d global_oa_vec = Eigen::Vector2d(0, 0);   //全局遮挡矢量
    Eigen::Vector2d oa_centroid_vec = Eigen::Vector2d(0, 0);   //点云质心矢量
};

#pragma region 视场锥体类
class Pyramid
{
public:
    Pyramid();
    Pyramid(double Ratio, double Height,std::string Name, Eigen::Vector3d Color, Eigen::Isometry3d T);
    ~Pyramid();

    void set_ratio_height(double Ratio, double Height, Eigen::Isometry3d T);
    void draw_pyramid(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    void remove_pyramid(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    bool is_point_in(Eigen::Vector4d);

    void cal_vec(Eigen::Isometry3d T);  //计算光轴矢量、图像

    Eigen::Vector4d center; //相机光心
    Eigen::Vector4d center_img; //图像中心，光心与图像中心连线为光轴，图像与光轴垂直，光轴长度1m

    Eigen::Vector4d point_o;  //原点矢量——单位矢量
    Eigen::Vector4d vec_n;  //光轴矢量——单位矢量
    Eigen::Vector4d vec_x;  //图像X方向矢量——单位矢量
    Eigen::Vector4d vec_y;  //图像Y方向矢量——单位矢量

    double height = -1;  //机器人坐标系下目标拍摄高度
    double ratio = 1;   //相机缩放比例

    Eigen::Isometry3d T_cam;    //相机坐标系至机器人坐标系变换矩阵
    std::string name = "Pyramid";   //锥体名称
    Eigen::Vector3d color;  //锥体颜色
    std::vector<Eigen::Vector4d> points;    //相机拍摄角点

    std::vector<Eigen::Vector4d> axis_points;    //图像坐标系;
};
#pragma endregion

#pragma region 辅助函数
void PCL_RLSave(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_RL, const rs2::points& points, Eigen::Isometry3d& T_3d_cam);  //强化学习点云采集
void PCL_Conversion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, const rs2::points& points, Eigen::Isometry3d& T_3d_cam, Pyramid& pyramid);  //将3D camera点云转化为pcl点云
void get_T(Eigen::Isometry3d& T_3d_cam_left, Eigen::Isometry3d& T_3d_cam_right, Eigen::Isometry3d& T_cam_main); //根据实际关节角度更新矩阵
void get_T(Eigen::Isometry3d& T_cam_main, std::vector<double> theta);   //根据给定关节角度更新矩阵
double get_occlusion_val(Pyramid& pyramid, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, cv::Mat& src);  //获得点云遮挡值，应用于实时遮挡检测
void get_occlusion_val(Pyramid& pyramid, pcl::PointCloud<pcl::PointXYZ>& cloud_in, pos_with_oa_info& pwoi);   //获得点云遮挡值（无绘制功能），应用于遮挡规避搜索
void set_focus_point(); //设置目标拍摄点

void get_occlusion_info_RL(Pyramid& pyramid, pcl::PointCloud<pcl::PointXYZ>& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>& cloud_out, pos_with_oa_info& pwoi); //求解遮挡矢量和全局遮挡矢量
Eigen::Vector2d get_delta_pos_RL(pos_with_oa_info& pwoi, double unreach_num);
std::vector<float> ProcessSingleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2);
Eigen::Vector2d get_delta_pos_RL_POINT(pos_with_oa_info& pwoi, double unreach_num, std::vector<float> point_cloud_data);
bool judge_init_pos(Pyramid& pyramid, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);  //判断正视位置是否存在遮挡
#pragma endregion

#pragma region 遮挡规避功能函数
void occlusion_avoidance(); //遮挡规避功能函数 
#pragma endregion

#pragma region MIDACO优化算法相关
std::pair<double, double> get_MIDACO_target(Pyramid& pyramid, pos_with_oa_info& pwoi_ori, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double step, std::vector<double> joint_now,
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
void problem_function(double* F, double* G, double* X, Pyramid& pyramid,
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
#pragma endregion