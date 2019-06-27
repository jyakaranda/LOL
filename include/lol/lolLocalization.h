/**
 * @file lolLocalization.h
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-05-04
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef __LOL_LOCALIZATION__
#define __LOL_LOCALIZATION__

#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <deque>
#include <string>
#include <thread>
#include <vector>
#include <array>
#include <chrono>

#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/angles.h>

#include <ceres/ceres.h>

#include "alego/utility.h"

namespace localization
{

using namespace ceres;

class LolLocalization
{
public:
  LolLocalization(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~LolLocalization() {}
  LolLocalization(const LolLocalization &l) = delete;

  void optimizeThread();

  void run();

private:
  // 点云地图中 intensity 为 keypose 的 id
  using PointType = pcl::PointXYZI;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_odom_;
  ros::Subscriber sub_corner_;
  ros::Subscriber sub_surf_;
  ros::Subscriber sub_outlier_;
  ros::Subscriber sub_initial_pose_;

  ros::Publisher pub_lol_pose_;
  ros::Publisher pub_corner_target_;
  ros::Publisher pub_surf_target_;
  ros::Publisher pub_corner_source_;
  ros::Publisher pub_surf_source_;
  ros::Publisher pub_test_;
  tf::TransformBroadcaster tf_broadcaster_;

  double tobe_optimized_[6];

  pcl::PointCloud<PointType>::Ptr laser_corner_;
  pcl::PointCloud<PointType>::Ptr laser_surf_;
  pcl::PointCloud<PointType>::Ptr laser_outlier_;
  pcl::PointCloud<PointType>::Ptr laser_corner_ds_;
  pcl::PointCloud<PointType>::Ptr laser_surf_ds_;
  pcl::PointCloud<PointType>::Ptr laser_outlier_ds_;

  double time_laser_corner_, time_laser_surf_, time_laser_outlier_, time_laser_odom_;
  bool new_laser_corner_, new_laser_surf_, new_laser_outlier_, new_laser_odom_;

  // 一个 keypose 对应三种点云，点云在 vector 中的 index 由 keypose 在 keyposes_3d_ 中的 index 决定，点云已根据 keypose 转换到 global 坐标系中
  std::vector<pcl::PointCloud<PointType>::Ptr> corner_keyframes_;
  std::vector<pcl::PointCloud<PointType>::Ptr> surf_keyframes_;
  std::vector<pcl::PointCloud<PointType>::Ptr> outlier_keyframes_;
  pcl::PointCloud<PointType>::Ptr keyposes_3d_;

  // 局部 target corner, surf 点云，作为局部地图进行配准
  pcl::PointCloud<PointType>::Ptr pc_corner_target_;
  pcl::PointCloud<PointType>::Ptr pc_surf_target_;
  pcl::PointCloud<PointType>::Ptr pc_corner_target_ds_;
  pcl::PointCloud<PointType>::Ptr pc_surf_target_ds_;
  PointType target_center_;
  std::vector<int> surround_keyposes_id_;
  std::vector<pcl::PointCloud<PointType>::Ptr> surround_corner_keyframes_;
  std::vector<pcl::PointCloud<PointType>::Ptr> surround_surf_keyframes_;
  std::vector<pcl::PointCloud<PointType>::Ptr> surround_outlier_keyframes_;
  int batch_cnt_;
  PointCloudT::Ptr local_corner_;
  PointCloudT::Ptr local_surf_;
  PointCloudT::Ptr local_outlier_;

  Eigen::Matrix4d tf_o2l_0_;
  Eigen::Matrix4d tf_m2l_0_;

  std::vector<int> point_search_idx_;
  std::vector<float> point_search_dist_;

  // 快速查找 cur_pose 附近的 keypose，进而提取附近点云作为 target map
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_keyposes_3d_;
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_corner_target_;
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf_target_;
  // params
  double surround_search_radius_;
  int surround_search_num_;

  // voxel filter
  pcl::VoxelGrid<PointType> ds_corner_;
  pcl::VoxelGrid<PointType> ds_surf_;
  pcl::VoxelGrid<PointType> ds_outlier_;
  pcl::VoxelGrid<PointType> ds_surround_keyposes_;

  // params
  double corner_leaf_, surf_leaf_, outlier_leaf_, surround_keyposes_leaf_;
  std::string fn_poses_, fn_corner_, fn_surf_, fn_outlier_;
  double target_update_dist_; // 与 target_center_ 偏移 target_update_dist_ 以上时更新局部 target map
  int batch_size_;

  double huber_s_;
  int max_iters_;
  double func_tolerance_;
  double gradient_tolerance_;
  double param_tolerance_;

  geometry_msgs::PoseStamped cur_laser_pose_, pre_laser_pose_;
  std::deque<geometry_msgs::PoseStamped> history_poses_;
  Eigen::Matrix4d tf_b2l_;
  Eigen::Matrix4d tf_m2o_;
  Eigen::Matrix4d tf_o2b_;
  Eigen::Matrix4d tf_m2o_update_;

  std::mutex mtx_;

  void odomCB(const nav_msgs::OdometryConstPtr &msg);
  void cornerCB(const sensor_msgs::PointCloud2ConstPtr &msg);
  void surfCB(const sensor_msgs::PointCloud2ConstPtr &msg);
  void outlierCB(const sensor_msgs::PointCloud2ConstPtr &msg);
  void initialPoseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

  /**
   * @brief 初始化工作，载入参数
   * 
   * @return true 
   * @return false 
   */
  bool init();

  /**
   * @brief 提取 p 附近的 corner, surf key frames 作为局部 target map 用于点云配准
   * 
   * @param p 
   */
  bool extractSurroundKeyFrames(const PointType &p);
};
} // namespace localization

#endif