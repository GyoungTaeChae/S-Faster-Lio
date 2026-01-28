#include "preprocess.h"
#include <Eigen/Core>
#include <csignal>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <math.h>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <unistd.h>
#include <visualization_msgs/msg/marker.hpp>

#include "IMU_Processing.hpp"
#include "voxmap/voxel_map.h"

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define PUBFRAME_PERIOD (20)

using MapType = VoxelMap<PointType>;

/*** Time Log Variables ***/
int add_point_size = 0, kdtree_delete_counter = 0;
bool pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true,
     path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0,
       filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int scan_count = 0, publish_count = 0;
int feats_down_size = 0, NUM_MAX_ITERATIONS = 0, pcd_save_interval = -1,
    pcd_index = 0;

bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<PointVector> Nearest_Points;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
deque<double> time_buffer;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

MapType::Options ivox_options_;
std::unique_ptr<MapType> ivox;

V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;

esekfom::esekf kf;

state_ikfom state_point;
Eigen::Vector3d pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());

// Global node pointer for logging and time
rclcpp::Node::SharedPtr g_node = nullptr;

void SigHandle(int sig) {
  flg_exit = true;
  RCLCPP_WARN(g_node->get_logger(), "catch sig %d", sig);
  sig_buffer.notify_all();
  rclcpp::shutdown();
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  mtx_buffer.lock();
  scan_count++;
  double preprocess_start_time = omp_get_wtime();
  double timestamp = rclcpp::Time(msg->header.stamp).seconds();
  if (timestamp < last_timestamp_lidar) {
    RCLCPP_ERROR(g_node->get_logger(), "lidar loop back, clear buffer");
    lidar_buffer.clear();
  }

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(timestamp);
  last_timestamp_lidar = timestamp;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;

void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg_in) {
  publish_count++;
  auto msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);

  if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
    msg->header.stamp = rclcpp::Time(
        static_cast<int64_t>((timediff_lidar_wrt_imu +
                              rclcpp::Time(msg_in->header.stamp).seconds()) *
                             1e9));
  }

  msg->header.stamp = rclcpp::Time(static_cast<int64_t>(
      (rclcpp::Time(msg_in->header.stamp).seconds() - time_diff_lidar_to_imu) *
      1e9));

  double timestamp = rclcpp::Time(msg->header.stamp).seconds();

  mtx_buffer.lock();

  if (timestamp < last_timestamp_imu) {
    RCLCPP_WARN(g_node->get_logger(), "imu loop back, clear buffer");
    imu_buffer.clear();
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int scan_num = 0;

bool sync_packages(MeasureGroup &meas) {
  if (lidar_buffer.empty() || imu_buffer.empty()) {
    return false;
  }

  /*** push a lidar scan ***/
  if (!lidar_pushed) {
    meas.lidar = lidar_buffer.front();
    meas.lidar_beg_time = time_buffer.front();
    if (meas.lidar->points.size() <= 5) {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
      RCLCPP_WARN(g_node->get_logger(), "Too few input point cloud!");
    } else if (meas.lidar->points.back().curvature / double(1000) <
               0.5 * lidar_mean_scantime) {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
    } else {
      scan_num++;
      lidar_end_time = meas.lidar_beg_time +
                       meas.lidar->points.back().curvature / double(1000);
      lidar_mean_scantime +=
          (meas.lidar->points.back().curvature / double(1000) -
           lidar_mean_scantime) /
          scan_num;
    }

    meas.lidar_end_time = lidar_end_time;

    lidar_pushed = true;
  }

  if (last_timestamp_imu < lidar_end_time) {
    return false;
  }

  /*** push imu data, and pop from imu buffer ***/
  double imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
  meas.imu.clear();
  while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
    imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
    if (imu_time > lidar_end_time)
      break;
    meas.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
  }

  lidar_buffer.pop_front();
  time_buffer.pop_front();
  lidar_pushed = false;
  return true;
}

void pointBodyToWorld(PointType const *const pi, PointType *const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(state_point.rot.matrix() *
                   (state_point.offset_R_L_I.matrix() * p_body +
                    state_point.offset_T_L_I) +
               state_point.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po) {
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(state_point.rot.matrix() *
                   (state_point.offset_R_L_I.matrix() * p_body +
                    state_point.offset_T_L_I) +
               state_point.pos);

  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(state_point.offset_R_L_I.matrix() * p_body_lidar +
                 state_point.offset_T_L_I);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

void map_incremental() {
  PointVector PointToAdd;
  PointVector PointNoNeedDownsample;
  PointToAdd.reserve(feats_down_size);
  PointNoNeedDownsample.reserve(feats_down_size);
  for (int i = 0; i < feats_down_size; i++) {
    pointBodyToWorld(&(feats_down_body->points[i]),
                     &(feats_down_world->points[i]));

    if (!Nearest_Points[i].empty() && flg_EKF_inited) {
      const PointVector &points_near = Nearest_Points[i];
      bool need_add = true;
      PointType mid_point;
      mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) *
                        filter_size_map_min +
                    0.5 * filter_size_map_min;
      mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) *
                        filter_size_map_min +
                    0.5 * filter_size_map_min;
      mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) *
                        filter_size_map_min +
                    0.5 * filter_size_map_min;
      float dist = calc_dist(feats_down_world->points[i], mid_point);
      if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
          fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
          fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {
        PointNoNeedDownsample.push_back(feats_down_world->points[i]);
        continue;
      }
      for (int j = 0; j < NUM_MATCH_POINTS; j++) {
        if ((int)points_near.size() < NUM_MATCH_POINTS)
          break;
        if (calc_dist(points_near[j], mid_point) < dist) {
          need_add = false;
          break;
        }
      }
      if (need_add)
        PointToAdd.push_back(feats_down_world->points[i]);
    } else {
      PointToAdd.push_back(feats_down_world->points[i]);
    }
  }

  double st_time = omp_get_wtime();
  ivox->AddPoints(PointToAdd);
  ivox->AddPoints(PointNoNeedDownsample);
  add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

void publish_frame_world(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        &pubLaserCloudFull_) {
  if (scan_pub_en) {
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort
                                                       : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
      pointBodyToWorld(&laserCloudFullRes->points[i],
                       &laserCloudWorld->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp =
        rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFull_->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
  }

  /**************** save map ****************/
  if (pcd_save_en) {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
      pointBodyToWorld(&feats_undistort->points[i],
                       &laserCloudWorld->points[i]);
    }

    static int scan_wait_num = 0;
    scan_wait_num++;

    if (scan_wait_num % 4 == 0)
      *pcl_wait_save += *laserCloudWorld;

    if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 &&
        scan_wait_num >= pcd_save_interval) {
      pcd_index++;
      string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") +
                            to_string(pcd_index) + string(".pcd"));
      pcl::PCDWriter pcd_writer;
      cout << "current scan saved to /PCD/" << all_points_dir << endl;
      pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
      pcl_wait_save->clear();
      scan_wait_num = 0;
    }
  }
}

void publish_frame_body(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        &pubLaserCloudFull_body) {
  int size = feats_undistort->points.size();
  PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                           &laserCloudIMUBody->points[i]);
  }

  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
  laserCloudmsg.header.stamp =
      rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
  laserCloudmsg.header.frame_id = "body";
  pubLaserCloudFull_body->publish(laserCloudmsg);
  publish_count -= PUBFRAME_PERIOD;
}

void publish_map(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        &pubLaserCloudMap) {
  sensor_msgs::msg::PointCloud2 laserCloudMap;
  pcl::toROSMsg(*featsFromMap, laserCloudMap);
  laserCloudMap.header.stamp =
      rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
  laserCloudMap.header.frame_id = "camera_init";
  pubLaserCloudMap->publish(laserCloudMap);
}

template <typename T> void set_posestamp(T &out) {
  out.pose.position.x = state_point.pos(0);
  out.pose.position.y = state_point.pos(1);
  out.pose.position.z = state_point.pos(2);

  auto q_ = Eigen::Quaterniond(state_point.rot.matrix());
  out.pose.orientation.x = q_.coeffs()[0];
  out.pose.orientation.y = q_.coeffs()[1];
  out.pose.orientation.z = q_.coeffs()[2];
  out.pose.orientation.w = q_.coeffs()[3];
}

void publish_odometry(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
        &pubOdomAftMapped,
    std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_broadcaster) {
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "body";
  odomAftMapped.header.stamp =
      rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
  set_posestamp(odomAftMapped.pose);
  pubOdomAftMapped->publish(odomAftMapped);

  auto P = kf.get_P();
  for (int i = 0; i < 6; i++) {
    int k = i < 3 ? i + 3 : i - 3;
    odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
    odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
    odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
    odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
    odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
    odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = odomAftMapped.header.stamp;
  t.header.frame_id = "camera_init";
  t.child_frame_id = "body";
  t.transform.translation.x = odomAftMapped.pose.pose.position.x;
  t.transform.translation.y = odomAftMapped.pose.pose.position.y;
  t.transform.translation.z = odomAftMapped.pose.pose.position.z;
  t.transform.rotation = odomAftMapped.pose.pose.orientation;
  tf_broadcaster->sendTransform(t);
}

void publish_path(
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath) {
  set_posestamp(msg_body_pose);
  msg_body_pose.header.stamp =
      rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
  msg_body_pose.header.frame_id = "camera_init";

  /*** if path is too large, the rviz will crash ***/
  static int jjj = 0;
  jjj++;
  if (jjj % 10 == 0) {
    path.poses.push_back(msg_body_pose);
    pubPath->publish(path);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("laserMapping");
  g_node = node;

  auto get_param = [&](const std::string &name, auto &var,
                       const auto &default_val) {
    node->declare_parameter(name, default_val);
    node->get_parameter(name, var);
  };

  get_param("publish.path_en", path_en, true);
  get_param("publish.scan_publish_en", scan_pub_en, true);
  get_param("publish.dense_publish_en", dense_pub_en, true);
  get_param("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
  get_param("max_iteration", NUM_MAX_ITERATIONS, 4);
  get_param("map_file_path", map_file_path, std::string(""));
  get_param("common.lid_topic", lid_topic, std::string("/livox/lidar"));
  get_param("common.imu_topic", imu_topic, std::string("/livox/imu"));
  get_param("common.time_sync_en", time_sync_en, false);
  get_param("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
  get_param("filter_size_corner", filter_size_corner_min, 0.5);
  get_param("filter_size_surf", filter_size_surf_min, 0.5);
  get_param("filter_size_map", filter_size_map_min, 0.5);
  get_param("cube_side_length", cube_len, 200.0);
  get_param("mapping.det_range", DET_RANGE, 300.f);
  get_param("mapping.fov_degree", fov_deg, 180.0);
  get_param("mapping.gyr_cov", gyr_cov, 0.1);
  get_param("mapping.acc_cov", acc_cov, 0.1);
  get_param("mapping.b_gyr_cov", b_gyr_cov, 0.0001);
  get_param("mapping.b_acc_cov", b_acc_cov, 0.0001);
  get_param("preprocess.blind", p_pre->blind, 0.01);
  get_param("preprocess.lidar_type", p_pre->lidar_type, (int)OUST64);
  get_param("preprocess.scan_line", p_pre->N_SCANS, 16);
  get_param("preprocess.timestamp_unit", p_pre->time_unit, (int)US);
  get_param("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
  get_param("point_filter_num", p_pre->point_filter_num, 2);
  get_param("feature_extract_enable", p_pre->feature_enabled, false);
  get_param("mapping.extrinsic_est_en", extrinsic_est_en, true);
  get_param("pcd_save.pcd_save_en", pcd_save_en, false);
  get_param("pcd_save.interval", pcd_save_interval, -1);
  get_param("mapping.extrinsic_T", extrinT, vector<double>());
  get_param("mapping.extrinsic_R", extrinR, vector<double>());

  float ivox_resolution = 0.5f;
  int ivox_nearby_type = 6;
  get_param("ivox_grid_resolution", ivox_resolution, 0.5f);
  get_param("ivox_nearby_type", ivox_nearby_type, 6);
  ivox_options_.resolution_ = ivox_resolution;
  if (ivox_nearby_type == 0) {
    ivox_options_.nearby_type_ = MapType::NearbyType::CENTER;
  } else if (ivox_nearby_type == 6) {
    ivox_options_.nearby_type_ = MapType::NearbyType::NEARBY6;
  } else if (ivox_nearby_type == 18) {
    ivox_options_.nearby_type_ = MapType::NearbyType::NEARBY18;
  } else if (ivox_nearby_type == 26) {
    ivox_options_.nearby_type_ = MapType::NearbyType::NEARBY26;
  } else {
    ivox_options_.nearby_type_ = MapType::NearbyType::NEARBY6;
  }

  ivox = std::make_unique<MapType>(ivox_options_);

  cout << "Lidar_type: " << p_pre->lidar_type << endl;
  path.header.stamp = node->now();
  path.header.frame_id = "camera_init";

  /*** ROS2 subscribe initialization ***/
  auto sub_pcl = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);
  auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(), imu_cbk);

  auto pubLaserCloudFull =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered",
                                                            100);
  auto pubLaserCloudFull_body =
      node->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/cloud_registered_body", 100);
  auto pubLaserCloudEffect =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected",
                                                            100);
  auto pubLaserCloudMap =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100);
  auto pubOdomAftMapped =
      node->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100);
  auto pubPath = node->create_publisher<nav_msgs::msg::Path>("/path", 100);

  auto tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min,
                                 filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min,
                                filter_size_map_min);

  shared_ptr<ImuProcess> p_imu1(new ImuProcess());
  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
  p_imu1->set_param(
      Lidar_T_wrt_IMU, Lidar_R_wrt_IMU, V3D(gyr_cov, gyr_cov, gyr_cov),
      V3D(acc_cov, acc_cov, acc_cov), V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov),
      V3D(b_acc_cov, b_acc_cov, b_acc_cov));

  signal(SIGINT, SigHandle);
  rclcpp::Rate rate(5000);

  while (rclcpp::ok()) {
    if (flg_exit)
      break;
    rclcpp::spin_some(node);

    if (sync_packages(Measures)) {
      double t00 = omp_get_wtime();

      if (flg_first_scan) {
        first_lidar_time = Measures.lidar_beg_time;
        p_imu1->first_lidar_time = first_lidar_time;
        flg_first_scan = false;
        continue;
      }

      p_imu1->Process(Measures, kf, feats_undistort);

      if (feats_undistort->empty() || (feats_undistort == NULL)) {
        RCLCPP_WARN(node->get_logger(), "No point, skip this scan!");
        continue;
      }

      state_point = kf.get_x();
      pos_lid =
          state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;

      flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME
                           ? false
                           : true;

      downSizeFilterSurf.setInputCloud(feats_undistort);
      downSizeFilterSurf.filter(*feats_down_body);
      feats_down_size = feats_down_body->points.size();

      if (feats_down_size < 5) {
        RCLCPP_WARN(node->get_logger(), "No point, skip this scan!");
        continue;
      }

      /*** iterated state estimation ***/
      Nearest_Points.resize(feats_down_size);
      kf.update_iterated_dyn_share_modified(
          LASER_POINT_COV, feats_down_body, *ivox, Nearest_Points,
          NUM_MAX_ITERATIONS, extrinsic_est_en);

      state_point = kf.get_x();
      pos_lid =
          state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;

      /******* Publish odometry *******/
      publish_odometry(pubOdomAftMapped, tf_broadcaster);

      /*** add the feature points to map ***/
      feats_down_world->resize(feats_down_size);
      map_incremental();

      /******* Publish points *******/
      if (path_en)
        publish_path(pubPath);
      if (scan_pub_en || pcd_save_en)
        publish_frame_world(pubLaserCloudFull);
      if (scan_pub_en && scan_body_pub_en)
        publish_frame_body(pubLaserCloudFull_body);

      // double t11 = omp_get_wtime();
      // std::cout << "feats_down_size: " << feats_down_size
      //           << "  Whole mapping time(ms):  " << (t11 - t00) * 1000
      //           << std::endl
      //           << std::endl;
    }

    rate.sleep();
  }

  return 0;
}
