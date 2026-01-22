# ROS 문법 정리

S-Faster-Lio 프로젝트의 src와 include 폴더에서 사용되는 ROS 문법을 정리한 문서입니다.

## 목차
1. [기본 헤더 및 초기화](#1-기본-헤더-및-초기화)
2. [파라미터 관리](#2-파라미터-관리)
3. [메시지 타입](#3-메시지-타입)
4. [Publisher와 Subscriber](#4-publisher와-subscriber)
5. [콜백 함수](#5-콜백-함수)
6. [시간 관리](#6-시간-관리)
7. [루프 및 스핀](#7-루프-및-스핀)
8. [로깅](#8-로깅)
9. [TF (Transform)](#9-tf-transform)
10. [PCL 변환](#10-pcl-변환)

---

## 1. 기본 헤더 및 초기화

### 헤더 파일 포함
```cpp
#include <ros/ros.h>
```

### ROS 노드 초기화
```cpp
int main(int argc, char** argv) {
  ros::init(argc, argv, "laserMapping");  // 노드 이름 지정
  ros::NodeHandle nh;                     // 노드 핸들 생성
  // ...
  return 0;
}
```

**사용 위치:**
- `src/laserMapping.cpp:479`

---

## 2. 파라미터 관리

### 파라미터 읽기
```cpp
nh.param<타입>("파라미터_이름", 변수, 기본값);
```

**예제:**
```cpp
nh.param<bool>("publish/path_en", path_en, true);
nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
```

**사용 위치:**
- `src/laserMapping.cpp:482-537`

---

## 3. 메시지 타입

### 센서 메시지
```cpp
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

sensor_msgs::PointCloud2::ConstPtr msg;    // 포인트 클라우드 메시지
sensor_msgs::Imu::ConstPtr imu_msg;        // IMU 메시지
```

### 네비게이션 메시지
```cpp
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

nav_msgs::Odometry odomAftMapped;          // 오도메트리 메시지
nav_msgs::Path path;                        // 경로 메시지
```

### 지오메트리 메시지
```cpp
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

geometry_msgs::PoseStamped msg_body_pose;  // 포즈 메시지
geometry_msgs::Vector3 vec;                 // 3D 벡터
```

### Livox 커스텀 메시지
```cpp
#include <livox_ros_driver/CustomMsg.h>

livox_ros_driver::CustomMsg::ConstPtr msg; // Livox 라이다 메시지
```

**사용 위치:**
- `src/laserMapping.cpp:1-17`
- `src/preprocess.h:1-4`

---

## 4. Publisher와 Subscriber

### Publisher 생성
```cpp
ros::Publisher pubLaserCloudFull =
    nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
ros::Publisher pubOdomAftMapped =
    nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
ros::Publisher pubPath =
    nh.advertise<nav_msgs::Path>("/path", 100000);
```

### Subscriber 생성
```cpp
ros::Subscriber sub_pcl =
    nh.subscribe(lid_topic, 200000, livox_pcl_cbk);
ros::Subscriber sub_imu =
    nh.subscribe(imu_topic, 200000, imu_cbk);
```

### 메시지 발행
```cpp
pubLaserCloudFull.publish(laserCloudmsg);
pubOdomAftMapped.publish(odomAftMapped);
pubPath.publish(path);
```

**사용 위치:**
- `src/laserMapping.cpp:562-577` (Publisher/Subscriber 생성)
- `src/laserMapping.cpp:357, 436, 474` (메시지 발행)

---

## 5. 콜백 함수

### 포인트 클라우드 콜백
```cpp
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  mtx_buffer.lock();
  // 메시지 처리
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}
```

### Livox 포인트 클라우드 콜백
```cpp
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
  mtx_buffer.lock();
  // 메시지 처리
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}
```

### IMU 콜백
```cpp
void imu_cbk(const sensor_msgs::Imu::ConstPtr& msg_in) {
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
  // IMU 데이터 처리
  imu_buffer.push_back(msg);
}
```

**사용 위치:**
- `src/laserMapping.cpp:106-122` (standard_pcl_cbk)
- `src/laserMapping.cpp:126-158` (livox_pcl_cbk)
- `src/laserMapping.cpp:160-187` (imu_cbk)

---

## 6. 시간 관리

### 현재 시간 가져오기
```cpp
ros::Time now = ros::Time::now();
path.header.stamp = ros::Time::now();
```

### 초 단위로 시간 변환
```cpp
double timestamp = msg->header.stamp.toSec();
```

### 초에서 ros::Time으로 변환
```cpp
ros::Time t = ros::Time().fromSec(lidar_end_time);
msg->header.stamp = ros::Time().fromSec(timestamp);
```

**사용 위치:**
- `src/laserMapping.cpp:110, 118, 134` (toSec)
- `src/laserMapping.cpp:166, 355, 404` (fromSec)
- `src/laserMapping.cpp:558` (now)

---

## 7. 루프 및 스핀

### 루프 레이트 설정
```cpp
ros::Rate rate(5000);  // 5000Hz

while (ros::ok()) {
  if (flg_exit) break;
  ros::spinOnce();     // 콜백 함수 처리

  // 메인 처리 로직

  rate.sleep();        // 레이트에 맞춰 대기
}
```

**사용 위치:**
- `src/laserMapping.cpp:594-673`

---

## 8. 로깅

### 로그 레벨별 출력
```cpp
ROS_INFO("IMU Initial Done");                    // 정보 메시지
ROS_WARN("catch sig %d", sig);                   // 경고 메시지
ROS_ERROR("lidar loop back, clear buffer");      // 에러 메시지
ROS_ASSERT(meas.lidar != nullptr);               // 단언문 (조건이 거짓이면 에러)
```

**사용 위치:**
- `src/laserMapping.cpp:102` (ROS_WARN)
- `src/laserMapping.cpp:111, 131` (ROS_ERROR)
- `src/IMU_Processing.hpp:368, 386` (ROS_ASSERT, ROS_INFO)
- `include/esekfom.hpp:173` (ROS_WARN)

---

## 9. TF (Transform)

### TransformBroadcaster 사용
```cpp
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

static tf::TransformBroadcaster br;
tf::Transform transform;
tf::Quaternion q;

// 위치 설정
transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                odomAftMapped.pose.pose.position.y,
                                odomAftMapped.pose.pose.position.z));

// 회전 설정
q.setW(odomAftMapped.pose.pose.orientation.w);
q.setX(odomAftMapped.pose.pose.orientation.x);
q.setY(odomAftMapped.pose.pose.orientation.y);
q.setZ(odomAftMapped.pose.pose.orientation.z);
transform.setRotation(q);

// 변환 브로드캐스트
br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp,
                                      "camera_init", "body"));
```

**사용 위치:**
- `src/laserMapping.cpp:449-461`

---

## 10. PCL 변환

### PCL에서 ROS 메시지로 변환
```cpp
#include <pcl_conversions/pcl_conversions.h>

sensor_msgs::PointCloud2 laserCloudmsg;
pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
laserCloudmsg.header.frame_id = "camera_init";
```

### ROS 메시지에서 PCL로 변환
```cpp
pcl::PointCloud<ouster_ros::Point> pl_orig;
pcl::fromROSMsg(*msg, pl_orig);
```

**사용 위치:**
- `src/laserMapping.cpp:354, 403, 412` (toROSMsg)
- `src/preprocess.cpp:192, 291` (fromROSMsg)
- `src/preprocess.h:784` (toROSMsg in pub_func)

---

## 주요 데이터 구조

### MeasureGroup (common_lib.h)
```cpp
struct MeasureGroup {
  double lidar_beg_time;
  double lidar_end_time;
  PointCloudXYZI::Ptr lidar;
  deque<sensor_msgs::Imu::ConstPtr> imu;
};
```

이 구조체는 한 프레임의 라이다 데이터와 해당 시간 범위 내의 IMU 데이터를 함께 묶어서 관리합니다.

**사용 위치:**
- `include/common_lib.h:39-49`
- `src/laserMapping.cpp:192-244` (sync_packages)

---

## 참고사항

1. **뮤텍스와 조건 변수**: ROS 콜백과 메인 스레드 간의 동기화를 위해 `std::mutex`와 `std::condition_variable`을 함께 사용합니다.
   ```cpp
   mutex mtx_buffer;
   condition_variable sig_buffer;
   ```

2. **deque 사용**: 버퍼링을 위해 ROS 메시지를 `std::deque`에 저장합니다.
   ```cpp
   deque<PointCloudXYZI::Ptr> lidar_buffer;
   deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
   ```

3. **헤더 설정**: ROS 메시지를 발행할 때는 항상 `header.stamp`와 `header.frame_id`를 설정해야 합니다.
