# ROS1 → ROS2 Humble Migration Guide

> S-Faster-Lio (LiDAR-Inertial Odometry) 프로젝트를 실제로 마이그레이션한 과정을 기반으로 작성된 가이드입니다.
> 이 문서를 통해 ROS1 C++ 프로젝트를 ROS2로 이식하는 전체 과정을 학습할 수 있습니다.

---

## 목차

1. [시작하기 전에 — ROS1과 ROS2의 핵심 차이](#1-시작하기-전에--ros1과-ros2의-핵심-차이)
2. [Step 1: package.xml 변환](#2-step-1-packagexml-변환)
3. [Step 2: CMakeLists.txt 변환](#3-step-2-cmakeliststxt-변환)
4. [Step 3: YAML 파라미터 파일 변환](#4-step-3-yaml-파라미터-파일-변환)
5. [Step 4: 헤더 파일 변환 (include 경로와 타입)](#5-step-4-헤더-파일-변환)
6. [Step 5: 메시지 타입과 스마트 포인터 변환](#6-step-5-메시지-타입과-스마트-포인터-변환)
7. [Step 6: 시간(Time) API 변환](#7-step-6-시간time-api-변환)
8. [Step 7: 노드, 파라미터, Publisher/Subscriber 변환](#8-step-7-노드-파라미터-publishersubscriber-변환)
9. [Step 8: TF 브로드캐스터 변환](#9-step-8-tf-브로드캐스터-변환)
10. [Step 9: 로깅 매크로 변환](#10-step-9-로깅-매크로-변환)
11. [Step 10: Launch 파일 변환](#11-step-10-launch-파일-변환)
12. [Step 11: RViz 설정 파일 변환](#12-step-11-rviz-설정-파일-변환)
13. [자주 발생하는 빌드 에러와 해결법](#13-자주-발생하는-빌드-에러와-해결법)
14. [파일별 변경 요약표](#14-파일별-변경-요약표)
15. [한눈에 보는 치트시트](#15-한눈에-보는-치트시트)

---

## 1. 시작하기 전에 — ROS1과 ROS2의 핵심 차이

마이그레이션 작업을 시작하기 전에, ROS2가 ROS1과 **구조적으로** 어떻게 다른지 이해해야 합니다.

### 빌드 시스템

| 항목 | ROS1 | ROS2 |
|------|------|------|
| 빌드 도구 | `catkin_make` / `catkin build` | `colcon build` |
| CMake 프레임워크 | `catkin` | `ament_cmake` |
| 패키지 manifest | `package.xml` format 1~2 | `package.xml` format 3 |
| install 과정 | 자동 (devel space) | **반드시 명시적 `install()` 필요** |

> **왜 install이 필요한가?**
> ROS1의 catkin은 `devel/` 폴더에 심볼릭 링크를 만들어 빌드 결과를 바로 사용할 수 있었습니다.
> ROS2의 colcon은 `install/` 폴더에 실제로 파일을 복사하므로, CMakeLists.txt에서 `install()` 명령을 빠뜨리면
> 실행 파일이나 launch 파일을 찾을 수 없습니다.

### 통신 시스템

| 항목 | ROS1 | ROS2 |
|------|------|------|
| 미들웨어 | TCPROS/UDPROS (자체) | DDS (표준 미들웨어) |
| 마스터 노드 | `roscore` 필수 | **필요 없음** (분산 Discovery) |
| QoS (통신 품질) | queue size만 설정 | Reliability, Durability, History 등 세밀한 제어 |

> **QoS가 중요한 이유:**
> ROS1에서는 subscriber에 큰 queue size (예: 200000)를 주면 끝이었지만,
> ROS2에서는 publisher와 subscriber의 QoS가 **호환**되어야 통신이 됩니다.
> 센서 데이터에는 `rclcpp::SensorDataQoS()`를 사용하면 Best Effort + Volatile로 설정되어
> 실시간 센서 토픽과 잘 맞습니다.

### 노드 구조

| 항목 | ROS1 | ROS2 |
|------|------|------|
| 노드 생성 | `ros::NodeHandle` (전역) | `rclcpp::Node` (객체 기반) |
| 파라미터 접근 | `nh.param<T>(...)` (암묵적 선언) | `declare_parameter` → `get_parameter` (명시적 선언 필수) |
| 파라미터 계층 구분자 | `/` (슬래시) | `.` (점) |

> **왜 declare가 필요한가?**
> ROS2에서는 "존재하지 않는 파라미터를 읽으려 하면 에러"가 발생합니다.
> 반드시 `declare_parameter(이름, 기본값)`으로 먼저 선언한 뒤 `get_parameter()`로 읽어야 합니다.
> 이는 실수로 잘못된 파라미터 이름을 사용하는 버그를 방지하기 위한 설계입니다.

---

## 2. Step 1: `package.xml` 변환

**package.xml**은 패키지의 메타정보와 의존성을 선언하는 파일입니다. ROS2에서는 format 3을 사용합니다.

### 변경 규칙

```xml
<!-- ROS1 (format 1 or 2) -->
<package>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>message_generation</build_depend>
  <run_depend>message_runtime</run_depend>
</package>
```

```xml
<!-- ROS2 (format 3) -->
<package format="3">
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>                          <!-- roscpp 대체 -->
  <buildtool_depend>rosidl_default_generators</buildtool_depend>  <!-- message_generation 대체 -->
  <exec_depend>rosidl_default_runtime</exec_depend>               <!-- message_runtime 대체 -->

  <!-- 커스텀 메시지가 있으면 반드시 추가 -->
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>   <!-- 반드시 명시 -->
  </export>
</package>
```

### 의존성 패키지 매핑표

| ROS1 패키지 | ROS2 패키지 | 설명 |
|-------------|------------|------|
| `catkin` | `ament_cmake` | 빌드 시스템 |
| `roscpp` | `rclcpp` | C++ 클라이언트 라이브러리 |
| `rospy` | `rclpy` | Python 클라이언트 (이 프로젝트에서는 제거) |
| `tf` | `tf2` + `tf2_ros` + `tf2_geometry_msgs` | TF가 ROS2에서는 세 패키지로 분리됨 |
| `pcl_ros` | `pcl_conversions` | PCL ↔ ROS 메시지 변환만 필요 |
| `message_generation` | `rosidl_default_generators` | 메시지 코드 생성기 |
| `message_runtime` | `rosidl_default_runtime` | 생성된 메시지 런타임 |

> **Tip:** ROS2에서는 `<depend>` 태그 하나로 build + exec depend를 동시에 선언할 수 있습니다.
> `<build_depend>` + `<exec_depend>`를 각각 쓰는 것보다 간결합니다.

> **Tip:** `<member_of_group>rosidl_interface_packages</member_of_group>`를 빠뜨리면
> 다른 패키지에서 이 패키지의 커스텀 메시지를 찾지 못합니다.

---

## 3. Step 2: `CMakeLists.txt` 변환

CMakeLists.txt는 가장 많이 바뀌는 파일 중 하나입니다. catkin의 매크로를 모두 ament_cmake로 교체해야 합니다.

### 전체 구조 비교

```cmake
# ===== ROS1 (catkin) =====
find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs nav_msgs tf message_generation
)
add_message_files(FILES Pose6D.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs)

add_executable(my_node src/main.cpp)
target_link_libraries(my_node ${catkin_LIBRARIES})
```

```cmake
# ===== ROS2 (ament_cmake) =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)            # 각 패키지를 개별로 find
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 메시지 생성 (catkin의 add_message_files + generate_messages 대체)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

add_executable(my_node src/main.cpp)
ament_target_dependencies(my_node rclcpp sensor_msgs)  # catkin_LIBRARIES 대체

# 생성된 메시지 헤더를 실행 파일에 연결
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(my_node ${cpp_typesupport_target})

# ★ ROS2에서는 반드시 install 해야 함!
install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz_cfg DESTINATION share/${PROJECT_NAME})

ament_package()   # 맨 마지막에 호출
```

### 핵심 포인트

1. **`find_package`를 개별로 호출**: catkin처럼 한번에 여러 패키지를 찾을 수 없음
2. **`ament_target_dependencies()`**: ROS2 패키지의 include path와 library를 자동 연결
3. **`rosidl_generate_interfaces()`**: 커스텀 메시지 생성. 반드시 `DEPENDENCIES`에 의존하는 메시지 패키지 명시
4. **`rosidl_get_typesupport_target()`**: 생성된 메시지 헤더를 실행 파일에서 사용하려면 이 링크 필요
5. **`install()` 필수**: 실행 파일, launch 파일, config 파일 등을 install 디렉토리에 복사
6. **`ament_package()`**: 반드시 CMakeLists.txt의 **맨 마지막**에 호출

> **주의:** `ament_package()`를 `install()` 보다 먼저 호출하면 에러가 발생합니다.

---

## 4. Step 3: YAML 파라미터 파일 변환

ROS2에서는 YAML 파라미터 파일의 구조가 다릅니다. **반드시 네임스페이스로 감싸야** 합니다.

### 변환 규칙

```yaml
# ===== ROS1 (그대로 사용) =====
common:
  lid_topic: "/ouster/points"
  imu_topic: "/ouster/imu"
mapping:
  acc_cov: 0.1
  gyr_cov: 0.1
```

```yaml
# ===== ROS2 (네임스페이스 래핑 필요) =====
/**:                    # ← 모든 노드에 적용 (노드 이름을 특정하려면 /노드이름: 사용)
  ros__parameters:      # ← 반드시 이 키 아래에 파라미터 배치 (밑줄 2개!)
    common:
      lid_topic: "/ouster/points"
      imu_topic: "/ouster/imu"
    mapping:
      acc_cov: 0.1
      gyr_cov: 0.1
```

> **왜 `ros__parameters`인가?**
> ROS2에서 YAML 파일은 범용 설정에도 사용될 수 있으므로, ROS 파라미터 영역을 명시적으로 구분합니다.
> `ros__parameters` 아래에 없는 키는 ROS2 파라미터로 인식되지 않습니다.

> **주의:** `ros_parameters` (밑줄 1개)가 아니라 `ros__parameters` (밑줄 **2개**)입니다!
> 이것은 ROS2에서 가장 흔한 실수 중 하나입니다.

### 파라미터 접근 시 구분자 변경

YAML에서 중첩된 파라미터를 코드에서 접근할 때의 구분자가 바뀝니다:

```cpp
// ROS1: 슬래시(/) 사용
nh.param<double>("mapping/acc_cov", acc_cov, 0.1);

// ROS2: 점(.) 사용
node->declare_parameter("mapping.acc_cov", 0.1);
node->get_parameter("mapping.acc_cov", acc_cov);
```

---

## 5. Step 4: 헤더 파일 변환

ROS2에서는 모든 메시지 헤더의 경로와 확장자가 바뀝니다.

### 헤더 경로 변환 규칙

**패턴:**  `<패키지명/메시지명.h>` → `<패키지명/msg/메시지명.hpp>`

```cpp
// ===== ROS1 헤더 =====
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>

// ===== ROS2 헤더 =====
#include <rclcpp/rclcpp.hpp>                         // ros/ros.h 대체
#include <sensor_msgs/msg/imu.hpp>                   // msg 서브디렉토리 추가, .hpp 확장자
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/transform_broadcaster.h>           // tf → tf2_ros
```

### 변환 규칙 정리

| 규칙 | ROS1 | ROS2 |
|------|------|------|
| 기본 헤더 | `<ros/ros.h>` | `<rclcpp/rclcpp.hpp>` |
| 메시지 경로 | `<패키지/메시지.h>` | `<패키지/msg/메시지.hpp>` |
| 서비스 경로 | `<패키지/서비스.h>` | `<패키지/srv/서비스.hpp>` |
| TF | `<tf/transform_broadcaster.h>` | `<tf2_ros/transform_broadcaster.h>` |
| 확장자 | `.h` | `.hpp` |
| 메시지 이름 | CamelCase 그대로 | snake_case 변환 (파일명) |

> **커스텀 메시지의 경우:**
> 메시지 이름이 CamelCase이면 파일명은 snake_case로 변환됩니다.
> 예: `Pose6D.msg` → `#include <s_faster_lio/msg/pose6_d.hpp>`
> (Pose6D → pose6_d로 변환됨)

> **흔한 빌드 에러:**
> ROS1에서 간접적으로 include 되던 STL 헤더 (`<deque>`, `<vector>` 등)가
> ROS2 헤더에서는 포함되지 않을 수 있습니다. `'deque' does not name a type` 같은 에러가 나면
> 해당 STL 헤더를 명시적으로 `#include` 해주세요.

---

## 6. Step 5: 메시지 타입과 스마트 포인터 변환

ROS2에서는 메시지 네임스페이스에 `msg`가 추가되고, 포인터 타입이 바뀝니다.

### 네임스페이스 변경

```cpp
// ROS1: 패키지명::메시지명
sensor_msgs::Imu
sensor_msgs::PointCloud2
nav_msgs::Odometry

// ROS2: 패키지명::msg::메시지명 (msg 네임스페이스 추가)
sensor_msgs::msg::Imu
sensor_msgs::msg::PointCloud2
nav_msgs::msg::Odometry
```

### 스마트 포인터 변경

```cpp
// ROS1: ConstPtr (boost::shared_ptr<const T>)
void callback(const sensor_msgs::Imu::ConstPtr& msg);
sensor_msgs::ImuConstPtr last_imu_;

// ROS2: SharedPtr (std::shared_ptr<T>)
void callback(const sensor_msgs::msg::Imu::SharedPtr msg);   // const 참조 제거 가능
sensor_msgs::msg::Imu::SharedPtr last_imu_;
```

> **왜 ConstPtr → SharedPtr인가?**
> ROS1은 `boost::shared_ptr`을 사용했고 메시지를 const로 전달했습니다.
> ROS2는 `std::shared_ptr`을 사용하며, 콜백에서 메시지를 수정해야 할 경우도 있어
> non-const SharedPtr이 기본입니다.

### 메시지 객체 생성 방식

```cpp
// ROS1
last_imu_.reset(new sensor_msgs::Imu());

// ROS2
last_imu_ = std::make_shared<sensor_msgs::msg::Imu>();
```

---

## 7. Step 6: 시간(Time) API 변환

ROS2의 시간 API는 ROS1과 크게 다릅니다. 마이그레이션에서 가장 실수하기 쉬운 부분입니다.

### stamp에서 초(seconds) 읽기

```cpp
// ROS1: 멤버 함수로 직접 변환
double t = msg->header.stamp.toSec();

// ROS2: rclcpp::Time 객체를 거쳐야 함
double t = rclcpp::Time(msg->header.stamp).seconds();
```

> **왜 직접 .toSec()을 쓸 수 없나?**
> ROS2의 `builtin_interfaces::msg::Time`은 단순한 데이터 구조체(sec, nanosec 필드)이므로
> 변환 메서드가 없습니다. `rclcpp::Time`으로 감싸야 `.seconds()` 등의 메서드를 사용할 수 있습니다.

### double(초) → stamp 변환

```cpp
// ROS1: fromSec() 사용
msg.header.stamp = ros::Time().fromSec(lidar_end_time);

// ROS2: 나노초(int64_t)로 변환하여 생성
msg.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
```

> **주의:** `1e9`를 곱하여 나노초로 변환할 때 반드시 `int64_t`로 캐스팅하세요.
> 부동소수점 오차로 인해 `uint64_t`를 사용하면 음수 시간에서 문제가 생길 수 있습니다.

### 현재 시간 가져오기

```cpp
// ROS1
ros::Time now = ros::Time::now();

// ROS2 (노드 객체 필요)
rclcpp::Time now = node->now();
```

### Rate (주기 제어)

```cpp
// ROS1
ros::Rate rate(5000);
rate.sleep();

// ROS2 (동일한 인터페이스)
rclcpp::Rate rate(5000);
rate.sleep();
```

---

## 8. Step 7: 노드, 파라미터, Publisher/Subscriber 변환

이 부분이 마이그레이션의 핵심입니다. ROS2에서는 노드가 **객체**이며, 모든 것이 노드를 통해 접근됩니다.

### 노드 초기화

```cpp
// ===== ROS1 =====
int main(int argc, char** argv) {
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  // nh로 파라미터, publisher, subscriber 모두 접근
}

// ===== ROS2 =====
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("laserMapping");
  // node->로 파라미터, publisher, subscriber 모두 접근
}
```

> **Global 함수에서 노드 접근이 필요할 때:**
> ROS1에서는 `ROS_WARN()`처럼 전역 함수에서 바로 로깅이 가능했습니다.
> ROS2에서는 logger 객체가 필요하므로, 전역 노드 포인터를 만들어 사용하는 패턴이 일반적입니다:
> ```cpp
> rclcpp::Node::SharedPtr g_node = nullptr;  // 전역
>
> void my_free_function() {
>   RCLCPP_WARN(g_node->get_logger(), "Something happened");
> }
>
> int main() {
>   auto node = std::make_shared<rclcpp::Node>("my_node");
>   g_node = node;  // 전역에 할당
> }
> ```

### 파라미터 읽기

```cpp
// ===== ROS1: 한 줄로 선언과 읽기를 동시에 =====
nh.param<double>("mapping/acc_cov", acc_cov, 0.1);

// ===== ROS2: 선언(declare) → 읽기(get) 두 단계 =====
node->declare_parameter("mapping.acc_cov", 0.1);      // 기본값과 함께 선언
node->get_parameter("mapping.acc_cov", acc_cov);       // 값 읽기
```

> **파라미터가 많을 때 유용한 패턴 — Lambda helper:**
> ```cpp
> auto get_param = [&](const std::string& name, auto& var, const auto& default_val) {
>   node->declare_parameter(name, default_val);
>   node->get_parameter(name, var);
> };
>
> // 이제 ROS1과 비슷한 느낌으로 사용 가능
> get_param("mapping.acc_cov", acc_cov, 0.1);
> get_param("common.lid_topic", lid_topic, std::string("/lidar"));
> ```

### Subscriber 생성

```cpp
// ===== ROS1 =====
ros::Subscriber sub = nh.subscribe("/imu", 200000, imu_callback);

// ===== ROS2 =====
auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "/imu",
    rclcpp::SensorDataQoS(),    // 센서 데이터에 적합한 QoS 프리셋
    imu_callback
);
```

> **QoS 프리셋 선택 가이드:**
> - `rclcpp::SensorDataQoS()`: 센서 데이터 (Best Effort, Volatile) — LiDAR, IMU 등
> - `rclcpp::SystemDefaultsQoS()`: 일반적인 토픽
> - `rclcpp::QoS(10)`: 단순히 depth만 지정 (Reliable, Volatile)
>
> 센서 데이터에 Reliable QoS를 사용하면 데이터 손실 시 재전송을 기다려 **지연**이 발생합니다.

### Publisher 생성

```cpp
// ===== ROS1 =====
ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/Odometry", 100);
pub.publish(msg);

// ===== ROS2 =====
auto pub = node->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100);
pub->publish(msg);    // -> (화살표) 사용 (SharedPtr)
```

### 메인 루프

```cpp
// ===== ROS1 =====
ros::Rate rate(5000);
while (ros::ok()) {
  ros::spinOnce();    // 콜백 한 번 처리
  // ... 처리 로직 ...
  rate.sleep();
}

// ===== ROS2 =====
rclcpp::Rate rate(5000);
while (rclcpp::ok()) {
  rclcpp::spin_some(node);   // 대기 중인 콜백 처리
  // ... 처리 로직 ...
  rate.sleep();
}
```

### Signal Handler

```cpp
// ===== ROS1 =====
void SigHandle(int sig) {
  ROS_WARN("catch sig %d", sig);
  ros::shutdown();
}

// ===== ROS2 =====
void SigHandle(int sig) {
  RCLCPP_WARN(g_node->get_logger(), "catch sig %d", sig);
  rclcpp::shutdown();
}
```

---

## 9. Step 8: TF 브로드캐스터 변환

ROS1의 `tf` 라이브러리는 ROS2에서 `tf2`로 완전히 대체되었습니다.

```cpp
// ===== ROS1 =====
#include <tf/transform_broadcaster.h>

static tf::TransformBroadcaster br;
tf::Transform transform;
transform.setOrigin(tf::Vector3(x, y, z));
tf::Quaternion q;
q.setRPY(0, 0, yaw);
transform.setRotation(q);
br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_init", "body"));
```

```cpp
// ===== ROS2 =====
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// 생성 시 노드를 전달해야 함
auto tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

// TransformStamped 메시지를 직접 채움
geometry_msgs::msg::TransformStamped t;
t.header.stamp = node->now();
t.header.frame_id = "camera_init";
t.child_frame_id = "body";
t.transform.translation.x = x;
t.transform.translation.y = y;
t.transform.translation.z = z;
t.transform.rotation = odom.pose.pose.orientation;  // Quaternion 직접 대입
tf_broadcaster->sendTransform(t);
```

> **핵심 차이:**
> - ROS1: `tf::TransformBroadcaster`는 기본 생성자로 생성 가능
> - ROS2: `tf2_ros::TransformBroadcaster`는 **반드시 노드를 인자로** 전달해야 함
> - ROS2: `geometry_msgs::msg::TransformStamped` 메시지를 직접 구성

---

## 10. Step 9: 로깅 매크로 변환

ROS2의 로깅은 **logger 객체**를 첫 번째 인자로 요구합니다.

### 변환 규칙

```cpp
// ===== ROS1 (전역 함수처럼 사용) =====
ROS_INFO("Initialized with %d points", count);
ROS_WARN("Low point count!");
ROS_ERROR("Failed to process scan");

// ===== ROS2 (logger 객체 필요) =====
// 노드 내부에서:
RCLCPP_INFO(node->get_logger(), "Initialized with %d points", count);
RCLCPP_WARN(node->get_logger(), "Low point count!");
RCLCPP_ERROR(node->get_logger(), "Failed to process scan");

// 노드 외부 (전역/static 함수)에서:
RCLCPP_WARN(rclcpp::get_logger("my_module"), "Something happened");
```

| ROS1 | ROS2 |
|------|------|
| `ROS_DEBUG(...)` | `RCLCPP_DEBUG(logger, ...)` |
| `ROS_INFO(...)` | `RCLCPP_INFO(logger, ...)` |
| `ROS_WARN(...)` | `RCLCPP_WARN(logger, ...)` |
| `ROS_ERROR(...)` | `RCLCPP_ERROR(logger, ...)` |
| `ROS_FATAL(...)` | `RCLCPP_FATAL(logger, ...)` |
| `ROS_ASSERT(cond)` | `assert(cond)` (표준 C++) |

> **`rclcpp::get_logger("이름")`의 활용:**
> 전역 함수, 헤더 파일, 클래스가 아닌 곳에서 로깅이 필요할 때 사용합니다.
> 이름은 자유롭게 지정할 수 있으며, 로그 출력에서 `[이름]`으로 표시됩니다.

---

## 11. Step 10: Launch 파일 변환

ROS2에서는 launch 파일을 **Python**으로 작성합니다. (XML도 지원하지만 Python이 공식 권장)

### 변환 예제

```xml
<!-- ===== ROS1 (XML) ===== -->
<launch>
  <rosparam command="load" file="$(find s_faster_lio)/config/ouster64.yaml" />
  <param name="feature_extract_enable" type="bool" value="0"/>
  <param name="point_filter_num" type="int" value="4"/>

  <node pkg="s_faster_lio" type="fastlio_mapping" name="laserMapping" output="screen"/>
  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find s_faster_lio)/rviz_cfg/loam_livox.rviz"/>
</launch>
```

```python
# ===== ROS2 (Python) =====
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # $(find 패키지) 대신 get_package_share_directory 사용
    pkg_dir = get_package_share_directory('s_faster_lio')

    # launch argument 선언 (roslaunch의 <arg> 태그 대체)
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2'
    )

    mapping_node = Node(
        package='s_faster_lio',
        executable='fastlio_mapping',     # type → executable
        name='laserMapping',
        output='screen',
        parameters=[
            # rosparam load → parameters 리스트에 yaml 파일 경로 전달
            os.path.join(pkg_dir, 'config', 'ouster64.yaml'),
            # 개별 파라미터는 dict로 전달
            {
                'feature_extract_enable': False,
                'point_filter_num': 4,
                'max_iteration': 3,
                'filter_size_surf': 0.5,
                'filter_size_map': 0.5,
                'cube_side_length': 1000.0,
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',                   # rviz → rviz2
        executable='rviz2',                # rviz → rviz2
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg_dir, 'rviz_cfg', 'loam_livox.rviz')],
    )

    return LaunchDescription([
        rviz_arg,
        mapping_node,
        rviz_node,
    ])
```

### 핵심 매핑

| ROS1 XML | ROS2 Python |
|----------|-------------|
| `$(find pkg)` | `get_package_share_directory('pkg')` |
| `<rosparam command="load" file="..."/>` | `parameters=[yaml_path]` in Node |
| `<param name="x" value="1"/>` | `parameters=[{'x': 1}]` in Node |
| `<node pkg="..." type="..." name="...">` | `Node(package='...', executable='...', name='...')` |
| `<arg name="x" default="true"/>` | `DeclareLaunchArgument('x', default_value='true')` |
| `if="$(arg x)"` | `condition=IfCondition(LaunchConfiguration('x'))` |
| `rviz` | `rviz2` |

> **주의:** ROS2 launch 파일의 파일명은 반드시 `.launch.py`로 끝나야 합니다.
> `colcon`과 `ros2 launch`가 이 패턴으로 launch 파일을 찾습니다.

---

## 12. Step 11: RViz 설정 파일 변환

RViz 설정 파일(.rviz)은 ROS2의 RViz2에서 포맷이 다릅니다.

### 플러그인 클래스 이름 변경

| ROS1 (RViz) | ROS2 (RViz2) | 설명 |
|-------------|-------------|------|
| `rviz/Grid` | `rviz_default_plugins/Grid` | 시각화 플러그인 |
| `rviz/PointCloud2` | `rviz_default_plugins/PointCloud2` | 시각화 플러그인 |
| `rviz/Odometry` | `rviz_default_plugins/Odometry` | 시각화 플러그인 |
| `rviz/Path` | `rviz_default_plugins/Path` | 시각화 플러그인 |
| `rviz/Displays` | `rviz_common/Displays` | 패널 |
| `rviz/Views` | `rviz_common/Views` | 패널 |
| `rviz/Time` | `rviz_common/Time` | 패널 |
| `rviz/Group` | `rviz_common/Group` | 그룹 |

> **규칙:**
> - 시각화 Display 플러그인: `rviz/` → `rviz_default_plugins/`
> - UI 패널/그룹: `rviz/` → `rviz_common/`

### Topic 구독 형식 변경

ROS2에서는 토픽 구독에 QoS 정보가 포함됩니다:

```yaml
# ===== ROS1 =====
Topic: /cloud_registered
Queue Size: 10
Unreliable: true

# ===== ROS2 =====
Topic:
  Depth: 1                        # Queue Size 대체
  Durability Policy: Volatile
  Filter size: 10
  History Policy: Keep Last
  Reliability Policy: Best Effort  # Unreliable: true 대체
  Value: /cloud_registered         # 토픽 이름은 Value 키에
```

### Tool 변경

```yaml
# ROS1: SetGoal 토픽
Topic: /move_base_simple/goal

# ROS2: Nav2에서 토픽 이름 변경
Topic:
  Value: /goal_pose
```

### 삭제할 항목

- `QMainWindow State`: ROS1 RViz의 Qt 윈도우 상태 바이너리 데이터. RViz2에서는 사용하지 않음

> **Tip:** RViz2에서 직접 설정을 변경하고 저장하면 올바른 형식의 .rviz 파일이 생성됩니다.
> 수동 변환이 어려우면 이 방법을 사용하는 것이 가장 확실합니다.

---

## 13. 자주 발생하는 빌드 에러와 해결법

### 에러 1: `'deque' does not name a type`
```
common_lib.h:46:3: error: 'deque' does not name a type
```
**원인:** ROS1 헤더가 간접적으로 포함하던 STL 헤더가 ROS2에서는 포함되지 않음
**해결:** `#include <deque>`를 명시적으로 추가

### 에러 2: `has no member named 'imu'`
```
IMU_Processing.hpp:143:14: error: 'const struct MeasureGroup' has no member named 'imu'
```
**원인:** 보통 위의 에러 1이 원인. `deque` 타입이 인식되지 않으면 해당 멤버 변수 선언 자체가 실패하여 연쇄 에러 발생
**해결:** 에러 1을 먼저 해결

### 에러 3: `parameter has not been declared`
```
rclcpp::exceptions::ParameterNotDeclaredException
```
**원인:** `get_parameter()` 전에 `declare_parameter()`를 호출하지 않음
**해결:** 반드시 declare → get 순서로 호출

### 에러 4: `ament_package() called before install()`
**원인:** CMakeLists.txt에서 `ament_package()`가 `install()` 보다 먼저 호출됨
**해결:** `ament_package()`를 파일 맨 마지막에 배치

### 에러 5: Launch 파일을 찾을 수 없음
```
Package 's_faster_lio' not found or no launch file found
```
**원인:** CMakeLists.txt에서 launch 디렉토리를 install하지 않음
**해결:** `install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})` 추가

---

## 14. 파일별 변경 요약표

| 파일 | 주요 변경 사항 |
|------|--------------|
| `package.xml` | format 3, catkin→ament_cmake, 의존성 전면 교체 |
| `CMakeLists.txt` | catkin→ament_cmake, rosidl 메시지 생성, install() 추가 |
| `config/*.yaml` (5개) | `/**:` + `ros__parameters:` 래핑 |
| `include/common_lib.h` | `#include <deque>` 추가, msg 네임스페이스, SharedPtr |
| `include/esekfom.hpp` | `rclcpp.hpp` include, RCLCPP_WARN |
| `src/preprocess.h` | `rclcpp.hpp`, msg 타입, Livox 코드 제거 |
| `src/preprocess.cpp` | msg 타입, avia_handler 제거, 기본값 OUST64 |
| `src/IMU_Processing.hpp` | msg 타입, rclcpp::Time API, RCLCPP 로깅 |
| `src/laserMapping.cpp` | 노드 객체화, declare/get 파라미터, tf2, QoS, spin_some |
| `launch/*.launch.py` (2개) | XML → Python launch 파일 신규 작성 |
| `rviz_cfg/loam_livox.rviz` | rviz_default_plugins, QoS Topic 구조 |
| `msg/Pose6D.msg` | 변경 없음 |

---

## 15. 한눈에 보는 치트시트

```
┌─────────────────────────────────────────────────────────────┐
│                  ROS1 → ROS2 Quick Reference                │
├──────────────────────┬──────────────────────────────────────┤
│ ros::init()          │ rclcpp::init()                       │
│ ros::NodeHandle      │ std::make_shared<rclcpp::Node>()     │
│ nh.param("a/b",v,d)  │ declare_parameter("a.b",d)          │
│                      │ get_parameter("a.b",v)               │
│ nh.subscribe(t,q,cb) │ create_subscription<T>(t,QoS,cb)    │
│ nh.advertise<T>(t,q) │ create_publisher<T>(t,depth)        │
│ ros::spinOnce()      │ rclcpp::spin_some(node)              │
│ ros::ok()            │ rclcpp::ok()                         │
│ ros::shutdown()      │ rclcpp::shutdown()                   │
│ ros::Time::now()     │ node->now()                          │
│ stamp.toSec()        │ rclcpp::Time(stamp).seconds()        │
│ Time().fromSec(t)    │ rclcpp::Time(int64_t(t*1e9))        │
│ ROS_WARN(...)        │ RCLCPP_WARN(logger, ...)             │
│ msg::Imu::ConstPtr   │ msg::Imu::SharedPtr                  │
│ sensor_msgs::Imu     │ sensor_msgs::msg::Imu                │
│ #include <X/Y.h>     │ #include <X/msg/y.hpp>               │
│ tf::TransformBr..    │ tf2_ros::TransformBroadcaster(node)  │
│ catkin_make          │ colcon build                          │
│ package format="1"   │ package format="3"                   │
│ $(find pkg)          │ get_package_share_directory('pkg')    │
│ rviz/PointCloud2     │ rviz_default_plugins/PointCloud2     │
│ Queue Size: 10       │ Depth: 10 + QoS policies             │
└──────────────────────┴──────────────────────────────────────┘
```
