#include <common_lib.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fstream>
#include <mutex>
#include <thread>

#include "esekfom.hpp"
#include "use-ikfom.hpp"

/*
이 hpp는 주로 다음을 포함:
IMU 데이터 전처리: IMU 초기화, IMU 전방 전파, 후방 전파로 모션 왜곡 보상
*/

#define MAX_INI_COUNT (10)  // 최대 반복 횟수
// 포인트의 시간 순서 판단 (주의: curvature에 타임스탬프가 저장됨)
const bool time_list(PointType& x, PointType& y) {
  return (x.curvature < y.curvature);
};

class ImuProcess {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void set_param(const V3D& transl,
                 const M3D& rot,
                 const V3D& gyr,
                 const V3D& acc,
                 const V3D& gyr_bias,
                 const V3D& acc_bias);
  Eigen::Matrix<double, 12, 12> Q;  // 노이즈 공분산 행렬, 논문 식(8)의 Q에 해당
  void Process(const MeasureGroup& meas,
               esekfom::esekf& kf_state,
               PointCloudXYZI::Ptr& pcl_un_);

  V3D cov_acc;              // 가속도 공분산
  V3D cov_gyr;              // 각속도 공분산
  V3D cov_acc_scale;        // 외부에서 전달된 초기 가속도 공분산
  V3D cov_gyr_scale;        // 외부에서 전달된 초기 각속도 공분산
  V3D cov_bias_gyr;         // 각속도 바이어스의 공분산
  V3D cov_bias_acc;         // 가속도 바이어스의 공분산
  double first_lidar_time;  // 현재 프레임의 첫 번째 포인트 클라우드 시간

 private:
  void IMU_init(const MeasureGroup& meas, esekfom::esekf& kf_state, int& N);
  void UndistortPcl(const MeasureGroup& meas,
                    esekfom::esekf& kf_state,
                    PointCloudXYZI& pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;     // 현재 프레임 포인트 클라우드 (왜곡 제거 전)
  sensor_msgs::ImuConstPtr last_imu_;  // 이전 프레임 imu
  vector<Pose6D> IMUpose;              // imu 자세 저장 (후방 전파용)
  M3D Lidar_R_wrt_IMU;                 // lidar에서 IMU로의 회전 외부 파라미터
  V3D Lidar_T_wrt_IMU;                 // lidar에서 IMU로의 평행 이동 외부 파라미터
  V3D mean_acc;                        // 가속도 평균값, 분산 계산용
  V3D mean_gyr;                        // 각속도 평균값, 분산 계산용
  V3D angvel_last;                     // 이전 프레임 각속도
  V3D acc_s_last;                      // 이전 프레임 가속도
  double start_timestamp_;             // 시작 타임스탬프
  double last_lidar_end_time_;         // 이전 프레임 종료 타임스탬프
  int init_iter_num = 1;               // 초기화 반복 횟수
  bool b_first_frame_ = true;          // 첫 번째 프레임 여부
  bool imu_need_init_ = true;          // imu 초기화 필요 여부
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1) {
  init_iter_num = 1;  // 초기화 반복 횟수
  Q = process_noise_cov();  // use-ikfom.hpp의 process_noise_cov를 호출하여 노이즈 공분산 초기화
  cov_acc = V3D(0.1, 0.1, 0.1);                // 가속도 공분산 초기화
  cov_gyr = V3D(0.1, 0.1, 0.1);                // 각속도 공분산 초기화
  cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);  // 각속도 바이어스 공분산 초기화
  cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);  // 가속도 바이어스 공분산 초기화
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;      // 이전 프레임 각속도 초기화
  Lidar_T_wrt_IMU = Zero3d;  // lidar에서 IMU로의 위치 외부 파라미터 초기화
  Lidar_R_wrt_IMU = Eye3d;   // lidar에서 IMU로의 회전 외부 파라미터 초기화
  last_imu_.reset(new sensor_msgs::Imu());  // 이전 프레임 imu 초기화
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()  // 파라미터 리셋
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init_ = true;                    // imu 초기화 필요 여부
  start_timestamp_ = -1;                    // 시작 타임스탬프
  init_iter_num = 1;                        // 초기화 반복 횟수
  IMUpose.clear();                          // imu 자세 클리어
  last_imu_.reset(new sensor_msgs::Imu());  // 이전 프레임 imu 초기화
  cur_pcl_un_.reset(new PointCloudXYZI());  // 현재 프레임 포인트 클라우드 (왜곡 제거 전) 초기화
}

// 외부 파라미터 전달
void ImuProcess::set_param(const V3D& transl,
                           const M3D& rot,
                           const V3D& gyr,
                           const V3D& acc,
                           const V3D& gyr_bias,
                           const V3D& acc_bias) {
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
  cov_gyr_scale = gyr;
  cov_acc_scale = acc;
  cov_bias_gyr = gyr_bias;
  cov_bias_acc = acc_bias;
}

// IMU 초기화: 시작 IMU 프레임의 평균값을 이용하여 상태량 x 초기화
void ImuProcess::IMU_init(const MeasureGroup& meas,
                          esekfom::esekf& kf_state,
                          int& N) {
  // MeasureGroup은 현재 처리 중인 모든 데이터를 나타내는 구조체로, IMU 큐와 한 프레임의 lidar 포인트 클라우드
  // 그리고 lidar의 시작 및 종료 시간을 포함. 중력, 자이로스코프 바이어스, acc 및 자이로스코프 공분산 초기화
  // 가속도 측정값을 단위 중력으로 정규화   **/
  V3D cur_acc, cur_gyr;

  if (b_first_frame_)  // 첫 번째 IMU 프레임인 경우
  {
    Reset();  // IMU 파라미터 리셋
    N = 1;    // 반복 횟수를 1로 설정
    b_first_frame_ = false;
    const auto& imu_acc =
        meas.imu.front()->linear_acceleration;  // IMU 초기 시점의 가속도
    const auto& gyr_acc =
        meas.imu.front()->angular_velocity;  // IMU 초기 시점의 각속도
    mean_acc << imu_acc.x, imu_acc.y,
        imu_acc.z;  // 첫 번째 프레임 가속도 값을 초기화 평균값으로 사용
    mean_gyr << gyr_acc.x, gyr_acc.y,
        gyr_acc.z;  // 첫 번째 프레임 각속도 값을 초기화 평균값으로 사용
    first_lidar_time =
        meas.lidar_beg_time;  // 현재 IMU 프레임에 해당하는 lidar 시작 시간을 초기 시간으로 설정
  }

  for (const auto& imu : meas.imu)  // 모든 IMU 데이터를 기반으로 평균값과 분산 계산
  {
    const auto& imu_acc = imu->linear_acceleration;
    const auto& gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc += (cur_acc - mean_acc) / N;  // 현재 프레임과 평균값의 차이를 기반으로 평균값 업데이트
    mean_gyr += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N +
              (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) / N;
    cov_gyr =
        cov_gyr * (N - 1.0) / N +
        (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) / N / N * (N - 1);

    N++;
  }

  state_ikfom init_state = kf_state.get_x();  // esekfom.hpp에서 x_의 상태 가져오기
  init_state.grav = -mean_acc / mean_acc.norm() *
                    G_m_s2;  // 평균 측정값의 단위 방향 벡터 * 중력 가속도 기본값

  init_state.bg = mean_gyr;  // 각속도 측정값을 자이로스코프 바이어스로 사용
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;  // lidar와 imu 외부 파라미터 전달
  init_state.offset_R_L_I = Sophus::SO3<double>(Lidar_R_wrt_IMU);
  kf_state.change_x(init_state);  // 초기화된 상태를 esekfom.hpp의 x_에 전달

  Matrix<double, 24, 24> init_P =
      MatrixXd::Identity(24, 24);  // esekfom.hpp에서 P_의 공분산 행렬 가져오기
  init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
  init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
  init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
  init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
  init_P(21, 21) = init_P(22, 22) = init_P(23, 23) = 0.00001;
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();

  // std::cout << "IMU init new -- init_state  " << init_state.pos  <<" " <<
  // init_state.bg <<" " << init_state.ba <<" " << init_state.grav << std::endl;
}

// 후방 전파
void ImuProcess::UndistortPcl(const MeasureGroup& meas,
                              esekfom::esekf& kf_state,
                              PointCloudXYZI& pcl_out) {
  /***이전 프레임 마지막 imu를 현재 프레임 앞쪽 imu에 추가 ***/
  auto v_imu = meas.imu;  // 현재 프레임의 IMU 큐 가져오기
  v_imu.push_front(last_imu_);  // 이전 프레임 마지막 imu를 현재 프레임 앞쪽 imu에 추가
  const double& imu_end_time =
      v_imu.back()->header.stamp.toSec();  // 현재 프레임 마지막 imu의 시간 가져오기
  const double& pcl_beg_time = meas.lidar_beg_time;  // 포인트 클라우드 시작 및 종료 타임스탬프
  const double& pcl_end_time = meas.lidar_end_time;

  // 포인트 클라우드의 각 포인트의 타임스탬프에 따라 포인트 클라우드 재정렬
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(),
       time_list);  // 여기서 curvature에 타임스탬프가 저장됨 (preprocess.cpp에서)

  state_ikfom imu_state =
      kf_state.get_x();  // 이전 KF 추정의 사후 상태를 이번 IMU 예측의 초기 상태로 사용
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel,
                               imu_state.pos, imu_state.rot.matrix()));
  // 초기 상태를 IMUpose에 추가, 시간 간격, 이전 프레임 가속도, 이전 프레임 각속도, 이전 프레임 속도, 이전 프레임 위치, 이전 프레임 회전 행렬 포함

  /*** 전방 전파 ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu,
      pos_imu;  // angvel_avr은 평균 각속도, acc_avr은 평균 가속도, acc_imu는 imu 가속도, vel_imu는 imu 속도, pos_imu는 imu 위치
  M3D R_imu;    // IMU 회전 행렬, 모션 왜곡 제거 시 사용

  double dt = 0;

  input_ikfom in;
  // 이번 추정의 모든 IMU 측정을 순회하며 적분, 이산 중간값 방법으로 전방 전파
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
    auto&& head = *(it_imu);      // 현재 프레임의 imu 데이터 가져오기
    auto&& tail = *(it_imu + 1);  // 다음 프레임의 imu 데이터 가져오기
    // 시간 순서 판단: 다음 프레임 타임스탬프가 이전 프레임 종료 타임스탬프보다 작으면 continue
    if (tail->header.stamp.toSec() < last_lidar_end_time_)
      continue;

    angvel_avr << 0.5 * (head->angular_velocity.x +
                         tail->angular_velocity.x),  // 중간값 적분
        0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
        0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr << 0.5 *
                   (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    acc_avr =
        acc_avr * G_m_s2 /
        mean_acc
            .norm();  // 중력 값을 통해 가속도 조정 (초기화된 IMU 크기*9.8로 나눔)

    // IMU 시작 시점이 이전 라이다 종료 시점보다 이르면 (이전 마지막 IMU를 이번 시작에 삽입했기 때문에 한 번 발생)
    if (head->header.stamp.toSec() < last_lidar_end_time_) {
      dt = tail->header.stamp.toSec() -
           last_lidar_end_time_;  // 이전 라이다 시점 끝에서 전파 시작
                                  // 이번 IMU 끝과의 시간 차이 계산
    } else {
      dt = tail->header.stamp.toSec() -
           head->header.stamp.toSec();  // 두 IMU 시점 간의 시간 간격
    }

    in.acc = acc_avr;  // 두 프레임 IMU의 중간값을 입력 in으로 사용, 전방 전파에 사용
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;  // 공분산 행렬 설정
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;

    kf_state.predict(dt, Q, in);  // IMU 전방 전파, 매번 전파 시간 간격은 dt

    imu_state = kf_state.get_x();  // IMU 상태를 적분 후 상태로 업데이트
    // 이전 프레임 각속도 업데이트 = 다음 프레임 각속도 - bias
    angvel_last = V3D(tail->angular_velocity.x, tail->angular_velocity.y,
                      tail->angular_velocity.z) -
                  imu_state.bg;
    // 이전 프레임 월드 좌표계 하 가속도 업데이트 = R*(가속도-bias) - g
    acc_s_last = V3D(tail->linear_acceleration.x, tail->linear_acceleration.y,
                     tail->linear_acceleration.z) *
                 G_m_s2 / mean_acc.norm();

    // std::cout << "acc_s_last: " << acc_s_last.transpose() << std::endl;
    // std::cout << "imu_state.ba: " << imu_state.ba.transpose() << std::endl;
    // std::cout << "imu_state.grav: " << imu_state.grav.transpose() <<
    // std::endl;
    acc_s_last = imu_state.rot * (acc_s_last - imu_state.ba) + imu_state.grav;
    // std::cout << "--acc_s_last: " << acc_s_last.transpose() << std::endl<<
    // std::endl;

    double&& offs_t = tail->header.stamp.toSec() -
                      pcl_beg_time;  // 다음 IMU 시점에서 이번 라이다 시작까지의 시간 간격
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel,
                                 imu_state.pos, imu_state.rot.matrix()));
  }

  // 마지막 IMU 측정도 추가
  dt = abs(pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);
  imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();  // 마지막 IMU 측정 저장, 다음 프레임에서 사용
  last_lidar_end_time_ =
      pcl_end_time;  // 이 프레임의 마지막 라이다 측정 종료 시간 저장, 다음 프레임에서 사용

  /***각 레이저 라이다 포인트의 왜곡 제거 (후방 전파)***/
  if (pcl_out.points.begin() == pcl_out.points.end())
    return;
  auto it_pcl = pcl_out.points.end() - 1;

  // 각 IMU 프레임 순회
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu << MAT_FROM_ARRAY(head->rot);  // 이전 프레임의 IMU 회전 행렬 가져오기
    // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
    vel_imu << VEC_FROM_ARRAY(head->vel);     // 이전 프레임의 IMU 속도 가져오기
    pos_imu << VEC_FROM_ARRAY(head->pos);     // 이전 프레임의 IMU 위치 가져오기
    acc_imu << VEC_FROM_ARRAY(tail->acc);     // 다음 프레임의 IMU 가속도 가져오기
    angvel_avr << VEC_FROM_ARRAY(tail->gyr);  // 다음 프레임의 IMU 각속도 가져오기

    // 이전에 포인트 클라우드를 시간 순으로 정렬했고, IMUpose도 시간 순으로 push했음
    // 이제 IMUpose의 끝에서부터 루프를 시작, 즉 시간이 가장 큰 곳부터 시작하므로
    // 포인트 클라우드 시간 > IMU head 시점만 판단하면 됨, 포인트 클라우드 시간 < IMU tail 판단 불필요
    for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
      dt = it_pcl->curvature / double(1000) -
           head->offset_time;  // 포인트에서 IMU 시작 시점까지의 시간 간격

      /*    P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei)    */

      M3D R_i(R_imu *
              Sophus::SO3<double>::exp(angvel_avr * dt)
                  .matrix());  // 포인트 it_pcl 시점의 회전: 이전 프레임의 IMU 회전 행렬
                               // * exp(다음 프레임 각속도*dt)

      V3D P_i(it_pcl->x, it_pcl->y,
              it_pcl->z);  // 포인트 시점의 위치 (라이다 좌표계)
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt -
               imu_state.pos);  // 포인트의 월드 위치 - 라이다 끝 월드 위치
      V3D P_compensate = imu_state.offset_R_L_I.matrix().transpose() *
                         (imu_state.rot.matrix().transpose() *
                              (R_i * (imu_state.offset_R_L_I.matrix() * P_i +
                                      imu_state.offset_T_L_I) +
                               T_ei) -
                          imu_state.offset_T_L_I);

      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin())
        break;
    }
  }
}

double T1, T2;
void ImuProcess::Process(const MeasureGroup& meas,
                         esekfom::esekf& kf_state,
                         PointCloudXYZI::Ptr& cur_pcl_un_) {
  // T1 = omp_get_wtime();

  if (meas.imu.empty()) {
    return;
  };
  ROS_ASSERT(meas.lidar != nullptr);

  if (imu_need_init_) {
    // 첫 번째 lidar 프레임
    IMU_init(meas, kf_state, init_iter_num);  // 처음 몇 프레임인 경우 IMU 파라미터 초기화 필요

    imu_need_init_ = true;

    last_imu_ = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();

    if (init_iter_num > MAX_INI_COUNT) {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
    }

    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  // T2 = omp_get_wtime();
  // cout<<"[ IMU Process ]: Time: "<<T2 - T1<<endl;
}
