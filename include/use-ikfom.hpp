#ifndef USE_IKFOM_H1
#define USE_IKFOM_H1

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <boost/bind.hpp>
#include <cstdlib>
#include <vector>

#include "common_lib.h"
#include "sophus/so3.hpp"

// 이 hpp는 주로 다음을 포함: 상태 변수 x, 입력 u의 정의, 그리고 전방 전파의 관련 행렬 함수

// 24차원의 상태량 x
struct state_ikfom {
  Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);
  Sophus::SO3<double> rot = Sophus::SO3<double>(Eigen::Matrix3d::Identity());
  Sophus::SO3<double> offset_R_L_I =
      Sophus::SO3<double>(Eigen::Matrix3d::Identity());
  Eigen::Vector3d offset_T_L_I = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d bg = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d ba = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d grav = Eigen::Vector3d(0, 0, -G_m_s2);
};

// 입력 u
struct input_ikfom {
  Eigen::Vector3d acc = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d gyro = Eigen::Vector3d(0, 0, 0);
};

// 노이즈 공분산 Q의 초기화 (논문 식(8)의 Q에 해당, IMU_Processing.hpp에서 사용)
Eigen::Matrix<double, 12, 12> process_noise_cov() {
  Eigen::Matrix<double, 12, 12> Q = Eigen::MatrixXd::Zero(12, 12);
  Q.block<3, 3>(0, 0) = 0.0001 * Eigen::Matrix3d::Identity();
  Q.block<3, 3>(3, 3) = 0.0001 * Eigen::Matrix3d::Identity();
  Q.block<3, 3>(6, 6) = 0.00001 * Eigen::Matrix3d::Identity();
  Q.block<3, 3>(9, 9) = 0.00001 * Eigen::Matrix3d::Identity();

  return Q;
}

// 논문 식(2)의 f에 해당
Eigen::Matrix<double, 24, 1> get_f(state_ikfom s, input_ikfom in) {
  // 해당 순서는 속도(3), 각속도(3), 외부파라미터 T(3), 외부파라미터 회전 R(3), 가속도(3), 각속도 바이어스(3), 가속도 바이어스(3), 위치(3), 논문 공식 순서와 일치하지 않음
  Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
  Eigen::Vector3d omega = in.gyro - s.bg;  // 입력된 imu의 각속도 (즉, 실제 측정값)
                                           // - 추정된 bias 값 (식의 1행에 해당)
  Eigen::Vector3d a_inertial =
      s.rot.matrix() *
      (in.acc -
       s.ba);  //  입력된 imu의 가속도, 먼저 월드 좌표계로 변환 (식의 3행에 해당)

  for (int i = 0; i < 3; i++) {
    res(i) = s.vel[i];      // 속도 (식의 2행에 해당)
    res(i + 3) = omega[i];  // 각속도 (식의 1행에 해당)
    res(i + 12) = a_inertial[i] + s.grav[i];  // 가속도 (식의 3행에 해당)
  }

  return res;
}

// 논문 식(7)의 Fx에 해당, 주의: 이 행렬은 dt를 곱하지 않았고, 단위 행렬도 더하지 않음
Eigen::Matrix<double, 24, 24> df_dx(state_ikfom s, input_ikfom in) {
  Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
  cov.block<3, 3>(0, 12) =
      Eigen::Matrix3d::Identity();       // 식(7)의 2행 3열에 해당, I
  Eigen::Vector3d acc_ = in.acc - s.ba;  // 측정 가속도 = a_m - bias

  cov.block<3, 3>(12, 3) =
      -s.rot.matrix() *
      Sophus::SO3<double>::hat(acc_);         // 식(7)의 3행 1열에 해당
  cov.block<3, 3>(12, 18) = -s.rot.matrix();  // 식(7)의 3행 5열에 해당

  cov.template block<3, 3>(12, 21) =
      Eigen::Matrix3d::Identity();  // 식(7)의 3행 6열에 해당, I
  cov.template block<3, 3>(3, 15) =
      -Eigen::Matrix3d::Identity();  // 식(7)의 1행 4열에 해당 (-I로 간소화)
  return cov;
}

// 논문 식(7)의 Fw에 해당, 주의: 이 행렬은 dt를 곱하지 않음
Eigen::Matrix<double, 24, 12> df_dw(state_ikfom s, input_ikfom in) {
  Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
  cov.block<3, 3>(12, 3) = -s.rot.matrix();  // 식(7)의 3행 2열에 해당, -R
  cov.block<3, 3>(3, 0) =
      -Eigen::Matrix3d::Identity();  // 식(7)의 1행 1열에 해당, -A(w dt)를 -I로 간소화
  cov.block<3, 3>(15, 6) =
      Eigen::Matrix3d::Identity();  // 식(7)의 4행 3열에 해당, I
  cov.block<3, 3>(18, 9) =
      Eigen::Matrix3d::Identity();  // 식(7)의 5행 4열에 해당, I
  return cov;
}

#endif