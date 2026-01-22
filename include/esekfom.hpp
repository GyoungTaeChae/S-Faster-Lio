#ifndef ESEKFOM_EKF_HPP1
#define ESEKFOM_EKF_HPP1

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <boost/bind.hpp>
#include <cstdlib>
#include <execution>
#include <vector>

#include "use-ikfom.hpp"
#include "voxmap/voxel_map.h"

// 이 hpp는 주로 다음을 포함: 광의 가감법, 전방 전파 메인 함수, 특징점 잔차 및 야코비 계산, ESKF 메인 함수

const double epsi = 0.001;  // ESKF 반복 시, dx<epsi이면 수렴으로 간주

namespace esekfom {
using namespace Eigen;

// 특징점이 맵에서 대응하는 평면 파라미터 (평면의 단위 법향벡터 및 현재 점에서 평면까지의 거리)
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));  // 유효 특징점
PointCloudXYZI::Ptr corr_normvect(
    new PointCloudXYZI(100000, 1));      // 유효 특징점에 대응하는 법향벡터
bool point_selected_surf[100000] = {1};  // 유효 특징점인지 판단

struct dyn_share_datastruct {
  bool valid;     // 유효 특징점 수량이 요구사항을 만족하는지
  bool converge;  // 반복 시 이미 수렴했는지
  Eigen::Matrix<double, Eigen::Dynamic, 1> h;  // 잔차 (식(14)의 z)
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
      h_x;  // 야코비 행렬 H (식(14)의 H)
};

// s-fast-lio의 esekf 클래스. 원본(fast-lio와 faster-lio의 대응 구현)과의 차이점은 readme에서 언급된 Sophus와 Eigen으로 MTK를 대체한 것뿐만 아니라, 모델을 변경(중력 가속도 관련 부분 제거)한 것입니다. 또한 작성자는 원본의 다양한 함수 포인터를 제거하고, 많은 구체적인 계산 단위(예: h 계산)를 esekf로 이동했습니다. 따라서 이 클래스는 현재 fast-lio에서 제안된 구체적인 상태-측정 모델과 결합되어 있습니다
class esekf {
 public:
  typedef Matrix<double, 24, 24> cov;              // 24x24 공분산 행렬
  typedef Matrix<double, 24, 1> vectorized_state;  // 24x1 벡터

  esekf(){};
  ~esekf(){};

  state_ikfom get_x() { return x_; }

  cov get_P() { return P_; }

  void change_x(state_ikfom& input_state) { x_ = input_state; }

  void change_P(cov& input_cov) { P_ = input_cov; }

  // 광의 덧셈 식(4)
  state_ikfom boxplus(state_ikfom x, Eigen::Matrix<double, 24, 1> f_) {
    state_ikfom x_r;
    x_r.pos = x.pos + f_.block<3, 1>(0, 0);

    x_r.rot = x.rot * Sophus::SO3<double>::exp(f_.block<3, 1>(3, 0));
    x_r.offset_R_L_I =
        x.offset_R_L_I * Sophus::SO3<double>::exp(f_.block<3, 1>(6, 0));

    x_r.offset_T_L_I = x.offset_T_L_I + f_.block<3, 1>(9, 0);
    x_r.vel = x.vel + f_.block<3, 1>(12, 0);
    x_r.bg = x.bg + f_.block<3, 1>(15, 0);
    x_r.ba = x.ba + f_.block<3, 1>(18, 0);
    x_r.grav = x.grav + f_.block<3, 1>(21, 0);

    return x_r;
  }

  // 전방 전파 식(4-8)
  void predict(double& dt,
               Eigen::Matrix<double, 12, 12>& Q,
               const input_ikfom& i_in) {
    Eigen::Matrix<double, 24, 1> f_ = get_f(x_, i_in);     // 식(3)의 f
    Eigen::Matrix<double, 24, 24> f_x_ = df_dx(x_, i_in);  // 식(7)의 df/dx
    Eigen::Matrix<double, 24, 12> f_w_ = df_dw(x_, i_in);  // 식(7)의 df/dw

    x_ = boxplus(x_, f_ * dt);  // 전방 전파 식(4)

    f_x_ = Matrix<double, 24, 24>::Identity() +
           f_x_ * dt;  // 이전 Fx 행렬의 항에 단위 행렬을 더하지 않았고 dt를 곱하지 않음, 여기서 보충

    P_ =
        (f_x_)*P_ * (f_x_).transpose() +
        (dt * f_w_) * Q * (dt * f_w_).transpose();  // 공분산 행렬 전파, 즉 식(8)
  }

  // 각 특징점의 잔차 및 H 행렬 계산
  void h_share_model(dyn_share_datastruct& ekfom_data,
                     PointCloudXYZI::Ptr& feats_down_body,
                     VoxelMap<PointType>& ivox,
                     vector<PointVector>& Nearest_Points,
                     bool extrinsic_est) {
    int feats_down_size = feats_down_body->points.size();
    laserCloudOri->clear();
    corr_normvect->clear();

    std::vector<int> index(feats_down_size);
    std::iota(index.begin(), index.end(), 0);
    std::for_each(
        std::execution::par_unseq, index.begin(), index.end(),
        [&](const int& i) {
          PointType& point_body = feats_down_body->points[i];
          PointType point_world;

          V3D p_body(point_body.x, point_body.y, point_body.z);
          // Lidar 좌표계의 점을 먼저 IMU 좌표계로 변환한 후, 전방 전파로 추정된 자세 x에 따라 월드 좌표계로 변환
          V3D p_global(x_.rot * (x_.offset_R_L_I * p_body + x_.offset_T_L_I) +
                       x_.pos);
          point_world.x = p_global(0);
          point_world.y = p_global(1);
          point_world.z = p_global(2);
          point_world.intensity = point_body.intensity;

          vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
          // Nearest_Points[i]를 출력해보니 point_world까지의 거리 기준으로 작은 것부터 큰 순서로 정렬된 vector임을 발견
          auto& points_near = Nearest_Points[i];

          if (ekfom_data.converge) {
            // point_world의 최근접 평면점 찾기
            ivox.GetClosestPoint(point_world, points_near, NUM_MATCH_POINTS);

            // 유효한 매칭점인지 판단, loam 시리즈와 유사하게 특징점 최근접 맵 포인트 수량>임계값, 거리<임계값 요구
            // 조건을 만족하는 경우만 true로 설정
            point_selected_surf[i] =
                points_near.size() < NUM_MATCH_POINTS        ? false
                : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                             : true;
          }
          if (!point_selected_surf[i])
            return;  // 해당 점이 조건을 만족하지 않으면 다음 단계를 진행하지 않음

          Matrix<float, 4, 1> pabcd;  // 평면점 정보
          // 해당 점을 무효점으로 설정하여 조건 만족 여부 판단
          point_selected_surf[i] = false;
          // 평면 방정식 ax+by+cz+d=0 피팅 및 점에서 평면까지 거리 계산
          if (esti_plane(pabcd, points_near, 0.1f)) {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                        pabcd(2) * point_world.z +
                        pabcd(3);  // 현재 점에서 평면까지의 거리

            // 잔차가 경험적 임계값보다 크면 해당 점을 유효점으로 간주, 즉 원점에 가까운 lidar 점일수록 점에서 평면까지 거리에 대한 요구사항이 엄격함
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
            // 잔차가 임계값보다 크면 해당 점을 유효점으로 간주
            if (s > 0.9) {
              point_selected_surf[i] = true;
              // 평면의 단위 법향벡터 및 현재 점에서 평면까지 거리 저장
              normvec->points[i].x = pabcd(0);
              normvec->points[i].y = pabcd(1);
              normvec->points[i].z = pabcd(2);
              normvec->points[i].intensity = pd2;
            }
          }
        });

    int effct_feat_num = 0;  // 유효 특징점의 수량
    for (int i = 0; i < feats_down_size; i++) {
      if (point_selected_surf[i])  // 요구사항을 만족하는 점에 대해
      {
        laserCloudOri->points[effct_feat_num] =
            feats_down_body->points[i];  // 이 점들을 laserCloudOri에 다시 저장
        corr_normvect->points[effct_feat_num] =
            normvec->points[i];  // 이 점들에 대응하는 법향벡터와 평면까지의 거리 저장
        effct_feat_num++;
      }
    }

    if (effct_feat_num < 1) {
      ekfom_data.valid = false;
      ROS_WARN("No Effective Points! \n");
      return;
    }

    // 야코비 행렬 H와 잔차 벡터 계산
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++) {
      V3D point_(laserCloudOri->points[i].x, laserCloudOri->points[i].y,
                 laserCloudOri->points[i].z);
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_);
      V3D point_I_ = x_.offset_R_L_I * point_ + x_.offset_T_L_I;
      M3D point_I_crossmat;
      point_I_crossmat << SKEW_SYM_MATRX(point_I_);

      // 대응하는 평면의 법향벡터 획득
      const PointType& norm_p = corr_normvect->points[i];
      V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

      // 야코비 행렬 H 계산
      V3D C(x_.rot.matrix().transpose() * norm_vec);
      V3D A(point_I_crossmat * C);
      if (extrinsic_est) {
        V3D B(point_crossmat * x_.offset_R_L_I.matrix().transpose() * C);
        ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
            VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
      } else {
        ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
            VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      }

      // 잔차: 점-평면 거리
      ekfom_data.h(i) = -norm_p.intensity;
    }
  }

  // 광의 뺄셈
  vectorized_state boxminus(state_ikfom x1, state_ikfom x2) {
    vectorized_state x_r = vectorized_state::Zero();

    x_r.block<3, 1>(0, 0) = x1.pos - x2.pos;

    x_r.block<3, 1>(3, 0) =
        Sophus::SO3<double>(x2.rot.matrix().transpose() * x1.rot.matrix())
            .log();
    x_r.block<3, 1>(6, 0) =
        Sophus::SO3<double>(x2.offset_R_L_I.matrix().transpose() *
                            x1.offset_R_L_I.matrix())
            .log();

    x_r.block<3, 1>(9, 0) = x1.offset_T_L_I - x2.offset_T_L_I;
    x_r.block<3, 1>(12, 0) = x1.vel - x2.vel;
    x_r.block<3, 1>(15, 0) = x1.bg - x2.bg;
    x_r.block<3, 1>(18, 0) = x1.ba - x2.ba;
    x_r.block<3, 1>(21, 0) = x1.grav - x2.grav;

    return x_r;
  }

  // ESKF
  void update_iterated_dyn_share_modified(double R,
                                          PointCloudXYZI::Ptr& feats_down_body,
                                          VoxelMap<PointType>& ivox,
                                          vector<PointVector>& Nearest_Points,
                                          int maximum_iter,
                                          bool extrinsic_est) {
    normvec->resize(int(feats_down_body->points.size()));

    dyn_share_datastruct dyn_share;
    dyn_share.valid = true;
    dyn_share.converge = true;
    int t = 0;
    // 여기서 x_와 P_는 각각 전방 전파 후의 상태량과 공분산 행렬, predict 함수를 먼저 호출한 후 이 함수를 호출하기 때문
    state_ikfom x_propagated = x_;
    cov P_propagated = P_;

    vectorized_state dx_new = vectorized_state::Zero();  // 24x1 벡터

    // maximum_iter는 칼만 필터의 최대 반복 횟수
    for (int i = -1; i < maximum_iter; i++) {
      dyn_share.valid = true;
      // 야코비 계산, 즉 점-평면 잔차의 도함수 H (코드에서는 h_x)
      h_share_model(dyn_share, feats_down_body, ivox, Nearest_Points,
                    extrinsic_est);

      if (!dyn_share.valid) {
        continue;
      }

      vectorized_state dx;
      dx_new = boxminus(x_, x_propagated);  // 식(18)의 x^k - x^

      // H 행렬은 희소 행렬로 처음 12열만 0이 아닌 원소를 가지며, 나머지 12열은 0
      // 따라서 여기서는 블록 행렬 형태로 계산하여 계산량 감소
      auto H = dyn_share.h_x;  // m x 12 행렬
      // 행렬 H^T * H
      Eigen::Matrix<double, 24, 24> HTH = Matrix<double, 24, 24>::Zero();
      HTH.block<12, 12>(0, 0) = H.transpose() * H;

      auto K_front = (HTH / R + P_.inverse()).inverse();
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;
      // 칼만 게인, 여기서 R은 상수로 간주
      K = K_front.block<24, 12>(0, 0) * H.transpose() / R;

      // 행렬 K * H
      Eigen::Matrix<double, 24, 24> KH = Matrix<double, 24, 24>::Zero();
      KH.block<24, 12>(0, 0) = K * H;
      // 식(18)
      Matrix<double, 24, 1> dx_ =
          K * dyn_share.h + (KH - Matrix<double, 24, 24>::Identity()) * dx_new;
      x_ = boxplus(x_, dx_);  // 식(18)

      dyn_share.converge = true;
      for (int j = 0; j < 24; j++) {
        // dx>epsi이면 수렴하지 않은 것으로 간주
        if (std::fabs(dx_[j]) > epsi) {
          dyn_share.converge = false;
          break;
        }
      }

      if (dyn_share.converge)
        t++;

      // 3번 반복해도 수렴하지 않으면 강제로 true로 설정, h_share_model 함수에서 근접점을 다시 찾음
      if (!t && i == maximum_iter - 2) {
        dyn_share.converge = true;
      }

      if (t > 1 || i == maximum_iter - 1) {
        P_ = (Matrix<double, 24, 24>::Identity() - KH) * P_;  // 식(19)
        return;
      }
    }
  }

 private:
  state_ikfom x_;
  cov P_ = cov::Identity();
};

}  // namespace esekfom

#endif  //  ESEKFOM_EKF_HPP1
