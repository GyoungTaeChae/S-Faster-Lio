# Eigen 문법 정리

S-Faster-Lio 프로젝트의 src와 include 폴더에서 사용되는 Eigen 라이브러리 문법을 정리한 문서입니다.

## 목차
1. [기본 헤더 및 타입](#1-기본-헤더-및-타입)
2. [벡터 타입](#2-벡터-타입)
3. [행렬 타입](#3-행렬-타입)
4. [쿼터니언](#4-쿼터니언)
5. [초기화 및 생성](#5-초기화-및-생성)
6. [기본 연산](#6-기본-연산)
7. [블록 연산](#7-블록-연산)
8. [선형 대수 연산](#8-선형-대수-연산)
9. [메모리 정렬](#9-메모리-정렬)
10. [Sophus와의 통합 (Lie Group)](#10-sophus와의-통합-lie-group)

---

## 1. 기본 헤더 및 타입

### 헤더 파일 포함
```cpp
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
```

### 타입 별칭 정의
```cpp
typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;
```

**사용 위치:**
- `include/common_lib.h:29-32`
- `include/use-ikfom.hpp:4-7`
- `include/esekfom.hpp:4-7`

---

## 2. 벡터 타입

### 3차원 벡터
```cpp
Eigen::Vector3d vec_d;    // double 타입 3D 벡터
Eigen::Vector3f vec_f;    // float 타입 3D 벡터
```

### 벡터 초기화
```cpp
// 직접 초기화
Eigen::Vector3d pos(0, 0, 0);
V3D Lidar_T_wrt_IMU(Zero3d);

// 값 설정
V3D cur_acc, cur_gyr;
cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

// 인덱스 접근
vec_d(0) = 1.0;  // x 좌표
vec_d(1) = 2.0;  // y 좌표
vec_d(2) = 3.0;  // z 좌표
```

### 동적 벡터
```cpp
Eigen::Matrix<double, Eigen::Dynamic, 1> dynamic_vec;  // 크기가 동적으로 변하는 벡터
dynamic_vec.resize(size);  // 크기 조정
```

**사용 위치:**
- `src/laserMapping.cpp:83, 92, 247` (V3D 사용)
- `src/IMU_Processing.hpp:142, 165-166` (Vector3d 초기화)
- `include/esekfom.hpp:33` (Dynamic 벡터)

---

## 3. 행렬 타입

### 3x3 행렬
```cpp
Eigen::Matrix3d mat_d;    // double 타입 3x3 행렬
Eigen::Matrix3f mat_f;    // float 타입 3x3 행렬
M3D Lidar_R_wrt_IMU;      // 별칭 사용
```

### 동적 크기 행렬
```cpp
Eigen::MatrixXd mat;      // 동적 크기의 double 행렬
mat.resize(rows, cols);   // 크기 조정
```

### 고정 크기 행렬
```cpp
Eigen::Matrix<double, 24, 24> cov;        // 24x24 행렬
Eigen::Matrix<double, 24, 1> vec_state;   // 24x1 열벡터
Eigen::Matrix<double, 12, 12> Q;          // 12x12 행렬
```

### 행렬 초기화
```cpp
M3D Eye3d = M3D::Identity();    // 단위 행렬
Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(rows, cols);   // 영 행렬
```

**사용 위치:**
- `src/laserMapping.cpp:84` (M3D 사용)
- `include/common_lib.h:34-37` (Identity 행렬)
- `include/use-ikfom.hpp:37-43` (고정 크기 행렬)
- `include/esekfom.hpp:41-42, 178` (동적/고정 행렬)

---

## 4. 쿼터니언

### 쿼터니언 생성
```cpp
Eigen::Quaterniond q(rotation_matrix);  // 회전 행렬로부터 생성
auto q = Eigen::Quaterniond(state_point.rot.matrix());
```

### 쿼터니언 계수 접근
```cpp
out.pose.orientation.x = q.coeffs()[0];
out.pose.orientation.y = q.coeffs()[1];
out.pose.orientation.z = q.coeffs()[2];
out.pose.orientation.w = q.coeffs()[3];
```

**사용 위치:**
- `src/laserMapping.cpp:424-428`

---

## 5. 초기화 및 생성

### 영벡터/영행렬
```cpp
vec.setZero();                           // 벡터를 0으로 설정
mat.setZero();                           // 행렬을 0으로 설정
Eigen::Matrix3d::Zero();                 // 영 행렬 생성
Eigen::MatrixXd::Zero(rows, cols);       // 동적 크기 영 행렬
```

### 단위 벡터/단위 행렬
```cpp
vec.setOnes();                           // 벡터를 1로 설정
Eigen::Matrix3d::Identity();             // 3x3 단위 행렬
Matrix<double, 24, 24>::Identity();      // 24x24 단위 행렬
```

### 상수로 초기화
```cpp
b.setOnes();
b *= -1.0f;  // 모든 원소를 -1로 설정
```

**사용 위치:**
- `src/IMU_Processing.hpp:98, 99, 111` (Zero3d 사용)
- `src/preprocess.cpp:484, 803, 816, 854, 874` (setZero)
- `include/common_lib.h:83-84` (setZero, setOnes)
- `include/esekfom.hpp:83, 213` (Identity)

---

## 6. 기본 연산

### 노름 (Norm)
```cpp
double length = vec.norm();              // 벡터 길이 (L2 노름)
vec.normalize();                         // 벡터 정규화 (길이를 1로)
V3D normalized = vec.normalized();       // 정규화된 벡터 반환 (원본 유지)
```

### 전치 (Transpose)
```cpp
Eigen::Matrix3d mat_t = mat.transpose();
```

### 내적 (Dot Product)
```cpp
double dot_result = vec1.dot(vec2);
types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
```

### Element-wise 연산
```cpp
// Element-wise 곱셈
cov_acc = cov_acc * (N - 1.0) / N +
          (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) / N;
```

### 행렬-벡터 곱
```cpp
V3D result = matrix * vector;
V3D p_global(state_point.rot.matrix() *
             (state_point.offset_R_L_I.matrix() * p_body +
              state_point.offset_T_L_I) +
             state_point.pos);
```

**사용 위치:**
- `src/laserMapping.cpp:248, 262` (matrix 곱)
- `src/preprocess.cpp:147, 241, 519, 900` (norm)
- `src/preprocess.cpp:632` (dot)
- `src/IMU_Processing.hpp:172, 174` (cwiseProduct)
- `include/common_lib.h:96` (norm)
- `include/esekfom.hpp:195` (transpose)

---

## 7. 블록 연산

### 블록 접근
```cpp
// 읽기
Eigen::Vector3d sub_vec = vec.block<3, 1>(0, 0);
Eigen::Matrix3d sub_mat = mat.block<3, 3>(0, 0);

// 쓰기
mat.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
Q.block<3, 3>(0, 0).diagonal() = cov_gyr;

// 대각선 원소 접근
Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
```

### 동적 크기 블록
```cpp
ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
    VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
```

**사용 위치:**
- `src/laserMapping.cpp:439-446` (블록 접근)
- `src/IMU_Processing.hpp:269-272` (블록 diagonal)
- `include/use-ikfom.hpp:39-42, 58-68` (블록 연산)
- `include/esekfom.hpp:58-68, 215-229, 272` (다양한 블록 연산)

---

## 8. 선형 대수 연산

### 선형 시스템 풀이 (Ax = b)
```cpp
// QR 분해를 이용한 최소제곱 해법
Eigen::Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
```

### 역행렬
```cpp
auto K_front = (HTH / R + P_.inverse()).inverse();
```

**사용 위치:**
- `include/common_lib.h:94` (colPivHouseholderQr)
- `include/esekfom.hpp:274` (inverse)

---

## 9. 메모리 정렬

### 클래스에서 메모리 정렬 보장
```cpp
class ImuProcess {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // ...
};
```

### STL 컨테이너에서 메모리 정렬
```cpp
typedef vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;

using PointVector =
    std::vector<PointType, Eigen::aligned_allocator<PointType>>;
```

**사용 위치:**
- `src/IMU_Processing.hpp:41` (EIGEN_MAKE_ALIGNED_OPERATOR_NEW)
- `include/common_lib.h:28` (aligned_allocator)
- `include/voxmap/voxel_map.h:17-18` (aligned_allocator)

---

## 10. Sophus와의 통합 (Lie Group)

Sophus는 Lie 군(Group)을 다루는 라이브러리로, Eigen과 함께 사용되어 회전을 표현합니다.

### SO(3) 회전 그룹
```cpp
#include "sophus/so3.hpp"

// SO(3) 타입 정의
Sophus::SO3<double> rot;

// 회전 행렬로부터 생성
Sophus::SO3<double> rot(rotation_matrix);

// 단위 행렬로 초기화
Sophus::SO3<double> rot = Sophus::SO3<double>(Eigen::Matrix3d::Identity());
```

### 행렬로 변환
```cpp
Eigen::Matrix3d R = rot.matrix();
```

### 지수 맵 (Exponential Map)
```cpp
// 축-각 표현에서 회전으로 변환
Sophus::SO3<double> rot = Sophus::SO3<double>::exp(omega_vec);

// 예제: 각속도를 이용한 회전 업데이트
x_r.rot = x.rot * Sophus::SO3<double>::exp(f_.block<3, 1>(3, 0));
```

### 로그 맵 (Logarithm Map)
```cpp
// 회전을 축-각 표현으로 변환
Eigen::Vector3d omega = rot.log();

// 예제: 두 회전 간의 차이
x_r.block<3, 1>(3, 0) =
    Sophus::SO3<double>(x2.rot.matrix().transpose() * x1.rot.matrix()).log();
```

### Hat 연산자 (반대칭 행렬)
```cpp
// 벡터를 반대칭 행렬로 변환
Eigen::Matrix3d skew_mat = Sophus::SO3<double>::hat(vector);

// 예제: 야코비 계산에서 사용
cov.block<3, 3>(12, 3) = -s.rot.matrix() * Sophus::SO3<double>::hat(acc_);
```

**사용 위치:**
- `include/use-ikfom.hpp:13, 20-22, 186` (SO3 정의 및 초기화)
- `include/use-ikfom.hpp:76` (hat 연산자)
- `include/esekfom.hpp:60-62` (exp 사용)
- `include/esekfom.hpp:217-222` (log 사용)
- `src/IMU_Processing.hpp:334` (exp 사용)

---

## 프로젝트 특화 매크로

### 벡터/행렬 변환 매크로
```cpp
// common_lib.h에 정의된 매크로
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

// 사용 예제
Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
M3D point_crossmat;
point_crossmat << SKEW_SYM_MATRX(point_);
```

**사용 위치:**
- `include/common_lib.h:20-22`
- `src/laserMapping.cpp:585-586`
- `src/IMU_Processing.hpp:317, 319-322`
- `include/esekfom.hpp:185, 187`

---

## 상태 벡터 구조체 예제

프로젝트에서 사용하는 24차원 상태 벡터:

```cpp
struct state_ikfom {
  Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);                    // 위치 (3)
  Sophus::SO3<double> rot = Sophus::SO3<double>(Eigen::Matrix3d::Identity());  // 회전 (3)
  Sophus::SO3<double> offset_R_L_I =                                 // LiDAR-IMU 회전 (3)
      Sophus::SO3<double>(Eigen::Matrix3d::Identity());
  Eigen::Vector3d offset_T_L_I = Eigen::Vector3d(0, 0, 0);          // LiDAR-IMU 변환 (3)
  Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);                    // 속도 (3)
  Eigen::Vector3d bg = Eigen::Vector3d(0, 0, 0);                     // 자이로 바이어스 (3)
  Eigen::Vector3d ba = Eigen::Vector3d(0, 0, 0);                     // 가속도 바이어스 (3)
  Eigen::Vector3d grav = Eigen::Vector3d(0, 0, -G_m_s2);            // 중력 (3)
};
```

**사용 위치:**
- `include/use-ikfom.hpp:18-28`

---

## 주요 사용 패턴

### 1. 좌표 변환
```cpp
// LiDAR 좌표계 → IMU 좌표계 → 월드 좌표계
V3D p_body(pi->x, pi->y, pi->z);
V3D p_global(state_point.rot.matrix() *
             (state_point.offset_R_L_I.matrix() * p_body +
              state_point.offset_T_L_I) +
             state_point.pos);
```

### 2. 공분산 업데이트
```cpp
P_ = (f_x_) * P_ * (f_x_).transpose() +
     (dt * f_w_) * Q * (dt * f_w_).transpose();
```

### 3. 칼만 게인 계산
```cpp
auto K_front = (HTH / R + P_.inverse()).inverse();
K = K_front.block<24, 12>(0, 0) * H.transpose() / R;
```

**사용 위치:**
- `src/laserMapping.cpp:246-265` (좌표 변환)
- `include/esekfom.hpp:86-88` (공분산)
- `include/esekfom.hpp:274-277` (칼만 게인)

---

## 참고사항

1. **고정 크기 vs 동적 크기**:
   - 성능이 중요한 경우 고정 크기 행렬/벡터 사용 (`Matrix3d`, `Vector3d`)
   - 크기가 런타임에 결정되는 경우 동적 크기 사용 (`MatrixXd`, `VectorXd`)

2. **메모리 정렬**:
   - SIMD 최적화를 위해 Eigen 타입을 클래스 멤버로 사용할 때는 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 필요
   - STL 컨테이너에서는 `Eigen::aligned_allocator` 사용

3. **Sophus 통합**:
   - 회전 표현 시 Lie 군 이론을 활용하여 수치적 안정성 향상
   - exp/log 맵을 통해 탄젠트 공간과 매니폴드 간 변환

4. **블록 연산 최적화**:
   - 큰 행렬의 일부만 수정할 때 블록 연산 사용으로 성능 향상
   - 희소 행렬 구조 활용 (예: 야코비 행렬의 처음 12열만 사용)
