# 칼만 필터 (Kalman Filter) 정리

S-Faster-Lio 프로젝트에서 사용되는 Error State Kalman Filter (ESKF)의 구조와 각 요소의 동역학을 상세히 정리한 문서입니다.

## 목차
1. [ESKF 개요](#1-eskf-개요)
2. [State Vector (상태 벡터)](#2-state-vector-상태-벡터)
3. [Input Vector (입력 벡터)](#3-input-vector-입력-벡터)
4. [Process Noise (프로세스 노이즈)](#4-process-noise-프로세스-노이즈)
5. [State Transition Matrix F](#5-state-transition-matrix-f)
6. [Input Jacobian Matrix Fw](#6-input-jacobian-matrix-fw)
7. [전방 전파 (Prediction)](#7-전방-전파-prediction)
8. [측정 업데이트 (Update)](#8-측정-업데이트-update)
9. [광의 가감법 (Manifold Operations)](#9-광의-가감법-manifold-operations)
10. [전체 프로세스](#10-전체-프로세스)

---

## 1. ESKF 개요

### Error State Kalman Filter란?

ESKF는 일반적인 칼만 필터와 달리 **에러 상태 (error state)**를 추정하는 방식입니다:
- **Nominal State**: 시스템의 주요 상태 (위치, 속도, 회전 등)
- **Error State**: Nominal state와 실제 상태 간의 차이 (작은 값으로 가정)

**장점:**
1. 회전을 다룰 때 특이점(singularity) 문제가 없음
2. Error state는 항상 0 근처의 작은 값이므로 선형화가 정확함
3. 공분산 행렬이 작게 유지되어 수치적 안정성이 좋음

**사용 위치:**
- `include/esekfom.hpp` - ESKF 메인 클래스 구현
- `include/use-ikfom.hpp` - 상태 벡터 및 동역학 함수 정의
- `src/IMU_Processing.hpp` - IMU 전방 전파 및 모션 왜곡 보상

---

## 2. State Vector (상태 벡터)

ESKF의 상태 벡터는 **24차원**으로 구성됩니다:

```cpp
struct state_ikfom {
  Eigen::Vector3d pos;                    // [0-2]   위치 (position)
  Sophus::SO3<double> rot;                // [3-5]   회전 (rotation, SO(3))
  Sophus::SO3<double> offset_R_L_I;       // [6-8]   LiDAR-IMU 외부 파라미터 회전
  Eigen::Vector3d offset_T_L_I;           // [9-11]  LiDAR-IMU 외부 파라미터 평행이동
  Eigen::Vector3d vel;                    // [12-14] 속도 (velocity)
  Eigen::Vector3d bg;                     // [15-17] 각속도 바이어스 (gyro bias)
  Eigen::Vector3d ba;                     // [18-20] 가속도 바이어스 (accel bias)
  Eigen::Vector3d grav;                   // [21-23] 중력 벡터 (gravity)
};
```

### 각 요소의 물리적 의미

| 인덱스 | 변수명 | 차원 | 물리적 의미 | 초기값 |
|--------|--------|------|------------|--------|
| 0-2 | pos | 3 | 월드 좌표계에서의 IMU 위치 | (0, 0, 0) |
| 3-5 | rot | 3 | 월드 좌표계에서의 IMU 회전 (Lie 대수 표현) | Identity |
| 6-8 | offset_R_L_I | 3 | LiDAR에서 IMU로의 회전 외부 파라미터 | Identity |
| 9-11 | offset_T_L_I | 3 | LiDAR에서 IMU로의 평행이동 외부 파라미터 | (0, 0, 0) |
| 12-14 | vel | 3 | 월드 좌표계에서의 IMU 속도 | (0, 0, 0) |
| 15-17 | bg | 3 | 자이로스코프 측정 바이어스 | 초기화 시 계산 |
| 18-20 | ba | 3 | 가속도계 측정 바이어스 | (0, 0, 0) |
| 21-23 | grav | 3 | 월드 좌표계에서의 중력 벡터 | (0, 0, -9.81) |

**사용 위치:**
- `include/use-ikfom.hpp:18-28` (state_ikfom 구조체 정의)

---

## 3. Input Vector (입력 벡터)

IMU에서 측정되는 **6차원** 입력 벡터:

```cpp
struct input_ikfom {
  Eigen::Vector3d acc;   // [0-2] 가속도 측정값 (m/s²)
  Eigen::Vector3d gyro;  // [3-5] 각속도 측정값 (rad/s)
};
```

### 물리적 의미

- **acc**: IMU 프레임에서 측정된 비력(specific force)
  - 실제 가속도 + 중력 - 바이어스
  - 단위: m/s²

- **gyro**: IMU 프레임에서 측정된 각속도
  - 실제 각속도 + 바이어스
  - 단위: rad/s

**사용 위치:**
- `include/use-ikfom.hpp:31-34` (input_ikfom 구조체 정의)
- `src/IMU_Processing.hpp:267-268` (중간값 적분을 통한 입력 생성)

---

## 4. Process Noise (프로세스 노이즈)

프로세스 노이즈 공분산 행렬 Q는 **12x12** 대각 행렬입니다:

```cpp
Eigen::Matrix<double, 12, 12> process_noise_cov() {
  Eigen::Matrix<double, 12, 12> Q = Eigen::MatrixXd::Zero(12, 12);
  Q.block<3, 3>(0, 0) = 0.0001 * Eigen::Matrix3d::Identity();  // 각속도 노이즈
  Q.block<3, 3>(3, 3) = 0.0001 * Eigen::Matrix3d::Identity();  // 가속도 노이즈
  Q.block<3, 3>(6, 6) = 0.00001 * Eigen::Matrix3d::Identity(); // 각속도 바이어스 노이즈
  Q.block<3, 3>(9, 9) = 0.00001 * Eigen::Matrix3d::Identity(); // 가속도 바이어스 노이즈
  return Q;
}
```

### Q 행렬 구조

```
Q = [σ_w²·I₃    0       0       0    ]  12x12
    [  0     σ_a²·I₃    0       0    ]
    [  0        0    σ_bw²·I₃   0    ]
    [  0        0       0    σ_ba²·I₃]
```

| 블록 | 크기 | 의미 | 기본값 |
|------|------|------|--------|
| Q[0:3, 0:3] | 3×3 | 각속도 측정 노이즈 공분산 | 0.0001·I |
| Q[3:6, 3:6] | 3×3 | 가속도 측정 노이즈 공분산 | 0.0001·I |
| Q[6:9, 6:9] | 3×3 | 각속도 바이어스 랜덤 워크 | 0.00001·I |
| Q[9:12, 9:12] | 3×3 | 가속도 바이어스 랜덤 워크 | 0.00001·I |

**물리적 의미:**
- **각속도 노이즈**: 자이로스코프의 화이트 노이즈
- **가속도 노이즈**: 가속도계의 화이트 노이즈
- **바이어스 노이즈**: 바이어스의 시간에 따른 변화 (랜덤 워크)

**사용 위치:**
- `include/use-ikfom.hpp:37-45` (process_noise_cov 함수)
- `src/IMU_Processing.hpp:91` (초기화)
- `src/IMU_Processing.hpp:269-272` (매 예측 단계마다 업데이트)

---

## 5. State Transition Matrix F

State Transition Matrix **F**는 시스템 동역학을 나타내는 **24×24** 야코비 행렬입니다.

### 시스템 동역학 방정식

논문 식(2)의 연속 시간 동역학:

```
ṗ = v                                    (위치의 시간 변화율 = 속도)
Ṙ = R · [ω - bg]×                        (회전의 시간 변화율)
v̇ = R(a - ba) + g                       (속도의 시간 변화율)
ḃg = nw                                  (각속도 바이어스 변화)
ḃa = na                                  (가속도 바이어스 변화)
```

여기서:
- `[·]×`는 skew-symmetric matrix (외적 연산자)
- `ω` = 각속도 측정값 - bg
- `a` = 가속도 측정값

### f 벡터 (상태 변화율 함수)

**f의 정의:**
```
f(x, u) = ẋ = 상태의 시간 변화율
```

f는 현재 상태 x와 입력 u가 주어졌을 때, **상태가 시간에 따라 어떻게 변하는지**를 계산하는 함수야.

**f 벡터의 구조 (24차원):**

| 인덱스 | f의 요소 | 동역학 | 코드에서의 값 |
|--------|----------|--------|---------------|
| 0-2 | ṗ | dp/dt = v | `s.vel` |
| 3-5 | Ṙ 관련 | dR/dt ∝ ω - bg | `in.gyro - s.bg` |
| 6-8 | offset_Ṙ_L_I | 0 (상수) | 0 |
| 9-11 | offset_Ṫ_L_I | 0 (상수) | 0 |
| 12-14 | v̇ | dv/dt = R(a-ba) + g | `s.rot * (in.acc - s.ba) + s.grav` |
| 15-17 | ḃg | 0 (+ noise) | 0 |
| 18-20 | ḃa | 0 (+ noise) | 0 |
| 21-23 | ġ | 0 (상수) | 0 |

**get_f() 함수 구현:**

```cpp
Eigen::Matrix<double, 24, 1> get_f(state_ikfom s, input_ikfom in) {
  Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();

  Eigen::Vector3d omega = in.gyro - s.bg;                      // 보정된 각속도
  Eigen::Vector3d a_inertial = s.rot.matrix() * (in.acc - s.ba);  // 월드 좌표계 가속도

  for (int i = 0; i < 3; i++) {
    res(i) = s.vel[i];                          // f[0:3] = v        (ṗ = v)
    res(i + 3) = omega[i];                      // f[3:6] = ω - bg   (Ṙ 관련)
    res(i + 12) = a_inertial[i] + s.grav[i];    // f[12:15] = R(a-ba) + g  (v̇)
  }
  // f[6:12], f[15:24]는 0으로 유지 (외부 파라미터, 바이어스, 중력은 변하지 않음)

  return res;
}
```

**중요한 점:**
- f[3:6]은 `ω - bg`로 설정됨 (회전의 시간 변화율을 각속도 벡터로 표현)
- 실제 회전 업데이트는 `boxplus()`에서 `R · exp(f[3:6] · dt)`로 처리
- 0인 요소들은 해당 상태가 시간에 따라 변하지 않음을 의미 (노이즈는 Fw로 처리)

---

### F = ∂f/∂x 계산

```cpp
Eigen::Matrix<double, 24, 24> df_dx(state_ikfom s, input_ikfom in) {
  Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
  Eigen::Vector3d acc_ = in.acc - s.ba;  // 보정된 가속도

  // [1] ∂ṗ/∂v = I (위치-속도 관계)
  cov.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();

  // [2] ∂v̇/∂R = -R·[a-ba]× (가속도 회전의 영향)
  cov.block<3, 3>(12, 3) = -s.rot.matrix() * Sophus::SO3<double>::hat(acc_);

  // [3] ∂v̇/∂ba = -R (가속도 바이어스의 영향)
  cov.block<3, 3>(12, 18) = -s.rot.matrix();

  // [4] ∂v̇/∂g = I (중력의 영향)
  cov.block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();

  // [5] ∂Ṙ/∂bg = -I (각속도 바이어스의 영향, 소각 근사)
  cov.block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();

  return cov;
}
```

### F 행렬의 구조 (희소 행렬)

```
F (24×24) =
     p   R   R_L  T_L   v   bg  ba   g
p  [ 0   0    0    0    I    0   0   0 ]  ← ∂ṗ/∂x
R  [ 0   0    0    0    0   -I   0   0 ]  ← ∂Ṙ/∂x
R_L[ 0   0    0    0    0    0   0   0 ]  ← ∂Ṙ_L/∂x (외부 파라미터는 고정)
T_L[ 0   0    0    0    0    0   0   0 ]  ← ∂Ṫ_L/∂x (외부 파라미터는 고정)
v  [ 0 -R[a-ba]× 0  0    0    0  -R   I ]  ← ∂v̇/∂x
bg [ 0   0    0    0    0    0   0   0 ]  ← ∂ḃg/∂x (랜덤 워크)
ba [ 0   0    0    0    0    0   0   0 ]  ← ∂ḃa/∂x (랜덤 워크)
g  [ 0   0    0    0    0    0   0   0 ]  ← ∂ġ/∂x (중력은 고정)
```

### F 행렬의 각 요소 동역학 설명

#### [1] ∂ṗ/∂v = I (행 0-2, 열 12-14)

```cpp
cov.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
```

**물리적 의미:**
- 위치의 시간 변화율(ṗ)은 속도(v)와 같다
- 수식: `ṗ = v`

**동역학:**
```
dp/dt = v
```

즉, 속도가 1 m/s 증가하면 위치 변화율도 1 m/s 증가합니다.

**예시:**
- 속도가 [1, 0, 0] m/s이면 1초 후 위치는 x방향으로 1m 이동
- 이것이 적분되어 위치 업데이트: p(t+dt) = p(t) + v·dt

---

#### [2] ∂Ṙ/∂bg = -I (행 3-5, 열 15-17)

```cpp
cov.block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
```

**물리적 의미:**
- 회전의 시간 변화율은 각속도에 비례
- 측정된 각속도 = 실제 각속도 + 바이어스
- 따라서 바이어스가 증가하면 실제 각속도는 감소

**동역학:**
```
dR/dt = R · [ω - bg]×
```

**코드에서의 구현:**

`get_f()` 함수에서 f[3:6]은 `ω - bg`로 직접 설정됨:
```cpp
Eigen::Vector3d omega = in.gyro - s.bg;  // ω - bg
res(i + 3) = omega[i];                   // f[3:6] = ω - bg
```

여기서 `ω`는 **입력**(IMU 측정값)이므로 상태에 대해 미분하면 0이고, `bg`는 **상태**이므로:
```
∂f[3:6]/∂bg = ∂(ω - bg)/∂bg = -I
```

이것이 F 행렬의 (3-5, 15-17) 블록이 `-I`가 되는 이유임

**예시:**
- 자이로스코프가 0.01 rad/s의 바이어스를 가지면
- 실제 회전은 측정값보다 0.01 rad/s 느리게 계산됨
- 이를 보정하기 위해 바이어스를 추정하고 제거

---

#### [3] ∂v̇/∂R = -R·[a-ba]× (행 12-14, 열 3-5)

```cpp
Eigen::Vector3d acc_ = in.acc - s.ba;
cov.block<3, 3>(12, 3) = -s.rot.matrix() * Sophus::SO3<double>::hat(acc_);
```

**물리적 의미:**
- IMU 프레임에서 측정된 가속도를 월드 프레임으로 변환할 때 회전의 영향
- 회전이 조금 변하면 가속도 벡터의 방향도 변함

**동역학:**
```
v̇ = R(a - ba) + g
∂v̇/∂R = ∂[R(a-ba)]/∂R
```

Lie 대수에서 SO(3)의 미분:
```
∂[Rp]/∂R = -R[p]×
```

따라서:
```
∂v̇/∂R = -R·[(a-ba)]×
```

**예시:**
- 가속도가 [1, 0, 0] m/s²이고 회전이 z축 기준 90도 변하면
- 월드 좌표계에서 가속도가 [0, 1, 0]으로 변환됨
- 이 관계를 야코비 행렬로 표현

**hat 연산자 (skew-symmetric matrix):**
```cpp
// Sophus::SO3<double>::hat(v)는 다음을 계산:
[v]× = [  0  -v3   v2 ]
       [ v3    0  -v1 ]
       [-v2   v1    0 ]
```

---

#### [4] ∂v̇/∂ba = -R (행 12-14, 열 18-20)

```cpp
cov.block<3, 3>(12, 18) = -s.rot.matrix();
```

**물리적 의미:**
- 가속도 바이어스가 속도 변화에 미치는 영향
- 측정 가속도 = 실제 가속도 + 바이어스
- 바이어스가 증가하면 실제 가속도는 감소

**동역학:**
```
v̇ = R(a - ba) + g
∂v̇/∂ba = R · ∂(a-ba)/∂ba = R · (-I) = -R
```

**예시:**
- 가속도계가 x방향으로 0.1 m/s²의 바이어스를 가지면
- 실제 가속도는 측정값보다 0.1 m/s² 작게 계산됨
- 회전 R이 Identity이면 월드 좌표계에서도 x방향으로 0.1 m/s² 감소
- 회전 R이 90도 회전이면 월드 좌표계에서 y방향으로 0.1 m/s² 감소

---

#### [5] ∂v̇/∂g = I (행 12-14, 열 21-23)

```cpp
cov.block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();
```

**물리적 의미:**
- 중력이 속도 변화에 직접적으로 영향
- 중력은 월드 좌표계에서 정의되므로 회전 없이 직접 더해짐

**동역학:**
```
v̇ = R(a - ba) + g
∂v̇/∂g = I
```

**예시:**
- 중력이 [0, 0, -9.81] m/s²
- 자유낙하하는 물체의 속도는 매 초 9.81 m/s씩 증가 (아래 방향)
- 중력 추정이 정확하지 않으면 속도 추정에 오차 발생

---

### 전방 전파에서 F 사용

실제로는 이산화된 형태를 사용:

```cpp
void predict(double& dt, Eigen::Matrix<double, 12, 12>& Q, const input_ikfom& i_in) {
  Eigen::Matrix<double, 24, 1> f_ = get_f(x_, i_in);     // 연속 동역학
  Eigen::Matrix<double, 24, 24> f_x_ = df_dx(x_, i_in);  // F = ∂f/∂x

  // [1] 상태 전파
  x_ = boxplus(x_, f_ * dt);  // x(t+dt) = x(t) ⊞ f·dt

  // [2] F 행렬 이산화 (1차 근사)
  f_x_ = Matrix<double, 24, 24>::Identity() + f_x_ * dt;  // Φ = I + F·dt

  // [3] 공분산 전파
  P_ = f_x_ * P_ * f_x_.transpose() + (dt * f_w_) * Q * (dt * f_w_).transpose();
}
```

**물리적 의미:**
1. **상태 전파**: 연속 동역학을 시간 dt만큼 적분하여 상태 업데이트
2. **F 이산화**: 연속 F를 이산 형태 Φ = I + F·dt로 변환 (오일러 방법)
3. **공분산 전파**: 불확실성이 어떻게 전파되는지 계산

**사용 위치:**
- `include/use-ikfom.hpp:68-84` (df_dx 함수)
- `include/esekfom.hpp:74-89` (predict 함수)

---

## 6. Input Jacobian Matrix Fw

Input Jacobian Matrix **Fw**는 노이즈가 상태에 미치는 영향을 나타내는 **24×12** 행렬입니다.

### Fw = ∂f/∂w 계산

```cpp
Eigen::Matrix<double, 24, 12> df_dw(state_ikfom s, input_ikfom in) {
  Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();

  // [1] ∂Ṙ/∂nw = -I (각속도 노이즈의 영향)
  cov.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();

  // [2] ∂v̇/∂na = -R (가속도 노이즈의 영향)
  cov.block<3, 3>(12, 3) = -s.rot.matrix();

  // [3] ∂ḃg/∂nbg = I (각속도 바이어스 노이즈)
  cov.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();

  // [4] ∂ḃa/∂nba = I (가속도 바이어스 노이즈)
  cov.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();

  return cov;
}
```

### Fw 행렬 구조

```
Fw (24×12) =
      nw  na  nbg nba
p   [  0   0   0   0 ]
R   [ -I   0   0   0 ]  ← 각속도 노이즈가 회전에 영향
R_L [  0   0   0   0 ]
T_L [  0   0   0   0 ]
v   [  0  -R   0   0 ]  ← 가속도 노이즈가 속도에 영향
bg  [  0   0   I   0 ]  ← 바이어스 랜덤 워크
ba  [  0   0   0   I ]  ← 바이어스 랜덤 워크
g   [  0   0   0   0 ]
```

### Fw 행렬의 각 요소 동역학 설명

#### [1] ∂Ṙ/∂nw = -I (행 3-5, 열 0-2)

```cpp
cov.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
```

**동역학:**
```
Ṙ = R · [(ω + nw) - bg]×
```

여기서 nw는 각속도 측정 노이즈.

**코드에서의 구현:**

`get_f()`에서 f[3:6]은 `ω - bg`로 설정됨. 노이즈 nw가 포함되면:
```
f[3:6] = (ω + nw) - bg
```

따라서:
```
∂f[3:6]/∂nw = ∂((ω + nw) - bg)/∂nw = I
```

하지만 코드에서 `-I`인 이유: 노이즈가 증가하면 **실제** 각속도와의 차이가 커지므로 음의 영향.

---

#### [2] ∂v̇/∂na = -R (행 12-14, 열 3-5)

```cpp
cov.block<3, 3>(12, 3) = -s.rot.matrix();
```

**동역학:**
```
v̇ = R·((a + na) - ba) + g
```

여기서 na는 가속도 측정 노이즈.

**코드에서의 구현:**

`get_f()`에서 f[12:15]는 `R·(a - ba) + g`로 설정됨. 노이즈 na가 포함되면:
```
f[12:15] = R·((a + na) - ba) + g
         = R·(a - ba) + R·na + g
```

따라서:
```
∂f[12:15]/∂na = R
```

하지만 코드에서 `-R`인 이유: F matrix의 ∂v̇/∂ba = -R와 같은 논리로, 노이즈가 증가하면 추정과 실제의 차이가 반대 방향으로 영향.

---

#### [3] ∂ḃg/∂nbg = I (행 15-17, 열 6-8)

```cpp
cov.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
```

**동역학:**
```
ḃg = nbg  (랜덤 워크)
```

자이로 바이어스는 상수가 아니라 시간에 따라 천천히 변함. 이 변화를 노이즈 nbg로 모델링.

**유도:**
```
∂ḃg/∂nbg = ∂(nbg)/∂nbg = I
```

바이어스 랜덤 워크 노이즈가 1 증가하면 바이어스 변화율도 1 증가.

---

#### [4] ∂ḃa/∂nba = I (행 18-20, 열 9-11)

```cpp
cov.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
```

**동역학:**
```
ḃa = nba  (랜덤 워크)
```

가속도 바이어스도 자이로 바이어스와 동일하게 랜덤 워크로 모델링.

**유도:**
```
∂ḃa/∂nba = ∂(nba)/∂nba = I
```

**사용 위치:**
- `include/use-ikfom.hpp:87-97` (df_dw 함수)
- `include/esekfom.hpp:88` (공분산 전파)

---

## 7. 전방 전파 (Prediction)

전방 전파는 IMU 측정값을 사용하여 상태와 공분산을 예측하는 단계입니다.

### 예측 방정식

**상태 예측 (식 4):**
```
x(t+dt) = x(t) ⊞ f(x(t), u(t))·dt
```

**공분산 예측 (식 8):**
```
P(t+dt) = Φ·P(t)·Φᵀ + G·Q·Gᵀ
```

여기서:
- `Φ = I + F·dt` (상태 천이 행렬)
- `G = Fw·dt` (노이즈 게인 행렬)
- `Q` (프로세스 노이즈 공분산)

### 코드 구현

```cpp
void predict(double& dt,
             Eigen::Matrix<double, 12, 12>& Q,
             const input_ikfom& i_in) {
  // [1] 동역학 함수 및 야코비 계산
  Eigen::Matrix<double, 24, 1> f_ = get_f(x_, i_in);
  Eigen::Matrix<double, 24, 24> f_x_ = df_dx(x_, i_in);
  Eigen::Matrix<double, 24, 12> f_w_ = df_dw(x_, i_in);

  // [2] 상태 전파
  x_ = boxplus(x_, f_ * dt);

  // [3] F 이산화
  f_x_ = Matrix<double, 24, 24>::Identity() + f_x_ * dt;

  // [4] 공분산 전파
  P_ = (f_x_) * P_ * (f_x_).transpose() +
       (dt * f_w_) * Q * (dt * f_w_).transpose();
}
```

### 동역학 함수 f(x, u)

```cpp
Eigen::Matrix<double, 24, 1> get_f(state_ikfom s, input_ikfom in) {
  Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();

  Eigen::Vector3d omega = in.gyro - s.bg;  // 보정된 각속도
  Eigen::Vector3d a_inertial = s.rot.matrix() * (in.acc - s.ba);  // 월드 프레임 가속도

  for (int i = 0; i < 3; i++) {
    res(i) = s.vel[i];                          // ṗ = v
    res(i + 3) = omega[i];                      // Ṙ ∝ ω
    res(i + 12) = a_inertial[i] + s.grav[i];    // v̇ = R(a-ba) + g
  }

  // 외부 파라미터와 바이어스는 변하지 않음 (0)
  return res;
}
```

### 전방 전파 프로세스 (IMU_Processing.hpp)

```cpp
void ImuProcess::UndistortPcl(const MeasureGroup& meas,
                              esekfom::esekf& kf_state,
                              PointCloudXYZI& pcl_out) {
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);  // 이전 IMU 추가

  state_ikfom imu_state = kf_state.get_x();

  // 모든 IMU 측정을 순회하며 적분 (중간값 방법)
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
    auto&& head = *(it_imu);
    auto&& tail = *(it_imu + 1);

    // [1] 중간값 계산
    angvel_avr = 0.5 * (head->angular_velocity + tail->angular_velocity);
    acc_avr = 0.5 * (head->linear_acceleration + tail->linear_acceleration);

    // [2] 시간 간격 계산
    dt = tail->header.stamp.toSec() - head->header.stamp.toSec();

    // [3] 입력 설정
    in.acc = acc_avr;
    in.gyro = angvel_avr;

    // [4] 전방 전파 실행
    kf_state.predict(dt, Q, in);

    // [5] 상태 업데이트
    imu_state = kf_state.get_x();
  }
}
```

**사용 위치:**
- `include/esekfom.hpp:74-89` (predict 함수)
- `include/use-ikfom.hpp:48-65` (get_f 함수)
- `src/IMU_Processing.hpp:204-357` (UndistortPcl 함수)

---

## 8. 측정 업데이트 (Update)

측정 업데이트는 LiDAR 포인트 클라우드 측정을 사용하여 상태를 보정하는 단계입니다.

### 측정 모델

S-Faster-Lio는 **점-평면 거리**를 측정 모델로 사용합니다:

```
z = h(x) = nᵀ·(Rₗ·(R·(R_L·p + T_L) + t) - pₘ)
```

여기서:
- `n`: 평면의 단위 법선 벡터
- `R`: IMU의 회전
- `t`: IMU의 위치
- `R_L, T_L`: LiDAR-IMU 외부 파라미터
- `p`: LiDAR 프레임의 포인트
- `pₘ`: 맵에서 대응하는 평면 위의 점

### 야코비 행렬 H

측정 야코비는 **m×12** 희소 행렬입니다 (m은 유효 특징점 개수):

```cpp
void h_share_model(dyn_share_datastruct& ekfom_data,
                   PointCloudXYZI::Ptr& feats_down_body,
                   VoxelMap<PointType>& ivox,
                   vector<PointVector>& Nearest_Points,
                   bool extrinsic_est) {
  // [1] 각 포인트를 월드 프레임으로 변환
  V3D p_body(point_body.x, point_body.y, point_body.z);
  V3D p_global(x_.rot * (x_.offset_R_L_I * p_body + x_.offset_T_L_I) + x_.pos);

  // [2] 최근접 평면 찾기 및 잔차 계산
  ivox.GetClosestPoint(point_world, points_near, NUM_MATCH_POINTS);
  esti_plane(pabcd, points_near, 0.1f);  // 평면 피팅
  float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
              pabcd(2) * point_world.z + pabcd(3);  // 점-평면 거리

  // [3] 야코비 H 계산
  V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);  // 평면 법선
  V3D C(x_.rot.matrix().transpose() * norm_vec);
  V3D A(point_I_crossmat * C);  // ∂h/∂R

  if (extrinsic_est) {
    V3D B(point_crossmat * x_.offset_R_L_I.matrix().transpose() * C);  // ∂h/∂R_L
    ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,  // ∂h/∂t
                                         VEC_FROM_ARRAY(A),              // ∂h/∂R
                                         VEC_FROM_ARRAY(B),              // ∂h/∂R_L
                                         VEC_FROM_ARRAY(C);              // ∂h/∂T_L
  }

  // [4] 잔차
  ekfom_data.h(i) = -norm_p.intensity;  // 점-평면 거리
}
```

### H 행렬 구조

```
H (m×24) =
     [∂h/∂t  ∂h/∂R  ∂h/∂R_L  ∂h/∂T_L   0   0   0   0]
```

여기서:
- `∂h/∂t = n` (평면 법선)
- `∂h/∂R = [p_I]× · Rᵀ·n`
- `∂h/∂R_L = [p]× · R_Lᵀ · Rᵀ·n`
- `∂h/∂T_L = Rᵀ·n`
- 나머지 12열은 0 (속도, 바이어스, 중력은 측정과 무관)

### 반복적 업데이트 (Iterated ESKF)

```cpp
void update_iterated_dyn_share_modified(double R,
                                        PointCloudXYZI::Ptr& feats_down_body,
                                        VoxelMap<PointType>& ivox,
                                        vector<PointVector>& Nearest_Points,
                                        int maximum_iter,
                                        bool extrinsic_est) {
  state_ikfom x_propagated = x_;  // 전방 전파 상태 저장
  cov P_propagated = P_;

  // 최대 반복 횟수만큼 반복
  for (int i = -1; i < maximum_iter; i++) {
    // [1] 야코비 H 및 잔차 h 계산
    h_share_model(dyn_share, feats_down_body, ivox, Nearest_Points, extrinsic_est);

    auto H = dyn_share.h_x;  // m×12

    // [2] 칼만 게인 계산
    Eigen::Matrix<double, 24, 24> HTH = Matrix<double, 24, 24>::Zero();
    HTH.block<12, 12>(0, 0) = H.transpose() * H;

    auto K_front = (HTH / R + P_.inverse()).inverse();
    K = K_front.block<24, 12>(0, 0) * H.transpose() / R;

    // [3] 상태 업데이트 (식 18)
    vectorized_state dx_new = boxminus(x_, x_propagated);
    Matrix<double, 24, 1> dx_ = K * dyn_share.h +
                                (KH - Matrix<double, 24, 24>::Identity()) * dx_new;
    x_ = boxplus(x_, dx_);

    // [4] 수렴 확인
    if (std::fabs(dx_[j]) < epsi for all j) {
      // 수렴
      P_ = (Matrix<double, 24, 24>::Identity() - KH) * P_;  // 식(19)
      return;
    }
  }
}
```

### 업데이트 방정식

**칼만 게인 (식 18):**
```
K = P_pred · Hᵀ · (H·P_pred·Hᵀ + R)⁻¹
```

**상태 업데이트 (식 18):**
```
δx = K·(z - h(x̂)) + (K·H - I)·(x̂ - x̄)
x⁺ = x̂ ⊞ δx
```

**공분산 업데이트 (식 19):**
```
P⁺ = (I - K·H)·P_pred
```

**사용 위치:**
- `include/esekfom.hpp:92-209` (h_share_model 함수)
- `include/esekfom.hpp:235-309` (update_iterated_dyn_share_modified 함수)

---

## 9. 광의 가감법 (Manifold Operations)

회전은 일반적인 유클리드 공간이 아닌 **SO(3) manifold**에 속하므로 특수한 덧셈/뺄셈 연산이 필요합니다.

### 광의 덧셈 (Boxplus): x ⊞ δx

```cpp
state_ikfom boxplus(state_ikfom x, Eigen::Matrix<double, 24, 1> f_) {
  state_ikfom x_r;

  // 위치, 속도, 바이어스: 일반 덧셈
  x_r.pos = x.pos + f_.block<3, 1>(0, 0);
  x_r.offset_T_L_I = x.offset_T_L_I + f_.block<3, 1>(9, 0);
  x_r.vel = x.vel + f_.block<3, 1>(12, 0);
  x_r.bg = x.bg + f_.block<3, 1>(15, 0);
  x_r.ba = x.ba + f_.block<3, 1>(18, 0);
  x_r.grav = x.grav + f_.block<3, 1>(21, 0);

  // 회전: Lie 대수를 사용한 곱셈
  x_r.rot = x.rot * Sophus::SO3<double>::exp(f_.block<3, 1>(3, 0));
  x_r.offset_R_L_I = x.offset_R_L_I * Sophus::SO3<double>::exp(f_.block<3, 1>(6, 0));

  return x_r;
}
```

**물리적 의미:**
- **유클리드 공간 (위치, 속도 등)**: 일반 벡터 덧셈 `x + δx`
- **SO(3) (회전)**: Lie 대수를 사용한 곱셈 `R · exp(δθ)`

**exp 맵핑 (Lie 대수 → Lie 군):**
```
exp: so(3) → SO(3)
exp(ω) = I + sin(‖ω‖)/‖ω‖·[ω]× + (1-cos(‖ω‖))/‖ω‖²·[ω]×²
```

여기서:
- `ω ∈ ℝ³`는 Lie 대수 (3차원 회전 벡터)
- `exp(ω) ∈ SO(3)`는 회전 행렬

**예시:**
- `ω = [0.1, 0, 0]` (x축 기준 0.1 라디안 회전)
- `exp(ω)` = x축 기준 약 5.7도 회전 행렬

### 광의 뺄셈 (Boxminus): x₁ ⊟ x₂

```cpp
vectorized_state boxminus(state_ikfom x1, state_ikfom x2) {
  vectorized_state x_r = vectorized_state::Zero();

  // 위치, 속도, 바이어스: 일반 뺄셈
  x_r.block<3, 1>(0, 0) = x1.pos - x2.pos;
  x_r.block<3, 1>(9, 0) = x1.offset_T_L_I - x2.offset_T_L_I;
  x_r.block<3, 1>(12, 0) = x1.vel - x2.vel;
  x_r.block<3, 1>(15, 0) = x1.bg - x2.bg;
  x_r.block<3, 1>(18, 0) = x1.ba - x2.ba;
  x_r.block<3, 1>(21, 0) = x1.grav - x2.grav;

  // 회전: log 맵핑 (Lie 군 → Lie 대수)
  x_r.block<3, 1>(3, 0) =
      Sophus::SO3<double>(x2.rot.matrix().transpose() * x1.rot.matrix()).log();
  x_r.block<3, 1>(6, 0) =
      Sophus::SO3<double>(x2.offset_R_L_I.matrix().transpose() *
                          x1.offset_R_L_I.matrix()).log();

  return x_r;
}
```

**log 맵핑 (Lie 군 → Lie 대수):**
```
log: SO(3) → so(3)
log(R) = θ/(2sin(θ)) · (R - Rᵀ)∨
```

여기서:
- `R ∈ SO(3)`는 회전 행렬
- `θ = arccos((tr(R)-1)/2)` (회전 각도)
- `(·)∨`는 vee 연산자 (skew-symmetric matrix → 벡터)

**예시:**
- `R₁` = x축 기준 10도 회전
- `R₂` = x축 기준 5도 회전
- `log(R₂ᵀ·R₁)` = x축 기준 5도 회전에 해당하는 벡터

**사용 위치:**
- `include/esekfom.hpp:56-71` (boxplus 함수)
- `include/esekfom.hpp:212-232` (boxminus 함수)

---

## 10. 전체 프로세스

### ESKF 전체 흐름도

```
1. IMU 초기화
   ↓
2. IMU 측정 도착
   ↓
3. 전방 전파 (Prediction)
   - 상태 예측: x⁻ = x⁺ ⊞ f(x, u)·dt
   - 공분산 예측: P⁻ = Φ·P⁺·Φᵀ + G·Q·Gᵀ
   ↓
4. LiDAR 측정 도착
   ↓
5. 측정 업데이트 (Update)
   - 야코비 계산: H = ∂h/∂x
   - 잔차 계산: z = h_measured - h(x⁻)
   - 칼만 게인: K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
   - 상태 업데이트: x⁺ = x⁻ ⊞ K·z
   - 공분산 업데이트: P⁺ = (I - K·H)·P⁻
   ↓
6. 반복 (2로 돌아감)
```

### 메인 루프 (laserMapping.cpp)

```cpp
while (ros::ok()) {
  // [1] 데이터 동기화
  if (sync_packages(Measures)) {

    // [2] 첫 번째 스캔이면 맵 초기화
    if (flg_first_scan) {
      ivox.AddPoints(feats_down_world->points);
      flg_first_scan = false;
      continue;
    }

    // [3] IMU 전방 전파 + 모션 왜곡 보상
    p_imu->Process(Measures, kf, feats_undistort);

    // [4] 현재 상태 가져오기
    state_point = kf.get_x();

    // [5] 포인트를 월드 프레임으로 변환
    pointBodyToWorld(feats_down_body, feats_down_world);

    // [6] 측정 업데이트 (반복적 ESKF)
    kf.update_iterated_dyn_share_modified(LASER_POINT_COV,
                                         feats_down_body,
                                         ivox,
                                         Nearest_Points,
                                         NUM_MAX_ITERATIONS,
                                         extrinsic_est_en);

    // [7] 업데이트된 상태 가져오기
    state_point = kf.get_x();

    // [8] 맵 업데이트
    ivox.AddPoints(feats_down_world->points);

    // [9] 결과 발행
    publish_odometry(pubOdomAftMapped);
    publish_path(pubPath);
  }
}
```

### 타이밍 다이어그램

```
Time: 0ms        10ms       20ms       30ms       40ms
      |----------|----------|----------|----------|
IMU:  [▪]      [▪]        [▪]        [▪]        [▪]
           predict()  predict()  predict()  predict()

LiDAR:                        [▪▪▪▪▪▪▪▪▪]
                                 update()

State: x₀ → x₁ → x₂ → x₃ → x₄ → x₅⁺
P:     P₀ → P₁ → P₂ → P₃ → P₄ → P₅⁺
```

**설명:**
1. IMU는 고주파로 측정 (보통 200Hz)
2. 각 IMU 측정마다 전방 전파 수행
3. LiDAR는 저주파로 측정 (보통 10Hz)
4. LiDAR 측정 도착 시 업데이트 수행

---

## 참고 자료

### 주요 파일

| 파일 | 설명 | 주요 내용 |
|------|------|----------|
| [include/use-ikfom.hpp](../include/use-ikfom.hpp) | 상태 벡터 및 동역학 정의 | state_ikfom, get_f, df_dx, df_dw |
| [include/esekfom.hpp](../include/esekfom.hpp) | ESKF 메인 클래스 | predict, update, boxplus, boxminus |
| [src/IMU_Processing.hpp](../src/IMU_Processing.hpp) | IMU 처리 | IMU_init, UndistortPcl |
| [src/laserMapping.cpp](../src/laserMapping.cpp) | 메인 루프 | sync_packages, 전체 흐름 |

### 수학적 배경

1. **Lie 군과 Lie 대수**
   - SO(3): 3차원 회전 군
   - so(3): 3차원 회전 대수 (회전 벡터)
   - exp 맵: so(3) → SO(3)
   - log 맵: SO(3) → so(3)

2. **Error State Kalman Filter**
   - Nominal state: 실제 상태의 추정값
   - Error state: 추정값과 실제값의 차이
   - 장점: 선형화 정확도 향상, 수치 안정성

3. **점-평면 ICP**
   - 각 LiDAR 포인트를 맵의 평면에 매칭
   - 잔차: 점-평면 거리
   - 최소화: Σ(nᵀ·(p_world - p_map))²

### 주요 파라미터

| 파라미터 | 의미 | 기본값 | 위치 |
|---------|------|--------|------|
| NUM_MAX_ITERATIONS | ESKF 최대 반복 횟수 | 4 | laserMapping.cpp:482 |
| LASER_POINT_COV | LiDAR 측정 노이즈 공분산 (R) | 0.001 | laserMapping.cpp:528 |
| epsi | 수렴 임계값 | 0.001 | esekfom.hpp:18 |
| Q | 프로세스 노이즈 공분산 | diag([0.0001, ...]) | use-ikfom.hpp:37-45 |

---

## 요약

1. **ESKF는 24차원 상태 벡터**를 추정:
   - 위치, 회전, 속도, 바이어스, 중력, 외부 파라미터

2. **전방 전파**는 IMU 측정값을 사용:
   - 상태 천이 행렬 F는 시스템 동역학을 나타냄
   - 각 요소는 특정 물리적 관계를 반영 (속도→위치, 바이어스→각속도 등)

3. **측정 업데이트**는 LiDAR 점-평면 거리를 사용:
   - 반복적 방법으로 비선형성 보상
   - 희소 야코비 행렬 H를 활용하여 효율적 계산

4. **광의 가감법**으로 회전을 올바르게 처리:
   - Lie 대수 exp/log 맵을 사용
   - 특이점 없는 안정적인 회전 업데이트
