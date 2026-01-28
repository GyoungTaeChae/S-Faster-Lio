# Multithreading 문법 정리

S-Faster-Lio 프로젝트의 src와 include 폴더에서 사용되는 멀티스레딩 및 병렬 처리 문법을 정리한 문서입니다.

## 목차
1. [기본 헤더](#1-기본-헤더)
2. [Mutex (뮤텍스)](#2-mutex-뮤텍스)
3. [Condition Variable (조건 변수)](#3-condition-variable-조건-변수)
4. [Thread (스레드)](#4-thread-스레드)
5. [Parallel Execution (병렬 실행)](#5-parallel-execution-병렬-실행)
6. [OpenMP](#6-openmp)
7. [동기화 패턴](#7-동기화-패턴)

---

## 1. 기본 헤더

### 멀티스레딩 관련 헤더
```cpp
#include <mutex>              // mutex, lock_guard
#include <thread>             // std::thread
#include <condition_variable> // condition_variable
#include <execution>          // parallel execution policies (C++17)
#include <omp.h>             // OpenMP
```

**사용 위치:**
- `src/laserMapping.cpp:21-22` (mutex, thread)
- `src/IMU_Processing.hpp:23` (thread)
- `include/esekfom.hpp:10` (execution)
- `include/voxmap/voxel_map.h:6` (execution)

---

## 2. Mutex (뮤텍스)

뮤텍스(Mutual Exclusion)는 공유 자원에 대한 동시 접근을 제어하는 동기화 메커니즘입니다.

### 뮤텍스 선언
```cpp
std::mutex mtx_buffer;
```

### 수동 잠금/해제
```cpp
void callback_function() {
  mtx_buffer.lock();      // 잠금 획득

  // 임계 영역 (Critical Section)
  // 공유 자원 접근
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(msg->header.stamp.toSec());

  mtx_buffer.unlock();    // 잠금 해제
}
```

### Lock Guard (자동 잠금/해제)
```cpp
{
  std::lock_guard<std::mutex> lock(mtx_buffer);
  // 스코프 내에서 자동으로 잠금 유지
  // 공유 자원 접근

  // 스코프를 벗어나면 자동으로 unlock
}
```

**주의사항:**
- `lock()`과 `unlock()`은 반드시 쌍으로 호출되어야 합니다.
- 예외 발생 시 unlock이 호출되지 않을 수 있으므로, `std::lock_guard` 사용을 권장합니다.

**사용 위치:**
- `src/laserMapping.cpp:45` (mutex 선언)
- `src/laserMapping.cpp:107, 120, 127, 156, 175, 185` (lock/unlock)

---

## 3. Condition Variable (조건 변수)

조건 변수는 특정 조건이 만족될 때까지 스레드를 대기시키는 동기화 메커니즘입니다.

### 조건 변수 선언
```cpp
std::condition_variable sig_buffer;
```

### 대기 (Wait)
```cpp
void consumer_thread() {
  std::unique_lock<std::mutex> lock(mtx_buffer);

  // 조건이 만족될 때까지 대기
  sig_buffer.wait(lock, []{ return !buffer.empty(); });

  // 조건이 만족되면 여기서 실행 계속
  // 공유 자원 사용
}
```

### 알림 (Notify)
```cpp
void producer_thread() {
  mtx_buffer.lock();

  // 공유 자원 수정
  buffer.push_back(data);

  mtx_buffer.unlock();

  // 대기 중인 스레드에게 알림
  sig_buffer.notify_all();  // 모든 대기 스레드 깨우기
  // 또는
  sig_buffer.notify_one();  // 하나의 대기 스레드만 깨우기
}
```

### Producer-Consumer 패턴
```cpp
// Producer (데이터 생성자)
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  mtx_buffer.lock();

  // 버퍼에 데이터 추가
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(msg->header.stamp.toSec());

  mtx_buffer.unlock();
  sig_buffer.notify_all();  // Consumer에게 알림
}

// Consumer (데이터 소비자)
while (ros::ok()) {
  // 데이터가 준비될 때까지 대기
  if (sync_packages(Measures)) {
    // 데이터 처리
  }
}
```

**사용 위치:**
- `src/laserMapping.cpp:46` (condition_variable 선언)
- `src/laserMapping.cpp:103, 121, 157, 186` (notify_all)

---

## 4. Thread (스레드)

### 스레드 생성
```cpp
#include <thread>

void thread_function(int arg1, double arg2) {
  // 스레드에서 실행할 코드
}

int main() {
  // 스레드 생성 및 시작
  std::thread t1(thread_function, 10, 3.14);

  // 메인 스레드 작업

  // 스레드 종료 대기
  t1.join();

  return 0;
}
```

### 람다 함수를 이용한 스레드
```cpp
std::thread t([&]() {
  // 스레드에서 실행할 코드
  // 캡처를 통해 외부 변수 접근 가능
});

t.join();
```

**참고:**
이 프로젝트에서는 ROS의 콜백 메커니즘을 주로 사용하며, 명시적인 `std::thread` 생성은 제한적으로 사용됩니다.

---

## 5. Parallel Execution (병렬 실행)

C++17의 병렬 알고리즘을 사용하여 루프를 병렬로 실행합니다.

### Execution Policies

```cpp
#include <execution>

// 순차 실행 (기본)
std::execution::seq

// 병렬 실행
std::execution::par

// 병렬 + 벡터화 실행 (SIMD)
std::execution::par_unseq

// 벡터화 실행
std::execution::unseq
```

### std::for_each를 이용한 병렬 루프

#### 병렬 + 벡터화 실행
```cpp
std::vector<int> index(size);
std::iota(index.begin(), index.end(), 0);  // 0, 1, 2, ..., size-1

std::for_each(
    std::execution::par_unseq,  // 병렬 + 벡터화
    index.begin(),
    index.end(),
    [&](const int& i) {
      // 각 인덱스 i에 대해 병렬로 실행
      // 스레드 안전성 주의!
      process_element(i);
    }
);
```

#### 벡터화 실행 (순서 보장 안됨)
```cpp
std::for_each(
    std::execution::unseq,  // 벡터화만
    points_to_add.begin(),
    points_to_add.end(),
    [this](const auto& pt) {
      // 각 포인트에 대해 실행
      // 순서가 보장되지 않음
      auto key = Pos2Grid(pt);
      // ...
    }
);
```

### 실제 사용 예제 1: 특징점 매칭
```cpp
// esekfom.hpp에서 특징점과 맵 매칭을 병렬로 수행
std::vector<int> index(feats_down_size);
std::iota(index.begin(), index.end(), 0);

std::for_each(
    std::execution::par_unseq,
    index.begin(),
    index.end(),
    [&](const int& i) {
      PointType& point_body = feats_down_body->points[i];
      PointType point_world;

      // 좌표 변환
      V3D p_body(point_body.x, point_body.y, point_body.z);
      V3D p_global(x_.rot * (x_.offset_R_L_I * p_body + x_.offset_T_L_I) + x_.pos);

      // 최근접점 검색
      auto& points_near = Nearest_Points[i];
      if (ekfom_data.converge) {
        ivox.GetClosestPoint(point_world, points_near, NUM_MATCH_POINTS);
      }

      // 평면 피팅 및 유효성 검사
      // ...
    }
);
```

### 실제 사용 예제 2: 복셀 맵에 포인트 추가
```cpp
// voxel_map.h에서 포인트 클라우드를 맵에 추가
void VoxelMap<PointType>::AddPoints(const PointVector& points_to_add) {
  std::for_each(
      std::execution::unseq,  // 순서 상관없이 병렬 실행
      points_to_add.begin(),
      points_to_add.end(),
      [this](const auto& pt) {
        auto key = Pos2Grid(pt);

        // 해시 맵에서 복셀 찾기
        auto iter = grids_map_.find(key);
        if (iter == grids_map_.end()) {
          // 새 복셀 생성
          // ...
        } else {
          // 기존 복셀에 포인트 추가
          iter->second->second.InsertPoint(pt);
        }
      }
  );
}
```

### 실제 사용 예제 3: 최근접점 검색
```cpp
// voxel_map.h에서 포인트 클라우드의 최근접점 검색
bool VoxelMap<PointType>::GetClosestPoint(
    const PointVector& cloud,
    PointVector& closest_cloud) {

  std::vector<size_t> index(cloud.size());
  for (int i = 0; i < cloud.size(); ++i) {
    index[i] = i;
  }
  closest_cloud.resize(cloud.size());

  std::for_each(
      std::execution::par_unseq,
      index.begin(),
      index.end(),
      [&cloud, &closest_cloud, this](size_t idx) {
        PointType pt;
        if (GetClosestPoint(cloud[idx], pt)) {
          closest_cloud[idx] = pt;
        } else {
          closest_cloud[idx] = PointType();
        }
      }
  );

  return true;
}
```

**주의사항:**
1. **데이터 경쟁(Data Race) 방지**: 각 스레드가 서로 다른 메모리 위치에 접근하도록 해야 합니다.
2. **공유 자원 접근**: 공유 자원에 접근할 때는 적절한 동기화가 필요합니다.
3. **실행 순서**: `par_unseq`와 `unseq`는 실행 순서를 보장하지 않습니다.

**사용 위치:**
- `include/esekfom.hpp:101-157` (par_unseq 사용)
- `include/voxmap/voxel_map.h:100, 249, 263` (par_unseq, unseq 사용)

---

## 6. OpenMP

OpenMP는 공유 메모리 멀티프로세싱 프로그래밍을 위한 API입니다.

### 헤더 포함
```cpp
#include <omp.h>
```

### 시간 측정
```cpp
double start_time = omp_get_wtime();

// 시간을 측정할 코드

double end_time = omp_get_wtime();
double elapsed = end_time - start_time;

std::cout << "Elapsed time: " << elapsed << " seconds" << std::endl;
```

### 병렬 for 루프 (프로젝트에서 미사용, 참고용)
```cpp
#pragma omp parallel for
for (int i = 0; i < n; i++) {
  // 병렬로 실행될 코드
  process(i);
}
```

### 임계 영역 (프로젝트에서 미사용, 참고용)
```cpp
#pragma omp critical
{
  // 한 번에 하나의 스레드만 실행
  shared_resource += value;
}
```

**사용 위치:**
- `src/laserMapping.cpp:6` (omp.h 헤더)
- `src/laserMapping.cpp:109, 128, 603, 665` (omp_get_wtime)
- `src/preprocess.cpp:97, 134` (omp_get_wtime)

---

## 7. 동기화 패턴

### Producer-Consumer 패턴

이 패턴은 데이터를 생성하는 스레드(Producer)와 소비하는 스레드(Consumer)를 분리합니다.

```cpp
// 공유 버퍼 및 동기화 객체
std::deque<DataType> buffer;
std::mutex mtx;
std::condition_variable cv;
bool done = false;

// Producer 스레드
void producer() {
  while (true) {
    DataType data = generate_data();

    {
      std::lock_guard<std::mutex> lock(mtx);
      buffer.push_back(data);
    }

    cv.notify_one();  // Consumer에게 알림

    if (should_stop()) {
      std::lock_guard<std::mutex> lock(mtx);
      done = true;
      cv.notify_all();
      break;
    }
  }
}

// Consumer 스레드
void consumer() {
  while (true) {
    std::unique_lock<std::mutex> lock(mtx);

    // 데이터가 준비되거나 종료 신호를 받을 때까지 대기
    cv.wait(lock, [&]{ return !buffer.empty() || done; });

    if (done && buffer.empty()) {
      break;
    }

    // 버퍼에서 데이터 가져오기
    DataType data = buffer.front();
    buffer.pop_front();

    lock.unlock();

    // 데이터 처리
    process_data(data);
  }
}
```

### 프로젝트의 실제 구현

#### 멀티스레드 아키텍처 개요

S-Faster-LIO의 멀티스레딩은 **Producer-Consumer 패턴**을 기반으로 하며, ROS 콜백 스레드(Producer)와 메인 처리 루프(Consumer)가 공유 버퍼를 통해 협력합니다.

```
┌─────────────────────────────────────────────────────────────────────┐
│                    S-Faster-LIO 스레드 구조                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────────┐   ┌─────────────────────┐                  │
│  │  ROS Callback Thread │   │  ROS Callback Thread │                 │
│  │    (LiDAR 센서)      │   │    (IMU 센서)        │                 │
│  │                     │   │                     │                  │
│  │  standard_pcl_cbk() │   │    imu_cbk()        │                  │
│  │  livox_pcl_cbk()    │   │                     │                  │
│  │   [Producer #1]     │   │   [Producer #2]     │                  │
│  └────────┬────────────┘   └────────┬────────────┘                  │
│           │                         │                               │
│           │  lock(mtx_buffer)       │  lock(mtx_buffer)             │
│           │  버퍼에 데이터 push      │  버퍼에 데이터 push            │
│           │  unlock(mtx_buffer)     │  unlock(mtx_buffer)           │
│           │  notify_all()           │  notify_all()                 │
│           │                         │                               │
│           ▼                         ▼                               │
│  ┌──────────────────────────────────────────────┐                   │
│  │            공유 버퍼 (Critical Section)        │                  │
│  │                                              │                   │
│  │  ┌──────────────┐  ┌───────────────────────┐ │                   │
│  │  │ lidar_buffer │  │ imu_buffer            │ │                   │
│  │  │ (deque)      │  │ (deque)               │ │                   │
│  │  └──────────────┘  └───────────────────────┘ │                   │
│  │  ┌──────────────┐  ┌───────────────────────┐ │                   │
│  │  │ time_buffer  │  │ last_timestamp_lidar  │ │                   │
│  │  │ (deque)      │  │ last_timestamp_imu    │ │                   │
│  │  └──────────────┘  └───────────────────────┘ │                   │
│  │                                              │                   │
│  │          보호: mtx_buffer (mutex)             │                   │
│  └──────────────────────┬───────────────────────┘                   │
│                         │                                           │
│                         │  ros::spinOnce()로 콜백 실행 후            │
│                         │  sync_packages()로 데이터 소비             │
│                         ▼                                           │
│  ┌──────────────────────────────────────────────┐                   │
│  │           Main Thread [Consumer]              │                  │
│  │                                              │                   │
│  │  while (ros::ok()) {                         │                   │
│  │    ros::spinOnce();     // 콜백 실행          │                   │
│  │    sync_packages();     // 버퍼에서 꺼냄      │                   │
│  │    Process → EKF → Map Update → Publish      │                   │
│  │    rate.sleep();        // 5000Hz 폴링        │                   │
│  │  }                                           │                   │
│  └──────────────────────────────────────────────┘                   │
│                                                                     │
│  ┌──────────────────────────────────────────────┐                   │
│  │           Signal Handler                      │                  │
│  │  SigHandle(SIGINT):                          │                   │
│  │    flg_exit = true                           │                   │
│  │    sig_buffer.notify_all()  → 대기 스레드 깨움 │                  │
│  └──────────────────────────────────────────────┘                   │
└─────────────────────────────────────────────────────────────────────┘
```

#### Race Condition 분석과 방지 설계

이 프로젝트에서 발생할 수 있는 Race Condition과 그 방지 메커니즘을 시각화합니다.

**Race Condition #1: 다중 Producer 동시 쓰기**

LiDAR 콜백과 IMU 콜백이 동시에 공유 버퍼에 쓰려 할 때 발생합니다.

```
  ❌ mutex 없이 (Race Condition 발생)
  ──────────────────────────────────────────────────────

  시간 ──────────────────────────────────────────────▶

  LiDAR 콜백:  ──[lidar_buffer.push_back()]──
                        ↕ 충돌! (동시 접근)
  IMU 콜백:       ──[imu_buffer.push_back()]──

  → deque 내부 상태 손상 가능 (정의되지 않은 동작)


  ✅ mtx_buffer로 보호 (실제 구현)
  ──────────────────────────────────────────────────────

  시간 ──────────────────────────────────────────────▶

  LiDAR 콜백:  ──[lock]──[push_back]──[unlock]──[notify_all]──
                                                │
  IMU 콜백:            ──[대기]──              ──[lock]──[push_back]──[unlock]──[notify_all]──
                        (blocked)
```

**Race Condition #2: Producer-Consumer 동시 접근**

콜백이 버퍼에 쓰는 동안 `sync_packages()`가 버퍼에서 읽으려 할 때 발생합니다.

```
  ❌ 보호 없이 (Race Condition 발생)
  ──────────────────────────────────────────────────────

  시간 ──────────────────────────────────────────────▶

  Producer:   ──[lidar_buffer.push_back()]──
                        ↕ 충돌!
  Consumer:      ──[lidar_buffer.front()]──[lidar_buffer.pop_front()]──

  → 읽기 중 데이터 변경, 반복자(iterator) 무효화 가능


  ✅ 실제 구현: ros::spinOnce() 기반 직렬화
  ──────────────────────────────────────────────────────

  시간 ──────────────────────────────────────────────▶

  Main Thread:
    ┌────────────────┐  ┌──────────────────────────────┐  ┌────────┐
    │ ros::spinOnce() │→│ sync_packages() + 처리        │→│ sleep  │→ ...
    │ (콜백 실행)      │  │ (콜백과 동일 스레드에서 실행)   │  │        │
    └────────────────┘  └──────────────────────────────┘  └────────┘
         │
         ├─ LiDAR 콜백 실행 (lock → push → unlock → notify)
         └─ IMU 콜백 실행   (lock → push → unlock → notify)

  핵심: ros::spinOnce()가 콜백을 메인 스레드에서 실행하므로,
        sync_packages()와 콜백은 자연스럽게 직렬화됩니다.
        mtx_buffer는 멀티스레드 subscriber 사용 시를 대비한 안전장치입니다.
```

**Race Condition #3: 타임스탬프 역행 (Loopback)**

센서가 재시작되어 타임스탬프가 과거로 돌아갈 때, 오래된 데이터가 버퍼에 남는 문제입니다.

```
  시간순 타임스탬프 기대:  t1 → t2 → t3 → t4 → t5

  실제 수신:              t1 → t2 → t3 → [센서 재시작] → t1' → t2'
                                          ↑
                                  t1' < t3 (타임스탬프 역행!)

  방지 메커니즘:
  ┌─────────────────────────────────────────────────────┐
  │  if (msg->header.stamp.toSec() < last_timestamp) {  │
  │    ROS_ERROR("lidar loop back, clear buffer");      │
  │    lidar_buffer.clear();   // 오래된 데이터 전부 제거 │
  │  }                                                  │
  └─────────────────────────────────────────────────────┘
  → 잘못된 시간 순서의 데이터로 SLAM이 오동작하는 것을 방지
```

**Race Condition #4: 종료 시점의 안전한 스레드 정리**

Ctrl+C (SIGINT) 수신 시, 대기 중인 스레드가 영원히 블록되는 것을 방지합니다.

```
  사용자: Ctrl+C

  SigHandle():
    ┌──────────────────────────┐
    │ flg_exit = true          │  ← 종료 플래그 설정
    │ sig_buffer.notify_all()  │  ← 모든 대기 스레드 깨움
    └──────────────────────────┘
             │
             ▼
  Main Loop:
    while (ros::ok()) {
      if (flg_exit) break;  ← 플래그 확인 후 루프 탈출
      ...
    }

  → notify_all()이 없으면 sig_buffer.wait() 상태의 스레드가
    영원히 깨어나지 못할 수 있음 (deadlock 유사 상태)
```

#### 동기화 프리미티브 관계도

```
  ┌─────────────────────────────────────────────────────────────┐
  │                 동기화 메커니즘 상호작용                       │
  │                                                             │
  │   mtx_buffer (mutex)                                        │
  │   ┌─────────────────────────────────────────┐               │
  │   │ 역할: 공유 버퍼의 상호 배제 보장           │              │
  │   │                                         │               │
  │   │ 보호 대상:                               │               │
  │   │   • lidar_buffer (deque)                │               │
  │   │   • imu_buffer (deque)                  │               │
  │   │   • time_buffer (deque)                 │               │
  │   │   • last_timestamp_lidar (double)       │               │
  │   │   • last_timestamp_imu (double)         │               │
  │   └─────────────────────────────────────────┘               │
  │         │                                                   │
  │         │ lock/unlock 쌍으로 임계 영역 형성                   │
  │         ▼                                                   │
  │   sig_buffer (condition_variable)                           │
  │   ┌─────────────────────────────────────────┐               │
  │   │ 역할: "데이터가 준비됨"을 알리는 신호       │              │
  │   │                                         │               │
  │   │ notify_all() 호출 시점:                  │               │
  │   │   • LiDAR 콜백에서 데이터 push 후         │              │
  │   │   • IMU 콜백에서 데이터 push 후           │               │
  │   │   • SIGINT 핸들러에서 종료 신호 전달 시    │               │
  │   │                                         │               │
  │   │ 의미: "버퍼에 새 데이터가 있으니 확인하라"  │               │
  │   └─────────────────────────────────────────┘               │
  │         │                                                   │
  │         ▼                                                   │
  │   flg_exit (bool)                                           │
  │   ┌─────────────────────────────────────────┐               │
  │   │ 역할: 프로그램 종료 신호 (플래그 변수)     │               │
  │   │                                         │               │
  │   │ SIGINT → flg_exit = true                │               │
  │   │       → notify_all() (대기 스레드 깨움)   │              │
  │   │       → 메인 루프 break                  │               │
  │   └─────────────────────────────────────────┘               │
  │                                                             │
  └─────────────────────────────────────────────────────────────┘
```

#### notify_all vs notify_one 선택 이유

```
  이 프로젝트에서 notify_all()을 사용하는 이유:

  notify_one():  대기 중인 스레드 중 "하나만" 깨움
  notify_all():  대기 중인 스레드 "전부" 깨움

  ┌─────────────────────────────────────────────────┐
  │  LiDAR 콜백 → notify_all()                      │
  │  IMU 콜백   → notify_all()                      │
  │  SigHandle  → notify_all()                      │
  └─────────────────────────────────────────────────┘

  이유 1: 안전성
    → sig_buffer를 wait()하는 스레드가 여러 개일 수 있으며,
      모두 깨워야 종료 시 deadlock을 방지할 수 있음

  이유 2: 종료 처리
    → SigHandle에서 flg_exit 설정 후 notify_all()로
      모든 대기 스레드를 확실히 깨워서 종료시킴

  이유 3: 단순함과 안정성
    → notify_one()은 미묘한 버그(lost wakeup)를 유발할 수 있음
    → notify_all()은 약간의 성능 비용으로 안전성을 확보
```

#### 전체 데이터 흐름 시퀀스

```
  시간 ──────────────────────────────────────────────────────────────▶

  LiDAR 센서:  ═══[scan 1]════════════[scan 2]════════════[scan 3]═══
  IMU 센서:    ═[i1]═[i2]═[i3]═[i4]═[i5]═[i6]═[i7]═[i8]═[i9]═[i10]═

  ROS 콜백     ┌lock─push─unlock─notify┐
  (LiDAR):     └───────────────────────┘

  ROS 콜백          ┌lock─push─unlock─notify┐   (각 IMU 메시지마다)
  (IMU):            └───────────────────────┘

  Main Loop:   [spinOnce]─[sync_packages]─────────────────[spinOnce]──
                               │
                               ├─ lidar_buffer에서 scan 1 꺼냄
                               ├─ imu_buffer에서 i1~i4 꺼냄
                               │   (scan 1 시간 범위에 해당하는 IMU)
                               ▼
                          ┌─────────────┐
                          │ MeasureGroup │
                          │ .lidar = scan1    │
                          │ .imu = [i1,i2,i3,i4] │
                          └──────┬──────┘
                                 │
                                 ▼
                     ┌─────────────────────┐
                     │ IMU 사전적분          │
                     │ 왜곡 보정             │
                     │ EKF 상태 추정         │
                     │ 맵 업데이트           │
                     │ 결과 퍼블리시          │
                     └─────────────────────┘
```

```cpp
// 전역 변수
mutex mtx_buffer;
condition_variable sig_buffer;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
bool flg_exit = false;

// Producer: ROS 콜백 (여러 개 가능)
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  mtx_buffer.lock();

  // 데이터 유효성 검사
  if (msg->header.stamp.toSec() < last_timestamp_lidar) {
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_buffer.clear();
  }

  // 버퍼에 데이터 추가
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(msg->header.stamp.toSec());

  mtx_buffer.unlock();
  sig_buffer.notify_all();  // Consumer에게 알림
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr& msg_in) {
  mtx_buffer.lock();

  // IMU 데이터 처리 및 저장
  imu_buffer.push_back(msg);

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

// Consumer: 메인 루프
int main(int argc, char** argv) {
  // ROS 초기화 및 Subscriber 설정

  ros::Rate rate(5000);
  while (ros::ok()) {
    if (flg_exit) break;

    ros::spinOnce();  // 콜백 처리

    // 동기화된 데이터 패키지 가져오기
    if (sync_packages(Measures)) {
      // LiDAR와 IMU 데이터가 준비됨
      // 데이터 처리 (SLAM 알고리즘)
      p_imu->Process(Measures, kf, feats_undistort);
      // ...
    }

    rate.sleep();
  }

  return 0;
}

// 동기화 함수
bool sync_packages(MeasureGroup& meas) {
  // 버퍼가 비어있으면 false 반환
  if (lidar_buffer.empty() || imu_buffer.empty()) {
    return false;
  }

  // LiDAR와 IMU 데이터를 시간 기준으로 동기화
  // ...

  return true;
}
```

**사용 위치:**
- `src/laserMapping.cpp:45-46` (동기화 객체)
- `src/laserMapping.cpp:106-187` (Producer 콜백)
- `src/laserMapping.cpp:192-244` (sync_packages)
- `src/laserMapping.cpp:596-673` (Consumer 메인 루프)

---

## 성능 최적화 팁

### 1. 락 최소화
```cpp
// 나쁜 예
mtx.lock();
data1 = process_data1();  // 시간이 오래 걸리는 작업
shared_buffer.push_back(data1);
mtx.unlock();

// 좋은 예
data1 = process_data1();  // 락 밖에서 처리
{
  std::lock_guard<std::mutex> lock(mtx);
  shared_buffer.push_back(data1);  // 최소한의 시간만 락 유지
}
```

### 2. 데이터 분할
```cpp
// 각 스레드가 독립적인 데이터 영역에 접근
std::for_each(
    std::execution::par_unseq,
    index.begin(),
    index.end(),
    [&](const int& i) {
      // result[i]는 각 스레드가 독립적으로 접근
      result[i] = process(input[i]);
    }
);
```

### 3. False Sharing 방지
```cpp
// 나쁜 예: 인접한 메모리 위치에 여러 스레드가 쓰기
struct Data {
  int thread1_data;
  int thread2_data;
};

// 좋은 예: 캐시 라인을 고려한 패딩
struct alignas(64) Data {
  int thread1_data;
  char padding[60];
  int thread2_data;
};
```

---

## 디버깅 팁

### 1. 데이터 경쟁 검출
```bash
# Thread Sanitizer 사용 (컴파일 시)
g++ -fsanitize=thread -g -o program program.cpp
./program
```

### 2. 데드락 방지
- 항상 같은 순서로 여러 락을 획득
- `std::lock`을 사용하여 동시에 여러 락 획득
- 타임아웃 설정 (`try_lock_for`)

### 3. 로깅
```cpp
#include <iostream>
#include <thread>

void log_thread_id(const std::string& msg) {
  std::cout << "[Thread " << std::this_thread::get_id() << "] "
            << msg << std::endl;
}
```

---

## 요약

| 기능 | 사용 도구 | 주요 용도 |
|------|----------|----------|
| 상호 배제 | `std::mutex` | 공유 자원 보호 |
| 조건 대기 | `std::condition_variable` | 이벤트 기반 동기화 |
| 병렬 루프 | `std::execution::par_unseq` | CPU 집약적 작업 가속 |
| 시간 측정 | `omp_get_wtime()` | 성능 프로파일링 |
| 자동 잠금 관리 | `std::lock_guard` | RAII 패턴 적용 |

---

## 참고사항

1. **스레드 안전성**: 공유 자원에 접근하는 모든 코드는 적절한 동기화가 필요합니다.

2. **병렬 실행 정책**: C++17의 실행 정책을 사용하면 컴파일러와 런타임이 자동으로 최적화합니다.

3. **락 순서**: 데드락을 방지하기 위해 항상 일관된 순서로 락을 획득하세요.

4. **예외 안전성**: `lock_guard`나 `unique_lock`을 사용하여 예외 발생 시에도 락이 해제되도록 보장하세요.

5. **성능**: 병렬화는 오버헤드가 있으므로, 충분히 큰 작업에만 적용하세요.
