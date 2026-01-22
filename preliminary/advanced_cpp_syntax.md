# 고급 C++ 문법 정리

S-Faster-Lio 프로젝트의 src와 include 폴더에서 사용되는 까다로운 C++ 문법을 정리한 문서입니다.

## 목차
1. [스마트 포인터 (Smart Pointers)](#1-스마트-포인터-smart-pointers)
2. [템플릿 (Templates)](#2-템플릿-templates)
3. [람다 함수 (Lambda Functions)](#3-람다-함수-lambda-functions)
4. [Move Semantics & Rvalue References](#4-move-semantics--rvalue-references)
5. [타입 추론 (auto, decltype)](#5-타입-추론-auto-decltype)
6. [타입 별칭 (typedef, using)](#6-타입-별칭-typedef-using)
7. [STL 컨테이너](#7-stl-컨테이너)
8. [매크로 (Macros)](#8-매크로-macros)
9. [RAII 패턴](#9-raii-패턴)
10. [네임스페이스 (Namespace)](#10-네임스페이스-namespace)
11. [constexpr & const](#11-constexpr--const)
12. [References & Pointers](#12-references--pointers)
13. [explicit & static](#13-explicit--static)
14. [구조체 초기화 및 Designated Initializers](#14-구조체-초기화-및-designated-initializers)

---

## 1. 스마트 포인터 (Smart Pointers)

스마트 포인터는 자동 메모리 관리를 제공하여 메모리 누수를 방지합니다.

### shared_ptr (공유 소유권)

```cpp
#include <memory>

// shared_ptr 생성
std::shared_ptr<PointCloudXYZI> ptr1(new PointCloudXYZI());

// make_shared 사용 (권장)
auto ptr2 = std::make_shared<PointCloudXYZI>();

// 복사: 참조 카운트 증가
std::shared_ptr<PointCloudXYZI> ptr3 = ptr2;

// 참조 카운트 확인
int count = ptr2.use_count();

// 포인터 접근
ptr2->points.push_back(point);
(*ptr2).points.size();

// reset: 포인터 해제
ptr2.reset();
ptr2.reset(new PointCloudXYZI());  // 새 객체로 교체
```

### unique_ptr (독점 소유권)

```cpp
// unique_ptr 생성
std::unique_ptr<MapType> ivox;

// make_unique 사용 (C++14)
ivox = std::make_unique<MapType>(ivox_options_);

// 소유권 이전 (복사 불가, 이동만 가능)
std::unique_ptr<MapType> ivox2 = std::move(ivox);
// 이제 ivox는 nullptr

// 포인터 접근
ivox->AddPoints(points);
```

### 프로젝트에서의 실제 사용

```cpp
// shared_ptr 예제
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
// PointCloudXYZI::Ptr는 boost::shared_ptr<PointCloudXYZI>의 별칭

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

// unique_ptr 예제
std::unique_ptr<MapType> ivox;
ivox = std::make_unique<MapType>(ivox_options_);
```

**왜 스마트 포인터를 사용하나?**
- 자동 메모리 해제 (소멸자에서 자동으로 delete 호출)
- 예외 안전성 보장
- 소유권 의미가 명확함

**사용 위치:**
- `src/laserMapping.cpp:70-75, 98, 554, 584` (shared_ptr)
- `src/laserMapping.cpp:81` (unique_ptr)
- `src/IMU_Processing.hpp:101, 116, 117` (reset)

---

## 2. 템플릿 (Templates)

템플릿은 타입에 독립적인 코드를 작성할 수 있게 해줍니다.

### 함수 템플릿

```cpp
// 기본 함수 템플릿
template <typename T>
void process(T value) {
  // T는 컴파일 타임에 결정됨
}

// 다중 타입 파라미터
template <typename T, typename U>
auto add(T a, U b) -> decltype(a + b) {
  return a + b;
}

// 프로젝트 예제: 좌표 변환
template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1>& pi, Matrix<T, 3, 1>& po) {
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(state_point.rot.matrix() *
               (state_point.offset_R_L_I.matrix() * p_body +
                state_point.offset_T_L_I) +
               state_point.pos);

  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

// 호출
Matrix<double, 3, 1> input, output;
pointBodyToWorld(input, output);  // T = double
```

### 프로젝트 예제: set_pose6d

```cpp
template <typename T>
auto set_pose6d(const double t,
                const Matrix<T, 3, 1>& a,
                const Matrix<T, 3, 1>& g,
                const Matrix<T, 3, 1>& v,
                const Matrix<T, 3, 1>& p,
                const Matrix<T, 3, 3>& R) {
  Pose6D rot_kp;
  rot_kp.offset_time = t;
  for (int i = 0; i < 3; i++) {
    rot_kp.acc[i] = a(i);
    rot_kp.gyr[i] = g(i);
    rot_kp.vel[i] = v(i);
    rot_kp.pos[i] = p(i);
    for (int j = 0; j < 3; j++)
      rot_kp.rot[i * 3 + j] = R(i, j);
  }
  return move(rot_kp);
}
```

### 클래스 템플릿

```cpp
// 기본 클래스 템플릿
template <typename T>
class MyClass {
 public:
  T value;
  void process(T input);
};

// 프로젝트 예제: VoxelMap
template <typename PointType = pcl::PointXYZ>
class VoxelMap {
 public:
  using KeyType = VoxelIndex;
  using NodeType = BasicVoxelNode<PointType>;
  using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

  void AddPoints(const PointVector& points_to_add);
  bool GetClosestPoint(const PointType& pt, PointType& closest_pt);
};

// 사용
VoxelMap<PointType> voxel_map;
```

### 템플릿 특수화

```cpp
// 일반 템플릿
template <typename T>
class Storage {
  T data;
};

// 부분 특수화
template <typename T>
class Storage<T*> {
  T* data;
  ~Storage() { delete data; }
};

// 완전 특수화
template <>
class Storage<bool> {
  unsigned char data;  // 메모리 최적화
};
```

**사용 위치:**
- `src/laserMapping.cpp:259-270, 419-429` (함수 템플릿)
- `include/common_lib.h:51-69, 77-111` (함수 템플릿)
- `include/voxmap/voxel_map.h:12-84` (클래스 템플릿)

---

## 3. 람다 함수 (Lambda Functions)

람다는 익명 함수를 정의하는 간결한 방법입니다.

### 기본 문법

```cpp
[capture](parameters) -> return_type { body }
```

### 캡처 방식

```cpp
int x = 10, y = 20;

// [=]: 값으로 캡처 (복사)
auto lambda1 = [=]() { return x + y; };

// [&]: 참조로 캡처
auto lambda2 = [&]() { x++; y++; };

// [x, &y]: x는 값으로, y는 참조로
auto lambda3 = [x, &y]() { y = x; };

// [this]: 클래스 멤버 접근
auto lambda4 = [this]() { return this->member; };

// [&, x]: 기본은 참조, x만 값으로
auto lambda5 = [&, x]() { return x + y; };

// [=, &y]: 기본은 값, y만 참조로
auto lambda6 = [=, &y]() { y = x; };
```

### 프로젝트 예제 1: 병렬 처리

```cpp
std::vector<int> index(feats_down_size);
std::iota(index.begin(), index.end(), 0);

std::for_each(
    std::execution::par_unseq,
    index.begin(),
    index.end(),
    [&](const int& i) {  // 참조로 캡처하여 외부 변수 접근
      PointType& point_body = feats_down_body->points[i];
      PointType point_world;

      V3D p_body(point_body.x, point_body.y, point_body.z);
      V3D p_global(x_.rot * (x_.offset_R_L_I * p_body + x_.offset_T_L_I) +
                   x_.pos);

      point_world.x = p_global(0);
      point_world.y = p_global(1);
      point_world.z = p_global(2);

      // 공유 변수 접근
      auto& points_near = Nearest_Points[i];
      if (ekfom_data.converge) {
        ivox.GetClosestPoint(point_world, points_near, NUM_MATCH_POINTS);
      }

      if (!point_selected_surf[i]) return;

      // ...
    }
);
```

### 프로젝트 예제 2: STL 알고리즘

```cpp
// std::for_each with lambda
std::for_each(
    nearby_grids_.begin(),
    nearby_grids_.end(),
    [&key, &candidates, &pt, this](const KeyType& delta) {
      auto dkey = key + delta;
      auto iter = grids_map_.find(dkey);
      if (iter != grids_map_.end()) {
        DistPoint dist_point;
        bool found = iter->second->second.NNPoint(pt, dist_point);
        if (found) {
          candidates.emplace_back(dist_point);
        }
      }
    }
);
```

### 반환 타입 추론

```cpp
// 반환 타입 자동 추론
auto lambda = [](int x, int y) { return x + y; };  // int 반환

// 명시적 반환 타입
auto lambda2 = [](int x, int y) -> double {
  return static_cast<double>(x) / y;
};
```

### mutable 람다

```cpp
int x = 10;

// 값으로 캡처된 변수를 수정하려면 mutable 필요
auto lambda = [x]() mutable {
  x++;  // OK: mutable이 있으면 캡처된 복사본을 수정 가능
  return x;
};

std::cout << lambda() << std::endl;  // 11
std::cout << x << std::endl;         // 10 (원본은 변하지 않음)
```

**사용 위치:**
- `include/esekfom.hpp:105-157` (병렬 처리 람다)
- `include/voxmap/voxel_map.h:100-111, 250-257, 264-290` (STL 알고리즘 람다)

---

## 4. Move Semantics & Rvalue References

Move semantics는 불필요한 복사를 피하고 성능을 향상시킵니다.

### Lvalue vs Rvalue

```cpp
int x = 10;      // x는 lvalue (이름이 있고 주소를 가짐)
int y = x + 5;   // x+5는 rvalue (임시값, 주소 없음)

int& ref = x;    // OK: lvalue reference
int& ref2 = 20;  // Error: rvalue를 lvalue reference에 바인딩 불가

int&& rref = 20; // OK: rvalue reference
```

### std::move

```cpp
std::vector<int> vec1 = {1, 2, 3, 4, 5};

// 복사 (느림)
std::vector<int> vec2 = vec1;

// 이동 (빠름)
std::vector<int> vec3 = std::move(vec1);
// vec1은 이제 비어있음 (이동 후 유효하지만 미지정 상태)
```

### 프로젝트 예제

```cpp
// set_pose6d 함수에서 move 사용
template <typename T>
auto set_pose6d(const double t,
                const Matrix<T, 3, 1>& a,
                const Matrix<T, 3, 1>& g,
                const Matrix<T, 3, 1>& v,
                const Matrix<T, 3, 1>& p,
                const Matrix<T, 3, 3>& R) {
  Pose6D rot_kp;
  // ... 데이터 채우기 ...
  return move(rot_kp);  // 복사 대신 이동으로 반환
}
```

### Move Constructor & Move Assignment

```cpp
class MyClass {
 public:
  int* data;

  // Move constructor
  MyClass(MyClass&& other) noexcept
      : data(other.data) {
    other.data = nullptr;  // 원본은 무효화
  }

  // Move assignment
  MyClass& operator=(MyClass&& other) noexcept {
    if (this != &other) {
      delete[] data;
      data = other.data;
      other.data = nullptr;
    }
    return *this;
  }
};
```

### Forwarding Reference (Universal Reference)

```cpp
template <typename T>
void process(T&& arg) {
  // T&&는 forwarding reference
  // lvalue나 rvalue 모두 받을 수 있음
  internal_process(std::forward<T>(arg));
}
```

**사용 위치:**
- `include/common_lib.h:68` (move 사용)
- `src/laserMapping.cpp:81` (unique_ptr와 move)

---

## 5. 타입 추론 (auto, decltype)

### auto 키워드

```cpp
// 기본 사용
auto x = 10;              // int
auto y = 3.14;            // double
auto str = "hello";       // const char*

// 포인터와 참조
auto ptr = &x;            // int*
auto& ref = x;            // int&
const auto& cref = x;     // const int&

// 복잡한 타입 간소화
std::vector<std::pair<int, std::string>> vec;
auto iter = vec.begin();  // 복잡한 iterator 타입을 auto로

// 프로젝트 예제
auto&& head = *(it_imu);
auto&& tail = *(it_imu + 1);

auto iter = grids_map_.find(key);
auto H = dyn_share.h_x;
```

### auto with const and references

```cpp
const std::vector<int> vec = {1, 2, 3};

auto a = vec[0];        // int (복사, const 제거)
const auto b = vec[0];  // const int
auto& c = vec[0];       // const int& (vec가 const이므로)
const auto& d = vec[0]; // const int&
```

### decltype

```cpp
int x = 10;
decltype(x) y = 20;  // y는 int 타입

// 반환 타입 추론
template <typename T, typename U>
auto add(T a, U b) -> decltype(a + b) {
  return a + b;
}

// C++14: 후행 반환 타입 없이도 가능
template <typename T, typename U>
auto add(T a, U b) {
  return a + b;
}
```

### decltype(auto)

```cpp
// auto: 값 타입으로 추론
// decltype(auto): 표현식의 정확한 타입(참조 포함)으로 추론

int x = 10;
int& get_ref() { return x; }

auto a = get_ref();           // int (복사)
decltype(auto) b = get_ref(); // int& (참조)
```

**사용 위치:**
- `src/laserMapping.cpp:237-238` (auto&&)
- `src/IMU_Processing.hpp:149-152, 163-164` (const auto&)
- `include/esekfom.hpp:269` (auto)
- `include/voxmap/voxel_map.h:99, 102, 106, 117` (auto)

---

## 6. 타입 별칭 (typedef, using)

### typedef (C++ 전통 방식)

```cpp
typedef unsigned int uint;
typedef std::vector<int> IntVector;

// 함수 포인터
typedef void (*FuncPtr)(int, double);

// 프로젝트 예제
typedef s_faster_lio::Pose6D Pose6D;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
typedef Vector3d V3D;
typedef Matrix3d M3D;
```

### using (C++11 권장 방식)

```cpp
using uint = unsigned int;
using IntVector = std::vector<int>;

// 템플릿 별칭 (typedef로는 불가능)
template <typename T>
using Vec = std::vector<T>;

Vec<int> v1;     // std::vector<int>
Vec<double> v2;  // std::vector<double>

// 프로젝트 예제
using KeyType = VoxelIndex;
using NodeType = BasicVoxelNode<PointType>;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using DistPoint = typename NodeType::DistPoint;
```

### typedef vs using 비교

```cpp
// 복잡한 함수 포인터
typedef void (*OldStyle)(int, double);
using NewStyle = void(*)(int, double);

// 템플릿 별칭 (using만 가능)
template <typename T>
using MyMap = std::map<std::string, T>;

MyMap<int> m1;      // std::map<std::string, int>
MyMap<double> m2;   // std::map<std::string, double>
```

**사용 위치:**
- `include/common_lib.h:25-32` (typedef)
- `include/voxmap/voxel_map.h:15-19` (using)
- `src/laserMapping.cpp:32` (using)

---

## 7. STL 컨테이너

### std::vector

```cpp
#include <vector>

std::vector<int> vec;

// 추가
vec.push_back(10);
vec.emplace_back(20);  // 생성자 인자를 직접 전달 (더 효율적)

// 접근
int x = vec[0];        // 범위 검사 없음 (빠름)
int y = vec.at(1);     // 범위 검사 있음 (안전)

// 크기
size_t size = vec.size();
bool empty = vec.empty();

// 메모리 예약
vec.reserve(1000);     // 재할당 방지

// 초기화
std::vector<int> vec2(10, 0);           // 10개의 0
std::vector<int> vec3 = {1, 2, 3, 4};   // 초기화 리스트
```

### std::deque (양방향 큐)

```cpp
#include <deque>

std::deque<int> dq;

// 양쪽 끝에서 추가/제거 (O(1))
dq.push_back(10);
dq.push_front(5);
dq.pop_back();
dq.pop_front();

// 프로젝트 예제
deque<double> time_buffer;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

time_buffer.push_back(timestamp);
double t = time_buffer.front();
time_buffer.pop_front();
```

### std::list (이중 연결 리스트)

```cpp
#include <list>

std::list<int> lst;

// 추가/제거 (O(1))
lst.push_back(10);
lst.push_front(5);

// 중간 삽입 (O(1) if iterator available)
auto it = lst.begin();
++it;
lst.insert(it, 7);

// 프로젝트 예제: LRU 캐시 구현
std::list<std::pair<KeyType, NodeType>> grids_cache_;

grids_cache_.push_front({key, NodeType(center, resolution)});
grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter);
grids_cache_.pop_back();
```

### std::unordered_map (해시맵)

```cpp
#include <unordered_map>

std::unordered_map<std::string, int> map;

// 삽입
map["key1"] = 10;
map.insert({"key2", 20});
map.emplace("key3", 30);  // 더 효율적

// 검색
auto it = map.find("key1");
if (it != map.end()) {
  int value = it->second;
}

// 프로젝트 예제
std::unordered_map<KeyType,
                   typename std::list<std::pair<KeyType, NodeType>>::iterator>
    grids_map_;

auto iter = grids_map_.find(key);
if (iter != grids_map_.end()) {
  iter->second->second.InsertPoint(pt);
}
```

### 컨테이너 선택 가이드

| 컨테이너 | 특징 | 사용 시기 |
|---------|------|----------|
| `vector` | 연속 메모리, 빠른 랜덤 접근 | 기본 선택, 끝에서 추가/삭제 |
| `deque` | 양쪽 끝에서 빠른 추가/삭제 | 양방향 큐 필요 시 |
| `list` | 이중 연결, 중간 삽입/삭제 빠름 | 중간 삽입/삭제 많을 때 |
| `unordered_map` | 해시 기반, O(1) 검색 | 빠른 키-값 검색 필요 시 |
| `map` | 트리 기반, 정렬됨 | 순서 유지 필요 시 |

**사용 위치:**
- `src/laserMapping.cpp:66-68` (deque)
- `include/voxmap/voxel_map.h:79-83` (unordered_map, list)

---

## 8. 매크로 (Macros)

### 기본 매크로

```cpp
#define PI_M (3.14159265358)
#define G_m_s2 (9.81)
#define NUM_MATCH_POINTS (5)

// 사용
double gravity = G_m_s2;
```

### 함수형 매크로

```cpp
// 간단한 매크로
#define MAX(a, b) ((a) > (b) ? (a) : (b))

// 주의: 부작용 가능
int x = 5;
int y = MAX(x++, 10);  // x++가 두 번 실행될 수 있음!

// 프로젝트 예제
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

// 사용
Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
// 전개: Lidar_T_wrt_IMU << extrinT[0], extrinT[1], extrinT[2];

M3D point_crossmat;
point_crossmat << SKEW_SYM_MATRX(point_);
```

### 조건부 컴파일

```cpp
#ifdef DEBUG
  std::cout << "Debug mode" << std::endl;
#endif

#ifndef HEADER_H
#define HEADER_H
// 헤더 내용
#endif

// 프로젝트 예제
#ifdef INNER_TIMER
  auto t1 = std::chrono::high_resolution_clock::now();
  // ... 타이밍 측정 코드 ...
  auto t2 = std::chrono::high_resolution_clock::now();
#endif
```

### Include Guards

```cpp
#ifndef COMMON_LIB_H1
#define COMMON_LIB_H1

// 헤더 파일 내용

#endif  // COMMON_LIB_H1
```

### 매크로 vs 함수/constexpr

```cpp
// 매크로 (나쁜 예)
#define SQUARE(x) x * x
int result = SQUARE(1 + 2);  // 1 + 2 * 1 + 2 = 5 (예상: 9)

// 함수 (좋은 예)
inline int square(int x) { return x * x; }
int result = square(1 + 2);  // 9

// constexpr (C++11, 더 좋은 예)
constexpr int square(int x) { return x * x; }
constexpr int result = square(3);  // 컴파일 타임 계산
```

**매크로를 피해야 하는 이유:**
1. 타입 체크 없음
2. 디버깅 어려움
3. 네임스페이스 무시
4. 예상치 못한 부작용

**매크로를 사용해야 하는 경우:**
1. 조건부 컴파일
2. Include guards
3. 문자열화 (`#` 연산자)

**사용 위치:**
- `include/common_lib.h:16-23` (상수 및 함수형 매크로)
- `src/laserMapping.cpp:28-30` (상수 매크로)
- `include/common_lib.h:1-2` (include guard)

---

## 9. RAII 패턴

RAII (Resource Acquisition Is Initialization)는 자원 관리의 핵심 패턴입니다.

### 개념

```cpp
// 나쁜 예: 수동 자원 관리
void bad_example() {
  int* ptr = new int[100];
  // ... 작업 ...
  if (error) {
    return;  // 메모리 누수!
  }
  delete[] ptr;
}

// 좋은 예: RAII
void good_example() {
  std::vector<int> vec(100);
  // ... 작업 ...
  if (error) {
    return;  // 자동으로 정리됨
  }
  // 스코프 끝에서 자동으로 정리됨
}
```

### RAII 예제: 스마트 포인터

```cpp
void process_data() {
  std::unique_ptr<Data> data(new Data());
  // 예외가 발생하더라도 자동으로 delete 호출됨
  data->process();
}  // 스코프 끝에서 자동 삭제
```

### RAII 예제: Lock Guard

```cpp
std::mutex mtx;

void thread_function() {
  std::lock_guard<std::mutex> lock(mtx);
  // 예외가 발생하더라도 자동으로 unlock 호출됨
  // 임계 영역
}  // 스코프 끝에서 자동 unlock
```

### 사용자 정의 RAII 클래스

```cpp
class FileHandle {
 public:
  FileHandle(const char* filename) {
    file_ = fopen(filename, "r");
    if (!file_) {
      throw std::runtime_error("Failed to open file");
    }
  }

  ~FileHandle() {
    if (file_) {
      fclose(file_);  // 자동 정리
    }
  }

  // 복사 방지
  FileHandle(const FileHandle&) = delete;
  FileHandle& operator=(const FileHandle&) = delete;

  // 이동 허용
  FileHandle(FileHandle&& other) : file_(other.file_) {
    other.file_ = nullptr;
  }

  FILE* get() { return file_; }

 private:
  FILE* file_;
};

// 사용
void read_file() {
  FileHandle file("data.txt");
  // 예외가 발생하더라도 파일 자동으로 닫힘
  // ...
}  // 스코프 끝에서 파일 자동으로 닫힘
```

**RAII의 장점:**
1. 자원 누수 방지
2. 예외 안전성
3. 코드 간결성
4. 실수 방지

**사용 위치:**
- 프로젝트 전체에서 스마트 포인터 사용
- mutex와 lock_guard 조합

---

## 10. 네임스페이스 (Namespace)

### 네임스페이스 정의

```cpp
namespace my_namespace {
  int value = 10;
  void function() {
    // ...
  }

  class MyClass {
    // ...
  };
}

// 사용
my_namespace::value = 20;
my_namespace::function();
my_namespace::MyClass obj;
```

### using 선언

```cpp
// 특정 이름만 가져오기
using std::cout;
using std::endl;

cout << "Hello" << endl;  // OK
std::cin >> x;             // std:: 여전히 필요
```

### using namespace (권장하지 않음)

```cpp
using namespace std;  // 전체 네임스페이스 가져오기

cout << "Hello" << endl;  // OK, but...
// 이름 충돌 위험!
```

### 프로젝트 예제

```cpp
// 네임스페이스 정의
namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

// using 선언
using namespace std;
using namespace Eigen;
```

### 익명 네임스페이스

```cpp
// 파일 스코프 전역 (static과 유사)
namespace {
  int internal_value = 10;  // 이 파일에서만 접근 가능

  void internal_function() {
    // ...
  }
}
```

**사용 위치:**
- `include/common_lib.h:13-14` (using namespace)
- `src/preprocess.h:69-79, 83-95, 97-124` (namespace 정의)
- `include/esekfom.hpp:21` (namespace esekfom)

---

## 11. constexpr & const

### const

```cpp
// 상수 변수
const int MAX_SIZE = 100;

// const 포인터
int x = 10, y = 20;
const int* ptr1 = &x;      // 포인터가 가리키는 값 수정 불가
int* const ptr2 = &x;      // 포인터 자체 수정 불가
const int* const ptr3 = &x; // 둘 다 수정 불가

// const 멤버 함수
class MyClass {
 public:
  int get_value() const {  // 멤버 변수 수정 불가
    return value_;
  }

  void set_value(int v) {  // 멤버 변수 수정 가능
    value_ = v;
  }

 private:
  int value_;
};
```

### constexpr (컴파일 타임 상수)

```cpp
// 컴파일 타임 상수
constexpr int BUFFER_SIZE = 1024;
constexpr double PI = 3.141592653589793;

// constexpr 함수 (컴파일 타임에 평가 가능)
constexpr int factorial(int n) {
  return n <= 1 ? 1 : (n * factorial(n - 1));
}

// 컴파일 타임에 계산됨
constexpr int result = factorial(5);  // 120

// 배열 크기로 사용 가능
int array[factorial(4)];  // int array[24]

// 프로젝트 예제
constexpr int STAT_PERIOD = 100000;
```

### const vs constexpr

```cpp
const int x = 10;          // 런타임 또는 컴파일 타임 상수
constexpr int y = 10;      // 컴파일 타임 상수만 가능

int arr1[x];               // 컴파일러에 따라 안 될 수 있음
int arr2[y];               // OK: constexpr는 항상 컴파일 타임

const int a = get_value(); // OK: 런타임에 결정
constexpr int b = get_value(); // Error: constexpr 함수여야 함
```

### const 함수 파라미터

```cpp
// const 참조: 복사 방지, 수정 방지
void process(const std::vector<int>& vec) {
  // vec를 읽기만 가능
}

// const 포인터
void process(const PointType* const pt) {
  // pt와 *pt 모두 수정 불가
}

// 프로젝트 예제
void pointBodyToWorld(PointType const* const pi, PointType* const po);
bool GetClosestPoint(const PointType& pt, PointType& closest_pt);
void AddPoints(const PointVector& points_to_add);
```

**사용 위치:**
- `src/laserMapping.cpp:40-42` (const 상수)
- `src/laserMapping.cpp:246` (const 함수 파라미터)
- `include/voxmap/voxel_map.h:181` (constexpr)

---

## 12. References & Pointers

### Lvalue Reference (&)

```cpp
int x = 10;
int& ref = x;  // ref는 x의 별칭

ref = 20;      // x도 20으로 변경됨
```

### Rvalue Reference (&&)

```cpp
int&& rref = 10;  // 임시값에 바인딩

std::string str = "hello";
std::string&& rref2 = std::move(str);
```

### 함수 파라미터로서의 참조

```cpp
// 값으로 전달 (복사)
void func1(std::vector<int> vec) {
  // vec는 복사본
}

// const 참조로 전달 (복사 방지, 읽기 전용)
void func2(const std::vector<int>& vec) {
  // 복사 없음, 수정 불가
}

// 참조로 전달 (복사 방지, 수정 가능)
void func3(std::vector<int>& vec) {
  // 복사 없음, 수정 가능
  vec.push_back(10);
}

// Rvalue 참조로 전달 (이동 의미론)
void func4(std::vector<int>&& vec) {
  // 소유권 이전 가능
}
```

### Pointer vs Reference

```cpp
int x = 10;

// 포인터
int* ptr = &x;
*ptr = 20;       // 역참조 필요
ptr = nullptr;   // 다시 할당 가능

// 참조
int& ref = x;
ref = 20;        // 직접 사용
// ref = y;      // 다시 바인딩 불가 (y의 값만 복사됨)
```

### 프로젝트 예제

```cpp
// const 참조 파라미터
void h_share_model(dyn_share_datastruct& ekfom_data,
                   PointCloudXYZI::Ptr& feats_down_body,
                   VoxelMap<PointType>& ivox,
                   vector<PointVector>& Nearest_Points,
                   bool extrinsic_est);

// auto&&: forwarding reference
auto&& head = *(it_imu);
auto&& tail = *(it_imu + 1);

// const 포인터
void pointBodyToWorld(PointType const* const pi, PointType* const po);
```

**사용 위치:**
- `src/laserMapping.cpp:237-238` (auto&&)
- `src/laserMapping.cpp:246` (const pointer)
- `include/esekfom.hpp:92-96` (참조 파라미터)

---

## 13. explicit & static

### explicit (암시적 변환 방지)

```cpp
class MyString {
 public:
  // explicit 없음: 암시적 변환 허용
  MyString(int size) : data_(new char[size]) {}

  ~MyString() { delete[] data_; }

 private:
  char* data_;
};

void process(MyString str) {
  // ...
}

int main() {
  process(10);  // OK: int -> MyString 암시적 변환
  MyString str = 10;  // OK: 암시적 변환
}
```

```cpp
class MyString {
 public:
  // explicit: 명시적 변환만 허용
  explicit MyString(int size) : data_(new char[size]) {}

  ~MyString() { delete[] data_; }

 private:
  char* data_;
};

void process(MyString str) {
  // ...
}

int main() {
  process(10);  // Error: 암시적 변환 불가
  MyString str = 10;  // Error: 암시적 변환 불가

  process(MyString(10));  // OK: 명시적 변환
  MyString str(10);  // OK: 직접 생성
}
```

### 프로젝트 예제

```cpp
// VoxelMap 생성자에서 explicit 사용
class VoxelMap {
 public:
  explicit VoxelMap(Options options) : options_(options) {
    options_.inv_resolution_ = 1.0 / options_.resolution_;
    GenerateNearbyGrids();
  }
};

// 사용
VoxelMap map(options);  // OK
// VoxelMap map = options;  // Error: explicit로 인해 불가
```

### static 멤버

```cpp
class Counter {
 public:
  static int count;  // 클래스 공유 변수

  Counter() { count++; }

  static int get_count() {  // static 멤버 함수
    return count;
    // this 포인터 없음
  }
};

// cpp 파일에서 초기화
int Counter::count = 0;

// 사용
Counter::count;          // 객체 없이 접근 가능
Counter::get_count();    // 객체 없이 호출 가능
```

### static 지역 변수

```cpp
void function() {
  static int call_count = 0;  // 한 번만 초기화
  call_count++;
  std::cout << "Called " << call_count << " times" << std::endl;
}

// 프로젝트 예제
void publish_odometry(const ros::Publisher& pubOdomAftMapped) {
  // ...
  static tf::TransformBroadcaster br;  // 한 번만 생성
  tf::Transform transform;
  // ...
  br.sendTransform(transform);
}
```

**사용 위치:**
- `include/voxmap/voxel_map.h:39` (explicit)
- `src/laserMapping.cpp:449` (static 지역 변수)
- `src/preprocess.cpp:131-132` (static 지역 변수)

---

## 14. 구조체 초기화 및 Designated Initializers

### 기본 초기화

```cpp
struct Point {
  double x;
  double y;
  double z;
};

// C++11 uniform initialization
Point p1{1.0, 2.0, 3.0};

// 기본값이 있는 구조체
struct Config {
  int width = 800;
  int height = 600;
  bool fullscreen = false;
};

Config cfg1;  // 기본값 사용
Config cfg2{1024, 768, true};  // 커스텀 값
```

### 프로젝트 예제: state_ikfom

```cpp
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

// 기본값으로 초기화
state_ikfom state;
```

### 프로젝트 예제: MeasureGroup

```cpp
struct MeasureGroup {
  MeasureGroup() {
    lidar_beg_time = 0.0;
    this->lidar.reset(new PointCloudXYZI());
  };

  double lidar_beg_time;
  double lidar_end_time;
  PointCloudXYZI::Ptr lidar;
  deque<sensor_msgs::Imu::ConstPtr> imu;
};
```

### C++20 Designated Initializers (참고용)

```cpp
// C++20부터 지원
struct Options {
  float resolution_ = 0.5;
  float inv_resolution_ = 10.0;
  int capacity_ = 1000000;
};

// Designated initializers
Options opt{
  .resolution_ = 1.0,
  .capacity_ = 500000
  // inv_resolution_은 기본값 사용
};
```

**사용 위치:**
- `include/use-ikfom.hpp:18-28` (기본값 초기화)
- `include/common_lib.h:39-49` (생성자 초기화)
- `include/voxmap/voxel_map.h:28-33` (Options 구조체)

---

## 추가 고급 개념

### std::pair & std::tuple

```cpp
// pair: 두 값 묶기
std::pair<int, std::string> p = {1, "hello"};
int first = p.first;
std::string second = p.second;

// make_pair
auto p2 = std::make_pair(10, "world");

// tuple: 여러 값 묶기
std::tuple<int, double, std::string> t = {1, 3.14, "tuple"};
int x = std::get<0>(t);
double y = std::get<1>(t);
std::string z = std::get<2>(t);

// C++17 structured binding
auto [a, b, c] = t;
```

### std::optional (C++17)

```cpp
#include <optional>

std::optional<int> maybe_value;

if (condition) {
  maybe_value = 42;
}

if (maybe_value.has_value()) {
  int value = maybe_value.value();
  // 또는
  int value = *maybe_value;
}

// value_or: 기본값 제공
int x = maybe_value.value_or(0);
```

### Range-based for loop

```cpp
std::vector<int> vec = {1, 2, 3, 4, 5};

// 값으로 순회
for (int x : vec) {
  std::cout << x << " ";
}

// 참조로 순회
for (int& x : vec) {
  x *= 2;  // 원본 수정
}

// const 참조로 순회 (권장)
for (const auto& x : vec) {
  std::cout << x << " ";
}

// 프로젝트 예제
for (const auto& imu : meas.imu) {
  const auto& imu_acc = imu->linear_acceleration;
  const auto& gyr_acc = imu->angular_velocity;
  // ...
}

for (const KeyType& delta : nearby_grids_) {
  // ...
}
```

---

## 모범 사례 요약

1. **스마트 포인터 사용**: 원시 포인터 대신 `shared_ptr`, `unique_ptr` 사용
2. **const 참조**: 큰 객체를 함수에 전달할 때 `const&` 사용
3. **auto 사용**: 복잡한 타입은 `auto`로 간소화
4. **RAII 패턴**: 자원 관리는 객체 생명주기에 맡기기
5. **범위 기반 for**: 명시적 iterator 대신 range-based for 사용
6. **emplace 사용**: `push_back` 대신 `emplace_back` 사용 (더 효율적)
7. **move 의미론**: 큰 객체 이동 시 `std::move` 사용
8. **explicit 생성자**: 단일 인자 생성자는 `explicit`으로 선언
9. **const 멤버 함수**: 멤버를 수정하지 않는 함수는 `const`로 선언
10. **using over typedef**: 타입 별칭은 `using` 사용 (특히 템플릿)

---

## 디버깅 팁

### 타입 확인

```cpp
#include <typeinfo>
#include <iostream>

template <typename T>
void print_type(T&& arg) {
  std::cout << typeid(arg).name() << std::endl;
}

// 사용
auto x = 10;
print_type(x);  // 타입 출력
```

### static_assert

```cpp
// 컴파일 타임 체크
static_assert(sizeof(int) == 4, "Int must be 4 bytes");

template <typename T>
void process(T value) {
  static_assert(std::is_integral<T>::value, "T must be integral type");
  // ...
}
```

### 컴파일러 경고 활용

```bash
# 모든 경고 활성화
g++ -Wall -Wextra -pedantic -o program program.cpp

# 추가 검사
g++ -Wshadow -Wconversion -Wsign-conversion
```
