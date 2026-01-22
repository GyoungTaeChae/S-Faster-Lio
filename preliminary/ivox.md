# iVox (Incremental Voxel Map) 정리

S-Faster-Lio 프로젝트에서 사용되는 iVox의 핵심 원리와 메서드를 상세히 정리한 문서입니다.

## 목차
1. [iVox 개요](#1-ivox-개요)
2. [자료구조 설계](#2-자료구조-설계)
3. [VoxelIndex (복셀 인덱스)](#3-voxelindex-복셀-인덱스)
4. [BasicVoxelNode (복셀 노드)](#4-basicvoxelnode-복셀-노드)
5. [VoxelMap 클래스](#5-voxelmap-클래스)
6. [해시 함수](#6-해시-함수)
7. [LRU 캐시 메커니즘](#7-lru-캐시-메커니즘)
8. [핵심 메서드](#8-핵심-메서드)
9. [근접 복셀 검색](#9-근접-복셀-검색)
10. [병렬 처리 최적화](#10-병렬-처리-최적화)
11. [성능 분석](#11-성능-분석)
12. [사용 예제](#12-사용-예제)

---

## 1. iVox 개요

### iVox란?

**iVox (Incremental Voxel Map)**는 3D 공간을 복셀(voxel)로 분할하여 포인트 클라우드를 효율적으로 저장하고 검색하는 자료구조입니다.

**핵심 특징:**
1. **해시 기반 복셀 맵**: unordered_map으로 O(1) 접근 시간
2. **LRU 캐시**: 최근 사용 순서로 복셀 관리, 메모리 효율성
3. **병렬 처리**: C++17 parallel algorithms 활용
4. **증분식 업데이트**: 새로운 포인트를 즉시 추가 가능
5. **다중 근접 검색**: CENTER, NEARBY6, NEARBY18, NEARBY26

### Faster-LIO의 iVox와의 차이

S-Faster-Lio의 iVox는 Faster-LIO의 구현을 간소화한 버전입니다:

| 항목 | Faster-LIO | S-Faster-Lio |
|------|------------|--------------|
| 템플릿 파라미터 | 5개 (IVoxType, DistType, etc.) | 1개 (PointType) |
| 복셀 타입 | LINEAR, PHC 지원 | LINEAR만 지원 |
| 키 타입 | Eigen::Matrix<int, 3, 1> | VoxelIndex 클래스 |
| 해시 함수 | 동일 | 동일 (유지) |

**장점:**
- 코드가 간결하고 이해하기 쉬움
- VoxelIndex 클래스로 인간친화적인 인터페이스
- LINEAR 버전만 사용하므로 성능 최적화 가능

**사용 위치:**
- `include/voxmap/voxel_map.h` - VoxelMap 클래스
- `include/voxmap/voxel_index.h` - VoxelIndex 클래스
- `include/voxmap/basic_voxel_node.h` - BasicVoxelNode 클래스
- `src/laserMapping.cpp:332` - iVox 초기화 및 사용

---

## 2. 자료구조 설계

### 전체 구조

```
VoxelMap
├── grids_map_: unordered_map<VoxelIndex, Iterator>
│   └── Key: VoxelIndex (복셀의 3D 좌표)
│   └── Value: Iterator (grids_cache_의 위치)
│
├── grids_cache_: list<pair<VoxelIndex, BasicVoxelNode>>
│   └── LRU 캐시 (가장 최근 사용된 것이 앞쪽)
│   └── pair<Key, Value>
│       ├── Key: VoxelIndex
│       └── Value: BasicVoxelNode (포인트 저장)
│
└── nearby_grids_: vector<VoxelIndex>
    └── 근접 복셀의 오프셋 (예: NEARBY6)
```

### 설계 철학

1. **해시 맵 (grids_map_)**
   - 복셀 인덱스로 빠른 검색 (O(1))
   - 직접 값을 저장하지 않고 캐시의 iterator 저장

2. **LRU 캐시 (grids_cache_)**
   - 최근 접근한 복셀을 앞쪽에 유지
   - 오래된 복셀은 자동으로 제거 (capacity 초과 시)
   - std::list로 O(1) splice 연산

3. **근접 그리드 (nearby_grids_)**
   - 미리 계산된 오프셋으로 근접 복셀 탐색
   - 중복 계산 방지

**사용 위치:**
- `include/voxmap/voxel_map.h:79-83`

---

## 3. VoxelIndex (복셀 인덱스)

### 클래스 정의

```cpp
class VoxelIndex {
 public:
  VoxelIndex() : coords_{0, 0, 0} {}
  VoxelIndex(int x, int y, int z) : coords_{x, y, z} {}

  int x() const { return coords_[0]; }
  int y() const { return coords_[1]; }
  int z() const { return coords_[2]; }

 private:
  int coords_[3];
};
```

### 역할

**VoxelIndex는 3D 공간에서 복셀의 위치를 나타내는 정수 좌표입니다.**

예시:
- 해상도 0.5m인 경우
- 포인트 (1.2, 0.8, -0.3)
- VoxelIndex = (2, 2, -1) = (round(1.2/0.5), round(0.8/0.5), round(-0.3/0.5))

### 주요 연산자

#### [1] 비교 연산자

```cpp
bool operator==(const VoxelIndex& other) const {
  return coords_[0] == other.coords_[0] &&
         coords_[1] == other.coords_[1] &&
         coords_[2] == other.coords_[2];
}

bool operator<(const VoxelIndex& other) const {
  if (coords_[0] != other.coords_[0]) return coords_[0] < other.coords_[0];
  if (coords_[1] != other.coords_[1]) return coords_[1] < other.coords_[1];
  return coords_[2] < other.coords_[2];
}
```

**용도:**
- `==`: unordered_map에서 키 비교
- `<`: 정렬 (필요시)

#### [2] 덧셈 연산자 (근접 복셀 계산)

```cpp
VoxelIndex operator+(const VoxelIndex& other) const {
  return VoxelIndex(coords_[0] + other.coords_[0],
                    coords_[1] + other.coords_[1],
                    coords_[2] + other.coords_[2]);
}
```

**용도:**
- 현재 복셀 + 오프셋 = 근접 복셀
- 예: VoxelIndex(5, 3, 2) + VoxelIndex(1, 0, 0) = VoxelIndex(6, 3, 2)

**사용 위치:**
- `include/voxmap/voxel_map.h:102` (GetClosestPoint에서 근접 복셀 계산)

#### [3] 해시 함수

```cpp
std::size_t hash() const {
  return size_t(((coords_[0]) * 73856093) ^
                ((coords_[1]) * 471943) ^
                ((coords_[2]) * 83492791)) % 10000000;
}
```

**원리:**
- 공간 해시 함수 (Spatial Hashing)
- 큰 소수로 곱한 후 XOR 연산
- Faster-LIO와 동일한 구현

**특징:**
- 균등 분포: 가까운 복셀도 다른 해시값
- 빠른 계산: 곱셈과 XOR만 사용

**사용 위치:**
- `include/voxmap/voxel_index.h:41-45`
- `include/voxmap/voxel_index.h:73-78` (std::hash 특수화)

### 정적 팩토리 메서드

#### FromEigenVector

```cpp
template <typename EigenVector>
static VoxelIndex FromEigenVector(const EigenVector& eigen, const double& s = 1.0) {
  return VoxelIndex(static_cast<int>(std::round(eigen[0] * s)),
                    static_cast<int>(std::round(eigen[1] * s)),
                    static_cast<int>(std::round(eigen[2] * s)));
}
```

**용도:**
- Eigen 벡터를 VoxelIndex로 변환
- `s`: inverse resolution (1/복셀 크기)

**예시:**
```cpp
Eigen::Vector3d pos(1.2, 0.8, -0.3);
double inv_res = 1.0 / 0.5;  // 해상도 0.5m
VoxelIndex idx = VoxelIndex::FromEigenVector(pos, inv_res);
// idx = (2, 2, -1)
```

#### FromPclPoint

```cpp
template <typename PclPointType>
static VoxelIndex FromPclPoint(const PclPointType& point, const double& s = 1.0) {
  return VoxelIndex(static_cast<int>(std::round(point.x * s)),
                    static_cast<int>(std::round(point.y * s)),
                    static_cast<int>(std::round(point.z * s)));
}
```

**용도:**
- PCL 포인트를 VoxelIndex로 변환

**사용 위치:**
- `include/voxmap/voxel_map.h:99` (GetClosestPoint)

---

## 4. BasicVoxelNode (복셀 노드)

### 클래스 정의

```cpp
template <typename PointT>
class BasicVoxelNode {
 public:
  BasicVoxelNode() = default;
  BasicVoxelNode(const PointT& center, const float& side_length) {}

  void InsertPoint(const PointT& pt);
  bool Empty() const;
  std::size_t Size() const;
  PointT GetPoint(const std::size_t idx) const;
  int KNNPointByCondition(std::vector<DistPoint>& dis_points,
                          const PointT& point,
                          const int& K,
                          const double& max_range);

 private:
  std::vector<PointT> points_;
};
```

### 역할

**BasicVoxelNode는 하나의 복셀 내부에 저장된 포인트들을 관리합니다.**

- 각 복셀은 여러 개의 포인트를 저장할 수 있음
- 단순히 std::vector로 관리 (LINEAR 버전)

### DistPoint 구조체

```cpp
struct DistPoint {
  double dist = 0;                  // 거리
  BasicVoxelNode* node = nullptr;   // 노드 포인터
  int idx = 0;                      // 포인트 인덱스

  DistPoint(const double d, BasicVoxelNode* n, const int i)
      : dist(d), node(n), idx(i) {}

  PointT Get() { return node->GetPoint(idx); }
  bool operator<(const DistPoint& rhs) { return dist < rhs.dist; }
};
```

**역할:**
- 거리 정보와 포인트를 함께 저장
- KNN 검색 결과를 담는 컨테이너
- 정렬을 위한 비교 연산자 제공

### 핵심 메서드

#### InsertPoint

```cpp
void InsertPoint(const PointT& pt) {
  points_.emplace_back(pt);
}
```

**역할:**
- 새로운 포인트를 복셀에 추가
- O(1) 연산 (vector의 push_back)

**사용 위치:**
- `include/voxmap/voxel_map.h:278, 285` (AddPoints)

#### KNNPointByCondition

```cpp
int KNNPointByCondition(std::vector<DistPoint>& dis_points,
                        const PointT& point,
                        const int& K,
                        const double& max_range) {
  std::size_t old_size = dis_points.size();

  // [1] 복셀 내 모든 포인트 순회
  for (const auto& pt : points_) {
    double d = distance2(pt, point);  // 거리 제곱 계산

    // [2] 범위 내 포인트만 추가
    if (d < max_range * max_range) {
      dis_points.emplace_back(DistPoint(d, this, &pt - points_.data()));
    }
  }

  // [3] K개만 유지 (nth_element로 부분 정렬)
  if (old_size + K < dis_points.size()) {
    std::nth_element(dis_points.begin() + old_size,
                     dis_points.begin() + old_size + K - 1,
                     dis_points.end());
    dis_points.resize(old_size + K);
  }

  return dis_points.size();
}
```

**알고리즘:**
1. 복셀 내 모든 포인트와의 거리 계산
2. max_range 이내인 포인트만 선택
3. nth_element로 K개만 유지 (부분 정렬)

**최적화:**
- 거리 제곱을 사용 (sqrt 계산 생략)
- nth_element는 O(n) (전체 정렬 O(nlogn)보다 빠름)

**distance2 함수:**
```cpp
template <typename PointT>
inline double distance2(const PointT& pt1, const PointT& pt2) {
  Eigen::Vector3f d = pt1.getVector3fMap() - pt2.getVector3fMap();
  return d.squaredNorm();
}
```

**사용 위치:**
- `include/voxmap/basic_voxel_node.h:83-156`
- `include/voxmap/voxel_map.h:148` (GetClosestPoint에서 호출)

---

## 5. VoxelMap 클래스

### 클래스 정의

```cpp
template <typename PointType = pcl::PointXYZ>
class VoxelMap {
 public:
  using KeyType = VoxelIndex;
  using NodeType = BasicVoxelNode<PointType>;
  using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
  using DistPoint = typename NodeType::DistPoint;

  enum class NearbyType {
    CENTER,    // 중심만
    NEARBY6,   // 6개 인접 (면 공유)
    NEARBY18,  // 18개 인접 (면+모서리 공유)
    NEARBY26,  // 26개 인접 (면+모서리+꼭짓점 공유)
  };

  struct Options {
    float resolution_ = 0.5;                        // 복셀 해상도
    float inv_resolution_ = 10.0;                   // 역 해상도
    NearbyType nearby_type_ = NearbyType::NEARBY6;  // 근접 타입
    std::size_t capacity_ = 1000000;                // 최대 용량
  };

  explicit VoxelMap(Options options);
  void AddPoints(const PointVector& points_to_add);
  bool GetClosestPoint(const PointType& pt, PointType& closest_pt);
  bool GetClosestPoint(const PointType& pt, PointVector& closest_pt,
                       int max_num = 5, double max_range = 5.0);
  bool GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud);

 private:
  void GenerateNearbyGrids();
  KeyType Pos2Grid(const PointType& pt) const;

  Options options_;
  std::unordered_map<KeyType, typename std::list<std::pair<KeyType, NodeType>>::iterator>
      grids_map_;
  std::list<std::pair<KeyType, NodeType>> grids_cache_;
  std::vector<KeyType> nearby_grids_;
};
```

### Options 구조체

| 필드 | 타입 | 기본값 | 의미 |
|------|------|--------|------|
| resolution_ | float | 0.5 | 복셀 크기 (m) |
| inv_resolution_ | float | 10.0 | 1/resolution (자동 계산) |
| nearby_type_ | NearbyType | NEARBY6 | 근접 복셀 검색 범위 |
| capacity_ | size_t | 1000000 | 최대 복셀 개수 |

**사용 예:**
```cpp
VoxelMap<PointType>::Options options;
options.resolution_ = 0.2;                     // 20cm 복셀
options.nearby_type_ = VoxelMap<PointType>::NearbyType::NEARBY6;
options.capacity_ = 500000;                    // 최대 50만 복셀

VoxelMap<PointType> ivox(options);
```

**사용 위치:**
- `src/laserMapping.cpp:332-335` (iVox 초기화)

---

## 6. 해시 함수

### 공간 해시 (Spatial Hashing)

```cpp
std::size_t hash() const {
  return size_t(((coords_[0]) * 73856093) ^
                ((coords_[1]) * 471943) ^
                ((coords_[2]) * 83492791)) % 10000000;
}
```

### 원리

**공간 해시는 3D 정수 좌표를 1D 해시값으로 매핑하는 함수입니다.**

1. **큰 소수 곱셈**
   - 73856093, 471943, 83492791은 큰 소수
   - 각 축에 다른 소수를 곱하여 충돌 방지

2. **XOR 연산**
   - 세 값을 XOR로 결합
   - 비트 분포를 균등하게 만듦

3. **모듈로 연산**
   - % 10000000으로 범위 제한
   - 해시 테이블 크기에 맞춤

### 예시

```cpp
VoxelIndex idx1(100, 200, 300);
VoxelIndex idx2(100, 200, 301);

size_t hash1 = idx1.hash();
size_t hash2 = idx2.hash();

// hash1 = ((100 * 73856093) ^ (200 * 471943) ^ (300 * 83492791)) % 10000000
//       = (7385609300 ^ 94388600 ^ 25047837300) % 10000000
//       = 22547918648 % 10000000
//       = 2547918648

// hash2는 완전히 다른 값
```

### 특징

**장점:**
1. **빠른 계산**: 곱셈, XOR, 모듈로만 사용
2. **균등 분포**: 인접한 복셀도 다른 해시값
3. **충돌 감소**: 큰 소수 사용으로 패턴 회피

**단점:**
- 완벽한 해시 함수는 아님 (충돌 가능)
- unordered_map의 체이닝으로 충돌 처리

**사용 위치:**
- `include/voxmap/voxel_index.h:41-45`

---

## 7. LRU 캐시 메커니즘

### LRU (Least Recently Used)

**최근에 사용되지 않은 복셀을 자동으로 제거하여 메모리를 효율적으로 관리합니다.**

### 자료구조

```cpp
// 해시 맵: VoxelIndex → list의 iterator
std::unordered_map<KeyType, typename std::list<std::pair<KeyType, NodeType>>::iterator>
    grids_map_;

// LRU 캐시: list<pair<VoxelIndex, BasicVoxelNode>>
std::list<std::pair<KeyType, NodeType>> grids_cache_;
```

### 동작 원리

```
초기 상태 (capacity = 3):
grids_map_   = {}
grids_cache_ = []

[1] 포인트 추가: VoxelIndex(1,1,1)
grids_map_   = {(1,1,1) → iter1}
grids_cache_ = [(1,1,1, Node1)]
                 ↑ 가장 최근

[2] 포인트 추가: VoxelIndex(2,2,2)
grids_map_   = {(1,1,1) → iter2, (2,2,2) → iter1}
grids_cache_ = [(2,2,2, Node2), (1,1,1, Node1)]
                 ↑ 가장 최근

[3] 포인트 추가: VoxelIndex(3,3,3)
grids_map_   = {(1,1,1) → iter3, (2,2,2) → iter2, (3,3,3) → iter1}
grids_cache_ = [(3,3,3, Node3), (2,2,2, Node2), (1,1,1, Node1)]
                 ↑ 가장 최근

[4] 포인트 추가: VoxelIndex(4,4,4) - 용량 초과!
- 가장 오래된 (1,1,1) 제거
grids_map_   = {(2,2,2) → iter3, (3,3,3) → iter2, (4,4,4) → iter1}
grids_cache_ = [(4,4,4, Node4), (3,3,3, Node3), (2,2,2, Node2)]
                 ↑ 가장 최근

[5] 포인트 추가: VoxelIndex(2,2,2) - 이미 존재
- (2,2,2)를 앞쪽으로 이동 (splice)
grids_map_   = {(2,2,2) → iter1, (3,3,3) → iter3, (4,4,4) → iter2}
grids_cache_ = [(2,2,2, Node2), (4,4,4, Node4), (3,3,3, Node3)]
                 ↑ 가장 최근 (splice로 이동)
```

### AddPoints 구현

```cpp
void AddPoints(const PointVector& points_to_add) {
  std::for_each(std::execution::unseq, points_to_add.begin(), points_to_add.end(),
                [this](const auto& pt) {
    auto key = Pos2Grid(pt);  // 포인트 → VoxelIndex

    auto iter = grids_map_.find(key);

    if (iter == grids_map_.end()) {
      // [Case 1] 새로운 복셀
      PointType center;
      center.x = key.x() * options_.resolution_;
      center.y = key.y() * options_.resolution_;
      center.z = key.z() * options_.resolution_;

      // 캐시 앞쪽에 추가
      grids_cache_.push_front({key, NodeType(center, options_.resolution_)});
      grids_map_.insert({key, grids_cache_.begin()});

      // 포인트 삽입
      grids_cache_.front().second.InsertPoint(pt);

      // 용량 초과 시 가장 오래된 것 제거
      if (grids_map_.size() >= options_.capacity_) {
        grids_map_.erase(grids_cache_.back().first);
        grids_cache_.pop_back();
      }
    } else {
      // [Case 2] 기존 복셀
      // 포인트 삽입
      iter->second->second.InsertPoint(pt);

      // 캐시 앞쪽으로 이동 (최근 사용)
      grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
      grids_map_[key] = grids_cache_.begin();
    }
  });
}
```

### splice 연산

**std::list::splice는 O(1)에 요소를 이동시킵니다.**

```cpp
// 문법
list.splice(destination_position, source_list, source_position);

// 예시
grids_cache_.splice(grids_cache_.begin(),    // 앞쪽으로
                   grids_cache_,              // 같은 list에서
                   iter->second);             // 이 위치의 요소를
```

**장점:**
- O(1) 연산 (노드 포인터만 변경)
- 메모리 복사 없음
- 최근 사용 순서 유지

**사용 위치:**
- `include/voxmap/voxel_map.h:262-290` (AddPoints)
- `include/voxmap/voxel_map.h:286` (splice 호출)

---

## 8. 핵심 메서드

### AddPoints

```cpp
void AddPoints(const PointVector& points_to_add);
```

**역할:**
- 포인트 클라우드를 복셀 맵에 추가

**알고리즘:**
1. 각 포인트의 VoxelIndex 계산
2. 해당 복셀이 존재하면 포인트 추가 + splice로 앞쪽 이동
3. 존재하지 않으면 새 복셀 생성 + 포인트 추가
4. 용량 초과 시 가장 오래된 복셀 제거

**병렬 처리:**
- `std::execution::unseq` 사용
- 벡터화 최적화 (SIMD)

**사용 위치:**
- `src/laserMapping.cpp:549, 667` (맵에 포인트 추가)

---

### GetClosestPoint (단일 포인트)

```cpp
bool GetClosestPoint(const PointType& pt, PointType& closest_pt);
```

**역할:**
- 주어진 포인트에 가장 가까운 맵 포인트 찾기

**알고리즘:**
```cpp
bool GetClosestPoint(const PointType& pt, PointType& closest_pt) {
  std::vector<DistPoint> candidates;

  // [1] 포인트의 VoxelIndex 계산
  auto key = KeyType::FromPclPoint(pt, options_.inv_resolution_);

  // [2] 근접 복셀 순회
  std::for_each(nearby_grids_.begin(), nearby_grids_.end(),
                [&](const KeyType& delta) {
    auto dkey = key + delta;  // 현재 복셀 + 오프셋
    auto iter = grids_map_.find(dkey);

    if (iter != grids_map_.end()) {
      // [3] 복셀 내 최근접점 찾기
      DistPoint dist_point;
      bool found = iter->second->second.NNPoint(pt, dist_point);
      if (found) {
        candidates.emplace_back(dist_point);
      }
    }
  });

  // [4] 후보 중 최소값 선택
  if (candidates.empty()) return false;

  auto iter = std::min_element(candidates.begin(), candidates.end());
  closest_pt = iter->Get();
  return true;
}
```

**예시:**
```
포인트 pt = (1.1, 0.9, 0.2)
resolution = 0.5

[1] key = (2, 2, 0)

[2] nearby_grids_ (NEARBY6) = {(0,0,0), (-1,0,0), (1,0,0), (0,1,0), (0,-1,0), (0,0,-1), (0,0,1)}

[3] 검색할 복셀:
    - (2,2,0) + (0,0,0)   = (2,2,0)   ✓
    - (2,2,0) + (-1,0,0)  = (1,2,0)   ✓
    - (2,2,0) + (1,0,0)   = (3,2,0)   ✓
    - (2,2,0) + (0,1,0)   = (2,3,0)   ✓
    - (2,2,0) + (0,-1,0)  = (2,1,0)   ✓
    - (2,2,0) + (0,0,-1)  = (2,2,-1)  ✓
    - (2,2,0) + (0,0,1)   = (2,2,1)   ✓

[4] 각 복셀에서 가장 가까운 포인트를 candidates에 추가

[5] candidates 중 최소 거리 선택
```

**사용 위치:**
- `include/voxmap/voxel_map.h:95-120`

---

### GetClosestPoint (조건부 K개)

```cpp
bool GetClosestPoint(const PointType& pt,
                     PointVector& closest_pt,
                     int max_num = 5,
                     double max_range = 5.0);
```

**역할:**
- 주어진 포인트에 가까운 K개의 포인트 찾기
- max_range 이내의 포인트만 선택

**알고리즘:**
```cpp
bool GetClosestPoint(const PointType& pt, PointVector& closest_pt,
                     int max_num, double max_range) {
  std::vector<DistPoint> candidates;
  candidates.reserve(max_num * nearby_grids_.size());

  auto key = Pos2Grid(pt);

  // [1] 근접 복셀에서 조건을 만족하는 포인트 수집
  for (const KeyType& delta : nearby_grids_) {
    auto dkey = key + delta;
    auto iter = grids_map_.find(dkey);

    if (iter != grids_map_.end()) {
      // KNNPointByCondition: 복셀 내에서 조건 만족하는 포인트 찾기
      iter->second->second.KNNPointByCondition(candidates, pt, max_num, max_range);
    }
  }

  if (candidates.empty()) return false;

  // [2] K개만 선택 (nth_element로 부분 정렬)
  if (candidates.size() > max_num) {
    std::nth_element(candidates.begin(), candidates.begin() + max_num - 1,
                     candidates.end());
    candidates.resize(max_num);
  }

  // [3] 최소값을 첫 번째로 이동
  std::nth_element(candidates.begin(), candidates.begin(), candidates.end());

  // [4] 결과 저장
  closest_pt.clear();
  for (auto& it : candidates) {
    closest_pt.emplace_back(it.Get());
  }

  return !closest_pt.empty();
}
```

**nth_element 설명:**

```cpp
std::nth_element(begin, nth, end);
```

- `nth` 위치에 정렬 후 n번째 요소를 배치
- `nth` 앞쪽은 `nth`보다 작거나 같음
- `nth` 뒤쪽은 `nth`보다 크거나 같음
- **시간 복잡도**: O(n) (평균)

**예시:**
```cpp
vector<int> v = {5, 3, 8, 1, 9, 2, 7};
nth_element(v.begin(), v.begin() + 3, v.end());
// 결과: v = {1, 2, 3, 5, 8, 9, 7} (또는 유사한 배치)
//                  ↑ 4번째 요소 (0-indexed로 3)
// 3 앞쪽 (1, 2, 3)은 모두 3 이하
// 3 뒤쪽 (5, 8, 9, 7)은 모두 3 이상
```

**사용 위치:**
- `include/voxmap/voxel_map.h:123-201`
- `include/esekfom.hpp:124` (ESKF update에서 최근접점 찾기)

---

### GetClosestPoint (포인트 클라우드)

```cpp
bool GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud);
```

**역할:**
- 포인트 클라우드의 각 포인트에 대해 최근접점 찾기

**알고리즘:**
```cpp
bool GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud) {
  // [1] 인덱스 배열 생성
  std::vector<size_t> index(cloud.size());
  for (int i = 0; i < cloud.size(); ++i) {
    index[i] = i;
  }

  closest_cloud.resize(cloud.size());

  // [2] 병렬 처리
  std::for_each(std::execution::par_unseq, index.begin(), index.end(),
                [&](size_t idx) {
    PointType pt;
    if (GetClosestPoint(cloud[idx], pt)) {
      closest_cloud[idx] = pt;
    } else {
      closest_cloud[idx] = PointType();  // 빈 포인트
    }
  });

  return true;
}
```

**병렬 처리:**
- `std::execution::par_unseq` 사용
- 각 포인트를 독립적으로 처리
- 멀티코어 CPU 활용

**사용 위치:**
- `include/voxmap/voxel_map.h:241-259`

---

## 9. 근접 복셀 검색

### NearbyType

```cpp
enum class NearbyType {
  CENTER,    // 중심만
  NEARBY6,   // 6개 인접
  NEARBY18,  // 18개 인접
  NEARBY26,  // 26개 인접
};
```

### GenerateNearbyGrids

```cpp
void GenerateNearbyGrids() {
  if (options_.nearby_type_ == NearbyType::CENTER) {
    nearby_grids_.emplace_back(KeyType());
  } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
    nearby_grids_ = {
      KeyType(0, 0, 0),   // 중심
      KeyType(-1, 0, 0),  // 왼쪽
      KeyType(1, 0, 0),   // 오른쪽
      KeyType(0, 1, 0),   // 앞
      KeyType(0, -1, 0),  // 뒤
      KeyType(0, 0, -1),  // 아래
      KeyType(0, 0, 1)    // 위
    };
  } else if (options_.nearby_type_ == NearbyType::NEARBY18) {
    nearby_grids_ = {
      // 중심
      KeyType(0, 0, 0),
      // 6개 면 공유
      KeyType(-1, 0, 0), KeyType(1, 0, 0),
      KeyType(0, 1, 0), KeyType(0, -1, 0),
      KeyType(0, 0, -1), KeyType(0, 0, 1),
      // 12개 모서리 공유
      KeyType(1, 1, 0), KeyType(-1, 1, 0),
      KeyType(1, -1, 0), KeyType(-1, -1, 0),
      KeyType(1, 0, 1), KeyType(-1, 0, 1),
      KeyType(1, 0, -1), KeyType(-1, 0, -1),
      KeyType(0, 1, 1), KeyType(0, -1, 1),
      KeyType(0, 1, -1), KeyType(0, -1, -1)
    };
  } else if (options_.nearby_type_ == NearbyType::NEARBY26) {
    nearby_grids_ = {
      // 중심 + 6개 면 + 12개 모서리 + 8개 꼭짓점
      // ... (26개 오프셋)
    };
  }
}
```

### 시각화

#### NEARBY6 (7개 복셀)

```
        Z
        ↑
        |
    [0,0,1]
        |
  Y ← [·] → [0,0,0]
        |
    [0,0,-1]

X축 방향:
[-1,0,0] ← [0,0,0] → [1,0,0]
```

**특징:**
- 중심 + 6개 면 공유 복셀
- 가장 빠른 검색
- 복셀 경계 근처 포인트에 충분

#### NEARBY18 (19개 복셀)

```
         +-------+-------+-------+
        /       /       /       /|
       /   ·   /   ·   /   ·   / |
      +-------+-------+-------+  |
     /       /       /       /|  +
    /   ·   /   ●   /   ·   / | /|
   +-------+-------+-------+  |/ |
  /       /       /       /|  +  |
 /   ·   /   ·   /   ·   / | /|  +
+-------+-------+-------+  |/ | /
|       |       |       |  +  |/
|   ·   |   ·   |   ·   | /|  +
|       |       |       |/ | /
+-------+-------+-------+  |/
|       |       |       |  +
|   ·   |   ·   |   ·   | /
|       |       |       |/
+-------+-------+-------+
```

**특징:**
- 중심 + 6개 면 + 12개 모서리 공유 복셀
- 균형잡힌 검색 범위

#### NEARBY26 (27개 복셀)

```
전체 3×3×3 큐브
```

**특징:**
- 모든 인접 복셀 (면 + 모서리 + 꼭짓점)
- 가장 정확한 검색
- 계산량 증가

### 성능 비교

| NearbyType | 검색 복셀 수 | 검색 시간 | 정확도 | 추천 상황 |
|-----------|------------|----------|--------|----------|
| CENTER | 1 | 가장 빠름 | 낮음 | 밀집된 맵 |
| NEARBY6 | 7 | 빠름 | 중간 | 일반적 사용 |
| NEARBY18 | 19 | 보통 | 높음 | 희소한 맵 |
| NEARBY26 | 27 | 느림 | 가장 높음 | 정확도 우선 |

**사용 위치:**
- `include/voxmap/voxel_map.h:209-238` (GenerateNearbyGrids)
- `src/laserMapping.cpp:334` (NEARBY18 사용)

---

## 10. 병렬 처리 최적화

### C++17 Parallel Algorithms

S-Faster-Lio의 iVox는 C++17의 병렬 실행 정책을 활용합니다.

### 실행 정책

```cpp
#include <execution>

std::execution::seq         // 순차 실행
std::execution::par         // 병렬 실행
std::execution::par_unseq   // 병렬 + 벡터화
std::execution::unseq       // 벡터화 (순차)
```

### AddPoints 병렬화

```cpp
void AddPoints(const PointVector& points_to_add) {
  std::for_each(std::execution::unseq,  // 벡터화
                points_to_add.begin(),
                points_to_add.end(),
                [this](const auto& pt) {
    // 포인트 처리
  });
}
```

**std::execution::unseq:**
- **벡터화 (Vectorization)**: SIMD 명령어 사용
- **순차 실행**: 멀티스레드 사용하지 않음
- **주의**: 동기화 프리미티브 (mutex 등) 사용 불가

**왜 unseq를 사용하나?**
- AddPoints는 각 포인트를 독립적으로 처리
- 공유 자료구조 (grids_map_, grids_cache_) 접근 시 동기화 필요
- par_unseq 사용 시 경쟁 조건 (race condition) 발생 가능
- unseq는 벡터화만 사용하여 안전하게 최적화

### GetClosestPoint 병렬화

```cpp
bool GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud) {
  std::for_each(std::execution::par_unseq,  // 병렬 + 벡터화
                index.begin(), index.end(),
                [&](size_t idx) {
    PointType pt;
    if (GetClosestPoint(cloud[idx], pt)) {
      closest_cloud[idx] = pt;
    }
  });
}
```

**std::execution::par_unseq:**
- **병렬 실행**: 멀티스레드로 작업 분산
- **벡터화**: SIMD 최적화
- **안전성**: 각 스레드는 독립적인 인덱스 처리

**왜 par_unseq를 사용하나?**
- 각 포인트의 최근접점 검색은 완전히 독립적
- 경쟁 조건 없음 (각 idx는 고유)
- 멀티코어 CPU 활용으로 큰 성능 향상

### ESEKF update 병렬화

```cpp
void h_share_model(dyn_share_datastruct& ekfom_data,
                   PointCloudXYZI::Ptr& feats_down_body,
                   VoxelMap<PointType>& ivox,
                   vector<PointVector>& Nearest_Points,
                   bool extrinsic_est) {
  std::vector<int> index(feats_down_size);
  std::iota(index.begin(), index.end(), 0);

  std::for_each(std::execution::par_unseq,  // 병렬 + 벡터화
                index.begin(), index.end(),
                [&](const int& i) {
    // 각 포인트를 월드 프레임으로 변환
    V3D p_body(point_body.x, point_body.y, point_body.z);
    V3D p_global(x_.rot * (x_.offset_R_L_I * p_body + x_.offset_T_L_I) + x_.pos);

    // 최근접 평면 찾기
    ivox.GetClosestPoint(point_world, points_near, NUM_MATCH_POINTS);

    // 평면 피팅 및 잔차 계산
    // ...
  });
}
```

**성능 향상:**
- 수백~수천 개의 포인트를 병렬 처리
- 멀티코어 CPU에서 4-8배 속도 향상

**사용 위치:**
- `include/voxmap/voxel_map.h:263` (AddPoints - unseq)
- `include/voxmap/voxel_map.h:249` (GetClosestPoint 벡터 - par_unseq)
- `include/esekfom.hpp:103-157` (h_share_model - par_unseq)

---

## 11. 성능 분석

### 시간 복잡도

| 연산 | 최선 | 평균 | 최악 | 설명 |
|------|------|------|------|------|
| AddPoints (1개) | O(1) | O(1) | O(n) | 해시 충돌 시 O(n) |
| GetClosestPoint (1개) | O(k·m) | O(k·m) | O(k·m) | k=근접 복셀 수, m=복셀당 포인트 수 |
| GetClosestPoint (N개) | O(N·k·m) | O(N·k·m/p) | O(N·k·m) | p=코어 수 (병렬) |
| LRU splice | O(1) | O(1) | O(1) | list의 splice |

### 공간 복잡도

```
메모리 사용량 = sizeof(grids_map_) + sizeof(grids_cache_)
             ≈ capacity × (sizeof(VoxelIndex) + sizeof(iterator))  // grids_map_
             + capacity × (sizeof(VoxelIndex) + sizeof(BasicVoxelNode))  // grids_cache_
             ≈ capacity × (12 + 8 + 12 + 24)  // 64비트 시스템
             ≈ capacity × 56 bytes

+ 포인트 저장:
  각 포인트 = sizeof(PointType) ≈ 16 bytes (x, y, z, intensity)

총 메모리 ≈ capacity × 56 + total_points × 16
```

**예시:**
- capacity = 1,000,000
- 평균 복셀당 포인트 = 10개
- 메모리 ≈ 1M × 56 + 10M × 16 ≈ 56 MB + 160 MB = 216 MB

### 성능 측정 (참고)

```cpp
// INNER_TIMER 매크로로 성능 측정 가능
#define INNER_TIMER
```

**측정 항목:**
- `knn`: KNNPointByCondition 실행 시간
- `nth`: nth_element 정렬 시간
- `dis`: 거리 계산 시간
- `put`: 후보 추가 시간

**사용 위치:**
- `include/voxmap/voxel_map.h:132-194` (GetClosestPoint 타이머)
- `include/voxmap/basic_voxel_node.h:89-153` (KNNPointByCondition 타이머)

---

## 12. 사용 예제

### 초기화

```cpp
// laserMapping.cpp:332-335
VoxelMap<PointType>::Options ivox_options;
ivox_options.resolution_ = 0.5;  // 50cm 복셀
ivox_options.nearby_type_ = VoxelMap<PointType>::NearbyType::NEARBY18;
ivox_options.capacity_ = 1000000;

VoxelMap<PointType> ivox(ivox_options);
```

### 포인트 추가

```cpp
// laserMapping.cpp:549, 667
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());

// 포인트 클라우드를 월드 프레임으로 변환
pointBodyToWorld(feats_down_body, feats_down_world);

// iVox에 추가
ivox.AddPoints(feats_down_world->points);
```

### 최근접점 검색 (단일)

```cpp
PointType query_point;
query_point.x = 1.0;
query_point.y = 2.0;
query_point.z = 0.5;

PointType closest_point;
bool found = ivox.GetClosestPoint(query_point, closest_point);

if (found) {
  std::cout << "Closest point: ("
            << closest_point.x << ", "
            << closest_point.y << ", "
            << closest_point.z << ")" << std::endl;
}
```

### 최근접점 검색 (K개)

```cpp
// esekfom.hpp:124
PointType query_point;
vector<PointType> closest_points;

bool found = ivox.GetClosestPoint(query_point,
                                  closest_points,
                                  5,      // K=5개
                                  5.0);   // max_range=5m

if (found) {
  std::cout << "Found " << closest_points.size() << " points" << std::endl;
  for (const auto& pt : closest_points) {
    std::cout << "  (" << pt.x << ", " << pt.y << ", " << pt.z << ")" << std::endl;
  }
}
```

### 최근접점 검색 (포인트 클라우드)

```cpp
PointCloudXYZI::Ptr query_cloud(new PointCloudXYZI());
PointCloudXYZI::Ptr closest_cloud(new PointCloudXYZI());

bool found = ivox.GetClosestPoint(query_cloud->points, closest_cloud->points);

if (found) {
  std::cout << "Matched " << closest_cloud->points.size() << " points" << std::endl;
}
```

### 통계 정보

```cpp
// 복셀 수
size_t num_grids = ivox.NumValidGrids();
std::cout << "Number of voxels: " << num_grids << std::endl;

// 포인트 통계
std::vector<float> stats = ivox.StatGridPoints();
std::cout << "Valid grids: " << stats[0] << std::endl;
std::cout << "Average points per grid: " << stats[1] << std::endl;
std::cout << "Max points in a grid: " << stats[2] << std::endl;
std::cout << "Min points in a grid: " << stats[3] << std::endl;
std::cout << "Std dev: " << stats[4] << std::endl;
```

**StatGridPoints 구현:**
```cpp
std::vector<float> StatGridPoints() const {
  int num = grids_cache_.size();
  int valid_num = 0, max = 0, min = 100000000;
  int sum = 0, sum_square = 0;

  for (auto& it : grids_cache_) {
    int s = it.second.Size();
    valid_num += s > 0;
    max = s > max ? s : max;
    min = s < min ? s : min;
    sum += s;
    sum_square += s * s;
  }

  float ave = float(sum) / num;
  float stddev = num > 1
      ? sqrt((float(sum_square) - num * ave * ave) / (num - 1))
      : 0;

  return std::vector<float>{valid_num, ave, max, min, stddev};
}
```

---

## 참고 자료

### 주요 파일

| 파일 | 설명 |
|------|------|
| [include/voxmap/voxel_map.h](../include/voxmap/voxel_map.h) | VoxelMap 클래스 |
| [include/voxmap/voxel_index.h](../include/voxmap/voxel_index.h) | VoxelIndex 클래스 |
| [include/voxmap/basic_voxel_node.h](../include/voxmap/basic_voxel_node.h) | BasicVoxelNode 클래스 |
| [src/laserMapping.cpp](../src/laserMapping.cpp) | iVox 사용 예제 |

### 관련 논문

1. **Faster-LIO: Lightweight Tightly Coupled Lidar-Inertial Odometry using Parallel Sparse Incremental Voxels**
   - iVox의 원본 구현
   - LINEAR와 PHC 두 가지 복셀 타입 제안

2. **Spatial Hashing**
   - 3D 공간을 해시로 매핑하는 기법
   - 충돌 감소를 위한 큰 소수 사용

### 알고리즘

1. **LRU 캐시 (Least Recently Used)**
   - 최근 사용 순서로 데이터 관리
   - O(1) 접근 및 업데이트

2. **nth_element**
   - 부분 정렬 알고리즘
   - O(n) 평균 복잡도

3. **Parallel Algorithms (C++17)**
   - `std::execution::par_unseq`: 병렬 + 벡터화
   - `std::execution::unseq`: 벡터화만

---

## 요약

1. **iVox는 해시 기반 복셀 맵**:
   - O(1) 평균 접근 시간
   - LRU 캐시로 메모리 효율성

2. **3계층 자료구조**:
   - VoxelIndex (키): 정수 좌표, 공간 해시
   - BasicVoxelNode (값): 포인트 저장, KNN 검색
   - VoxelMap (컨테이너): 해시맵 + LRU 캐시

3. **병렬 처리 최적화**:
   - C++17 parallel algorithms
   - 멀티코어 CPU 활용
   - 4-8배 성능 향상

4. **유연한 근접 검색**:
   - NEARBY6/18/26 지원
   - 정확도와 속도 trade-off

5. **증분식 업데이트**:
   - 실시간으로 포인트 추가
   - 자동으로 오래된 복셀 제거
