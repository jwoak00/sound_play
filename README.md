# sound_play_pkg

ROS 2 기반 자율주행 차량용 오디오 피드백 시스템으로, 다양한 차량 이벤트(차선 이탈 경고, ACC, 모드 전환, 차선 변경 등)에 대응하여 사운드를 재생합니다. 우선순위 기반 재생 큐, 반복 경고음, 속도 변화 알림 등을 지원하며, 런타임에 파라미터로 사운드 매핑을 유연하게 구성할 수 있습니다.

---

## 주요 특징

### 아키텍처
- **노드 이름**: `sound_play` (클래스: `SoundPlayNode`)
- **언어**: C++17, ROS 2 Humble
- **헤더 분리**: 재사용 가능한 공개 헤더 (`include/sound_play_pkg/sound_play_node.hpp`)
- **멀티스레드**: 재생 큐 워커 스레드 + 우선순위 경고음 전용 스레드

### 지원 기능
- **이벤트 기반 재생**: 10가지 차량 이벤트에 대한 개별 토픽 구독
  - **LDWS** (차선 이탈 경고) - `std_msgs/Bool`
  - **ACC** (어댑티브 크루즈 컨트롤) 레벨 0~3 - `std_msgs/UInt8`
  - **System Failure** (시스템 오류) - `std_msgs/Bool`
  - **Mode Switch** (자율주행/수동 모드 전환) - `std_msgs/Bool`
  - **Driving Disable Area** (주행 불가 구역) - `std_msgs/Bool`
  - **Lane Change** (좌/우/완료/취소) - `std_msgs/Bool`
  - **Speed Limit** (속도 제한 변화, 5~80km/h, 5km/h 단위) - `std_msgs/Float32` (m/s)

- **우선순위 재생 시스템**
  - 경고음(LDWS, ACC)은 최우선 순위로 즉시 반복 재생
  - 일반 사운드는 경고음 종료 후 큐에서 순차 재생
  - ACC 레벨에 따른 동적 경고음 간격 조정:
    - Level 1: 600ms
    - Level 2: 400ms
    - Level 3+: 200ms
    - LDWS only: 500ms

- **유연한 사운드 매핑**
  - 런치 파라미터로 이벤트 키와 파일명 매핑 (`sounds.mapping`)
  - YAML-like 문자열 파싱 지원: `{key: filename, key2: filename2, ...}`
  - 기본 매핑 제공 + 런타임 오버라이드 가능
  - 파싱 실패 시 기본 매핑 유지 + 경고 로그

- **핫 리로딩**
  - `sounds_dir` 파라미터로 사운드 파일 경로 직접 지정
  - 파일 추가/변경 시 노드 재시작만으로 반영 (빌드 불필요)
  - 미지정 시 자동으로 패키지 설치 경로 사용

- **재생 엔진**
  - **WAV**: `aplay` (ALSA) - 낮은 지연시간
  - **MP3**: `ffplay` (FFmpeg) - 압축 파일 지원
  - 동기식 재생으로 프로세스 완료 보장
  - fork/exec 기반 독립 프로세스 실행

---

## 시스템 요구사항

### ROS 2 의존성
- **ROS 2 배포판**: Humble Hawksbill 이상
- **필수 패키지**: 
  - `rclcpp` - ROS 2 C++ 클라이언트 라이브러리
  - `std_msgs` - 표준 메시지 타입
  - `ament_index_cpp` - 패키지 경로 해석
  - `rcl_interfaces` - 파라미터 디스크립터

### 시스템 의존성
오디오 재생을 위한 시스템 도구가 필요합니다:

```bash
sudo apt-get update
sudo apt-get install -y alsa-utils ffmpeg
```

- `aplay` (alsa-utils) - WAV 파일 재생
- `ffplay` (ffmpeg) - MP3 파일 재생

### 빌드 도구
- CMake 3.8+
- C++17 지원 컴파일러 (GCC 7+, Clang 5+)

---

## 파일 구조

```
sound_play_pkg/
├── CMakeLists.txt              # 빌드 설정
├── package.xml                 # ROS 2 패키지 메타데이터
├── README.md                   # 본 문서
├── include/
│   └── sound_play_pkg/
│       └── sound_play_node.hpp # 공개 노드 헤더
├── src/
│   └── sound_play_node.cpp     # 노드 구현 + 메인
├── launch/
│   └── sound_play.launch.xml   # XML 런치 파일
├── sounds/                     # 기본 사운드 파일 디렉토리
│   ├── beep.wav
│   ├── autonomous_mode.wav
│   ├── driver_mode.wav
│   ├── driving_disable_area.mp3
│   ├── lane_*.mp3
│   ├── 5.mp3 ~ 80.mp3
│   └── ...
└── resource/
    └── sound_play_pkg
```

---

## 빌드 & 설치

### 워크스페이스 빌드

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select sound_play_pkg
source install/setup.bash
```

### Release 빌드 (최적화)

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sound_play_pkg
```

---

## 실행

### 기본 실행

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch sound_play_pkg sound_play.launch.xml
```

### 디버그 모드

```bash
ROS_LOG_LEVEL=DEBUG ros2 launch sound_play_pkg sound_play.launch.xml
```

실행 시 다음과 같은 로그가 출력됩니다:

```
[INFO] [sound_play]: LDWS topic: /tmp/ldws_warning
[INFO] [sound_play]: ACC topic: /tmp/acc_level
[INFO] [sound_play]: System failure topic: /tmp/system_failure
...
[INFO] [sound_play]: sounds dir: /home/ok/ros2_ws/src/sound_play_pkg/sounds
```

---

## 파라미터 설정

### 파라미터 목록

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `sounds_dir` | string | (패키지 share/sounds) | 사운드 파일 디렉토리 절대 경로 |
| `sounds.mapping` | string | (기본 매핑) | 이벤트 키→파일명 매핑 (YAML 형식) |
| `topics.ldws` | string | "" | LDWS 경고 토픽 (`std_msgs/Bool`) |
| `topics.acc` | string | "" | ACC 레벨 토픽 (`std_msgs/UInt8`, 0~3) |
| `topics.system_failure` | string | "" | 시스템 오류 토픽 (`std_msgs/Bool`) |
| `topics.autonomous_mode` | string | "" | 자율주행 모드 토픽 (`std_msgs/Bool`) |
| `topics.driver_mode` | string | "" | 수동 모드 토픽 (`std_msgs/Bool`) |
| `topics.driving_disable_area` | string | "" | 주행 불가 구역 토픽 (`std_msgs/Bool`) |
| `topics.lane_change_right` | string | "" | 우측 차선 변경 토픽 (`std_msgs/Bool`) |
| `topics.lane_change_left` | string | "" | 좌측 차선 변경 토픽 (`std_msgs/Bool`) |
| `topics.lane_change_finish` | string | "" | 차선 변경 완료 토픽 (`std_msgs/Bool`) |
| `topics.lane_change_cancel` | string | "" | 차선 변경 취소 토픽 (`std_msgs/Bool`) |
| `topics.speed` | string | "/planning/..." | 속도 제한 토픽 (`std_msgs/Float32`, m/s) |

### 기본 사운드 매핑

```cpp
{
  "warning_beep": "beep.wav",
  "system_failure": "beep.wav",
  "autonomous_mode": "autonomous_mode.wav",
  "driver_mode": "driver_mode.wav",
  "driving_disable_area": "driving_disable_area.mp3",
  "lane_change_right": "lane_right.mp3",
  "lane_change_left": "lane_left.mp3",
  "lane_change_finish": "lane_finish.mp3",
  "lane_change_cancel": "lane_cancle.mp3"
}
```

### 런치 파일 예시

`launch/sound_play.launch.xml`:

```xml
<?xml version="1.0"?>
<launch>
  <node pkg="sound_play_pkg" exec="sound_play" name="sound_play" output="screen">
    <!-- 사운드 파일 디렉토리 -->
    <param name="sounds_dir" value="/home/ok/ros2_ws/src/sound_play_pkg/sounds" />
    
    <!-- 사운드 매핑 (YAML 형식 문자열) -->
    <param name="sounds.mapping" type="str" value="{warning_beep: beep.wav, system_failure: beep.wav, autonomous_mode: autonomous_mode.wav, driver_mode: driver_mode.wav, driving_disable_area: driving_disable_area.mp3, lane_change_right: lane_right.mp3, lane_change_left: lane_left.mp3, lane_change_cancel: lane_cancle.mp3, lane_change_finish: lane_finish.mp3}" />

    <!-- 토픽 구독 설정 -->
    <param name="topics.ldws" value="/tmp/ldws_warning" />
    <param name="topics.acc" value="/tmp/acc_level" />
    <param name="topics.system_failure" value="/tmp/system_failure" />
    <param name="topics.autonomous_mode" value="/tmp/autonomous_mode" />
    <param name="topics.driver_mode" value="/tmp/driver_mode" />
    <param name="topics.driving_disable_area" value="/tmp/driving_disable_area" />
    <param name="topics.lane_change_right" value="/tmp/lane_change_right" />
    <param name="topics.lane_change_left" value="/tmp/lane_change_left" />
    <param name="topics.lane_change_finish" value="/tmp/lane_change_finish" />
    <param name="topics.lane_change_cancel" value="/tmp/lane_change_cancel" />
    <param name="topics.speed" value="/planning/scenario_planning/max_velocity" />
  </node>
</launch>
```

---

## 사용 예시

### 이벤트 트리거

각 이벤트는 해당 토픽에 메시지를 퍼블리시하여 트리거할 수 있습니다:

#### LDWS 경고 활성화
```bash
ros2 topic pub /tmp/ldws_warning std_msgs/msg/Bool "{data: true}"
```

#### ACC 레벨 2 설정
```bash
ros2 topic pub /tmp/acc_level std_msgs/msg/UInt8 "{data: 2}"
```

#### 자율주행 모드 진입
```bash
ros2 topic pub --once /tmp/autonomous_mode std_msgs/msg/Bool "{data: true}"
```

#### 우측 차선 변경 알림
```bash
ros2 topic pub --once /tmp/lane_change_right std_msgs/msg/Bool "{data: true}"
```

#### 속도 제한 변경 (60km/h = 16.67 m/s)
```bash
ros2 topic pub --once /planning/scenario_planning/max_velocity std_msgs/msg/Float32 "{data: 16.67}"
```

### 사운드 파일 추가 (핫 리로딩)

새로운 사운드 파일을 추가하고 즉시 사용할 수 있습니다:

```bash
# 1. 사운드 파일 복사
cp my_custom_sound.mp3 ~/ros2_ws/src/sound_play_pkg/sounds/

# 2. 런치 파일에서 매핑 추가
# sounds.mapping 파라미터에 "custom_key: my_custom_sound.mp3" 추가

# 3. 노드 재시작 (빌드 불필요!)
ros2 launch sound_play_pkg sound_play.launch.xml
```

---

## 아키텍처 상세

### 스레드 구조

노드는 3개의 주요 스레드로 동작합니다:

1. **메인 스레드** (`rclcpp::spin`)
   - ROS 2 콜백 처리
   - 토픽 메시지 수신
   - 재생 큐에 사운드 추가

2. **재생 큐 워커** (`queue_worker`)
   - 일반 사운드 재생 큐 처리
   - 우선순위 경고음 대기
   - FIFO 순서로 순차 재생

3. **경고음 워커** (`warning_worker`)
   - LDWS/ACC 경고음 전용
   - 레벨별 간격으로 반복 재생
   - 우선순위 플래그로 일반 재생 블록

### 재생 흐름

```
Event Topic → handle_*() → enqueue_sound() → sound_map_ lookup
                                                     ↓
                                            enqueue_file()
                                                     ↓
                                            playback_queue_
                                                     ↓
queue_worker() → wait_for_priority_clear() → play_blocking()
                                                     ↓
                                        fork() + execvp(aplay/ffplay)
                                                     ↓
                                              waitpid() (블로킹)
```

### 우선순위 메커니즘

- `priority1_active_` 플래그로 경고음 활성 상태 표시
- 일반 재생은 `wait_for_priority_clear()`에서 대기
- 경고음 종료 시 `priority_cv_` 조건 변수로 알림
- ACC 레벨/LDWS 상태 변화 시 `update_warning_state()` 호출

### 사운드 매핑 파싱

런치 파라미터 `sounds.mapping`은 다음 형식을 지원합니다:

```
{key1: value1, key2: value2, ...}
key1: value1, key2: value2
key1=value1, key2=value2
```

- 공백, 따옴표 자동 제거
- 콜론(`:`) 또는 등호(`=`) 구분자 지원
- 파싱 실패 시 기본 매핑 유지

---

## 트러블슈팅

### 일반 문제

| 증상 | 원인 | 해결 방법 |
|------|------|----------|
| 사운드 재생 안 됨 | 오디오 도구 미설치 | `sudo apt install alsa-utils ffmpeg` |
| "Sound file missing" 경고 | 파일 경로 오류 | `sounds_dir` 확인, 파일 존재 확인 |
| "Sound mapping missing" 경고 | 매핑 키 오류 | `sounds.mapping` 파라미터 확인 |
| 노드 즉시 종료 | 파라미터 오류 | 로그에서 에러 메시지 확인 |
| MP3 재생 실패 | ffmpeg 미설치 | `sudo apt install ffmpeg` |
| WAV 재생 실패 | alsa 미설치/설정 | `aplay -l` 로 장치 확인 |

### 디버깅 방법

#### 1. 상세 로그 확인
```bash
ROS_LOG_LEVEL=DEBUG ros2 launch sound_play_pkg sound_play.launch.xml
```

#### 2. 토픽 모니터링
```bash
# 구독 중인 토픽 확인
ros2 node info /sound_play

# 특정 토픽 메시지 확인
ros2 topic echo /tmp/ldws_warning
```

#### 3. 파라미터 확인
```bash
# 현재 파라미터 값 조회
ros2 param list /sound_play
ros2 param get /sound_play sounds_dir
ros2 param get /sound_play sounds.mapping
```

#### 4. 오디오 시스템 테스트
```bash
# aplay 직접 테스트
aplay ~/ros2_ws/src/sound_play_pkg/sounds/beep.wav

# ffplay 직접 테스트
ffplay -nodisp -autoexit ~/ros2_ws/src/sound_play_pkg/sounds/driving_disable_area.mp3
```

### 런치 파일 오류

#### ValueError: Got invalid type identifier 'string'
- **원인**: XML에서 잘못된 타입 지정자 사용
- **해결**: `type="string"` → `type="str"` 변경

#### SyntaxError: unterminated string literal
- **원인**: 파라미터 value에 여러 줄 사용
- **해결**: 한 줄로 작성하거나 이스케이프 처리

---

## 성능 고려사항

### 지연시간
- **WAV (aplay)**: ~10-50ms 시작 지연
- **MP3 (ffplay)**: ~100-200ms 디코딩 + 시작 지연
- **Fork/exec 오버헤드**: ~5-10ms

### 리소스 사용
- **메모리**: ~10-20MB (노드 + 스레드)
- **CPU**: 재생 중 각 프로세스당 ~1-5%
- **디스크**: 사운드 파일 크기에 따라 가변

### 동시 재생 제한
- 일반 사운드: 순차 재생 (큐 기반)
- 경고음: 독립 스레드로 즉시 재생
- 시스템 리소스에 따라 동시 프로세스 수 제한 가능

---

## 보안 고려사항

### 파일 경로 검증
- `sounds_dir`은 신뢰할 수 있는 경로만 사용
- 사용자 입력을 직접 파일명으로 사용하지 않음
- 디렉토리 탐색 공격 방지 (상대 경로 제한)

### 프로세스 관리
- 자식 프로세스 적절히 정리 (waitpid)
- 좀비 프로세스 방지
- 시그널 핸들링으로 안전한 종료

### 권한 관리
- 오디오 장치 접근 권한 필요 (`audio` 그룹)
- 사운드 파일 읽기 권한 확인

---

## 확장 가능성

### 향후 개선 아이디어

1. **Action 인터페이스**
   - 재생 완료 피드백
   - 재생 취소/중단 기능
   - 재생 상태 퍼블리싱

2. **고급 믹싱**
   - GStreamer 백엔드 지원
   - 볼륨 제어 파라미터
   - 페이드 인/아웃 효과

3. **동적 재구성**
   - 런타임 파라미터 업데이트
   - 사운드 매핑 핫 리로드
   - 토픽 동적 구독/해제

4. **성능 최적화**
   - 사운드 파일 사전 로드/캐싱
   - 재생 풀링 (프로세스 재사용)
   - 비동기 재생 옵션

5. **모니터링**
   - 재생 통계 퍼블리싱
   - 오류 카운터/진단 정보
   - 성능 메트릭 수집

---

## 라이선스

이 패키지는 MIT 라이선스 하에 배포됩니다 (또는 프로젝트에 맞게 수정).

---


**Repository**: https://github.com/jwoak00/sound_play_pkg  
**Branch**: v2.0

---

## 참고 자료

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [rclcpp API Reference](https://docs.ros2.org/latest/api/rclcpp/)
- [ALSA Project](https://www.alsa-project.org/)
- [FFmpeg Documentation](https://ffmpeg.org/documentation.html)

---

**작성일**: 2025-10-14  
**버전**: 2.0  
**ROS 2 배포판**: Humble Hawksbill
