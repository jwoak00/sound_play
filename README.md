# sound_play_pkg

**Short description (EN)**: Simple ROS 2 node that plays local WAV/MP3 files selected by incoming std_msgs/UInt8 IDs. Supports single/overlap playback modes, duplicate-ID edge suppression, and hot‑reloading via a configurable sounds directory.

**요약 (KO)**: `std_msgs/UInt8` ID 값을 받아 미리 매핑된 사운드(WAV/MP3)를 재생하는 ROS 2 패키지. 동일 ID 연속 수신 시 새 ID가 올 때까지 재생 1회만 수행(엣지 트리거). `sounds_dir` 파라미터로 소스 폴더를 직접 지정해 빌드 없이 파일 교체가 가능하며, 재생 모드는 단일(`single`) 또는 중첩(`overlap`)을 지원.

```bash
git clone -b feature/sounds_dir https://github.com/jwoak00/sound_play_pkg.git sound_play_pkg_feature
```

---

## 주요 특징

- 노드: `sound_play` (`SoundPlayNode`)
- 구독 토픽: 파라미터 `topic` (기본 런치 예: `/sound_id`)
- 메시지 타입: `std_msgs/UInt8`
- ID → 파일 매핑: `file_ids`, `file_names` (동일 길이 배열)
- 재생 모드:
  - `single`: 새 요청 시 기존 프로세스 종료 후 재생
  - `overlap`: 여러 사운드 동시에 허용
- 동일 ID 반복 수신 시: 첫 수신만 재생하고 이후 동일 값은 무시 (새 ID 들어오면 다시 1회 재생)
- 지원 포맷: `.wav`(aplay), `.mp3`(ffplay). 기타 확장자도 ffplay로 시도.
- `sounds_dir` 파라미터로 소스 디렉토리 직접 지정 가능 → 사운드 파일 추가 후 build 불필요(launch 재시작만)
- (옵션) 설치 경로에 없고 `sounds_dir`가 지정되었을 때 지정 경로 우선 사용


---

## 파일 구조 예시

```
   sound_play_pkg
    ├── README.md
    ├── launch
    │   └── sound_play.launch.py
    ├── package.xml
    ├── resource
    │   └── sound_play_pkg
    ├── setup.cfg
    ├── setup.py
    ├── sound
    │   ├── test_0.mp3
    │   ├── test_1.mp3
    │   ├── test_2.mp3
    │   ├── test_3.wav
    │   ├── test_4.wav
    │   ├── test_5.wav
    └── sound_play_pkg
        ├── __init__.py
        ├── __pycache__
        └── sound_play.py

```

---

## 의존

- ROS 2 (rclpy, std_msgs, launch_ros, ament_index_python)
- 시스템 실행파일:
  - `aplay` (alsa-utils)
  - `ffplay` (ffmpeg)
```bash
sudo apt-get update
sudo apt-get install -y alsa-utils ffmpeg
```

---

## 빌드 & 실행

```bash
cd ~/your_ws
colcon build --symlink-install --packages-select sound_play_pkg
source install/setup.bash
ros2 launch sound_play_pkg sound_play.launch.py
```

로그 예:
```
[INFO] [sound_play]: Listening on /sound_id, sounds dir: .../share/sound_play_pkg/sound
```

---

## 파라미터 (런치에서 설정)

| 이름          | 타입        | 설명 |
|---------------|-------------|------|
| `topic`       | string      | 구독할 UInt8 토픽 |
| `play_mode`   | string      | `single` 또는 `overlap` |
| `file_ids`    | int[]       | 사운드 ID 목록 |
| `file_names`  | string[]    | 각 ID에 대응되는 파일명 (동일 길이) |
| `sounds_dir`  | string      | (선택) 사운드 파일이 존재하는 디렉토리 – 지정 시 설치 share/sound 대신 이 경로 사용 |

런치 예시: [`launch/sound_play.launch.py`](launch/sound_play.launch.py)

```python
parameters=[{
  'topic': '/sound_id',
  'play_mode': 'single',
  'file_ids':  [0, 1, 2, 3, 4, 5],
  'file_names': ['test_0.mp3', 'test_1.mp3', 'test_2.mp3', 'test_3.wav', 'test_4.wav', 'test_5.wav'],
}]
```

---

## 사용 예

ID 2 재생:
```bash
ros2 topic pub --once /sound_id std_msgs/msg/UInt8 "{data: 2}"
```

같은 2를 반복 퍼블리시 → 무시  
다른 3 퍼블리시 → 새 재생

새 파일(test_6.mp3) 추가 & 즉시 사용 (rebuild 불필요):
```bash
cp new.mp3 ~/ros2_ws/src/sound_play_pkg/sound/test_6.mp3
# launch 파일 file_ids/file_names 에 6, 'test_6.mp3' 추가 후
ros2 launch sound_play_pkg sound_play.launch.py
ros2 topic pub --once /sound_id std_msgs/msg/UInt8 "{data: 6}"
```

---

## 동작 흐름

1. 초기 파라미터 로드 (`topic`, `play_mode`, `file_ids`, `file_names`, 선택적 `sounds_dir`).
2. `sounds_dir` 지정 시 그 경로 사용, 미지정이면 설치된 패키지 share 내부 sound.
3. 매핑 검증 후 ID→파일 dict 구성.
4. UInt8 메시지 수신 시 이전 ID와 동일하면 무시(엣지 트리거).
5. 단일 모드에서 기존 재생 프로세스가 있으면 종료 후 새 재생.
6. MP3는 `ffplay`, WAV는 `aplay` (그 외 확장자는 `ffplay`) 사용.
7. 종료 시 (Ctrl+C) 프로세스 정리 후 안전하게 rclpy.shutdown.

코어 구현: [`sound_play_pkg/sound_play.py`](sound_play_pkg/sound_play.py) → `SoundPlayNode.on_msg()` / `_build_mapping()`.


---

## 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| File not found | 설치 share 또는 지정 `sounds_dir`에 파일 없음 | 파일 존재/권한 확인, `file_names` 매핑 재확인 |
| 매핑 오류 예외 | file_ids / file_names 길이 불일치 | 런치 파라미터 수정 |
| mp3 재생 실패 | ffplay 없음 | `sudo apt install ffmpeg` |
| 즉시 종료 | 필수 파라미터 공백 또는 매핑 오류 | 런치 파라미터/매핑 길이 재검증 |
| 반복 재생 안 됨 | 동일 ID만 지속 입력 | 새 ID 전송해야 재생 재개 (엣지 트리거) |
| mp3 무음 / 지연 | ffmpeg 설치 이슈 또는 파일 손상 | 다른 mp3로 교체, `ffplay` 단독 실행 테스트 |

### 디버그 팁
```bash
ROS_LOG_LEVEL=DEBUG ros2 launch sound_play_pkg sound_play.launch.py
```
출력되는 매핑/경로 로그를 확인하여 잘못된 파라미터를 빠르게 찾을 수 있습니다.

### 보안/성능 유의점
- 외부로부터 임의 ID를 받아 민감 경로 재생하지 않도록 `sounds_dir`를 신뢰 가능한 경로로 제한하세요.
- 매우 짧은 간격으로 서로 다른 ID를 퍼블리시하면 ffplay/aplay 프로세스가 많이 생성될 수 있으므로 필요 시 윈도잉(디바운스) 로직을 추가 고려.

### 향후 개선 아이디어
- status 퍼블리시(topic)로 현재 재생 상태 알림
- Action 인터페이스 (재생 완료 피드백)
- 볼륨 파라미터 / 믹싱 백엔드(GStreamer 등) 지원
- 캐시 및 파일 사전 존재 검사로 첫 재생 지연 감소
