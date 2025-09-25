# sound_play_pkg

**Short description (EN)**: Simple ROS 2 node that plays local wav/mp3 files selected by incoming std_msgs/UInt8 IDs, with single / overlap playback modes and duplicate-ID suppression.

**요약 (KO)**: `std_msgs/UInt8` ID 값을 받아 미리 매핑된 사운드(wav/mp3)를 재생하는 ROS 2 패키지. 동일 ID가 연속으로 들어오면 새 ID가 올 때까지 재생을 한 번만 수행(엣지 트리거). 재생 모드는 한 번에 하나(`single`) 또는 겹침(`overlap`) 지원.

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

---

## 파일 구조

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
    │   ├── idle.wav
    │   ├── start.mp3
    │   ├── stop.wav
    │   ├── test_0.mp3
    │   ├── tmp.mp3
    │   └── warn.mp3
    └── sound_play_pkg
        ├── __init__.py
        ├── __pycache__
        │   ├── __init__.cpython-310.pyc
        │   └── sound_play.cpython-310.pyc
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
cd ~/ros2_ws
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

런치 예시: [`launch/sound_play.launch.py`](launch/sound_play.launch.py)

```python
parameters=[{
  'topic': '/sound_id',
  'play_mode': 'single',
  'file_ids':  [0, 1, 2, 3, 4, 5],
  'file_names': ['idle.wav', 'start.mp3', 'stop.wav', 'warn.mp3', 'tmp.mp3', 'test_0.mp3'],
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

---

## 동작 흐름

1. 노드 시작 시 패키지 share 디렉토리에서 `sound/` 경로 결정
2. `file_ids` + `file_names` 검증 → 매핑 딕셔너리 생성
3. 메시지 수신:
   - 이전 ID와 같으면 리턴
   - 새 ID면 기존 재생(단일 모드) 중지 후 새 프로세스 실행
4. 프로세스 핸들 reference 보관 (single 모드만)

코어 구현: [`sound_play_pkg/sound_play.py`](sound_play_pkg/sound_play.py) 의 `SoundPlayNode.on_msg()`

---

## 확장 아이디어 (미구현)

- `sounds_dir` 파라미터로 외부 경로 지정
- 재생 상태 퍼블리시(/sound_play/status)
- 서비스/액션 인터페이스로 동기 재생 제어
- 동일 ID 무시 타임아웃(stale timeout) 옵션
- 볼륨 파라미터 (`ffplay -volume`, `amixer` 연계)

요청 시 추가 가능.

---

## 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| File not found | 사운드 파일 미설치/이름 불일치 | sound/ 폴더 파일명 확인 |
| 매핑 오류 예외 | file_ids / file_names 길이 불일치 | 런치 파라미터 수정 |
| mp3 재생 실패 | ffplay 없음 | `sudo apt install ffmpeg` |
| 즉시 종료 | 필수 파라미터 공백 | 런치 파라미터 채우기 |

---

## 라이선스

Apache License 2.0 (코드), 사운드 파일은 별도 라이선스 확인 필요.

---

## 변경 로그 (요약)

- 0.1.0: 기본 기능, 중복 ID 억제, launch 파라미터 기반 매핑.
