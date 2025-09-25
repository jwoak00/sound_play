# sound_play

**ROS 2 패키지** `sound_play_pkg`는 `std_msgs/UInt8` 메시지를 받아 값에 따라 미리 지정된 `.wav` 또는 `.mp3` 파일을 재생하는 노드(`sound_play`)를 제공합니다.

---

## 기능 요약

- **노드 이름**: `sound_play`  
- **구독 토픽**: `/sound_id` (`std_msgs/UInt8`)  
- **동작**:  
  - 수신한 값(예: 0, 1, 2, …)에 대응하는 사운드 파일을 `sound/` 폴더에서 찾아 재생  
  - `.wav` 파일은 `aplay`, `.mp3` 파일은 `ffplay` 사용  
  - `single` 모드: 새로운 소리가 재생되면 기존 재생 중지  
  - `overlap` 모드: 여러 소리를 동시에 재생  

---

## 설치 및 빌드

```bash
cd ~/ros2_ws/src
git clone https://github.com/jwoak00/sound_play.git
cd ..
colcon build --symlink-install
source install/setup.bash
