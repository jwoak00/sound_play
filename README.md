# sound_play

**ROS 2 Sound Playback Package** | **ROS 2 사운드 재생 패키지**

A ROS 2 package that provides audio playback functionality by subscribing to `std_msgs/UInt8` messages and playing corresponding sound files based on the received values.

ROS 2 패키지 `sound_play`는 `std_msgs/UInt8` 메시지를 구독하여 수신된 값에 따라 해당하는 사운드 파일을 재생하는 기능을 제공합니다.

---

## 🎵 Features | 기능

- **Multi-format Support**: Supports both `.wav` and `.mp3` audio files  
  **다중 포맷 지원**: `.wav` 및 `.mp3` 오디오 파일 지원
- **Playback Modes**: Single mode (stops current sound) and overlap mode (simultaneous playback)  
  **재생 모드**: 단일 모드 (현재 사운드 중지) 및 중첩 모드 (동시 재생)
- **Dynamic Sound Mapping**: Maps integer values to sound files automatically  
  **동적 사운드 매핑**: 정수 값을 사운드 파일에 자동 매핑
- **ROS 2 Integration**: Seamless integration with ROS 2 ecosystem  
  **ROS 2 통합**: ROS 2 생태계와의 원활한 통합

---

## 📋 Requirements | 요구사항

- **ROS 2** (Humble Hawksbill or later recommended)
- **Python 3** (3.8+)
- **Audio System**: 
  - `aplay` for `.wav` files (usually pre-installed on Linux)
  - `ffplay` for `.mp3` files (`sudo apt install ffmpeg`)

---

## 🚀 Installation | 설치

### Method 1: Clone and Build | 방법 1: 클론 및 빌드

```bash
# Create workspace if it doesn't exist
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/jwoak00/sound_play.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select sound_play_pkg
source install/setup.bash
```

### Method 2: Direct Download | 방법 2: 직접 다운로드

```bash
cd ~/ros2_ws/src
wget -O sound_play.tar.gz https://github.com/jwoak00/sound_play/archive/main.tar.gz
tar -xzf sound_play.tar.gz
mv sound_play-main sound_play
cd ~/ros2_ws
colcon build --packages-select sound_play_pkg
source install/setup.bash
```

---

## 📁 Directory Structure | 디렉터리 구조

```
sound_play/
├── README.md
├── package.xml
├── setup.py
├── sound_play_pkg/
│   ├── __init__.py
│   └── sound_play_node.py
├── launch/
│   └── sound_play.launch.py
└── sound/
    ├── 0.wav
    ├── 1.mp3
    ├── 2.wav
    └── ...
```

---

## 🎮 Usage | 사용법

### 1. Start the Node | 노드 시작

```bash
# Method 1: Direct node execution
ros2 run sound_play_pkg sound_play

# Method 2: Using launch file
ros2 launch sound_play_pkg sound_play.launch.py

# Method 3: With custom parameters
ros2 run sound_play_pkg sound_play --ros-args -p mode:=overlap -p sound_dir:=/custom/path
```

### 2. Play Sounds | 사운드 재생

```bash
# Play sound file corresponding to ID 0 (sound/0.wav or sound/0.mp3)
ros2 topic pub /sound_id std_msgs/UInt8 "data: 0"

# Play sound file corresponding to ID 5
ros2 topic pub /sound_id std_msgs/UInt8 "data: 5"
```

### 3. Programming Example | 프로그래밍 예제

**Python:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

class SoundController(Node):
    def __init__(self):
        super().__init__('sound_controller')
        self.publisher_ = self.create_publisher(UInt8, '/sound_id', 10)
    
    def play_sound(self, sound_id):
        msg = UInt8()
        msg.data = sound_id
        self.publisher_.publish(msg)
        self.get_logger().info(f'Playing sound ID: {sound_id}')

def main():
    rclpy.init()
    controller = SoundController()
    
    # Play different sounds
    controller.play_sound(0)  # Welcome sound
    controller.play_sound(1)  # Alert sound
    controller.play_sound(2)  # Success sound
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**C++:**
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

class SoundController : public rclcpp::Node {
public:
    SoundController() : Node("sound_controller") {
        publisher_ = this->create_publisher<std_msgs::msg::UInt8>("/sound_id", 10);
    }
    
    void play_sound(uint8_t sound_id) {
        auto message = std_msgs::msg::UInt8();
        message.data = sound_id;
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Playing sound ID: %d", sound_id);
    }

private:
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<SoundController>();
    
    // Play different sounds
    controller->play_sound(0);  // Welcome sound
    controller->play_sound(1);  // Alert sound
    
    rclcpp::shutdown();
    return 0;
}
```

---

## ⚙️ Configuration | 설정

### Node Parameters | 노드 매개변수

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `mode` | string | `"single"` | Playback mode: `"single"` or `"overlap"` |
| `sound_dir` | string | `"sound/"` | Directory containing sound files |
| `volume` | double | `1.0` | Audio volume (0.0 - 1.0) |
| `timeout` | double | `10.0` | Maximum playback duration in seconds |

**Korean | 한국어:**

| 매개변수 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `mode` | string | `"single"` | 재생 모드: `"single"` 또는 `"overlap"` |
| `sound_dir` | string | `"sound/"` | 사운드 파일이 있는 디렉터리 |
| `volume` | double | `1.0` | 오디오 볼륨 (0.0 - 1.0) |
| `timeout` | double | `10.0` | 최대 재생 시간 (초) |

### Topics | 토픽

| Topic | Type | Description |
|-------|------|-------------|
| `/sound_id` | `std_msgs/UInt8` | Sound ID to play (subscribes) |
| `/sound_status` | `std_msgs/String` | Current playback status (publishes) |

**Korean | 한국어:**

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/sound_id` | `std_msgs/UInt8` | 재생할 사운드 ID (구독) |
| `/sound_status` | `std_msgs/String` | 현재 재생 상태 (발행) |

---

## 🎯 Sound File Management | 사운드 파일 관리

### Adding Sound Files | 사운드 파일 추가

1. Place your audio files in the `sound/` directory
2. Name them with numeric IDs: `0.wav`, `1.mp3`, `2.wav`, etc.
3. Supported formats: `.wav`, `.mp3`

**Korean:**
1. `sound/` 디렉터리에 오디오 파일을 배치
2. 숫자 ID로 파일명 지정: `0.wav`, `1.mp3`, `2.wav` 등
3. 지원 형식: `.wav`, `.mp3`

### File Naming Convention | 파일 명명 규칙

```
sound/
├── 0.wav    # ID 0 → Welcome message
├── 1.mp3    # ID 1 → Alert sound
├── 2.wav    # ID 2 → Success notification
├── 3.mp3    # ID 3 → Error sound
└── ...
```

---

## 🔧 Troubleshooting | 문제 해결

### Common Issues | 일반적인 문제

**1. No sound output | 소리가 나지 않음**
```bash
# Check audio system
pulseaudio --check
alsamixer

# Test audio
speaker-test -t wav
```

**2. File not found errors | 파일을 찾을 수 없음 오류**
```bash
# Check file permissions
ls -la sound/
chmod 644 sound/*.wav sound/*.mp3

# Verify file paths
ros2 param get /sound_play sound_dir
```

**3. ffplay not found | ffplay를 찾을 수 없음**
```bash
# Install ffmpeg
sudo apt update
sudo apt install ffmpeg
```

### Debug Mode | 디버그 모드

```bash
# Enable debug logging
ros2 run sound_play_pkg sound_play --ros-args --log-level debug

# Monitor topics
ros2 topic echo /sound_id
ros2 topic echo /sound_status
```

---

## 🤝 Contributing | 기여

We welcome contributions! Please follow these steps:

기여를 환영합니다! 다음 단계를 따라주세요:

1. **Fork** the repository | 저장소를 **포크**
2. **Create** a feature branch | 기능 브랜치 **생성**
   ```bash
   git checkout -b feature/amazing-feature
   ```
3. **Commit** your changes | 변경사항 **커밋**
   ```bash
   git commit -m 'Add some amazing feature'
   ```
4. **Push** to the branch | 브랜치에 **푸시**
   ```bash
   git push origin feature/amazing-feature
   ```
5. **Open** a Pull Request | Pull Request **열기**

### Development Setup | 개발 환경 설정

```bash
# Clone for development
git clone https://github.com/jwoak00/sound_play.git
cd sound_play

# Install development dependencies
pip3 install pytest rclpy-pytest colcon-common-extensions

# Run tests
colcon test --packages-select sound_play_pkg
```

---

## 📄 License | 라이선스

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

이 프로젝트는 MIT 라이선스 하에 라이선스가 부여됩니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.

---

## 📞 Support | 지원

- **Issues**: [GitHub Issues](https://github.com/jwoak00/sound_play/issues)
- **Discussions**: [GitHub Discussions](https://github.com/jwoak00/sound_play/discussions)
- **Documentation**: [Wiki](https://github.com/jwoak00/sound_play/wiki)

---

## 🙏 Acknowledgments | 감사의 말

- ROS 2 community for the excellent documentation
- Contributors who help improve this package
- Open source audio libraries that make this possible

ROS 2 커뮤니티의 훌륭한 문서와 이 패키지 개선에 도움을 주신 기여자들, 그리고 이를 가능하게 한 오픈 소스 오디오 라이브러리에 감사드립니다.

---

## 📈 Roadmap | 로드맵

- [ ] **v2.0**: GUI control panel
- [ ] **v2.1**: Text-to-speech integration
- [ ] **v2.2**: Streaming audio support
- [ ] **v2.3**: Advanced audio effects
- [ ] **v3.0**: Multi-language support enhancement

---

*For more examples and detailed documentation, please visit our [Wiki](https://github.com/jwoak00/sound_play/wiki).*

*더 많은 예제와 자세한 문서는 [Wiki](https://github.com/jwoak00/sound_play/wiki)를 참조하세요.*