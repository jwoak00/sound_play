#include "sound_play_pkg/sound_play_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <map>
#include <regex>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace
{

std::string trim_copy(const std::string & input)
{
  const auto first = input.find_first_not_of(" \t\n\r");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = input.find_last_not_of(" \t\n\r");
  return input.substr(first, last - first + 1);
}

std::string strip_quotes(const std::string & input)
{
  if (input.size() >= 2) {
    const char front = input.front();
    const char back = input.back();
    if ((front == '"' && back == '"') || (front == '\'' && back == '\'')) {
      return input.substr(1, input.size() - 2);
    }
  }
  return input;
}

std::map<std::string, std::string> parse_mapping(const std::string & raw)
{
  std::map<std::string, std::string> result;
  auto data = trim_copy(raw);
  if (data.empty()) {
    return result;
  }

  if (data.front() == '{' && data.back() == '}') {
    data = trim_copy(data.substr(1, data.size() - 2));
  }

  std::string current;
  bool in_quote = false;
  char quote_char = '\0';

  for (char c : data) {
    if ((c == '"' || c == '\'') && (!in_quote || c == quote_char)) {
      in_quote = !in_quote;
      quote_char = in_quote ? c : '\0';
    }

    if (!in_quote && (c == ',' || c == '\n')) {
      const auto trimmed = trim_copy(current);
      if (!trimmed.empty()) {
        const auto colon_pos = trimmed.find(':');
        const auto equal_pos = trimmed.find('=');
        const auto delim_pos = colon_pos != std::string::npos ? colon_pos : equal_pos;
        if (delim_pos != std::string::npos) {
          auto key = strip_quotes(trim_copy(trimmed.substr(0, delim_pos)));
          auto value = strip_quotes(trim_copy(trimmed.substr(delim_pos + 1)));
          if (!key.empty() && !value.empty()) {
            result[key] = value;
          }
        }
      }
      current.clear();
      continue;
    }

    current.push_back(c);
  }

  const auto trimmed = trim_copy(current);
  if (!trimmed.empty()) {
    const auto colon_pos = trimmed.find(':');
    const auto equal_pos = trimmed.find('=');
    const auto delim_pos = colon_pos != std::string::npos ? colon_pos : equal_pos;
    if (delim_pos != std::string::npos) {
      auto key = strip_quotes(trim_copy(trimmed.substr(0, delim_pos)));
      auto value = strip_quotes(trim_copy(trimmed.substr(delim_pos + 1)));
      if (!key.empty() && !value.empty()) {
        result[key] = value;
      }
    }
  }

  return result;
}

std::map<std::string, std::string> parse_mapping_array(const std::vector<std::string> & raw)
{
  std::map<std::string, std::string> result;
  for (const auto & entry : raw) {
    auto token = trim_copy(entry);
    if (token.empty()) {
      continue;
    }

    const auto colon_pos = token.find(':');
    const auto equal_pos = token.find('=');
    const auto delim_pos = colon_pos != std::string::npos ? colon_pos : equal_pos;
    if (delim_pos == std::string::npos) {
      continue;
    }

    auto key = strip_quotes(trim_copy(token.substr(0, delim_pos)));
    auto value = strip_quotes(trim_copy(token.substr(delim_pos + 1)));
    if (!key.empty() && !value.empty()) {
      result[key] = value;
    }
  }
  return result;
}

constexpr rcutils_duration_value_t kWarnThrottleMs = 2000;
constexpr int MIN_SPEED_KMH = 5;
constexpr int MAX_SPEED_KMH = 80;
constexpr int SPEED_STEP_KMH = 5;

}  // namespace

namespace sound_play_pkg
{
namespace fs = std::filesystem;

std::map<std::string, std::string> SoundPlayNode::default_sound_map()
{
  return {
    {"warning_beep", "beep.wav"},
    {"system_failure", "system_failure.wav"},
    {"autonomous_mode", "autonomous_mode.wav"},
    {"driver_mode", "driver_mode.wav"},
    {"driving_disable_area", "driving_disable_area.mp3"},
    {"lane_change_right", "lane_right.mp3"},
    {"lane_change_left", "lane_left.mp3"},
    {"lane_change_cancel", "lane_cancle.mp3"},
    {"lane_change_finish", "lane_finish.mp3"}
  };
}

SoundPlayNode::SoundPlayNode()
: rclcpp::Node("sound_play")
{
  rcl_interfaces::msg::ParameterDescriptor dyn_desc;
  dyn_desc.dynamic_typing = true;

  declare_parameter<std::string>("sounds_dir", "", dyn_desc);
  declare_parameter<std::string>("topics.ldws", "", dyn_desc);
  declare_parameter<std::string>("topics.acc", "", dyn_desc);
  declare_parameter<std::string>("topics.system_failure", "", dyn_desc);
  declare_parameter<std::string>("topics.autonomous_mode", "", dyn_desc);
  declare_parameter<std::string>("topics.driver_mode", "", dyn_desc);
  declare_parameter<std::string>("topics.driving_disable_area", "", dyn_desc);
  declare_parameter<std::string>("topics.lane_change_right", "", dyn_desc);
  declare_parameter<std::string>("topics.lane_change_left", "", dyn_desc);
  declare_parameter<std::string>("topics.lane_change_finish", "", dyn_desc);
  declare_parameter<std::string>("topics.lane_change_cancel", "", dyn_desc);
  declare_parameter<std::string>("topics.speed", "/planning/scenario_planning/max_velocity", dyn_desc);
  declare_parameter<std::string>("sounds.mapping", "", dyn_desc);
  declare_parameter<int>("speed.debounce_ms", 300, dyn_desc); 

  sound_map_ = default_sound_map();

  const auto mapping_param = get_parameter("sounds.mapping");
  std::map<std::string, std::string> overrides;
  if (mapping_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    const auto raw = mapping_param.as_string();
    if (!raw.empty()) {
      overrides = parse_mapping(raw);
      if (overrides.empty()) {
        RCLCPP_WARN(get_logger(), "Failed to parse sounds.mapping parameter");
      }
    }
  } else if (mapping_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    const auto raw = mapping_param.as_string_array();
    if (!raw.empty()) {
      overrides = parse_mapping_array(raw);
      if (overrides.empty()) {
        RCLCPP_WARN(get_logger(), "Failed to parse sounds.mapping string array");
      }
    }
  }

  if (!overrides.empty()) {
    for (const auto & item : overrides) {
      sound_map_[item.first] = item.second;
    }
  }

  const auto sounds_param = get_parameter("sounds_dir").as_string();
  if (!sounds_param.empty()) {
    sounds_dir_ = fs::path{sounds_param};
  } else {
    const auto share_dir = ament_index_cpp::get_package_share_directory("sound_play_pkg");
    sounds_dir_ = fs::path{share_dir} / "sounds";
  }

  if (!fs::exists(sounds_dir_) || !fs::is_directory(sounds_dir_)) {
    RCLCPP_WARN(get_logger(), "sounds_dir not found: %s", sounds_dir_.string().c_str());
  }

  // debounce_ms atomic 변수 초기화
  debounce_ms_.store(get_parameter("speed.debounce_ms").as_int());

  queue_thread_ = std::thread(&SoundPlayNode::queue_worker, this);
  warning_thread_ = std::thread(&SoundPlayNode::warning_worker, this);
  speed_thread_ = std::thread(&SoundPlayNode::speed_debounce_worker, this);  // 추가

  init_subscriptions();

  RCLCPP_INFO(get_logger(), "sounds dir: %s", sounds_dir_.string().c_str());
}

SoundPlayNode::~SoundPlayNode()
{
  queue_running_.store(false);
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    std::queue<fs::path> empty;
    std::swap(playback_queue_, empty);
  }
  queue_cv_.notify_all();
  priority_cv_.notify_all();
  if (queue_thread_.joinable()) {
    queue_thread_.join();
  }

  warning_thread_running_.store(false);
  warning_cv_.notify_all();
  if (warning_thread_.joinable()) {
    warning_thread_.join();
  }

  // 속도 스레드 정리
  speed_thread_running_.store(false);
  speed_cv_.notify_all();
  if (speed_thread_.joinable()) {
    speed_thread_.join();
  }
}

void SoundPlayNode::init_subscriptions()
{
  const auto ldws_topic = get_parameter("topics.ldws").as_string();
  if (!ldws_topic.empty()) {
    ldws_sub_ = create_subscription<std_msgs::msg::Bool>(
      ldws_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::Bool::SharedPtr msg) { handle_ldws(msg->data); });
  }

  const auto acc_topic = get_parameter("topics.acc").as_string();
  if (!acc_topic.empty()) {
    acc_sub_ = create_subscription<std_msgs::msg::UInt8>(
      acc_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::UInt8::SharedPtr msg) { handle_acc(msg->data); });
  }

  const auto system_topic = get_parameter("topics.system_failure").as_string();
  if (!system_topic.empty()) {
    system_failure_sub_ = create_subscription<std_msgs::msg::Bool>(
      system_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          enqueue_sound("system_failure");
        }
      });
  }

  const auto autonomous_topic = get_parameter("topics.autonomous_mode").as_string();
  if (!autonomous_topic.empty()) {
    autonomous_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      autonomous_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::Bool::SharedPtr msg) { handle_autonomous_mode(msg->data); });
  }

  const auto driver_topic = get_parameter("topics.driver_mode").as_string();
  if (!driver_topic.empty()) {
    driver_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      driver_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::Bool::SharedPtr msg) { handle_driver_mode(msg->data); });
  }

  const auto disable_topic = get_parameter("topics.driving_disable_area").as_string();
  if (!disable_topic.empty()) {
    driving_disable_area_sub_ = create_subscription<std_msgs::msg::Bool>(
      disable_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::Bool::SharedPtr msg) { handle_driving_disable_area(msg->data); });
  }

  const auto lane_right_topic = get_parameter("topics.lane_change_right").as_string();
  if (!lane_right_topic.empty()) {
    lane_change_right_sub_ = create_subscription<std_msgs::msg::Bool>(
      lane_right_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          enqueue_sound("lane_change_right");
        }
      });
  }

  const auto lane_left_topic = get_parameter("topics.lane_change_left").as_string();
  if (!lane_left_topic.empty()) {
    lane_change_left_sub_ = create_subscription<std_msgs::msg::Bool>(
      lane_left_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          enqueue_sound("lane_change_left");
        }
      });
  }

  const auto lane_finish_topic = get_parameter("topics.lane_change_finish").as_string();
  if (!lane_finish_topic.empty()) {
    lane_change_finish_sub_ = create_subscription<std_msgs::msg::Bool>(
      lane_finish_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          enqueue_sound("lane_change_finish");
        }
      });
  }

  const auto lane_cancel_topic = get_parameter("topics.lane_change_cancel").as_string();
  if (!lane_cancel_topic.empty()) {
    lane_change_cancel_sub_ = create_subscription<std_msgs::msg::Bool>(
      lane_cancel_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          enqueue_sound("lane_change_cancel");
        }
      });
  }

  const auto speed_topic = get_parameter("topics.speed").as_string();
  if (!speed_topic.empty()) {
    velocity_sub_ = create_subscription<tier4_planning_msgs::msg::VelocityLimit>(
      speed_topic, rclcpp::QoS{10},
      [this](const tier4_planning_msgs::msg::VelocityLimit::SharedPtr msg) { handle_velocity(msg); });
  }
}

void SoundPlayNode::handle_ldws(bool active)
{
  const bool previous = ldws_active_.exchange(active);
  if (previous != active) {
    RCLCPP_INFO(get_logger(), "LDWS %s", active ? "ON" : "OFF");
  }
  update_warning_state();
}

void SoundPlayNode::handle_acc(uint8_t level)
{
  const uint8_t clamped = std::min<uint8_t>(static_cast<uint8_t>(3), level);
  const uint8_t previous = acc_level_.exchange(clamped);
  if (previous != clamped) {
    RCLCPP_INFO(get_logger(), "ACC level: %u", clamped);
  }
  update_warning_state();
}

void SoundPlayNode::handle_driver_mode(bool active)
{
  if (active && !driver_mode_active_) {
    enqueue_sound("driver_mode");
  }

  driver_mode_active_ = active;
  if (active) {
    autonomous_mode_active_ = false;
  }
}

void SoundPlayNode::handle_autonomous_mode(bool active)
{
  if (!active) {
    autonomous_mode_active_ = false;
    return;
  }

  if (driving_disable_area_active_) {
    enqueue_sound("driving_disable_area");
    driver_mode_active_ = true;
    autonomous_mode_active_ = false;
    return;
  }

  if (!autonomous_mode_active_) {
    enqueue_sound("autonomous_mode");
  }

  autonomous_mode_active_ = true;
  driver_mode_active_ = false;
}

void SoundPlayNode::handle_driving_disable_area(bool active)
{
  driving_disable_area_active_ = active;
}

void SoundPlayNode::handle_velocity(const tier4_planning_msgs::msg::VelocityLimit::SharedPtr msg)
{
  const double kmh = msg->max_velocity * 3.6;
  if (kmh < 0.0) {
    return;
  }

  int step = static_cast<int>(kmh / SPEED_STEP_KMH) * SPEED_STEP_KMH;
  if (step < MIN_SPEED_KMH) {
    step = 0;
  }

  step = std::min(step, MAX_SPEED_KMH);

  const int previous = last_speed_step_.load();
  
  // 속도가 변경된 경우에만 처리 (증가/감소 모두)
  if (step != previous) {
    last_speed_step_.store(step);
    enqueue_speed_sound(step);
  }
}

void SoundPlayNode::enqueue_speed_sound(int speed_step) {
  pending_speed_step_.store(speed_step, std::memory_order_release);
  speed_changed_.store(true, std::memory_order_release);
  speed_cv_.notify_one();
}

void SoundPlayNode::speed_debounce_worker()
{
  std::unique_lock<std::mutex> lock(speed_mutex_);
  
  while (speed_thread_running_) {
    // 1. 새 입력 대기
    speed_cv_.wait(lock, [this]() {
      return !speed_thread_running_.load() || speed_changed_.load();
    });

    if (!speed_thread_running_) break;

    // 2. 타이머 리셋 방식: 새 입력마다 타이머 재시작
    auto last_change_time = std::chrono::steady_clock::now();
    const auto debounce_duration = std::chrono::milliseconds(debounce_ms_.load());
    
    // 3. 안정화될 때까지 대기 (새 입력 없이 debounce_duration 경과)
    while (true) {
      const auto now = std::chrono::steady_clock::now();
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_change_time);
      
      if (elapsed >= debounce_duration) {
        break;  // 안정화 완료
      }
      
      // 남은 시간만큼 대기
      const auto remaining = debounce_duration - elapsed;
      const auto wait_result = speed_cv_.wait_for(lock, remaining);
      
      // 새 입력이 들어오면 타이머 리셋
      if (wait_result == std::cv_status::no_timeout && speed_changed_.load()) {
        last_change_time = std::chrono::steady_clock::now();
      }
    }

    // 4. 재생 (최종 안정화된 값)
    if (speed_changed_.load()) {
      const int speed = pending_speed_step_.load(std::memory_order_acquire);
      speed_changed_.store(false, std::memory_order_release);
      
      if (speed >= MIN_SPEED_KMH && speed <= MAX_SPEED_KMH) {
        const std::string filename = std::to_string(speed) + ".mp3";
        lock.unlock();
        enqueue_file(filename);
        lock.lock();
      }
    }
  }
}

void SoundPlayNode::enqueue_file(const std::string & file_name)
{
  fs::path file_path = sounds_dir_ / file_name;
  if (!fs::exists(file_path)) {
    RCLCPP_WARN(get_logger(), "Sound file missing: %s", file_path.string().c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    playback_queue_.push(file_path);
  }
  queue_cv_.notify_one();
}

void SoundPlayNode::enqueue_sound(const std::string & key)
{
  const auto it = sound_map_.find(key);
  if (it == sound_map_.end()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kWarnThrottleMs,
      "Sound mapping missing for key '%s'", key.c_str());
    return;
  }

  enqueue_file(it->second);
}

void SoundPlayNode::queue_worker()
{
  while (true) {
    fs::path next_file;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() {
        return !queue_running_.load() || !playback_queue_.empty();
      });

      if (!queue_running_.load()) {
        return;
      }

      next_file = playback_queue_.front();
      playback_queue_.pop();
    }

    if (!wait_for_priority_clear()) {
      return;
    }

    play_blocking(next_file);
  }
}

bool SoundPlayNode::wait_for_priority_clear()
{
  std::unique_lock<std::mutex> lock(priority_mutex_);
  priority_cv_.wait(lock, [this]() {
    return !queue_running_.load() || !priority1_active_.load();
  });

  return queue_running_.load();
}

void SoundPlayNode::warning_worker()
{
  std::unique_lock<std::mutex> lock(warning_mutex_);
  while (warning_thread_running_) {
    warning_cv_.wait(lock, [this]() {
      return !warning_thread_running_.load() || warning_active_.load();
    });

    if (!warning_thread_running_) {
      break;
    }

    while (warning_thread_running_ && warning_active_.load()) {
      const auto it = sound_map_.find("warning_beep");
      if (it == sound_map_.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), kWarnThrottleMs,
          "Sound mapping missing for key 'warning_beep'");
        lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(warning_interval_ms_.load()));
        lock.lock();
        continue;
      }

      const fs::path beep_path = sounds_dir_ / it->second;
      const auto interval = std::chrono::milliseconds(warning_interval_ms_.load());

      lock.unlock();
      play_blocking(beep_path, false);
      std::this_thread::sleep_for(interval);
      lock.lock();
    }
    priority_cv_.notify_all();
  }
}

void SoundPlayNode::update_warning_state()
{
  const bool active = ldws_active_.load() || acc_level_.load() > 0;

  int interval_ms = 500;
  const auto level = acc_level_.load();
  if (level == 1) {
    interval_ms = 600;
  } else if (level == 2) {
    interval_ms = 400;
  } else if (level == 3) {
    interval_ms = 200;
  }

  warning_interval_ms_.store(interval_ms);
  warning_active_.store(active);
  priority1_active_.store(active);

  warning_cv_.notify_all();
  if (!active) {
    priority_cv_.notify_all();
  }
}

void SoundPlayNode::play_blocking(const fs::path & file_path, bool log_info)
{
  if (file_path.empty() || !fs::exists(file_path)) {
    return;
  }

  const auto ext = file_path.extension().string();
  std::vector<std::string> command;

  if (ext == ".wav" || ext == ".WAV") {
    command = {"aplay", "-q", file_path.string()};
  } else if (ext == ".mp3" || ext == ".MP3" || ext == ".ogg" || ext == ".OGG") {
    command = {"ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", file_path.string()};
  } else {
    RCLCPP_WARN(get_logger(), "Unsupported file format: %s", ext.c_str());
    return;
  }

  if (log_info) {
    RCLCPP_INFO(get_logger(), "Playing: %s", file_path.filename().string().c_str());
  }

  pid_t pid = fork();
  
  // fork 실패 처리
  if (pid < 0) {
    RCLCPP_ERROR(get_logger(), "fork() failed: %s", strerror(errno));
    return;
  }
  
  if (pid == 0) {
    // 자식 프로세스
    std::vector<char *> argv;
    argv.reserve(command.size() + 1);
    for (const auto & token : command) {
      argv.push_back(const_cast<char *>(token.c_str()));
    }
    argv.push_back(nullptr);
    execvp(argv[0], argv.data());
    _exit(127);
  }

  // 부모 프로세스: waitpid 에러 처리
  if (pid > 0) {
    int status = 0;
    pid_t result = waitpid(pid, &status, 0);
    
    if (result == -1) {
      RCLCPP_ERROR(get_logger(), "waitpid() failed: %s", strerror(errno));
    } else if (WIFEXITED(status)) {
      const int exit_code = WEXITSTATUS(status);
      if (exit_code != 0) {
        RCLCPP_WARN(get_logger(), 
          "Audio process exited with code %d for file: %s", 
          exit_code, file_path.filename().string().c_str());
      }
    } else if (WIFSIGNALED(status)) {
      RCLCPP_WARN(get_logger(), 
        "Audio process killed by signal %d", WTERMSIG(status));
    }
  }
}

}  // namespace sound_play_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sound_play_pkg::SoundPlayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
