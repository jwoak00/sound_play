#include "sound_play_pkg/sound_play_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <chrono>
#include <map>
#include <utility>
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

std::vector<std::string> split_entries(const std::string & input)
{
  std::vector<std::string> tokens;
  std::string current;
  bool in_quote = false;
  char quote_char = '\0';

  for (char c : input) {
    if ((c == '"' || c == '\'') && (!in_quote || c == quote_char)) {
      if (in_quote) {
        in_quote = false;
        quote_char = '\0';
      } else {
        in_quote = true;
        quote_char = c;
      }
    }

    if (!in_quote && (c == ',' || c == '\n')) {
      const auto trimmed = trim_copy(current);
      if (!trimmed.empty()) {
        tokens.push_back(trimmed);
      }
      current.clear();
      continue;
    }

    current.push_back(c);
  }

  const auto trimmed = trim_copy(current);
  if (!trimmed.empty()) {
    tokens.push_back(trimmed);
  }
  return tokens;
}

std::map<std::string, std::string> parse_mapping_string(const std::string & raw)
{
  std::map<std::string, std::string> result;
  auto data = trim_copy(raw);
  if (data.empty()) {
    return result;
  }

  if (data.front() == '{' && data.back() == '}') {
    data = trim_copy(data.substr(1, data.size() - 2));
  }

  for (const auto & entry : split_entries(data)) {
    const auto colon_pos = entry.find(':');
    const auto equal_pos = entry.find('=');
    const auto delim_pos = colon_pos != std::string::npos ? colon_pos : equal_pos;
    if (delim_pos == std::string::npos) {
      continue;
    }

    auto key = strip_quotes(trim_copy(entry.substr(0, delim_pos)));
    auto value = strip_quotes(trim_copy(entry.substr(delim_pos + 1)));
    if (!key.empty() && !value.empty()) {
      result[key] = value;
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
    {"lane_change_finish", "lane_cancle.mp3"},
    {"lane_change_cancel", "lane_finish.mp3"}
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

  sound_map_ = default_sound_map();

  const auto mapping_param = get_parameter("sounds.mapping");
  std::map<std::string, std::string> overrides;
  if (mapping_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    const auto raw = mapping_param.as_string();
    if (!raw.empty()) {
      overrides = parse_mapping_string(raw);
      if (overrides.empty()) {
        RCLCPP_WARN(
          get_logger(), "Failed to parse sounds.mapping parameter. Using default sound map.");
      }
    }
  } else if (mapping_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    const auto raw = mapping_param.as_string_array();
    if (!raw.empty()) {
      overrides = parse_mapping_array(raw);
      if (overrides.empty()) {
        RCLCPP_WARN(
          get_logger(), "Failed to parse sounds.mapping string array. Using default sound map.");
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
    sounds_dir_ = fs::path{share_dir} / "sound";
  }

  if (!fs::exists(sounds_dir_) || !fs::is_directory(sounds_dir_)) {
    RCLCPP_WARN(get_logger(), "sounds_dir not found: %s", sounds_dir_.string().c_str());
  }

  queue_thread_ = std::thread(&SoundPlayNode::queue_worker, this);
  warning_thread_ = std::thread(&SoundPlayNode::warning_worker, this);

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
}

void SoundPlayNode::init_subscriptions()
{
  const auto ldws_topic = get_parameter("topics.ldws").as_string();
  if (!ldws_topic.empty()) {
    ldws_sub_ = create_subscription<std_msgs::msg::Bool>(
      ldws_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::Bool::SharedPtr msg) { handle_ldws(msg->data); });
    RCLCPP_INFO(get_logger(), "LDWS topic: %s", ldws_topic.c_str());
  }

  const auto acc_topic = get_parameter("topics.acc").as_string();
  if (!acc_topic.empty()) {
    acc_sub_ = create_subscription<std_msgs::msg::UInt8>(
      acc_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::UInt8::SharedPtr msg) { handle_acc(msg->data); });
    RCLCPP_INFO(get_logger(), "ACC topic: %s", acc_topic.c_str());
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
    RCLCPP_INFO(get_logger(), "System failure topic: %s", system_topic.c_str());
  }

  const auto autonomous_topic = get_parameter("topics.autonomous_mode").as_string();
  if (!autonomous_topic.empty()) {
    autonomous_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      autonomous_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::Bool::SharedPtr msg) { handle_autonomous_mode(msg->data); });
    RCLCPP_INFO(get_logger(), "Autonomous mode topic: %s", autonomous_topic.c_str());
  }

  const auto driver_topic = get_parameter("topics.driver_mode").as_string();
  if (!driver_topic.empty()) {
    driver_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      driver_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::Bool::SharedPtr msg) { handle_driver_mode(msg->data); });
    RCLCPP_INFO(get_logger(), "Driver mode topic: %s", driver_topic.c_str());
  }

  const auto disable_topic = get_parameter("topics.driving_disable_area").as_string();
  if (!disable_topic.empty()) {
    driving_disable_area_sub_ = create_subscription<std_msgs::msg::Bool>(
      disable_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::Bool::SharedPtr msg) { handle_driving_disable_area(msg->data); });
    RCLCPP_INFO(get_logger(), "Driving disable area topic: %s", disable_topic.c_str());
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
    RCLCPP_INFO(get_logger(), "Lane change right topic: %s", lane_right_topic.c_str());
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
    RCLCPP_INFO(get_logger(), "Lane change left topic: %s", lane_left_topic.c_str());
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
    RCLCPP_INFO(get_logger(), "Lane change finish topic: %s", lane_finish_topic.c_str());
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
    RCLCPP_INFO(get_logger(), "Lane change cancel topic: %s", lane_cancel_topic.c_str());
  }

  const auto speed_topic = get_parameter("topics.speed").as_string();
  if (!speed_topic.empty()) {
    velocity_sub_ = create_subscription<std_msgs::msg::Float32>(
      speed_topic, rclcpp::QoS{10},
      [this](const std_msgs::msg::Float32::SharedPtr msg) { handle_velocity(msg); });
    RCLCPP_INFO(get_logger(), "Velocity topic: %s", speed_topic.c_str());
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

void SoundPlayNode::handle_velocity(const std_msgs::msg::Float32::SharedPtr msg)
{
  const double kmh = msg->data * 3.6;
  if (kmh < 0.0) {
    return;
  }

  int step = static_cast<int>(kmh / 5.0) * 5;
  if (step < 5) {
    last_speed_step_ = step;
    return;
  }

  step = std::min(step, 80);

  const int previous = last_speed_step_.load();
  if (step > previous) {
    enqueue_file(std::to_string(step) + ".mp3");
  }

  last_speed_step_ = step;
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
  const auto sound_file = get_sound_file(key);
  if (sound_file.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kWarnThrottleMs,
      "Sound mapping missing for key '%s'", key.c_str());
    return;
  }

  enqueue_file(sound_file);
}

std::filesystem::path SoundPlayNode::resolve_sound_path(const std::string & key) const
{
  const auto sound_file = get_sound_file(key);
  if (sound_file.empty()) {
    return {};
  }
  return sounds_dir_ / sound_file;
}

std::string SoundPlayNode::get_sound_file(const std::string & key) const
{
  const auto it = sound_map_.find(key);
  if (it == sound_map_.end()) {
    return {};
  }
  return it->second;
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
      const auto beep_path = resolve_sound_path("warning_beep");
      const auto interval = std::chrono::milliseconds(warning_interval_ms_.load());

      if (beep_path.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), kWarnThrottleMs,
          "Sound mapping missing for key 'warning_beep'");
        lock.unlock();
        std::this_thread::sleep_for(interval);
        lock.lock();
        continue;
      }

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
  } else if (level >= 3) {
    interval_ms = 200;
  } else if (ldws_active_.load()) {
    interval_ms = 500;
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
  if (file_path.empty()) {
    return;
  }

  if (!fs::exists(file_path)) {
    RCLCPP_WARN(get_logger(), "Sound file missing: %s", file_path.string().c_str());
    return;
  }

  const auto command = build_command(file_path);
  if (command.empty()) {
    RCLCPP_ERROR(get_logger(), "Failed to build command for file: %s", file_path.string().c_str());
    return;
  }

  if (log_info) {
    RCLCPP_INFO(get_logger(), "Play -> %s", file_path.filename().string().c_str());
  } else {
    RCLCPP_DEBUG(get_logger(), "Play (priority) -> %s", file_path.filename().string().c_str());
  }

  pid_t pid = launch_process(command);
  if (pid <= 0) {
    return;
  }

  int status = 0;
  waitpid(pid, &status, 0);
}

std::vector<std::string> SoundPlayNode::build_command(const fs::path & file_path)
{
  const auto ext = file_path.extension().string();
  if (ext == ".wav" || ext == ".WAV") {
    return {"aplay", "-q", file_path.string()};
  }

  if (ext == ".mp3" || ext == ".MP3") {
    return {"ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", file_path.string()};
  }

  return {};
}

pid_t SoundPlayNode::launch_process(const std::vector<std::string> & command)
{
  pid_t pid = fork();
  if (pid == 0) {
    std::vector<char *> argv;
    argv.reserve(command.size() + 1);
    for (const auto & token : command) {
      argv.push_back(const_cast<char *>(token.c_str()));
    }
    argv.push_back(nullptr);
    execvp(argv[0], argv.data());
    _exit(127);
  }

  if (pid < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to fork for audio playback");
  }

  return pid;
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

