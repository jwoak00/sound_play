#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace sound_play_pkg
{
namespace fs = std::filesystem;

class SoundPlayNode : public rclcpp::Node
{
public:
  SoundPlayNode()
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

  ~SoundPlayNode() override
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

    warning_thread_running_ = false;
    warning_cv_.notify_all();
    if (warning_thread_.joinable()) {
      warning_thread_.join();
    }
  }

private:
  void init_subscriptions()
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
            enqueue_file("system_failure.wav");
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
            enqueue_file("lane_right.mp3");
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
            enqueue_file("lane_left.mp3");
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
            enqueue_file("lane_cancle.mp3");
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
            enqueue_file("lane_finish.mp3");
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

  void handle_ldws(bool active)
  {
    const bool previous = ldws_active_.exchange(active);
    if (previous != active) {
      RCLCPP_INFO(get_logger(), "LDWS %s", active ? "ON" : "OFF");
    }
    update_warning_state();
  }

  void handle_acc(uint8_t level)
  {
    const uint8_t clamped = std::min<uint8_t>(static_cast<uint8_t>(3), level);
    const uint8_t previous = acc_level_.exchange(clamped);
    if (previous != clamped) {
      RCLCPP_INFO(get_logger(), "ACC level: %u", clamped);
    }
    update_warning_state();
  }

  void handle_driver_mode(bool active)
  {
    if (active && !driver_mode_active_) {
      enqueue_file("driver_mode.wav");
    }

    driver_mode_active_ = active;
    if (active) {
      autonomous_mode_active_ = false;
    }
  }

  void handle_autonomous_mode(bool active)
  {
    if (!active) {
      autonomous_mode_active_ = false;
      return;
    }

    if (driving_disable_area_active_) {
      enqueue_file("driving_disable_area.mp3");
      driver_mode_active_ = true;
      autonomous_mode_active_ = false;
      return;
    }

    if (!autonomous_mode_active_) {
      enqueue_file("autonomous_mode.wav");
    }

    autonomous_mode_active_ = true;
    driver_mode_active_ = false;
  }

  void handle_driving_disable_area(bool active)
  {
    driving_disable_area_active_ = active;
  }

  void handle_velocity(const std_msgs::msg::Float32::SharedPtr msg)
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

  void enqueue_file(const std::string & file_name)
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

  void queue_worker()
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

  bool wait_for_priority_clear()
  {
    std::unique_lock<std::mutex> lock(priority_mutex_);
    priority_cv_.wait(lock, [this]() {
      return !queue_running_.load() || !priority1_active_.load();
    });

    return queue_running_.load();
  }

  void warning_worker()
  {
    std::unique_lock<std::mutex> lock(warning_mutex_);
    while (warning_thread_running_) {
      warning_cv_.wait(lock, [this]() {
        return !warning_thread_running_ || warning_active_.load();
      });

      if (!warning_thread_running_) {
        break;
      }

      while (warning_thread_running_ && warning_active_.load()) {
        lock.unlock();
        play_blocking(sounds_dir_ / "beep.wav", false);
        const auto interval = std::chrono::milliseconds(warning_interval_ms_.load());
        std::this_thread::sleep_for(interval);
        lock.lock();
      }
      priority_cv_.notify_all();
    }
  }

  void update_warning_state()
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

  void play_blocking(const fs::path & file_path, bool log_info = true)
  {
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

  std::vector<std::string> build_command(const fs::path & file_path)
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

  pid_t launch_process(const std::vector<std::string> & command)
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

  fs::path sounds_dir_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ldws_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr acc_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr system_failure_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr driver_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr driving_disable_area_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lane_change_right_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lane_change_left_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lane_change_finish_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lane_change_cancel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_sub_;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::queue<fs::path> playback_queue_;
  std::atomic<bool> queue_running_{true};
  std::thread queue_thread_;

  std::mutex priority_mutex_;
  std::condition_variable priority_cv_;
  std::atomic<bool> priority1_active_{false};

  std::mutex warning_mutex_;
  std::condition_variable warning_cv_;
  std::thread warning_thread_;
  std::atomic<bool> warning_thread_running_{true};
  std::atomic<bool> warning_active_{false};
  std::atomic<int> warning_interval_ms_{500};

  std::atomic<bool> ldws_active_{false};
  std::atomic<uint8_t> acc_level_{0};

  bool driver_mode_active_{false};
  bool autonomous_mode_active_{false};
  bool driving_disable_area_active_{false};
  std::atomic<int> last_speed_step_{0};
};

}  // namespace sound_play_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sound_play_pkg::SoundPlayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

