#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

namespace sound_play_pkg
{

class SoundPlayNode : public rclcpp::Node
{
public:
  SoundPlayNode();
  ~SoundPlayNode() override;

private:
  void init_subscriptions();

  void handle_ldws(bool active);
  void handle_acc(uint8_t level);
  void handle_driver_mode(bool active);
  void handle_autonomous_mode(bool active);
  void handle_driving_disable_area(bool active);
  void handle_velocity(const tier4_planning_msgs::msg::VelocityLimit::SharedPtr msg);

  void enqueue_file(const std::string & file_name);
  void enqueue_sound(const std::string & key);
  
  // 속도 음원 전용 큐 관리
  void enqueue_speed_sound(int speed_step);
  void speed_debounce_worker();

  void queue_worker();
  bool wait_for_priority_clear();
  void warning_worker();
  void update_warning_state();

  void play_blocking(const std::filesystem::path & file_path, bool log_info = true);

  static std::map<std::string, std::string> default_sound_map();

  std::filesystem::path sounds_dir_;
  std::map<std::string, std::string> sound_map_;

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
  rclcpp::Subscription<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr velocity_sub_;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::queue<std::filesystem::path> playback_queue_;
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

  // 속도 음원 디바운싱용
  std::mutex speed_mutex_;
  std::condition_variable speed_cv_;
  std::thread speed_thread_;
  std::atomic<bool> speed_thread_running_{true};
  std::atomic<int> pending_speed_step_{-1};  // -1 = 대기 중인 속도 없음
  std::atomic<bool> speed_changed_{false};

  std::atomic<int> debounce_ms_{300};
};

}  // namespace sound_play_pkg
