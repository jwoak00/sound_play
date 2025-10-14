#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <csignal>
#include <cerrno>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <filesystem>
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

    this->declare_parameter<std::string>("topic", "", dyn_desc);
    this->declare_parameter<std::string>("play_mode", "", dyn_desc);
    this->declare_parameter<std::vector<int64_t>>("file_ids", {}, dyn_desc);
    this->declare_parameter<std::vector<std::string>>("file_names", {}, dyn_desc);
    this->declare_parameter<std::string>("sounds_dir", "", dyn_desc);

    topic_ = this->get_parameter("topic").as_string();
    if (topic_.empty()) {
      RCLCPP_ERROR(get_logger(), "Required parameter 'topic' not set");
      throw std::runtime_error("missing required parameter: topic");
    }

    play_mode_ = this->get_parameter("play_mode").as_string();
    if (play_mode_ != "single" && play_mode_ != "overlap") {
      RCLCPP_ERROR(get_logger(), "Parameter 'play_mode' must be 'single' or 'overlap'");
      throw std::runtime_error("invalid or missing play_mode");
    }

    const auto sounds_param = this->get_parameter("sounds_dir").as_string();
    if (!sounds_param.empty()) {
      sounds_dir_ = fs::weakly_canonical(fs::path{sounds_param});
    } else {
      const auto share_dir = ament_index_cpp::get_package_share_directory("sound_play_pkg");
      sounds_dir_ = fs::path{share_dir} / "sound";
    }

    if (!fs::exists(sounds_dir_) || !fs::is_directory(sounds_dir_)) {
      RCLCPP_WARN(get_logger(), "sounds_dir not found: %s", sounds_dir_.string().c_str());
    }

    build_mapping();
    subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
      topic_, rclcpp::QoS{10}, std::bind(&SoundPlayNode::on_message, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Listening on %s, sounds dir: %s", topic_.c_str(), sounds_dir_.string().c_str());
  }

  ~SoundPlayNode() override
  {
    stop_current();
    reap_children();
  }

private:
  void build_mapping()
  {
    const auto ids_param = this->get_parameter("file_ids").as_integer_array();
    const auto names_param = this->get_parameter("file_names").as_string_array();

    if (ids_param.empty() || ids_param.size() != names_param.size()) {
      RCLCPP_ERROR(
        get_logger(),
        "file_ids / file_names must be same-length non-empty lists (provided lengths: %zu / %zu)",
        ids_param.size(), names_param.size());
      throw std::runtime_error("invalid file_ids/file_names parameters");
    }

    for (size_t i = 0; i < ids_param.size(); ++i) {
      file_map_[static_cast<int>(ids_param[i])] = names_param[i];
    }

    if (file_map_.empty()) {
      RCLCPP_ERROR(get_logger(), "Resulting file mapping empty");
      throw std::runtime_error("empty mapping after processing parameters");
    }
  }

  void on_message(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    reap_children();

    const int value = static_cast<int>(msg->data);
    if (last_played_id_ && *last_played_id_ == value) {
      return;
    }

    auto it = file_map_.find(value);
    if (it == file_map_.end()) {
      RCLCPP_WARN(get_logger(), "No file mapped for value=%d", value);
      last_played_id_ = value;
      return;
    }

    fs::path file_path = fs::weakly_canonical(sounds_dir_ / it->second);
    if (!fs::exists(file_path)) {
      RCLCPP_ERROR(get_logger(), "File not found: %s", file_path.string().c_str());
      last_played_id_ = value;
      return;
    }

    const auto command = build_command(file_path);
    if (command.empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to build command for file: %s", file_path.string().c_str());
      last_played_id_ = value;
      return;
    }

    RCLCPP_INFO(get_logger(), "Play(%d) -> %s", value, file_path.filename().string().c_str());

    if (play_mode_ == "single") {
      stop_current();
      current_pid_ = launch_process(command);
    } else {
      launch_process(command);
    }

    last_played_id_ = value;
  }

  std::vector<std::string> build_command(const fs::path & file_path)
  {
    const auto ext = file_path.extension().string();
    if (ext == ".wav" || ext == ".WAV") {
      return {"aplay", "-q", file_path.string()};
    }

    // Default to ffplay for mp3 or any other extension.
    return {"ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", file_path.string()};
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

  void stop_current()
  {
    if (current_pid_ <= 0) {
      return;
    }

    if (kill(current_pid_, SIGTERM) != 0) {
      if (errno != ESRCH) {
        RCLCPP_WARN(get_logger(), "Failed to terminate process %d", current_pid_);
      }
    }

    int status = 0;
    waitpid(current_pid_, &status, 0);
    current_pid_ = -1;
  }

  void reap_children()
  {
    int status = 0;
    pid_t pid = 0;
    do {
      pid = waitpid(-1, &status, WNOHANG);
    } while (pid > 0);
  }

  std::string topic_;
  std::string play_mode_;
  fs::path sounds_dir_;
  std::unordered_map<int, std::string> file_map_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
  std::optional<int> last_played_id_;
  pid_t current_pid_{-1};
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

