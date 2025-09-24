#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <array>
#include <string>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

struct RawTerminal {
  termios oldt{}; int oldfl{-1}; bool ok{false};
  RawTerminal() {
    if (!isatty(STDIN_FILENO)) return;
    if (tcgetattr(STDIN_FILENO, &oldt) != 0) return;
    termios t = oldt;
    t.c_lflag &= ~(ICANON | ECHO);
    t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &t) != 0) return;
    oldfl = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldfl | O_NONBLOCK);
    ok = true;
  }
  ~RawTerminal() {
    if (!ok) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    if (oldfl != -1) fcntl(STDIN_FILENO, F_SETFL, oldfl);
  }
};

class KeyboardTeleop : public rclcpp::Node {
public:
  KeyboardTeleop()
  : Node("dummy_keyboard_teleop"),
    pub_rate_hz_(declare_parameter<double>("pub_rate_hz", 100.0)),
    step_deg_(declare_parameter<double>("step_deg", 0.5)),
    j1_min_(declare_parameter<double>("j1_min_deg", -180.0)),
    j1_max_(declare_parameter<double>("j1_max_deg",  180.0)),
    use_current_on_start_(declare_parameter<bool>("use_current_on_start", true))
  {
    // —— 用绝对服务名，避免命名空间不一致 —— //
    cli_start_   = create_client<std_srvs::srv::Trigger>("/start");
    cli_home_    = create_client<std_srvs::srv::Trigger>("/home");
    cli_reset_   = create_client<std_srvs::srv::Trigger>("/reset");
    cli_disable_ = create_client<std_srvs::srv::Trigger>("/disable");
    cli_stop_    = create_client<std_srvs::srv::Trigger>("/stop");
    cli_setmode_ = create_client<std_srvs::srv::SetBool>("/set_mode");

    // 订阅 joint_states（度）
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&KeyboardTeleop::on_joint_state, this, std::placeholders::_1));

    // 发布 dummy_control（保持绝对话题名，避免命名空间错位）
    pub_ctrl_ = create_publisher<std_msgs::msg::Float64MultiArray>("/dummy_control",
                                                                   rclcpp::SystemDefaultsQoS());

    // 控制发布定时器（始终发布，机器人在“指令模式”会忽略）
    auto dt = std::chrono::duration<double>(1.0 / std::max(1.0, pub_rate_hz_));
    ctrl_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(dt),
      std::bind(&KeyboardTeleop::on_ctrl_timer, this));

    // 键盘轮询（10 ms）
    key_timer_ = create_wall_timer(10ms, std::bind(&KeyboardTeleop::on_key_timer, this));

    // 初始 desired
    desired_ = use_current_on_start_ ? std::array<double,6>{0.0,-0.0,90.0,0.0,0.0,0.0}
                                     : std::array<double,6>{0,0,0,0,0,0};

    print_help();
  }

private:
  void print_help() {
    RCLCPP_INFO(get_logger(),
      "Keyboard teleop:\n"
      "  [1]=START  [2]=HOME  [3]=RESET  [4]=DISABLE  [5]=STOP  [6]=MODE:JOINT_CONTROL\n"
      "  [A/a]=J1 -step   [D/d]=J1 +step   [Q/q]=quit\n"
      "  step=%.3f deg  pub_rate=%.1f Hz  limits=[%.1f, %.1f] deg",
      step_deg_, pub_rate_hz_, j1_min_, j1_max_);
  }

  // joint_states 回调
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() >= 6) {
      for (size_t i=0;i<6;++i) current_[i] = msg->position[i];
      if (!have_current_ && use_current_on_start_) {
        have_current_ = true;
        desired_ = current_;
      }
    }
  }

  // 定时发布当前 desired（不依赖模式）
  void on_ctrl_timer() {
    std_msgs::msg::Float64MultiArray m;
    m.data.assign(desired_.begin(), desired_.end());
    pub_ctrl_->publish(m);
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                          "pub /dummy_control: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                          desired_[0],desired_[1],desired_[2],desired_[3],desired_[4],desired_[5]);
  }

  // 非阻塞键盘读取
  void on_key_timer() {
    static RawTerminal rt; (void)rt;

    char c;
    while (true) {
      ssize_t n = ::read(STDIN_FILENO, &c, 1);
      if (n <= 0) break;

      switch (c) {
        case '1': call_trigger(cli_start_,   "START");   break;
        case '2': call_trigger(cli_home_,    "HOME");    break;
        case '3': call_trigger(cli_reset_,   "RESET");   break;
        case '4': call_trigger(cli_disable_, "DISABLE"); break;
        case '5': call_trigger(cli_stop_,    "STOP");    break;
        case '6': set_mode_joint_control(true);          break;

        case 'a': case 'A': step_j1(-step_deg_); break;
        case 'd': case 'D': step_j1(+step_deg_); break;

        case 'q': case 'Q':
          RCLCPP_INFO(get_logger(), "Quit requested.");
          rclcpp::shutdown();
          return;

        default: break;
      }
    }
  }

  void step_j1(double delta_deg) {
    desired_[0] = std::clamp(desired_[0] + delta_deg, j1_min_, j1_max_);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
                         "J1 desired = %.3f deg", desired_[0]);
  }

  // 调 Trigger
  void call_trigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& cli,
                    const char* name) {
    if (!cli->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "%s: service not ready (waiting...)", name);
      return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    (void)cli->async_send_request(req,
      [this,name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f){
        auto res = f.get();
        RCLCPP_INFO(get_logger(), "%s -> %s: %s",
                    name, res->success ? "OK":"FAIL", res->message.c_str());
      });
  }

  // set_mode
  void set_mode_joint_control(bool on) {
    if (!cli_setmode_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "set_mode: service not ready (waiting...)");
      return;
    }
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = on;
    (void)cli_setmode_->async_send_request(req,
      [this,on](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f){
        auto res = f.get();
        RCLCPP_INFO(get_logger(), "set_mode(%s) -> %s: %s",
                    on ? "JOINT_CONTROL" : "COMMAND",
                    res->success ? "OK":"FAIL", res->message.c_str());
      });
  }

private:
  // 参数
  double pub_rate_hz_;
  double step_deg_;
  double j1_min_, j1_max_;
  bool   use_current_on_start_;

  // 状态
  std::array<double,6> current_{0,0,0,0,0,0};
  std::array<double,6> desired_{0,0,0,0,0,0};
  bool have_current_{false};

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_ctrl_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_start_, cli_home_, cli_reset_, cli_disable_, cli_stop_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cli_setmode_;
  rclcpp::TimerBase::SharedPtr key_timer_, ctrl_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
