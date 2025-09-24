#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <array>
#include <optional>
#include <mutex>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>

#include "base_communication/dummy_robot.hpp"

using namespace std::chrono_literals;

static double max_abs_diff(const std::array<double,6>& a, const std::array<double,6>& b) {
  double m=0; for (size_t i=0;i<6;++i) m = std::max(m, std::fabs(a[i]-b[i])); return m;
}

class DummyRobotControl : public rclcpp::Node {
public:
  DummyRobotControl()
  : Node("dummy_robot_control"),
    device_(declare_parameter<std::string>("device", "/dev/ttyACM0")),
    baud_(declare_parameter<int>("baud", 115200)),
    control_rate_hz_(declare_parameter<double>("control_rate_hz", 100.0)),   // 统一定时器频率（控制主频）
    status_rate_joint_hz_(declare_parameter<double>("status_rate_joint_hz", 100.0)), // 关节模式下的查询频率
    status_rate_cmd_hz_(declare_parameter<double>("status_rate_cmd_hz", 100.0)),    // 指令模式下的查询频率
    min_tx_gap_ms_(declare_parameter<int>("min_tx_gap_ms", 5)),
    keepalive_ms_(declare_parameter<int>("keepalive_ms", 150)), // 即使未变化，最久多久发一次控制帧
    eps_deg_(declare_parameter<double>("eps_deg", 0.02)),
    control_speed_param_(declare_parameter<double>("control_speed", -1.0)),
    robot_(device_, static_cast<unsigned>(baud_), std::chrono::milliseconds(500))
  {
    RCLCPP_INFO(get_logger(), "Opening serial on %s @ %d", device_.c_str(), baud_);
    robot_.start();

    // 绝对名，避免命名空间错位
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());

    // 订阅控制目标：只缓存最新，不直接发串口
    cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/dummy_control", rclcpp::SystemDefaultsQoS().keep_last(1),
      std::bind(&DummyRobotControl::on_joint_command, this, std::placeholders::_1));

    // 服务
    srv_start_   = create_service<std_srvs::srv::Trigger>("/start",
                     std::bind(&DummyRobotControl::srv_start, this, std::placeholders::_1, std::placeholders::_2));
    srv_home_    = create_service<std_srvs::srv::Trigger>("/home",
                     std::bind(&DummyRobotControl::srv_home, this, std::placeholders::_1, std::placeholders::_2));
    srv_reset_   = create_service<std_srvs::srv::Trigger>("/reset",
                     std::bind(&DummyRobotControl::srv_reset, this, std::placeholders::_1, std::placeholders::_2));
    srv_disable_ = create_service<std_srvs::srv::Trigger>("/disable",
                     std::bind(&DummyRobotControl::srv_disable, this, std::placeholders::_1, std::placeholders::_2));
    srv_stop_    = create_service<std_srvs::srv::Trigger>("/stop",
                     std::bind(&DummyRobotControl::srv_stop, this, std::placeholders::_1, std::placeholders::_2));
    srv_set_mode_= create_service<std_srvs::srv::SetBool>("/set_mode",
                     std::bind(&DummyRobotControl::srv_set_mode, this, std::placeholders::_1, std::placeholders::_2));

    joint_names_ = {"joint1","joint2","joint3","joint4","joint5","joint6"};

    // 统一定时器：TDM 调度控制/查询
    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
    main_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&DummyRobotControl::main_tick, this));

    // 计算分频（保证 >=1）
    recompute_dividers();

    RCLCPP_INFO(get_logger(),
      "Ready. Mode=%s | control=%.1f Hz | status(joint)=%.1f Hz | status(cmd)=%.1f Hz | min_gap=%dms | eps=%.3f deg",
      joint_control_mode_ ? "JOINT_CONTROL" : "COMMAND",
      control_rate_hz_, status_rate_joint_hz_, status_rate_cmd_hz_, min_tx_gap_ms_, eps_deg_);
  }

private:
  // ———————————— TDM 调度 ————————————
  void recompute_dividers() {
    // joint_status_div_ = std::max(1, static_cast<int>(std::round(control_rate_hz_ / std::max(0.1, status_rate_joint_hz_))));
    // cmd_status_div_   = std::max(1, static_cast<int>(std::round(control_rate_hz_ / std::max(0.1, status_rate_cmd_hz_))));
    // tick_ = 0;

      // 关节模式：至少保留一半时隙给控制
  joint_status_div_ = std::max(2, static_cast<int>(
      std::round(control_rate_hz_ / std::max(0.1, status_rate_joint_hz_))));

  // 指令模式下只查不控，div>=1 即可
  cmd_status_div_   = std::max(1, static_cast<int>(
      std::round(control_rate_hz_ / std::max(0.1, status_rate_cmd_hz_))));

  tick_ = 0;

  }

  void main_tick() {
    ++tick_;
    const auto now = std::chrono::steady_clock::now();

    // 选择本 tick 是否“查询时隙”
    const int div = joint_control_mode_ ? joint_status_div_ : cmd_status_div_;
    const bool query_slot = (tick_ % div == 0);

    if (joint_control_mode_) {
      if (query_slot) {
        // —— 关节模式：这一个时隙只做查询，不发控制 —— //
        query_and_publish();
      } else {
        // —— 关节模式：这一个时隙只做控制，不做查询 —— //
        control_once(now);
      }
    } else {
      // —— 指令模式：不发控制，只按分频查询 —— //
      if (query_slot) query_and_publish();
    }
  }

  // 查询一次并发布 joint_states（独占串口；与控制完全错开）
  void query_and_publish() {
    std::array<double,6> q{};
    {
      std::lock_guard<std::mutex> lk(io_lock_);
      if (!robot_.get_joint_positions(q, 20ms)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "get_joint_positions timeout");
        return;
      }
      last_tx_time_ = std::chrono::steady_clock::now();
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = joint_names_;
    msg.position.assign(q.begin(), q.end());
    joint_pub_->publish(std::move(msg));
  }

  // 控制时隙：有变化才发；满足最小间隔；周期性 keepalive
  void control_once(const std::chrono::steady_clock::time_point& now) {
    // 尚无目标则不发
    std::array<double,6> target{};
    {
      std::lock_guard<std::mutex> lk(desired_mtx_);
      if (!have_desired_) return;
      target = desired_q_;
    }

    // 变化是否足够大（去抖）
    bool need_send = !has_last_sent_ || (max_abs_diff(target, last_sent_q_) > eps_deg_);

    // keepalive：即使没变化，隔一段时间也补发一帧
    if (!need_send && has_last_sent_) {
      if (now - last_sent_time_ >= std::chrono::milliseconds(keepalive_ms_))
        need_send = true;
    }
    if (!need_send) return;

    // 与上次任何串口活动至少间隔 min_tx_gap_ms_
    if (now - last_tx_time_ < std::chrono::milliseconds(min_tx_gap_ms_)) return;

    std::optional<double> spd = std::nullopt;
    if (control_speed_param_ >= 0.0) spd = control_speed_param_;

    {
      std::lock_guard<std::mutex> lk(io_lock_);
      // 真正下发（fire-and-forget）
      robot_.move_joints(target, spd);
      last_tx_time_ = std::chrono::steady_clock::now();
    }
    last_sent_q_ = target;
    last_sent_time_ = now;
    has_last_sent_ = true;

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
      "TX control: [%.2f %.2f %.2f %.2f %.2f %.2f]",
      target[0],target[1],target[2],target[3],target[4],target[5]);
  }

  // ———————————— 订阅：只缓存最新 ————————————
  void on_joint_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 6) {
      RCLCPP_WARN(get_logger(), "dummy_control needs 6 values, got %zu", msg->data.size());
      return;
    }
    std::lock_guard<std::mutex> lk(desired_mtx_);
    for (size_t i=0;i<6;++i) desired_q_[i] = msg->data[i];
    have_desired_ = true;
  }

  // ———————————— 模式切换 & 服务 ————————————
  void srv_set_mode(const std_srvs::srv::SetBool::Request::SharedPtr req,
                    std_srvs::srv::SetBool::Response::SharedPtr res) {
    joint_control_mode_ = req->data;
    res->success = true;
    res->message = joint_control_mode_ ? "mode=JOINT_CONTROL" : "mode=COMMAND";
    // 模式切换后重置分频计数，避免刚切过去就连发控制/查询
    tick_ = 0; has_last_sent_ = false;
    RCLCPP_INFO(get_logger(), "Switched mode: %s", res->message.c_str());
  }

  void srv_start (const std_srvs::srv::Trigger::Request::SharedPtr,
                  std_srvs::srv::Trigger::Response::SharedPtr res) {
    std::lock_guard<std::mutex> lk(io_lock_);
    bool ok = robot_.start_robot(5000ms);
    res->success = ok; res->message = ok ? "Started" : "Start failed";
    last_tx_time_ = std::chrono::steady_clock::now();
  }
  void srv_disable(const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
    std::lock_guard<std::mutex> lk(io_lock_);
    bool ok = robot_.disable_robot(5000ms);
    res->success = ok; res->message = ok ? "Disabled" : "Disable failed";
    last_tx_time_ = std::chrono::steady_clock::now();
  }
  void srv_stop  (const std_srvs::srv::Trigger::Request::SharedPtr,
                  std_srvs::srv::Trigger::Response::SharedPtr res) {
    std::lock_guard<std::mutex> lk(io_lock_);
    bool ok = robot_.stop_robot(5000ms);
    res->success = ok; res->message = ok ? "Stopped" : "Stop failed";
    last_tx_time_ = std::chrono::steady_clock::now();
  }

  // HOME/RESET：只发一次命令，然后通过“低频查询”判断稳定（注意：期间 TDM 仍在运行）
  void srv_home  (const std_srvs::srv::Trigger::Request::SharedPtr,
                  std_srvs::srv::Trigger::Response::SharedPtr res) {
    {
      std::lock_guard<std::mutex> lk(io_lock_);
      robot_.send("!HOME\r\n");
      last_tx_time_ = std::chrono::steady_clock::now();
    }
    bool ok = wait_until_stable(0.2, 15, 120s, 100ms);
    res->success = ok; res->message = ok ? "Homed by feedback" : "Home timeout (not stable)";
    RCLCPP_INFO(get_logger(), "[HOME] %s", res->message.c_str());
  }

  void srv_reset (const std_srvs::srv::Trigger::Request::SharedPtr,
                  std_srvs::srv::Trigger::Response::SharedPtr res) {
    {
      std::lock_guard<std::mutex> lk(io_lock_);
      robot_.send("!RESET\r\n");
      last_tx_time_ = std::chrono::steady_clock::now();
    }
    bool ok = wait_until_stable(0.2, 15, 120s, 100ms);
    res->success = ok; res->message = ok ? "Reset complete by feedback" : "Reset timeout (not stable)";
    RCLCPP_INFO(get_logger(), "[RESET] %s", res->message.c_str());
  }

  // 关节稳定性判定（调用查询函数；TDM 会安排查询时隙，因此不会和控制“贴身”）
  bool wait_until_stable(double tol_deg,
                         int stable_samples,
                         std::chrono::milliseconds timeout,
                         std::chrono::milliseconds poll)
  {
    using Clock = std::chrono::steady_clock;
    auto t0 = Clock::now();
    std::array<double,6> prev{}, cur{};
    bool has_prev = false;
    int ok_cnt = 0;

    while (Clock::now() - t0 < timeout) {
      {
        std::lock_guard<std::mutex> lk(io_lock_);
        robot_.get_joint_positions(cur, 20ms);
        last_tx_time_ = std::chrono::steady_clock::now();
      }
      if (has_prev) {
        double md=0; for (size_t i=0;i<6;++i) md = std::max(md, std::fabs(cur[i]-prev[i]));
        if (md < tol_deg) { if (++ok_cnt >= stable_samples) return true; }
        else ok_cnt = 0;
      } else {
        has_prev = true;
      }
      prev = cur;
      std::this_thread::sleep_for(poll);
    }
    return false;
  }

private:
  // 参数
  std::string device_;
  int         baud_;
  double      control_rate_hz_;
  double      status_rate_joint_hz_;
  double      status_rate_cmd_hz_;
  int         min_tx_gap_ms_;
  int         keepalive_ms_;
  double      eps_deg_;
  double      control_speed_param_;

  // 机器人与并发
  DummyRobot robot_;
  std::mutex io_lock_;

  // 目标缓存（订阅回调只写）
  std::mutex desired_mtx_;
  std::array<double,6> desired_q_{0,0,0,0,0,0};
  bool have_desired_{false};

  // 最近一次真正下发的控制帧
  std::array<double,6> last_sent_q_{0,0,0,0,0,0};
  bool has_last_sent_{false};
  std::chrono::steady_clock::time_point last_sent_time_{};

  // 最近任一串口 TX/RX 时间（用来做 min_tx_gap）
  std::chrono::steady_clock::time_point last_tx_time_{};

  // 模式
  bool joint_control_mode_{false};

  // TDM 分频
  rclcpp::TimerBase::SharedPtr main_timer_;
  int tick_{0};
  int joint_status_div_{1};
  int cmd_status_div_{1};

  // ROS 接口
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_, srv_home_, srv_reset_, srv_disable_, srv_stop_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_mode_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DummyRobotControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
