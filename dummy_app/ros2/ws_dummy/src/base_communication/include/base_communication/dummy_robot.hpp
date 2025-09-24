#pragma once
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include "base_communication/serial_link.hpp"

class DummyRobot : public SerialLink {
public:
  explicit DummyRobot(const std::string& dev,
                      unsigned baud,
                      std::chrono::milliseconds default_timeout = std::chrono::milliseconds(500));

  // 控制命令：返回 true 表示成功（见 cpp 中各自默认超时）
  bool start_robot (std::chrono::milliseconds timeout = std::chrono::milliseconds(-1));
  bool stop_robot  (std::chrono::milliseconds timeout = std::chrono::milliseconds(-1));
  bool home_robot  (std::chrono::milliseconds timeout = std::chrono::milliseconds(-1));
  bool reset_robot (std::chrono::milliseconds timeout = std::chrono::milliseconds(-1));
  bool disable_robot(std::chrono::milliseconds timeout = std::chrono::milliseconds(-1));

  // 查询：成功返回 true，并把 6 轴数据写入 out（单位：度）
  bool get_joint_positions(std::array<double,6>& out,
                           std::chrono::milliseconds timeout = std::chrono::milliseconds(-1));
  bool get_link_pose(std::array<double,6>& out,
                     std::chrono::milliseconds timeout = std::chrono::milliseconds(-1));

  // 关节移动（逗号分隔，无空格；可携带 speed）
  // Fire-and-forget：只发送指令，不等待回包；timeout 参数被忽略
  bool move_joints(const std::array<double,6>& q,
                   std::optional<double> speed = std::nullopt,
                   std::chrono::milliseconds timeout = std::chrono::milliseconds(-1));
  bool move_joints(double j1, double j2, double j3,
                   double j4, double j5, double j6,
                   std::optional<double> speed = std::nullopt,
                   std::chrono::milliseconds timeout = std::chrono::milliseconds(-1));

private:
  // ---- SerialLink 的行回调：分类入队（带上限）----
  void on_line_(const std::string& line);

  // 等“任意回包”一行（给 send_and_expect_ok 用）
  bool wait_next_any_line_(std::string& out, std::chrono::milliseconds timeout);
  // 等“状态行 ok+6值”（给 get_* 用）
  bool wait_next_status_line_(std::string& out, std::chrono::milliseconds timeout);

  // 发送命令并等待“ok”；accept_no_response=true 时在超时且完全没回包也算成功
  bool send_and_expect_ok(const std::string& cmd,
                          std::chrono::milliseconds timeout,
                          bool accept_no_response = false);

  // 工具
  static bool is_ok_like(const std::string& trimmed);
  static bool parse_ok_six(const std::string& line, std::array<double,6>& out);

private:
  std::chrono::milliseconds default_timeout_;

  // 所有回包（ACK/日志/状态等全部）
  std::mutex m_all_;
  std::condition_variable cv_all_;
  std::deque<std::string> all_lines_;

  // 仅状态行："ok <6个数>"
  std::mutex m_stat_;
  std::condition_variable cv_stat_;
  std::deque<std::string> status_lines_;

  // 队列上限（可按需调整）
  static constexpr size_t MAX_ALL_LINES    = 200;
  static constexpr size_t MAX_STATUS_LINES = 50;
};
