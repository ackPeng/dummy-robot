#include "base_communication/dummy_robot.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

using namespace std::chrono_literals;

namespace { constexpr const char* EOL = "\r\n"; }

// ================= 构造 & 回调 =================
DummyRobot::DummyRobot(const std::string& dev, unsigned baud,
                       std::chrono::milliseconds default_timeout)
: SerialLink(dev, baud, [this](const std::string& line){ this->on_line_(line); }),
  default_timeout_(default_timeout)
{}

void DummyRobot::on_line_(const std::string& line) {
  // 1) 全量队列：所有回包
  {
    std::lock_guard<std::mutex> lk(m_all_);
    all_lines_.push_back(line);
    if (all_lines_.size() > MAX_ALL_LINES) all_lines_.pop_front();
    cv_all_.notify_one();
  }

  // 2) 状态队列：仅 "ok <6个数>"
  std::array<double,6> dummy{};
  if (parse_ok_six(line, dummy)) {
    std::lock_guard<std::mutex> lk2(m_stat_);
    status_lines_.push_back(line);
    if (status_lines_.size() > MAX_STATUS_LINES) status_lines_.pop_front();
    cv_stat_.notify_one();
  }
}

// ================= 阻塞等待函数 =================
bool DummyRobot::wait_next_any_line_(std::string& out, std::chrono::milliseconds timeout) {
  if (timeout.count() < 0) timeout = default_timeout_;
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  std::unique_lock<std::mutex> lk(m_all_);
  while (all_lines_.empty()) {
    if (cv_all_.wait_until(lk, deadline) == std::cv_status::timeout) return false;
  }
  out = std::move(all_lines_.front());
  all_lines_.pop_front();
  return true;
}

bool DummyRobot::wait_next_status_line_(std::string& out, std::chrono::milliseconds timeout) {
  if (timeout.count() < 0) timeout = default_timeout_;
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  std::unique_lock<std::mutex> lk(m_stat_);
  while (status_lines_.empty()) {
    if (cv_stat_.wait_until(lk, deadline) == std::cv_status::timeout) return false;
  }
  out = std::move(status_lines_.front());
  status_lines_.pop_front();
  return true;
}

// ================= 工具函数 =================
static inline std::string trim_copy(const std::string& s) {
  auto first = std::find_if_not(s.begin(), s.end(), [](unsigned char c){ return std::isspace(c); });
  auto last  = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char c){ return std::isspace(c); }).base();
  if (first >= last) return std::string{};
  return std::string(first, last);
}

bool DummyRobot::is_ok_like(const std::string& trimmed) {
  // 小写化
  std::string s; s.reserve(trimmed.size());
  for (unsigned char c : trimmed) s.push_back(static_cast<char>(std::tolower(c)));

  auto right_is_boundary = [](char c){
    return std::isspace((unsigned char)c) || std::ispunct((unsigned char)c);
  };

  // 1) 以 "ok" 开头：ok / OK / ok; / ok 123 ...
  if (s.rfind("ok", 0) == 0) return true;

  // 2) 中间/结尾包含 "ok"，且右边为边界："... ok", "... 15ok"
  for (size_t i = 0; i + 1 < s.size(); ++i) {
    if (s[i]=='o' && s[i+1]=='k') {
      if (i + 2 >= s.size() || right_is_boundary(s[i+2])) return true;
    }
  }
  return false;
}

bool DummyRobot::parse_ok_six(const std::string& line, std::array<double,6>& out) {
  if (line.size() < 2) return false;
  // 必须以 "ok"/"OK" 开头（你的设备状态行是这种格式）
  if (!((line[0]=='o'||line[0]=='O') && (line[1]=='k'||line[1]=='K'))) return false;

  std::istringstream iss(line.substr(2));
  iss >> std::ws;
  for (size_t i = 0; i < 6; ++i) {
    if (!(iss >> out[i])) return false;
  }
  return true;
}

// ================= 控制命令 =================
bool DummyRobot::send_and_expect_ok(const std::string& cmd,
                                    std::chrono::milliseconds timeout,
                                    bool accept_no_response) {
  this->send(std::string(cmd).append(EOL));

  const auto deadline = std::chrono::steady_clock::now() +
    ((timeout.count() < 0) ? default_timeout_ : timeout);

  bool saw_any_line = false;
  std::string line;

  while (std::chrono::steady_clock::now() < deadline) {
    auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    if (!wait_next_any_line_(line, remain)) break;
    saw_any_line = true;

    auto trimmed = trim_copy(line);
    if (is_ok_like(trimmed)) return true;

    // （可选）检测错误关键字快速失败：
    // std::string low = trimmed;
    // std::transform(low.begin(), low.end(), low.begin(), ::tolower);
    // if (low.find("err") != std::string::npos || low.find("fail") != std::string::npos) return false;
  }

  if (accept_no_response && !saw_any_line) return true;
  return false;
}

bool DummyRobot::start_robot (std::chrono::milliseconds t){
  if (t.count() < 0) t = 5000ms;
  return send_and_expect_ok("!START",  t, /*accept_no_response=*/false);
}
bool DummyRobot::stop_robot  (std::chrono::milliseconds t){
  if (t.count() < 0) t = 5000ms;
  return send_and_expect_ok("!STOP",   t, /*accept_no_response=*/false);
}
bool DummyRobot::disable_robot(std::chrono::milliseconds t){
  if (t.count() < 0) t = 5000ms;
  return send_and_expect_ok("!DISABLE",t, /*accept_no_response=*/false);
}
bool DummyRobot::home_robot  (std::chrono::milliseconds t){
  if (t.count() < 0) t = 120000ms;
  return send_and_expect_ok("!HOME",   t, /*accept_no_response=*/false);
}
bool DummyRobot::reset_robot (std::chrono::milliseconds t){
  if (t.count() < 0) t = 120000ms;
  return send_and_expect_ok("!RESET",  t, /*accept_no_response=*/false);
}

// ================= 查询：只看“状态队列”，并冲刷到最新 =================
bool DummyRobot::get_joint_positions(std::array<double,6>& out,
                                     std::chrono::milliseconds timeout) {
  this->send(std::string("#GETJPOS").append(EOL));

  std::string line;
  std::array<double,6> tmp{}, latest{};
  bool seen = false;

  auto deadline = std::chrono::steady_clock::now() + ((timeout.count() < 0) ? default_timeout_ : timeout);
  while (std::chrono::steady_clock::now() < deadline) {
    auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    if (!wait_next_status_line_(line, remain)) break;
    if (parse_ok_six(line, tmp)) { latest = tmp; seen = true; break; }
  }
  if (!seen) return false;

  // 极短“冲刷窗口”：吞掉立即可得的更多状态行，以“最新一条”为准
  constexpr int MAX_DRAIN = 100;
  auto drain_end = std::chrono::steady_clock::now() + 2ms;
  for (int i = 0; i < MAX_DRAIN; ++i) {
    if (std::chrono::steady_clock::now() >= drain_end) break;
    if (!wait_next_status_line_(line, 0ms)) break; // 非阻塞：队列空立即返回
    if (parse_ok_six(line, tmp)) latest = tmp;
  }

  out = latest;
  return true;
}

bool DummyRobot::get_link_pose(std::array<double,6>& out,
                               std::chrono::milliseconds timeout) {
  this->send(std::string("#GETLPOS").append(EOL));

  std::string line;
  std::array<double,6> tmp{}, latest{};
  bool seen = false;

  auto deadline = std::chrono::steady_clock::now() + ((timeout.count() < 0) ? default_timeout_ : timeout);
  while (std::chrono::steady_clock::now() < deadline) {
    auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    if (!wait_next_status_line_(line, remain)) break;
    if (parse_ok_six(line, tmp)) { latest = tmp; seen = true; break; }
  }
  if (!seen) return false;

  constexpr int MAX_DRAIN = 100;
  auto drain_end = std::chrono::steady_clock::now() + 2ms;
  for (int i = 0; i < MAX_DRAIN; ++i) {
    if (std::chrono::steady_clock::now() >= drain_end) break;
    if (!wait_next_status_line_(line, 0ms)) break;
    if (parse_ok_six(line, tmp)) latest = tmp;
  }

  out = latest;
  return true;
}

// ================= 关节移动（Fire-and-forget） =================
bool DummyRobot::move_joints(const std::array<double,6>& q,
                             std::optional<double> speed,
                             std::chrono::milliseconds /*timeout*/) {
  std::ostringstream oss;
  // 如需固定小数位：oss << std::fixed << std::setprecision(2);
  oss << '>' << q[0] << ',' << q[1] << ',' << q[2] << ','
               << q[3] << ',' << q[4] << ',' << q[5];
  if (speed.has_value()) oss << ',' << *speed;
  oss << EOL;

  this->send(oss.str());   // 仅发送，不等待任何回包
  return true;
}

bool DummyRobot::move_joints(double j1, double j2, double j3,
                             double j4, double j5, double j6,
                             std::optional<double> speed,
                             std::chrono::milliseconds timeout) {
  return move_joints(std::array<double,6>{j1,j2,j3,j4,j5,j6}, speed, timeout);
}
