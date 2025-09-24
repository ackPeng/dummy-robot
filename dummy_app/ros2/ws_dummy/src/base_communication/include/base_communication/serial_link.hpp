#pragma once
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <array>
#include <chrono>
#include <deque>
#include <functional>
#include <string>
#include <thread>
#include <vector>

class SerialLink {
public:
  using FrameHandler = std::function<void(const std::string&)>;

  SerialLink(std::string dev, unsigned baud, FrameHandler on_frame);
  ~SerialLink();

  // 启动 I/O 线程并打开串口
  void start();

  // 停止 I/O、关闭串口（阻塞等待 I/O 线程退出）
  void stop();

  // 线程安全：投递要发送的命令（建议包含行结束符）
  void send(std::string msg);

  // 可选：启动固定频率回调（如 1 kHz 周期任务）
  void start_rate_loop(std::chrono::microseconds period, std::function<void()> cb);

private:
  void open_and_start();
  void start_rx();
  void kick_tx();
  void on_tick(const boost::system::error_code& ec);
  void handle_error(const boost::system::error_code& ec);
  void retry_later();

  std::string dev_;
  unsigned baud_{};
  FrameHandler on_frame_;

  boost::asio::io_context io_;
  // work guard 防止 io_context.run() 提前返回
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_;
  boost::asio::serial_port sp_;
  boost::asio::steady_timer timer_;
  boost::asio::steady_timer reconnect_;
  std::jthread th_;

  std::array<uint8_t, 4096> rx_buf_{};
  std::vector<uint8_t> rx_acc_;     // 累积缓冲（生产可替换为环形缓冲）

  std::deque<std::string> tx_q_;
  bool tx_in_flight_ = false;

  std::function<void()> loop_cb_;
  std::chrono::microseconds period_{1000};
};
