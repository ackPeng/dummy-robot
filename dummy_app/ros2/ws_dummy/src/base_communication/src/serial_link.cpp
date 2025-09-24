#include "base_communication/serial_link.hpp"
#include <algorithm>
#include <cstdio>

SerialLink::SerialLink(std::string dev, unsigned baud, FrameHandler on_frame)
  : dev_(std::move(dev))
  , baud_(baud)
  , on_frame_(std::move(on_frame))
  , work_(boost::asio::make_work_guard(io_))
  , sp_(io_)
  , timer_(io_)
  , reconnect_(io_) {}

SerialLink::~SerialLink() {
  stop();
}

void SerialLink::start() {
  // 启动 I/O 线程
  th_ = std::jthread([this]{ io_.run(); });
  // 在 I/O 线程中打开串口并开始工作
  boost::asio::post(io_, [this]{ open_and_start(); });
}

void SerialLink::stop() {
  // 异步关闭资源，然后等待线程退出
  boost::asio::post(io_, [this]{
    boost::system::error_code ec;
    timer_.cancel();
    reconnect_.cancel();
    sp_.close(ec);
  });
  if (th_.joinable()) th_.join(); // jthread 析构也会 join，这里显式更明确
}

void SerialLink::send(std::string msg) {
  boost::asio::post(io_, [this, m = std::move(msg)]{
    tx_q_.push_back(m);
    if (!tx_in_flight_) kick_tx();
  });
}

void SerialLink::start_rate_loop(std::chrono::microseconds period, std::function<void()> cb) {
  boost::asio::post(io_, [this, period, cb]{
    period_ = period;
    loop_cb_ = cb;
    timer_.expires_after(period_);
    timer_.async_wait([this](const boost::system::error_code& ec){ on_tick(ec); });
  });
}

void SerialLink::open_and_start() {
  boost::system::error_code ec;
  sp_.open(dev_, ec);
  if (ec) {
    retry_later();
    return;
  }
  sp_.set_option(boost::asio::serial_port_base::baud_rate(baud_));
  sp_.set_option(boost::asio::serial_port_base::character_size(8));
  sp_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  sp_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  sp_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

  start_rx();
}

void SerialLink::start_rx() {
  sp_.async_read_some(boost::asio::buffer(rx_buf_),
    [this](const boost::system::error_code& ec, std::size_t n){
      if (ec) { handle_error(ec); return; }

      rx_acc_.insert(rx_acc_.end(), rx_buf_.begin(), rx_buf_.begin() + n);

      // 简单的行分隔解析（\n），需要更健壮可改成 COBS/CRC
      for (;;) {
        auto it = std::find(rx_acc_.begin(), rx_acc_.end(), '\n');
        if (it == rx_acc_.end()) break;

        std::string line(rx_acc_.begin(), it);
        if (!line.empty() && line.back() == '\r') line.pop_back();

        if (on_frame_) on_frame_(line);

        rx_acc_.erase(rx_acc_.begin(), it + 1);
      }

      start_rx();
    });
}

void SerialLink::kick_tx() {
  if (tx_q_.empty()) { tx_in_flight_ = false; return; }
  tx_in_flight_ = true;

  auto &front = tx_q_.front();
  boost::asio::async_write(sp_, boost::asio::buffer(front),
    [this](const boost::system::error_code& ec, std::size_t /*bytes*/){
      if (ec) { handle_error(ec); return; }
      tx_q_.pop_front();
      kick_tx();
    });
}

void SerialLink::on_tick(const boost::system::error_code& ec) {
  if (ec) return;
  if (loop_cb_) loop_cb_();

  // 累加到期时间：避免逐次 expires_after 带来的漂移
  timer_.expires_at(timer_.expiry() + period_);
  timer_.async_wait([this](const boost::system::error_code& e){ on_tick(e); });
}

void SerialLink::handle_error(const boost::system::error_code& /*ec*/) {
  boost::system::error_code ignore;
  timer_.cancel();
  reconnect_.cancel();
  sp_.close(ignore);
  retry_later();
}

void SerialLink::retry_later() {
  reconnect_.expires_after(std::chrono::milliseconds(500));
  reconnect_.async_wait([this](const boost::system::error_code& /*ec*/){
    open_and_start();
  });
}
