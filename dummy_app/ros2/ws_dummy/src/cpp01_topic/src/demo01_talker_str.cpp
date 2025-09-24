/*
  需求：固定频率发布文本hello dummy 每发布一条后面的文本后缀编号+1
  流程：
  1、包含头文件
  2、初始化ROS2客户端
  3、自定义节点类
    3.1、创建消息发布方
    3.2、创建定时器
    3.3、组织并发布消息
  4、调用spin函数，传入自定义类对象
  5、释放资源


*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <cstddef>
#include <rclcpp/timer.hpp>
#include <string>

using namespace std::chrono_literals;

class Talker: public rclcpp::Node{
public:
  Talker():Node("talker_node_cpp"){
    RCLCPP_INFO(this->get_logger(),"发布方节点");
    // 3.1、创建消息发布方
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    // 3.2、创建定时器
    timer_ = this->create_wall_timer(1s, std::bind(&Talker::on_timer,this));
    count = 0;

  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_; 
  size_t count;
  void on_timer(){
    // 3.3、组织并发布消息
    auto message = std_msgs::msg::String();
    message.data = "hello dummy" + std::to_string(count++);
    RCLCPP_INFO(this->get_logger(),"发布方发布消息：%s",message.data.c_str());
    publisher_->publish(message);
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
