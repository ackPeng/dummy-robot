/*
    订阅发布方发布的消息并且在终端输出
    流程：
        1、包含头文件
        2、初始化ros2客户端
        3、自定义节点类
            3.1、创建订阅方
            3.2、解析并输出数据
        4、调用Spin函数并传入自定义节点类指针
        5、释放资源
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <functional>
#include <rclcpp/subscription.hpp>


class Listener: public rclcpp::Node{
public:
    Listener():Node("listner_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"订阅方创建");
            // 3.1、创建订阅方
            subscription_ = this->create_subscription<std_msgs::msg::String>("chatter",10, std::bind(&Listener::do_cb,this,std::placeholders::_1));
            
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    void do_cb(const std_msgs::msg::String &msg){
        // 3.2、解析并输出数据
        RCLCPP_INFO(this->get_logger(),"订阅到的消息是：%s",msg.data.c_str());
    };

};


int main(int argc,char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
}