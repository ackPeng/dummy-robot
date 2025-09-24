#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>


//原始的方式
/*
int main(int argc, char **argv)
{
  // init ros2
  rclcpp::init(argc,argv);
  // create ros2 node
  auto node = rclcpp::Node::make_shared("dummy_app_node");
  // print log
  RCLCPP_INFO(node->get_logger(),"hello dummy");
  // release sourece
  rclcpp::shutdown();
  return 0;
}

*/

//更推荐的方式 自定义一个类继承 rclcpp::Node类，这样可以在一个进程创建多个节点，节点之间的通信效率相较于每个进程一个节点更高
class Mynode:public rclcpp::Node{
public:
  Mynode():Node("hello_node_cpp"){
    RCLCPP_INFO(this->get_logger(),"hello dummy");
  }

};


int main(int argc,char **argv){
  rclcpp::init(argc,argv);
  
  auto node = std::make_shared<Mynode>();

  rclcpp::shutdown();

}
