#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <stdio.h>       
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>
#include <chrono>
#include <thread>

class Configuration : public rclcpp::Node {
  public:
    Configuration()
    : Node("sil_node") {
      
      this->declare_parameter("sil_path", "/dev/ttyUSB1");

      publisher_ = this->create_publisher<std_msgs::msg::String>("SIL_Color", 10);

      subscription_ = this->create_subscription<std_msgs::msg::String>(
        "SIL_Color", 10, std::bind(&Configuration::topic_callback, this, std::placeholders::_1));

      //Opens LED strip communication
      fd = open(this->get_parameter("sil_path").as_string().c_str(), O_RDWR);

      if(fd == -1){
          RCLCPP_ERROR(this->get_logger(),"Error opening file (%i)\n",errno);
          rclcpp::shutdown();
      }
      //Set baud rate to 115200
      struct termios options;
      tcgetattr(fd, &options);
      cfsetispeed(&options, B115200);
      cfsetospeed(&options, B115200);
      tcsetattr(fd, TCSANOW, &options);
    }

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    int fd;

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
      //Gives time for LED strip color to be visible
      int nbytes = write(fd, msg->data.c_str(), sizeof(msg->data.c_str()));

      if(nbytes == -1){
          RCLCPP_ERROR(this->get_logger(),"writing to file %i\n",errno);
          exit(1);
      }
    }
};

//needs to add subscriber coming from the wheels_controller_node.cpp
//from class Configuration here in LifecycleSIL
class LifecycleSIL : public rclcpp_lifecycle::LifecycleNode{
  public: 
    explicit LifecycleSIL(const std::string& node_name, bool intra_process_comms = false)
    : rclcpp_lifecycle::LifecycleNode(node_name, 
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {}

    void topic_callback(){
      static size_t count = 0;
      auto msg = std::make_unique<std_msgs::msg::String>();
      msg->data = "Lifecycle HelloWorld #" + std::to_string(++count);

      // Print the current state for demo purposes
      if (!pub_->is_activated()) {
        RCLCPP_INFO(
          get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
      } else {
        RCLCPP_INFO(
          get_logger(), "Lifecycle publisher is active. Publishing: [%s]", msg->data.c_str());
      }
      pub_->publish(std::move(msg));
    }


    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &){
      pub_ = this->create_publisher<std_msgs::msg::String>("SIL_Color", 10);
      timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&LifecycleSIL::topic_callback, this)
      );

      RCLCPP_INFO(get_logger(), "on_configure() is called.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state){
      LifecycleNode::on_activate(state);

      RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
      std::this_thread::sleep_for(std::chrono::seconds(2));

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state){
      LifecycleNode::on_deactivate(state);

      RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &){
      timer_.reset();
      pub_.reset();

      RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & state){
      timer_.reset();
      pub_.reset();

      RCUTILS_LOG_INFO_NAMED(
        get_name(),
        "on shutdown is called from state %s.",
        state.label().c_str()
      );

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    
  private:
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<LifecycleSIL> sil_node = std::make_shared<LifecycleSIL>("sil_pub");

  exe.add_node(sil_node->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}
