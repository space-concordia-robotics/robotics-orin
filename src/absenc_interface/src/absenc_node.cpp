#include "absenc.h"
#include "math.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <chrono>
#include <array>
#include <cmath>
#include <string>
/*Include autogenerated message header*/
#include "absenc_interface/msg/encoder_values.hpp"
// #include "arm_controller/msg/arm_motor_values.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
    using namespace std::chrono_literals;
class AbsEnc : public rclcpp::Node
{
  public:
    AbsEnc()
    : Node("absenc_node")
    {
          /*
        Set default parameter for the filepath of the absenc. To set, use 
        ros2 run absenc_interface absenc_node --ros-args -p absenc_polling_rate:=1000
        or during runtime (when the node is already running) as 
        ros2 param set /absenc_node absenc_polling_rate 1000 
      */
      this->declare_parameter("absenc_path", "/dev/ttyUSB0");
      this->declare_parameter("absenc_polling_rate", 100);

      angles_publisher = this->create_publisher<absenc_interface::msg::EncoderValues>("absenc_values", 10);

      arm_publisher = this->create_publisher<std_msgs::msg::String>("arm_command", 10);
      
      timer = this->create_wall_timer(
      std::chrono::milliseconds(this->get_parameter("absenc_polling_rate").as_int()), 
      std::bind(&AbsEnc::absEncPollingCallback, this));

      /*
        Open serial to RS485 device
      */
      ABSENC_Error_t err = ABSENC::OpenPort(this->get_parameter("absenc_path").as_string().c_str(),B57600);
      if(err.error != 0){
        RCLCPP_ERROR(this->get_logger(),"Error opening file (%i)\n",err.error);
      }

      subscription = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&AbsEnc::ikValuesCallback, this, std::placeholders::_1));

      subscription_cad_mouse = this->create_subscription<sensor_msgs::msg::Joy>(
      "cad_mouse_joy", 10, std::bind(&AbsEnc::cadValuesCallback, this, std::placeholders::_1));
      
      arm_controller_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("arm_values",10);
    }

  private:
    
   void absEncPollingCallback()
  {
    auto message = absenc_interface::msg::EncoderValues();     

    ABSENC_Meas_t absenc_meas_1,absenc_meas_2,absenc_meas_3;

    ABSENC::PollSlave(1,&absenc_meas_1);
    ABSENC::PollSlave(2,&absenc_meas_2);
    ABSENC::PollSlave(3,&absenc_meas_3);
    
    message.angle_1 = absenc_meas_1.angval < 0 ? absenc_meas_1.angval + 180.f : absenc_meas_1.angval - 180;
    message.angle_2 = absenc_meas_2.angval;
    message.angle_3 = absenc_meas_3.angval < 0 ? 180 + absenc_meas_3.angval : absenc_meas_3.angval - 180.f;

    // std::cout << message.angle_1 << std::endl;
    // std::cout << message.angle_2 << std::endl;
    // std::cout << message.angle_3 << std::endl;
    
    // hold onto value for control system
    abs_angles[1] = message.angle_1;
    abs_angles[2] = message.angle_2;
    abs_angles[3] = message.angle_3;
  
    //TODO : Can simplify this a bit. 
    angles_publisher->publish(message);
  }

    void cadValuesCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
      if (msg->axes.size() < 6) {
        RCLCPP_ERROR(this->get_logger(),"Axes of cad mouse wrong dimension");
      }
      if (msg->buttons.size() < 2) {
        RCLCPP_ERROR(this->get_logger(),"Axes of cad mouse wrong dimension");
      }

      x = msg->axes[0];
      y = msg->axes[1];
      z = msg->axes[2];
      pitch = msg->axes[3];
      roll = msg->axes[4];
      yaw = msg->axes[5];
      leftButton = msg->buttons[0];
      rightButton = msg->buttons[1];

      if (rightButton) {
        controlEndEffector();
      }
    }

    void ikValuesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
      // Convert ik angles to degrees
      for (int i = 0; i < 4; i++) {
        ik_angles[i] = msg->position[i] * 180.0 / M_PI;
      }
      if (rightButton) {
        // Then should be controlling the end effector directly; abort here
        return;
      }

      std::string arm_command = "set_motor_speeds ";

      auto arm_msg_2 = std_msgs::msg::Float32MultiArray();


      // auto layout = std_msgs::msg::MultiArrayLayout();
      // auto dim = std::msgs::msg::MultiArrayDimension();

      std::vector<float> angles(6);


      std::cout << "Angles: ";
      for (int i = 0; i < std::size(ik_angles); i++) {
        std::cout << "ik " << ik_angles[i] << " abs " << abs_angles[i] << " ";
      }
      std::cout << "\n";


      // Set the base motor speed from the cad mouse directly
      float yaw_scaled = yaw * 0.75;
      float yaw_clamped = yaw_scaled > 255.0 ? 255.0 : yaw_scaled;
      yaw_clamped = yaw_scaled < -255.0 ? -255.0 : yaw_scaled;
      angles[0] = yaw_clamped;
      arm_command += std::to_string(angles[0]);
      arm_command += " ";

      // Only the three middle motors have values provided by IK
      for (int i = 1; i < std::size(ik_angles); i++) {
        float absenc_angle = abs_angles[i];
        float ik_angle = ik_angles[i];
        float motor_sign = motor_signs[i];

        float absolute_difference = std::abs(absenc_angle - ik_angle);
        int difference_sign = absenc_angle - ik_angle >= 0 ? 1 : -1;

        if (absolute_difference <= 1) {
          // NICK SHIT
          angles[i] = 0;
          arm_command += "0 ";
        } else {
          // Get motor value from difference from desired position, with gain 2000
          float value = ((absolute_difference) / 180.0) * 2000.0;
          // Clamp
          value = value > 255.0 ? 255.0 : value;
          // Account for sign differences
          value *= difference_sign;
          value *= motor_sign;
          // Add to msg
          int intVal = (int)value;
          
          // NICK SHIT
          angles[i] = value;

          arm_command += std::to_string(intVal);
          arm_command += " ";
        }
      }
        
      // Add 0 values for two last values
      arm_command += "0 0";
        // NICK SHIT
      angles[4] = 0.f;
      angles[5] = 0.f;
      
      auto arm_msg = std_msgs::msg::String();
      
      arm_msg.data = arm_command;

      arm_publisher->publish(arm_msg);
      
      arm_msg_2.data = angles;

      arm_controller_publisher->publish(arm_msg_2);
    }

    void controlEndEffector() {

    }

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_cad_mouse;
    rclcpp::Publisher<absenc_interface::msg::EncoderValues>::SharedPtr angles_publisher;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_controller_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_publisher;

    // Both these store 4 angles to be ready if a 4th encoder is added
    std::array<float, 4> ik_angles = {0.0, 0.0, 0.0, 0.0};
    std::array<float, 4> abs_angles = {0.0, 0.0, 0.0, 0.0};
    // Controls if motor sign is aligned with encoder direction
    std::array<int, 4> motor_signs = {1, -1, -1, -1};
    // Store the axes of the cad mouse
    float x, y, z, pitch, roll, yaw;
    // Store button states of cad mouse
    int leftButton, rightButton;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AbsEnc>());
  rclcpp::shutdown();
  return 0;
}
