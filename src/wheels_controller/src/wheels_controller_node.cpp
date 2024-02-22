#include "wheels_controller_node.h"

WheelsControllerNode::WheelsControllerNode(): Node("wheels_controller") {
    this->declare_parameter("can_path", "can0");
    // if(CANController::configureCAN("can0") != SUCCESS){
    //     RCLCPP_ERROR(this->get_logger(),"Error accessing CAN interface \n");
    //     rclcpp::shutdown();
    // }
    //CANController::printStatus();
    /*
     * Connect all the rev motor controllers to the system.
     */
    // RevMotorController::registerDevice(3);

//    motorControllers.at(DEVICE_5_ID)->sendConfiguration(BRUSHED)
//    motorControllers.at(DEVICE_6_ID)->sendConfiguration(BRUSHED)

    /*
     * This command will inform a motor controller to start transmitting periodic status frames.
     */
    // RevMotorController::requestStatusFrame();
    
    // update_group = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
    // timer = this->create_wall_timer( 20ms, std::bind(&WheelsControllerNode::pollControllersCallback, this));
    
    joy_msg_callback = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&WheelsControllerNode::JoyMessageCallback, this, std::placeholders::_1)
            );

    twist_msg_callback = this->create_subscription<geometry_msgs::msg::Twist>(
            "twist_wheels", 10, std::bind(&WheelsControllerNode::TwistMessageCallback, this, std::placeholders::_1)
            );
    twist_msg_publisher = this->create_publisher<geometry_msgs::msg::Twist>("twist_wheels", 10);        
}
void WheelsControllerNode::JoyMessageCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg){  
    std::cout << joy_msg->axes[0] << std::endl;
    float linear_y_axes_val = joy_msg->axes[1];
    float angular_z_axes_val = joy_msg->axes[3];
    
    geometry_msgs::msg::Twist twist_msg = geometry_msgs::msg::Twist{};
    twist_msg.linear.x = linear_y_axes_val;
    twist_msg.angular.z = angular_z_axes_val;
    
    twist_msg_publisher->publish(twist_msg);
}

void WheelsControllerNode::pollControllersCallback(){
    // std::cout << "Timer: " << std::this_thread::get_id() << std::endl;
    struct can_frame frame;
    // RCLCPP_INFO(this->get_logger(),"Error accessing CAN interface \n");
    RevMotorController::velocityControl(3,1000);
    // RevMotorController::voltagePercentControl(3,0.1);
//    struct{uint8_t motor_id, frameType frame_type} frame
    // uint8_t status = CANController::readFrame(frame);

    // if(frame.can_id == 0x82052c80) return;
    // if(frame.can_id != 0x82051803 && frame.can_id != 0x82051843 && frame.can_id != 0x82051883 
    // && frame.can_id != 0x820518c3 && frame.can_id != 0x82051983 && frame.can_id != 0x82051943 && frame.can_id != 0x820519c3) {
    // std::cout << std::hex << frame.can_id << "\n";
    //       for(int i = 0 ; i < frame.can_dlc ; i++){
    //             std::cout << std::hex << (int)frame.data[i] << ",";
    //         }
    // std::cout << "\n";
    
    // std::cout << "fuck you " << "\n";
    
//    if(frameInfo.type == PERIODIC_FRAME_0){
//        /*
//        * The motor controller status is contained here
//        */
//    }
//    else if(frameInfo.type == PERIODIC_FRAME_1){
//
//        float motor_rpm;
//        uint8_t motor_temp;
//        float voltage_low;
//
//        // 32 bit motor velocity
//        memcpy(&motor_rpm, &msg.data[0], 4);
//
//        // 8 bit temperature
//        memcpy(&motor_temp, &msg.data[4], 1);
//
//        // 12 bit voltage
//        memcpy(&voltage_low, &msg.data[5], 1);
//
//        float voltage = (float) (msg.data[6] & 0x0F) + (float) msg.data[5] / 255;
//        std::cout << ("\nVoltage : ");
//        std::cout << (voltage);
//        float current = ((float) (msg.data[6] >> 4)) / 15 + (float) msg.data[7];
//   }
}
void WheelsControllerNode::TwistMessageCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    float linear_y = msg->linear.x;
    float angular_z = msg->angular.z;

    float slip_track = 1.2f;

    // This can be derived from the equation in the paper.
    float right_wheels_velocity = linear_y - ( angular_z * slip_track * 0.5f);
    float left_wheels_velocity = linear_y + ( angular_z * slip_track * 0.5f);

    // std::cout << "Right wheels velocity : " << right_wheels_velocity << "\n";
    // std::cout << "Left wheels velocity : " << left_wheels_velocity  << "\n";
    

    float right_wheels_vel_rpm = right_wheels_velocity * 68.2;
    float left_wheels_vel_rpm = left_wheels_velocity * 68.2;

    std::cout << "Right wheels velocity PPM : " << right_wheels_vel_rpm << "\n";
    std::cout << "Left wheels velocity PM : " << left_wheels_vel_rpm  << "\n";
    
    /*
     * Compute the voltage that each wheel neeeds
     */
//    for(auto controller : motor_controllers){
//        controller.second->voltagePercentControl(0.1);
//    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::executors::MultiThreadedExecutor exec;
    
    // auto controller_node = std::make_shared<WheelsControllerNode>();
    // exec.add_node(controller_node);
    // exec.spin();

    rclcpp::spin(std::make_shared<WheelsControllerNode>());
    rclcpp::shutdown();
    return 0;
}