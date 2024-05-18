#include "joy_mapping_node.h"
#include <byteswap.h>

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_msg_publisher;


JoyMappingNode::JoyMappingNode(): Node("joy_mapping_controller") {
    arm_vals_msg_publisher =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "arm_values", rclcpp::QoS(10));

    joy_values_msg_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "joy", 10, std::bind(&JoyMappingNode::JoyMessageCallback, this, std::placeholders::_1)
        );

    
}



void JoyMappingNode::JoyMessageCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
    // Must hold L1 and R1 to get arm control
    if(joy_msg->buttons[4] == 0 || joy_msg->buttons[5] == 0){
        return;
    }


    // Or when L1 and R1 get fuckiing reammped?
    // if(joy_msg->buttons[9] == 0 || joy_msg->buttons[10] == 0){
    //     return;
    // }
    //  if( ! ( joy_msg->buttons[9] == 0 && joy_msg->buttons[10] == 1 ) ){
    //     return;
    // }


    // LEFT HORIZ : axes[0]
    // LEFT VERT : axes[1]
    // if(joy_msg->buttons[4] == 1){
    //     joy_msg->axes[]
    // }
    // RIGHT HORIZ : axes[3]
    // RIGHT VERT : axes[4]
    
    // L2 : axes [2]
    // R2 : axes [5]


    // joy_msg->axes[2] = (joy_msg->axes[2] - 1.f)/2.f;
    
    // joy_msg->axes[5] = (joy_msg->axes[5] - 1.f)/2.f;

    /*
        Hold R2 or L2 to control the turning the base rotation
        
    */
    float rotation = (joy_msg->axes[2] - joy_msg->axes[5]) / 2;

    // float rotation;

    // if(joy_msg->axes[4] < 1){
    //    rotation = -(joy_msg->axes[4] - 1.f)/2.f;
    // }
    // else if(joy_msg->axes[5] < 1){
    //    rotation = (joy_msg->axes[5] - 1.f)/2.f;
    // }
    
    
    //Map so L2 and R2 control base, left x, left y, right x then control the rest of the 12v motors
    float speeds[6] = {joy_msg->axes[2] - joy_msg->axes[5],joy_msg->axes[0],joy_msg->axes[1],joy_msg->axes[3],0,0};
    // float speeds[6] = {rotation,joy_msg->axes[0],-joy_msg->axes[1],-joy_msg->axes[3],0,0};
    

    
    // circle-square controls the first smart servo
    //speeds[4] = joy_msg->buttons[1] - joy_msg->buttons[3];
    // triangle-cross controls the second smart servo
    //speeds[5] = joy_msg->buttons[2] - joy_msg->buttons[0];


    // circle-square controls the first smart servo (Other fukcing mapping?)
    speeds[4] = joy_msg->buttons[1] - joy_msg->buttons[2];
    // triangle-cross controls the second smart servo
    speeds[5] = joy_msg->buttons[3] - joy_msg->buttons[0];

    // publish the array
    // change speeds to a float32multiarray
    std_msgs::msg::Float32MultiArray speeds2;
    speeds2 = {speeds[0], speeds[1], speeds[2], speeds[3], speeds[4], speeds[5]};
    arm_vals_msg_publisher->publish(speeds2);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyMappingNode>());
    rclcpp::shutdown();
    return 0;
}
