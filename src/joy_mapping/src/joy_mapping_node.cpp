#include "joy_mapping_node.h"
#include <byteswap.h>

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


    // std::cout << "Motor speeds " << speeds[0] << " "  << speeds[1] << " "  << speeds[2] << " "  << " "  << speeds[3] << " "  << speeds[4] << " "  << speeds[5] << std::endl;
    // // RIGHT BUMPER
    // if(joy_msg->buttons[7] == 1){
    //     speeds[2] = speeds[5] * -1.f;
    // }
    
    
    uint8_t out_buf[1 + 1 + sizeof(float)*6 + 1] ={};
    out_buf[0] = SET_MOTOR_SPEED;
    out_buf[1] = sizeof(float)*6;

    for(int i = 0 ; i < 6 ; i++){
        float speed = speeds[i] * 250.f;
        memcpy(&out_buf[ (i*sizeof(float)) +2],&speed,sizeof(float));
        
    }
    out_buf[26] = 0x0A;

    /* THIS FUCKING LINE CAUSES THE LINUX KERNEL TO CRASH WHEN USED (DMA ERROR???!!?!)
    // tcflush(fd, TCIOFLUSH); 
    */
    if (isLocal) {
        RCLCPP_INFO(this->get_logger(),"Speeds from joystick : %f %f %f %f %f %f\n", speeds[0], speeds[1], speeds[2], speeds[3], speeds[4], speeds[5]);
    } else {
        int status = write(fd,out_buf,sizeof(out_buf));
        if(status == -1){
            RCLCPP_ERROR(this->get_logger(),"Error : %d\n", errno);
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyMappingNode>());
    rclcpp::shutdown();
    return 0;
}
