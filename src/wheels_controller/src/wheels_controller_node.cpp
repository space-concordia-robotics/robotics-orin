#include "wheels_controller_node.h"

WheelsControllerNode::WheelsControllerNode(): Node("wheels_controller") {
    this->declare_parameter("can_path", "can0");
    this->declare_parameter("multiplier", 2000);
    

    if(CANController::configureCAN("can0") != SUCCESS){
        RCLCPP_ERROR(this->get_logger(),"Error accessing CAN interface \n");
        rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(),"Initialized node : %s\n",this->get_name());

    /*
     * This command will inform a motor controller to start transmitting periodic status frames.
     */
    // RevMotorController::requestStatusFrame();
    
    // update_group = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
    
    timer = this->create_wall_timer( 50ms, std::bind(&WheelsControllerNode::pollControllersCallback, this));
    
    joy_msg_callback = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&WheelsControllerNode::JoyMessageCallback, this, std::placeholders::_1)
            );

    twist_msg_callback = this->create_subscription<geometry_msgs::msg::Twist>(
            "twist_wheels", 10, std::bind(&WheelsControllerNode::TwistMessageCallback, this, std::placeholders::_1)
            );
    twist_msg_publisher = this->create_publisher<geometry_msgs::msg::Twist>("twist_wheels", 10);        
}
void WheelsControllerNode::JoyMessageCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg){  
    // Only move if holding down R1 only (that is, L1 has to be unpressed and R1 pressed)

    // if(joy_msg->buttons[4] != 0 || joy_msg->buttons[5] != 1) 
    //     return;

    
     // Only move if holding down R1 only (that is, L1 has to be unpressed and R1 pressed) (FUCKY VERSION)
    // if(joy_msg->buttons[9] != 0 || joy_msg->buttons[10] != 1){
    //     return;
    // }

     if( ! ( joy_msg->buttons[9] == 1 && joy_msg->buttons[10] == 0 ) ){
        return;
    }



    float linear_y_axes_val = joy_msg->axes[1];
    float angular_z_axes_val = joy_msg->axes[2];

    geometry_msgs::msg::Twist twist_msg = geometry_msgs::msg::Twist{};
    twist_msg.linear.x = linear_y_axes_val;
    twist_msg.angular.z = angular_z_axes_val;

    twist_msg_publisher->publish(twist_msg);
}   

void WheelsControllerNode::pollControllersCallback(){
    

    float slip_track = 1.2f;

    // This can be derived from the equation in the paper (to be fucking linked).
    float right_wheels_velocity = this->linear_y - ( this->angular_z * slip_track * 0.5f);
    float left_wheels_velocity = this->linear_y + ( this->angular_z * slip_track * 0.5f);

    float right_wheels_vel_rpm = right_wheels_velocity * this->get_parameter("multiplier").as_int();
    float left_wheels_vel_rpm = left_wheels_velocity * this->get_parameter("multiplier").as_int();
    
    /*
        Motors 3 and 6 are currently brushed, hence they will not run at the same speed as the brushless motors.
    */
    RevMotorController::velocityControl(1,right_wheels_vel_rpm);    
    RevMotorController::velocityControl(2,right_wheels_vel_rpm);
    RevMotorController::velocityControl(3,right_wheels_vel_rpm*0.5);
    
    RevMotorController::velocityControl(4,left_wheels_vel_rpm);
    RevMotorController::velocityControl(5,left_wheels_vel_rpm);
    RevMotorController::velocityControl(6,left_wheels_vel_rpm*0.5);
    
    uint64_t mask = 0x7E;
    RevMotorController::startMotor(mask);

    
}

void WheelsControllerNode::AccelerateTwist(geometry_msgs::msg::Twist twist_msg){
    static float last_speed_change_ms;
    static float last_linear_speed = 0.0f;
    static float last_angular_speed = 0.0f;
    static float expire_rate = 3000.f;

    bool is_expired = false;
    
    float twist_linear = twist_msg.linear.x;
    float twist_angular = twist_msg.angular.z;

    std::chrono::milliseconds time_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    
    if(last_speed_change_ms != 0.f){
        if( (time_ms.count() - last_speed_change_ms) > expire_rate){
            std::cout << "Command expired\n";
            is_expired = true;
        }
        else {
            is_expired = false;
        }

    }

    if (last_speed_change_ms < 1e-7 || is_expired){
        last_linear_speed = 0;
        last_angular_speed = 0;
        last_speed_change_ms = time_ms.count()- expire_rate / initial_ramp_factor;
    }
    float delta =  time_ms.count() - last_speed_change_ms;
    
    AccelerateValue(last_linear_speed, twist_linear, linear_acceleration_rate, delta);
    AccelerateValue(last_angular_speed, twist_angular, angular_acceleration_rate, delta);
    
    last_linear_speed = linear_y;
    last_angular_speed = angular_z;

    last_speed_change_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count();

    // return linear, angular;
    
}

float WheelsControllerNode::AccelerateValue(float current, float desired, float rate, float dt) {
    /*
    Accelerates the current speed to a desired speed at a certain rate while
    considering a certain time difference. Ex : Current Speed 0.3 m/s, desired speed 0.5 m/s,
    if the rate is 0.1 m/s^2 and dt is 100 milliseconds then the new speed should be 0.31 m/s.
    */
    if( (desired-current) < 1e-6 )
        return desired;

    if(desired < 1e-6)
        return 0;

    if(desired < current)
        rate = -rate;

    float new_value = current + rate * dt /1000;

    if(abs(new_value) > abs(desired)){
        new_value = desired;
    }
    std::cout << new_value << "\n";
    return new_value;
}

void WheelsControllerNode::TwistMessageCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->linear_y = msg->linear.x;
    this->angular_z = msg->angular.z;    

    AccelerateTwist(*msg);

    std::cout  << "Linear  : " << linear_y << "\n";
    std::cout << "Angular : " << angular_z  << "\n";
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::executors::MultiThreadedExecutor exec;
    
    auto controller_node = std::make_shared<WheelsControllerNode>();
    exec.add_node(controller_node);
    exec.spin();

    // rclcpp::spin(std::make_shared<WheelsControllerNode>());
    rclcpp::shutdown();
    return 0;
}
