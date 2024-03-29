
#ifndef WHEELS_CONTROLLER_NODE_H
#define WHEELS_CONTROLLER_NODE_H
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
// #include "arm_controller/msg/arm_motor_values.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <JetsonGPIO.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <unistd.h>
#include <cstdint>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <iostream>
#include <fcntl.h>
#define SET_MOTOR_SPEED 0x4E

class ArmControllerNode : public rclcpp::Node{
public:
    ArmControllerNode();
    void JoyMessageCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    void ArmMessageCallback(const std_msgs::msg::Float32MultiArray::SharedPtr arm_val_msgs);

private :
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_msg_callback;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr arm_vals_msg_callback;
    
    int fd;
};

#endif