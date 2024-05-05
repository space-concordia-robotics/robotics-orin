#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "absenc_interface/msg/encoder_values.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "std_msgs/msg/string.hpp"

#define NO_ERROR            0
#define ERR_SERIAL_FAILURE  1
#define ERR_SLAVE_INVALID   2
#define ERR_NO_RESPONSE     3
#define ERR_FRAME_CORRUPTED 4
typedef struct {
    int error; 
    int cause; 
    int line; 
} ABSENC_Error_t;

// Gets string corresponding to error code
const char* strAbsencErr(int err);

typedef struct {
    uint8_t slvnum; 
    uint16_t status; 
    double angval; 
    double angspd; 
} ABSENC_Meas_t; 

#define no_error (ABSENC_Error_t{0, 0, __LINE__})

extern void ABSENC_ReportError(ABSENC_Error_t err); 
extern void ABSENC_PrintMeas(const ABSENC_Meas_t * meas); 

class AbsencDriver {
public:    
    static ABSENC_Error_t OpenPort(const char* path, uint16_t baud_rate, int& s_fd);
    static ABSENC_Error_t PollSlave(int slvnum, ABSENC_Meas_t * meas, int s_fd);
    static ABSENC_Error_t ClosePort(int s_fd);
};

class Absenc: public rclcpp::Node{
public:
    Absenc();
    ~Absenc();
private:
    void absEncPollingCallback();
    void cadValuesCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void ikValuesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void controlEndEffector();

    int s_fd = -1;

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