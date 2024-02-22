//
// Created by nik on 08/02/24.
//
#include "command_prefixes.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <unistd.h>
#include <cstdint>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <termios.h>
#include <net/if.h>
#include <iostream>
#define STATUS_BUFFER_SIZE 1000

enum status{
    SUCCESS = 0,
    CAN_ERROR
};

class CANController {
    
    public:

    inline static int s_Socket = 0;
    
    inline static int s_BufferLength = 0;
    
    inline static char* s_StatusBuffer = nullptr;

   static uint8_t configure(const char* fd_path){

    s_StatusBuffer = new char[1000];
    struct sockaddr_can addr{};
    s_Socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(s_Socket == -1){
        sprintf(s_StatusBuffer,"Error with socket, errno : %i\n",errno);
        return CAN_ERROR;
    }
    struct ifreq ifr{};
    strcpy(ifr.ifr_name,fd_path);
    if(ioctl(s_Socket, SIOCGIFINDEX, &ifr) == -1){
        s_BufferLength = sprintf(s_StatusBuffer,"Error with ioctl, errno : %i\n",errno);
        return CAN_ERROR;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if(bind(s_Socket, (struct sockaddr *)&addr, sizeof(addr)) == -1){
        sprintf(s_StatusBuffer,"Error with bind, errno : %i\n",errno);
        return CAN_ERROR;
    }
    
    return SUCCESS;
}

static uint8_t sendFrame(struct can_frame& frame){
    if (write(s_Socket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        sprintf(s_StatusBuffer,"Error with send frame, did not write all data \n");
        return CAN_ERROR;
    }
    sprintf(s_StatusBuffer,"Write complete\n");
    return SUCCESS;
}

static uint8_t readFrame(struct can_frame& frame){
    auto nbytes = read(s_Socket, &frame, sizeof(struct can_frame));

    if (nbytes == -1) {
        sprintf(s_StatusBuffer,"read : error %i\n",errno);
        return CAN_ERROR;
    }
    /*
     * ssize_t cast to avoid compiler warning.
     */
    if (nbytes < (ssize_t)sizeof(struct can_frame)) {
        sprintf(s_StatusBuffer,"read: incomplete CAN frame\n");
        return CAN_ERROR;
    }
    return SUCCESS;
    
}
static void closeDevice(){
    close(s_Socket);
}

};


int main(){
    auto status = CANController::configure("can0");   
    if(status != SUCCESS){
        for(int i = 0 ; i < CANController::s_BufferLength ; i++){
            std::cout << CANController::s_StatusBuffer[i];
        }
    } 
    struct can_frame frame{};
    frame.can_id = 0x000502C0;
    frame.can_dlc = 1;
    frame.can_id |= CAN_EFF_FLAG;
    frame.data[0] = 1;

    CANController::sendFrame(frame);

    while(true){
    if(CANController::readFrame(frame) != SUCCESS){
        std::cerr << "Fuck you : " << errno;
        exit(-1);
    }
    std::cout << std::hex << frame.can_id << std::endl;
    // for(int i = 0 ; i < frame.can_dlc ; i++){
    //     std::cout << std::hex << (int)frame.data[i] << ",";
    // }
    }
    // // For a start move command, the id field needs to be added with 0x80. This command is only issued once per move.
    // frame.can_id = (COMMAND_PREFIX_MOVE_MOTORS << 8) | (0x03 + 0x80);
    // frame.can_dlc = 8;
    // frame.can_id |= CAN_EFF_FLAG;
    // float percent = 0.1;
    // memcpy(frame.data,&percent, sizeof(float));
    // if(CANController::sendFrame(frame) != SUCCESS){
    // std::cerr << errno;
    //         exit(-1);
    // }

        // while(true){
        //     if(CANController::readFrame(frame) != SUCCESS){
        //         std::cerr << "Fuck you : " << errno;
        //         exit(-1);
        //     }
        //     std::cout << std::hex << frame.can_id;
        //     for(int i = 0 ; i < frame.can_dlc ; i++){
        //         std::cout << std::hex << (int)frame.data[i] << ",";
        //     }
            std::cout << "\n";
        
        // while(true){
     

        //     uint64_t buf_data = (1ULL << 0x03);
        //     frame.can_id = COMMAND_PREFIX_MAINTAIN_SPEED;
        //     frame.can_id |= CAN_EFF_FLAG;
        //     frame.can_dlc = 8;
        //     memcpy(frame.data,&buf_data,sizeof(buf_data));
        //     if(CANController::sendFrame(frame) != SUCCESS){
        //             std::cerr << errno;
        //             exit(-1);
        //     }
        //     usleep(200000);
        // }
    
 
    CANController::closeDevice();
    std::cout << "fuck you";

}