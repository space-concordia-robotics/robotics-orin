#include <cstdint>
#include <iostream>
#include <array>
#include <vector>

#include <thread>
#include <chrono>
#include <cstring>

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

class UbxPacket;

class UbxAbstractConnection
{
protected:
    virtual bool recvFrame(std::vector<uint8_t>&) = 0;
    virtual bool sendFrame(std::vector<uint8_t>&) = 0;

public:
    virtual ~UbxAbstractConnection() = default;

    virtual void logMessage(const std::string& message)
    {
        std::cout << "Log: " << message << std::endl;
    }

    virtual void sendPacket(UbxPacket& packet) = 0;
    template <typename T>
    typename std::enable_if<std::is_base_of<UbxPacket, T>::value, void>::type
    recvPacket(T& packet);
};



