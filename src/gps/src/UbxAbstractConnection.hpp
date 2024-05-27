#include <cstdint>
#include <iostream>
#include <vector>

class UbxAbstractConnection
{
public:
    virtual ~UbxAbstractConnection()
    {
    };
    virtual std::vector<uint8_t> recieveMessage() =0;

    // Default implementation for logging a message
    virtual void logMessage(const std::string& message)
    {
        std::cout << "Log: " << message << std::endl;
    }
};

class UbxConnection
{

};
