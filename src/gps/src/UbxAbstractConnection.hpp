#include <cstdint>
#include <iostream>
#include <vector>

class UbxAbstractConnection
{
public:
    virtual ~UbxAbstractConnection()
    {
    };
    virtual std::vector<uint8_t> receiveMessage() =0;

    virtual void logMessage(const std::string& message)
    {
        std::cout << "Log: " << message << std::endl;
    }
    virtual void connect(const std::string_view fileName) = 0;
};

class UbxConnection
{

};
