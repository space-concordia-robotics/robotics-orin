#include <vector>
#include <cstdint>

class UbxPacket
{

private:
    const uint16_t preamble;
    const uint8_t category;
    const uint8_t id;
    const std::vector<uint8_t> payload;
    const uint16_t chksum;
    const std::vector<uint8_t> serializedFrame;

public:
    virtual ~UbxPacket() {}
    UbxPacket(std::vector<uint8_t> frame)
    {
    }

    UbxPacket(uint8_t category, uint8_t id, std::vector<uint8_t> payload) : preamble{0x6285}, category{category}, id{id}, payload{payload}, chksum{}, serializedFrame{}
    {
        // parent::function()
    }

    std::vector<uint8_t> serialize()
    {
        return this->serializedFrame;
    }
};
