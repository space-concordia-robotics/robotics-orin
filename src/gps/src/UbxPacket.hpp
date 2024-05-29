#pragma once
#include <vector>
#include <cstdint>

class UbxPacket
{
protected:
    uint16_t preamble;
    uint8_t category;
    uint8_t id;
    std::vector<uint8_t> payload;
    uint16_t chksum;
    std::vector<uint8_t> serializedFrame;
    uint16_t calculateCheckSum(const std::vector<uint8_t>& payload);

public:
    virtual ~UbxPacket();

    // Deserializing constructor
    UbxPacket(std::vector<uint8_t> frame);

    // Serializing constructor
    UbxPacket(uint8_t category, uint8_t id, const std::vector<uint8_t> payload);

    // getter
    virtual std::vector<uint8_t> serialize() const;

    uint8_t category1() const
    {
        return category;
    }

    uint8_t id1() const
    {
        return id;
    }

    std::vector<uint8_t> payload1() const
    {
        return payload;
    }

    uint16_t chksum1() const
    {
        return chksum;
    }

    std::vector<uint8_t> serialized_frame() const
    {
        return serializedFrame;
    }
};
