#include "UbxPacket.hpp"

UbxPacket::UbxPacket(uint8_t category, uint8_t id, const std::vector<uint8_t> payload): category{category}, id{id},
    payload{std::move(payload)},
    chksum{calculateCheckSum(payload)}
{
}

UbxPacket::~UbxPacket()
{
}


uint16_t UbxPacket::calculateCheckSum(const std::vector<uint8_t>& payload)
{
    // TODO: Fill in fucntion
}

std::vector<uint8_t> UbxPacket::serialize() const
{
    return this->serializedFrame;
}
