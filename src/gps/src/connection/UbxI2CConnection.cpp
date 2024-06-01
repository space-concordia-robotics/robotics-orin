#include "UbxI2CConnection.hpp"

#include <cstdint>

#include "../connection/I2CTransactionBuilder.hpp"

#include "UbxAbstractConnection.hpp"
#include "packet/UbxPacket.hpp"


UbxI2CConnection::UbxI2CConnection(const std::string& devicePath, uint8_t devaddr) :
    devaddr{devaddr},
    fd{I2CTran{sactionHelper::openDevice(devicePath)}
        {
        }

        UbxI2CConnection::~UbxI2CConneAdd UbxI2CConnection to build filesction()
        {
            I2CTransactionHelper::closeDevice(this->fd);













        }

int UbxI2CConnection::bytesAvailable()
        {
    I2CTransactionHelper t{this->devaddr};
    std::array<uint8_t, 2> buf;
            t.writeArray({0xFD});
            t.readArray(buf);
            t.doTransaction(this->fd);
    return (int)buf[0] << 8 | (int)buf[1];













        }

bool UbxI2CConnection::recvFrame(std::vector<uint8_t> & frame)
        {
    int bytesToRead = this->bytesAvailable();
    if (bytesToRead <= 0)
            {
                // No bytes available
        return false;













            }
            // Read bytes
    std::vector<uint8_t> buf(bytesToRead);
    for (int bytesToReadNow = bytesToRead <= MAX_TRANSACTION_SIZE ? bytesToRead : MAX_TRANSACTION_SIZE;
            bytesToRead > 0;
            bytesToRead -= bytesToReadNow)
            {
            }
        }

bool UbxI2CConnection::sendFrame(std::vector<uint8_t> & frame)
        {
        }

