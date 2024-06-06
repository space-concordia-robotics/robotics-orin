#include <iostream>
#include <cstdint>
#include <array>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "I2CTransactionBuilder.hpp"

int I2CTransactionHelper::openDevice(const std::string& path)
{
    // "/dev/i2c-x"
    int fd = open(path.c_str(), O_RDWR);
    if (fd < 0)
    {
        // TODO: throw exception?
        std::cerr << "Failed to open i2C device " << path << ". Error: " << strerror(errno) << std::endl;
        errno = 0;
        return -1;
    }
    return fd;
}

void I2CTransactionHelper::closeDevice(int fd)
{
    int status = close(fd);
    if (fd < 0)
    {
        // TODO: throw exception?
        std::cerr << "Failed to close i2C device. Error: " << strerror(errno) << std::endl;
        errno = 0;
    }
}

I2CTransactionHelper::I2CTransactionHelper(uint8_t devAddr) :
    I2CTransactionHelper(devAddr, 1)
{
}

I2CTransactionHelper::I2CTransactionHelper(uint8_t devAddr, int reserveSize) :
    devAddr{devAddr}
{
    this->segments.reserve(reserveSize);
}

// Write with raw pointer
I2CTransactionHelper& I2CTransactionHelper::writeRawPtr(uint8_t* buffer, int length)
{
    struct i2c_msg segment = this->buildSegment(
        buffer,
        length
    );
    segment.flags = 0;
    this->segments.push_back(segment);
    return *this;
}

// Read into raw pointer
I2CTransactionHelper& I2CTransactionHelper::readRawPtr(uint8_t* buffer, int length)
{
    struct i2c_msg segment = this->buildSegment(
        buffer,
        length
    );
    segment.flags = 0;
    this->segments.push_back(segment);
    segment.flags = I2C_M_RD;
    return *this;
}

// Write with vector container
I2CTransactionHelper& I2CTransactionHelper::writeVector(std::vector<uint8_t>& buffer)
{
    return this->write(buffer.data(), buffer.size());
}

// Read into vector container
I2CTransactionHelper& I2CTransactionHelper::readVector(std::vector<uint8_t>& buffer)
{
    return this->read(buffer.data(), buffer.size());
}

// Write with array container / literals
template <std::size_t N>
I2CTransactionHelper& I2CTransactionHelper::writeArray(std::array<uint8_t, N>& buffer)
{
    return this->write(buffer.data(), N);
}

I2CTransactionHelper& I2CTransactionHelper::writeArray(std::initializer_list<uint8_t> ilist)
{
    std::vector<uint8_t> buffer(ilist);
    return this->write(buffer.data(), buffer.size());
}

// Read into array container
template <std::size_t N>
I2CTransactionHelper& I2CTransactionHelper::readArray(std::array<uint8_t, N>& buffer)
{
    return this->read(buffer.data(), N);
}

// Write with arbitrary value / reference (will this work?)
template <typename T>
I2CTransactionHelper& I2CTransactionHelper::writeAny(T&& data)
{
    return this->write(reinterpret_cast<uint8_t*>(&data), sizeof(data));
}

// Read into arbitrary reference
template <typename T>
I2CTransactionHelper& I2CTransactionHelper::readAny(T& data)
{
    return this->read(reinterpret_cast<uint8_t*>(&data), sizeof(data));
}

int I2CTransactionHelper::doTransaction(int fd)
{
    if (this->segments.size() == 0)
    {
        // Nothing to perform. Return as successful.
        return 0;
    }

    struct i2c_rdwr_ioctl_data transaction;
    transaction.msgs = segments.data();
    transaction.nmsgs = segments.size();

    // Perform compound I2C transaction
    int nbytes = ioctl(fd, I2C_RDWR, &transaction);

    if (nbytes < 0)
    {
        // TODO: print out more information on the transaction?
        // TODO: make this an exception?
        std::cerr << "[I2C Error] Transaction error " << strerror(errno) << std::endl;
        errno = 0;
        return errno;
    }
    return 0;
}
