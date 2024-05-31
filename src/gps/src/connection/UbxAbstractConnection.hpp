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
    virtual ~UbxAbstractConnection()
    {
    }

    virtual void logMessage(const std::string& message)
    {
        std::cout << "Log: " << message << std::endl;
    }

    virtual void sendPacket(UbxPacket& packet);
    virtual void recvPacket(); // here's the template thing we talked about
    // recvPacket<T extends UbxPacket>(T & packet);
};


class I2CTransactionBuilder
{
private:
    uint8_t devAddr; // can make this const
    std::vector<struct i2c_msg> segments; // can make this const

    struct i2c_msg buildSegment(uint8_t* buffer, int length)
    {
        struct i2c_msg segment;
        segment.addr = this->devAddr;
        segment.buf = buffer;
        segment.len = length;
        return segment;
    }

public:
    I2CTransactionBuilder(uint8_t devAddr)
    {
        I2CTransactionBuilder(devAddr, 1);
    }

    I2CTransactionBuilder(uint8_t devAddr, int reserveSize)
    {
        this->devAddr = devAddr;
        this->segments.reserve(reserveSize);
    }

    // Write with raw pointer
    I2CTransactionBuilder& write(uint8_t* buffer, int length)
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
    I2CTransactionBuilder& read(uint8_t* buffer, int length)
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
    I2CTransactionBuilder& write(std::vector<uint8_t>& buffer)
    {
        return this->write(buffer.data(), buffer.size());
    }

    // Read into vector container
    I2CTransactionBuilder& read(std::vector<uint8_t>& buffer)
    {
        return this->read(buffer.data(), buffer.size());
    }

    // Write with array container / literals
    template <std::size_t N>
    I2CTransactionBuilder& write(std::array<uint8_t, N>& buffer)
    {
        return this->write(buffer.data(), N);
    }

    // Read into array container
    template <std::size_t N>
    I2CTransactionBuilder& read(std::array<uint8_t, N>& buffer)
    {
        return this->read(buffer.data(), N);
    }

    // Write with arbitrary value / reference (will this work?)
    template <typename T>
    I2CTransactionBuilder& write(T&& data)
    {
        return this->write(reinterpret_cast<uint8_t*>(&data), sizeof(data));
    }

    // Read into arbitrary reference
    template <typename T>
    I2CTransactionBuilder& read(T& data)
    {
        return this->read(reinterpret_cast<uint8_t*>(&data), sizeof(data));
    }

    int doTransaction(int fd)
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
};


class UbxI2CConnection : public UbxAbstractConnection
{
private:
    int fd;
    uint8_t devAddr;
    int pollingWaitTime;
    int pollingMaxPolls;
    int maxTransactionLength;

    int bytesAvailable()
    {
        I2CTransactionBuilder b(this->devAddr, 2);
        uint16_t nBytes = 0;
        b.write(0xFD).read(nBytes).doTransaction(this->fd);
        return nBytes;
    }

    void platformSleep(int ms)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

protected:
    bool sendFrame(std::vector<uint8_t>& frame)
    {
        I2CTransactionBuilder b(this->devAddr, 1);
        int result = b.write(frame).doTransaction(this->fd);
        return result == 0;
    }

    bool recvFrame(std::vector<uint8_t>& frame)
    {
        int bytesToRead = 0;
        for (int i = 0; i < this->pollingMaxPolls; i++)
        {
            bytesToRead = this->bytesAvailable();
            if (bytesToRead > 0)
            {
                break;
            }
            this->platformSleep(this->pollingWaitTime);
        }
        if (bytesToRead == 0)
        {
            return false;
        }
        frame.resize(bytesToRead);
        uint8_t* ptr = frame.data();
        for (int index = 0; bytesToRead > 0;)
        {
            int bytesToReadNow = (bytesToRead > this->maxTransactionLength) ? this->maxTransactionLength : bytesToRead;
            I2CTransactionBuilder b(this->devAddr);
            int status = b.read(ptr, bytesToReadNow).doTransaction(this->fd);
            if (status != 0)
            {
                return false;
            }
            ptr += bytesToReadNow;
            bytesToRead -= bytesToReadNow;
        }
        return true;
    }
};




