#pragma once

#include <cstdint>
#include <string>
#include <vector>

class UbxDeserializerException : public std::exception
{
private:
    std::vector<uint8_t> payload; // these are all read only, can we make optimizations?
    uint32_t readIndex;
    uint32_t readLength;
    std::string message;

public:
    explicit UbxDeserializerException(const std::vector<uint8_t>& payload, uint32_t readIndex, uint32_t readLength,
                                      const std::string& message)
    {
        // Initialize this in a better way?
        this->payload = payload;
        this->readIndex = readIndex;
        this->readLength = readLength;
        this->message = message;
    }

    virtual const char* what() const noexcept override
    {
        // How can I build a message like this:
        // "Error reading index <readIndex> length <readLength>: <message> (<payload bytes in hex>)"
        // Example: "Error reading 4 length 1: Payload drained. (64 00 1A 93)"
    }
};

class UbxPayloadDrainedException : public UbxDeserializerException
{
public:
    // Only change is the constructor now contains the error message
    explicit UbxPayloadDrainedException(const std::vector<uint8_t>& payload, uint32_t readIndex, uint32_t readLength)
        : UbxDeserializerException::UbxDeserializerException(payload, readIndex, readLength, "Payload drained.")
    {
    }
};

// TODO: fix
//
// class UbxPayloadExcessException : public UbxDeserializerException
// {
// public:
//     // Only change is the constructor now contains the error message
//     explicit UbxPayloadExcessException(const std::vector<uint8_t>& payload, uint32_t currentIndex)
//         : UbxDeserializerException(payload, readIndex, payload, "Payload has more bytes.")
//     {
//     }
// };


class UbxDeserializer
{
private:
    std::vector<uint8_t> payload;
    uint32_t index;
    uint32_t bitfieldBuffer;

    uint8_t readByteNochk()
    {
        return this->payload.at(this->index++);
    }

public:
    UbxDeserializer(std::vector<uint8_t> payload)
    {
        this->payload = payload;
        this->index = 0;
    }

    void reset()
    {
        this->index = 0;
    }

    void assertBytesAvailable(uint32_t length)
    {
        uint32_t requiredLength = this->index + length;
        if (this->payload.size() < requiredLength)
        {
            throw UbxPayloadDrainedException(this->payload, this->index, length);
        }
    }

    void assertEmpty()
    {
        if (this->index < this->payload.size())
        {
            //TODO: uncomment after class error is fixed
            // throw UbxPayloadExcessException(this->payload);
        }
    }

    uint8_t readU1()
    {
        this->assertBytesAvailable(1);
        return this->readByteNochk();
    }

    int8_t readS1()
    {
        this->assertBytesAvailable(1);
        return static_cast<int8_t>(this->readByteNochk());
    }

    uint16_t readU2()
    {
        this->assertBytesAvailable(2);
        uint16_t data;
        data = static_cast<uint16_t>(this->readByteNochk());
        data |= (static_cast<uint16_t>(this->readByteNochk())) << 8;
        return data;
    }

    int16_t readI2()
    {
        return static_cast<int16_t>(this->readU2());
    }

    uint32_t readU4()
    {
        this->assertBytesAvailable(4);
        uint32_t data;
        data = static_cast<uint32_t>(this->readByteNochk());
        data |= (static_cast<uint32_t>(this->readByteNochk())) << 8;
        data |= (static_cast<uint32_t>(this->readByteNochk())) << 16;
        data |= (static_cast<uint32_t>(this->readByteNochk())) << 24;
        return data;
    }

    int32_t readI4()
    {
        return static_cast<int32_t>(this->readU4());
    }

    void readByteArray(uint8_t* data, uint32_t length)
    {
        this->assertBytesAvailable(length);
        for (int i = 0; i < length; i++)
        {
            data[i] = this->readByteNochk();
        }
    }

    std::vector<uint8_t> readVector(uint32_t length)
    {
        this->assertBytesAvailable(length);
        std::vector<uint8_t> data;
        data.resize(length);
        this->readByteArray(data.data(), data.size());
        return data;
    }

    float readR4()
    {
        float data;
        this->readByteArray(reinterpret_cast<uint8_t*>(&data), 4);
        return data;
    }

    double readR8()
    {
        double data;
        this->readByteArray(reinterpret_cast<uint8_t*>(&data), 8);
        return data;
    }

    char readCh()
    {
        this->assertBytesAvailable(1);
        return static_cast<char>(this->readByteNochk());
    }

    void readX1()
    {
        this->bitfieldBuffer = static_cast<uint32_t>(this->readU1());
    }

    void readX2()
    {
        this->bitfieldBuffer = static_cast<uint32_t>(this->readU2());
    }

    void readX4()
    {
        this->bitfieldBuffer = this->readU4();
    }

    bool readXF(int bit)
    {
        if (bit < 0 || bit > 31)
        {
            return false;
        }
        return !!(this->bitfieldBuffer & (1 << bit));
    }

    uint32_t readXU(int msb, int lsb)
    {
        if (msb < 0 || lsb < 0 || msb > 31 || lsb > 31 || msb < lsb)
        {
            return 0;
        }
        uint32_t temp = this->bitfieldBuffer >> lsb;
        if (msb < 31)
        {
            uint32_t mask = (1 << msb - lsb + 1) - 1;
            return temp & mask;
        }
        else
        {
            return temp;
        }
    }

    int32_t readXI(int msb, int lsb)
    {
        if (msb < 0 || lsb < 0 || msb > 31 || lsb > 31 || msb < lsb)
        {
            return 0;
        }
        int32_t temp = static_cast<int32_t>(this->bitfieldBuffer) >> lsb;
        if (msb < 31)
        {
            uint32_t mask = (1 << msb - lsb + 1) - 1;
            return temp & mask;
        }
        else
        {
            return temp;
        }
    }

    int32_t readXS(int msb, int lsb)
    {
        if (msb < 0 || lsb < 0 || msb > 31 || lsb > 31 || msb <= lsb) // Note the <=, we need at least 2 bits
        {
            return 0;
        }
        uint32_t magnitude = this->readXU(msb - 1, lsb);
        return this->readXF(msb) ? -magnitude : magnitude;
    }
};

