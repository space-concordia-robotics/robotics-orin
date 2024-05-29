#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <exception>
#include <string>
#include <vector>

#define I2C_MAX_TRANSACTION 16

#define GPS_I2C_ADDR 0x42

typedef struct
{
    int fd;
} GPS_Device_t;

// TODO: constructor; leave constructor empty for now, the constructor will open GPS device

static void GPS_UBXI2C_LoadWrSegment(struct i2c_msg &segment, uint8_t *buf, int len)
{
    segment.addr = GPS_I2C_ADDR;
    segment.flags = 0;
    segment.buf = buf;
    segment.len = len;
}

static void GPS_UBXI2C_LoadRdSegment(struct i2c_msg &segment, uint8_t *buf, int len)
{
    segment.addr = GPS_I2C_ADDR;
    segment.flags = I2C_M_RD;
    segment.buf = buf;
    segment.len = len;
}

static int GPS_UBXI2C_DoTransaction(const GPS_Device_t *obj, struct i2c_msg *segments, int transactionCount)
{
    if (transactionCount == 0)
    {
        return 0;
    }

    struct i2c_rdwr_ioctl_data transaction;
    transaction.msgs = segments;
    transaction.nmsgs = transactionCount;

    // Perform compound I2C transaction
    int nbytes = ioctl(obj->fd, I2C_RDWR, &transaction);

    // TODO: proper error checking
    if (nbytes < 0)
    {
        fprintf(stderr, "[ERR ] Line %d: I2C transaction error (%s)\n", __LINE__, strerror(errno));
        errno = 0;
        return 1;
    }
    return 0;
}

static int GPS_UBXI2C_BytesAvailable(const GPS_Device_t *obj, uint16_t *size)
{
    struct i2c_msg transactionSegments[2];
    // Segment 1: set address pointer to 0xFD
    uint8_t writeBuffer[1] = {0xFD};
    GPS_UBXI2C_LoadWrSegment(&transactionSegments[0], writeBuffer, 1);

    // Segment 2: read 2 bytes
    uint8_t readBuffer[2];
    GPS_UBXI2C_LoadRdSegment(&transactionSegments[1], readBuffer, 2);

    // Perform compound transaction
    int status = GPS_UBXI2C_DoTransaction(obj, transactionSegments, 2);
    if (status == 0)
    {
        *size = readBuffer[0] | readBuffer[1] << 8;
    }
    else
    {
        *size = 0;
    }
    return status;
}

// Public method
// TODO: encapsulate buffer + capacity as an object
int GPS_UBXI2C_Read(const GPS_Device_t *obj, uint8_t *buffer, uint32_t capacity, uint32_t *frameSize0)
{
    uint16_t frameSize;
    int result = GPS_UBXI2C_BytesAvailable(obj, &frameSize);
    if (result != 0)
        return result;
    if (frameSize > capacity)
    {
        fprintf(stderr, "[WARN] Line %d: Receiving %d bytes but buffer capacity is only %d\n", __LINE__, frameSize, capacity);
        frameSize = capacity;
    }
    *frameSize0 = frameSize;
    if (frameSize == 0)
    {
        // Nothing to read
        return 0;
    }
    // Initiate reads not exceeding max transfer length
    int bytesToRead = frameSize;
    uint8_t i2cReadBuffer[I2C_MAX_TRANSACTION];
    for (int index = 0;;)
    {
        int bytesToReadNow = (bytesToRead > I2C_MAX_TRANSACTION) ? I2C_MAX_TRANSACTION : bytesToRead;
        struct i2c_msg transactionSegment;
        GPS_UBXI2C_LoadRdSegment(&transactionSegment, i2cReadBuffer, bytesToReadNow);
        int status = GPS_UBXI2C_DoTransaction(obj, &transactionSegment, 1);
        if (status != 0)
        {
            return status;
        }
        memcpy(buffer + index, i2cReadBuffer, bytesToReadNow);
        index += bytesToReadNow;
        bytesToRead -= bytesToReadNow;
    }
    return 0;
}

// Public method
// TODO: encapsulate buffer + length as an object
int GPS_UBXI2C_Write(const GPS_Device_t *obj, uint8_t *buffer, uint32_t length)
{
    if (length == 0)
    {
        fprintf(stderr, "[WARN] Line %d: Attempting to send 0 bytes.\n");
        return 0;
    }
    struct i2c_msg transactionSegment;
    GPS_UBXI2C_LoadWrSegment(&transactionSegment, buffer, length);
    int status = GPS_UBXI2C_DoTransaction(obj, &transactionSegment, 1);
    return status;
}

/* -------- UBX packet deserialization -------- */

class UbxDeserializerException : public std::exception
{
private:
    std::vector<uint8_t> payload; // these are all read only, can we make optimizations?
    uint32_t readIndex;
    uint32_t readLength;
    std::string message;

public:
    explicit UbxDeserializerException(const std::vector<uint8_t> &payload, uint32_t readIndex, uint32_t readLength, const std::string &message)
    {
        // Initialize this in a better way?
        this->payload = payload;
        this->readIndex = readIndex;
        this->readLength = readLength;
        this->message = message;
    }

    virtual const char *what() const noexcept override
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
    explicit UbxPayloadDrainedException(const std::vector<uint8_t> &payload, uint32_t readIndex, uint32_t readLength)
        : UbxDeserializerException::UbxDeserializerException(payload, readIndex, readLength, "Payload drained.")
    {
    }
};

class UbxDeserializer
{
private:
    std::vector<uint8_t> payload;
    uint32_t index;

    void assertBytesAvailable(uint32_t length) {
        uint32_t requiredLength = this->index + length; 
        if(this->payload.size() < requiredLength) {
            throw UbxPayloadDrainedException(this->payload, this->index, length); 
        }
    }

    uint8_t readByte() {
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

    uint8_t readU1() {
        this->assertBytesAvailable(1); 
        return this->readByte(); 
    }

    int8_t readS1() {
        this->assertBytesAvailable(1); 
        return static_cast<int8_t>(this->readByte()); 
    }

    uint16_t readU2() {
        this->assertBytesAvailable(2); 
        uint16_t data; 
        data  =  static_cast<uint16_t>(this->readByte()); 
        data |= (static_cast<uint16_t>(this->readByte())) << 8; 
        return data; 
    }

    int16_t readI2() {
        return static_cast<int16_t>(this->readU2()); 
    }

    uint32_t readU4() {
        this->assertBytesAvailable(4); 
        uint32_t data; 
        data  =  static_cast<uint32_t>(this->readByte()); 
        data |= (static_cast<uint32_t>(this->readByte())) << 8; 
        data |= (static_cast<uint32_t>(this->readByte())) << 16; 
        data |= (static_cast<uint32_t>(this->readByte())) << 24; 
        return data; 
    }

    int32_t readI4() {
        return static_cast<int32_t>(this->readU4()); 
    }

    void readByteArray(uint8_t * data, uint32_t length) {
        this->assertBytesAvailable(length); 
        for(int i = 0; i < length; i++) {
            data[i] = this->readByte(); 
        }
    }

    float readR4() {
        float data; 
        this->readByteArray(reinterpret_cast<uint8_t *>(&data), 4); 
        return data; 
    }

    double readR8() {
        double data; 
        this->readByteArray(reinterpret_cast<uint8_t *>(&data), 8); 
        return data; 
    }
    
    char readCh() {
        this->assertBytesAvailable(1); 
        return static_cast<char>(this->readByte()); 
    }

}; 
