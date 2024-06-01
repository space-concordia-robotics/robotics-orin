#include "absenc.h"
#include <iostream>

const char* strAbsencErr(int err)
{
    switch (err)
    {
    case NO_ERROR:
        return "No error occurred";
    case ERR_SERIAL_FAILURE:
        return "Serial failure";
    case ERR_SLAVE_INVALID:
        return "Slave invalid";
    case ERR_NO_RESPONSE:
        return "No response";
    case ERR_FRAME_CORRUPTED:
        return "Frame corrupted";
    default:
        return "Unknown code";
    }
}

ABSENC_Error_t AbsencDriver::OpenPort(const char* file_name, uint16_t baud_rate, int& socket_fd)
{
    errno = 0;

    socket_fd = open(file_name, O_RDWR);
    if (socket_fd < 0)
    {
        int errno0 = errno;
        errno = 0;
        return ABSENC_Error_t{
            ERR_SERIAL_FAILURE,
            errno0,
            __LINE__,
        };
    }
    struct termios ttycfg;
    ttycfg.c_cflag = CS8 | CREAD | CLOCAL; // 8N1, ignore modem signals
    ttycfg.c_lflag = 0;
    ttycfg.c_iflag = 0;
    ttycfg.c_oflag = 0;
    ttycfg.c_line = 0;
    ttycfg.c_cc[VTIME] = 1; // 100ms timeout
    ttycfg.c_cc[VMIN] = 0; // Return anything read so far

    cfsetispeed(&ttycfg,B57600);
    cfsetospeed(&ttycfg,B57600);

    if (tcsetattr(socket_fd, TCSANOW, &ttycfg) > 0)
    {
        int errno0 = errno;
        errno = 0;
        return ABSENC_Error_t{
            ERR_SERIAL_FAILURE,
            errno0,
            __LINE__,
        };
    }
    return no_error;
}

ABSENC_Error_t AbsencDriver::PollSlave(int slave_number, ABSENC_Meas_t* measurement_data, int socket_fd)
{
    if (slave_number < 0 || slave_number > 9)
    {
        return ABSENC_Error_t{
            ERR_SLAVE_INVALID,
            0,
            __LINE__,
        };
    }
    tcflush(socket_fd, TCIOFLUSH);

    char txbuf[2];
    txbuf[0] = '#';
    txbuf[1] = '0' + slave_number;
    int nsend = write(socket_fd, txbuf, sizeof(txbuf));
    if (nsend < 0)
    {
        int errno0 = errno;
        errno = 0;
        return ABSENC_Error_t{
            ERR_SERIAL_FAILURE,
            errno0,
            __LINE__,
        };
    }
    // tcdrain(socket_fd); // Flush TX buffer? seems not needed

    char sof;
    for (int i = 0; i < 50; i++)
    {
        // Ensure SOF search always ends
        int nrecv = read(socket_fd, &sof, 1);
        if (nrecv < 0)
        {
            int errno0 = errno;
            errno = 0;
            return ABSENC_Error_t{
                ERR_SERIAL_FAILURE,
                errno0,
                __LINE__,
            };
        }
        if (nrecv == 0)
        {
            return ABSENC_Error_t{
                ERR_NO_RESPONSE,
                0,
                __LINE__,
            };
        }
        if (sof == '#') break;
        // Not SOF, maybe noise on the bus, search for another one
    }
    if (sof != '#')
    {
        return ABSENC_Error_t{
            ERR_FRAME_CORRUPTED,
            0,
            __LINE__,
        };
    }

    char rxbuf[21]; // "X -> AAAA, BBBB, CCCC", ignore the \r\n
    int nrecv = read(socket_fd, rxbuf, sizeof(rxbuf));
    if (nrecv < 0)
    {
        int errno0 = errno;
        errno = 0;
        return ABSENC_Error_t{
            ERR_SERIAL_FAILURE,
            errno0,
            __LINE__,
        };
    }

    if (nrecv < (int)sizeof(rxbuf))
    {
        return ABSENC_Error_t{
            ERR_FRAME_CORRUPTED,
            0,
            __LINE__,
        };
    }

    /*
    TODO : Explain how this works for the people with a skill issue
   */
    uint16_t rawdata[3];
    int index = 5;
    for (int i = 0; i < 3; i++)
    {
        uint16_t val = 0;
        for (int j = 0; j < 4; j++)
        {
            uint8_t nib = rxbuf[index++];
            if (nib >= '0' && nib <= '9') nib = nib - '0';
            else if (nib >= 'A' && nib <= 'F') nib = nib - 'A' + 10;
            else if (nib >= 'a' && nib <= 'f') nib = nib - 'a' + 10;
            else
                return ABSENC_Error_t{
                    ERR_FRAME_CORRUPTED,
                    0,
                    __LINE__,
                };
            val = (val << 4) | nib;
        }
        index += 2;
        rawdata[i] = val;
    }

    measurement_data->slvnum = slave_number;
    measurement_data->status = rawdata[0];
    measurement_data->angval = ((double)(int16_t)rawdata[1]) / 65536.0 * 360.0;
    // TODO: fix calibration code on MCU side
    measurement_data->angspd = (double)(int16_t)rawdata[2]; // TODO: this conversion formula

    return no_error;
}

ABSENC_Error_t AbsencDriver::ClosePort(int socket_fd)
{
    close(socket_fd);
    return no_error;
}
