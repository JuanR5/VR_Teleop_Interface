// C library headers
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// Linux headers
#include <fcntl.h>      // Contains file controls like O_RDWR
#include <errno.h>      // Error integer and strerror() function
#include <termios.h>    // Contains POSIX terminal control definitions
#include <unistd.h>     // write(), read(), close()


struct __attribute__((__packed__)) DataStatus
{
    uint16_t app_took_too_long:1;
    uint16_t overrange:1;
    uint16_t invalid_measurements:1;
    uint16_t raw_measurements:1;
    uint16_t:12; //reserved
};

struct __attribute__((__packed__)) AppOutput
{
    struct DataStatus status;
    float forces[6];
    uint32_t timestamp;
    float temperature;
};

struct __attribute__((__packed__)) RxFrame
{
    uint8_t header;
    struct AppOutput data;
    uint16_t crc16_ccitt;
} frame;

const uint8_t frame_header = 0xAA;


// https://www.nongnu.org/avr-libc/user-manual/group__util__crc.html#ga1c1d3ad875310cbc58000e24d981ad20

#define lo8(x) ((x)&0xFF)
#define hi8(x) (((x)>>8)&0xFF)

uint16_t crc_ccitt_update (uint16_t crc, uint8_t data)
{
    data ^= lo8 (crc);
    data ^= data << 4;
    return ((((uint16_t)data << 8) | hi8 (crc)) ^ (uint8_t)(data >> 4)
            ^ ((uint16_t)data << 3));
}

static inline uint16_t calcCrc16X25(uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    while(len--)
        crc = crc_ccitt_update(crc, *data++);
    return ~crc;
}



int main()
{
    /* Open the serial port. Change device path as needed.
    * Currently set to an standard FTDI USB-UART converter
    */
    int serial_port = open("/dev/ttyUSB0", O_RDWR);

    if (serial_port < 0) {
      printf("Error while opening device.\n");
      printf("errno = %d\n", errno);
      return 1;
    }

    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Disable parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag |= CS8; // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 460800
    cfsetispeed(&tty, B460800);
    cfsetospeed(&tty, B460800);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    while (1) {
        printf("Trying to connect.\n");
        /* read the next available byte and check if is the header
        * make sure to unget it after. emulating peek. Using static
        * variable for the sync flag will preserve status of sync
        * between calls */
        bool frameSync_ = false;
        while (!frameSync_) {
            uint8_t possible_header;
            /* read bytes 1 by 1 to find the header */
            read(serial_port, (char*)&possible_header, sizeof(possible_header));
            //printf("Sync read byte: %#06x\n",possible_header);
            if (possible_header == frame_header) {
                /* read the remaining frame to make check also CRC */
                read(serial_port, (char*)&frame.data, sizeof(frame.data));
                read(serial_port, (char*)&frame.crc16_ccitt, sizeof(frame.crc16_ccitt));
                if (frame.crc16_ccitt == calcCrc16X25((uint8_t*)&frame.data, sizeof(frame.data))) {
                    printf("Frame synced with 0x%X header\n", frame_header);
                    frameSync_ = true;
                }
                else {
                    /* if there is a frame that included the header 0xAA in
                    a fixed position. Could be the above checking mechanism
                    will get stuck because will find the wrong value as header
                    then will remove from the buffer n bytes where n the size
                    of the frame and then will find again exactly the same
                    situation the wrong header. So we read on extra byte to make
                    sure next time will start from the position that is size of frame
                    plus 1. It works */
                    char dummy;
                    read(serial_port, (char*)&dummy, sizeof(dummy));
                    printf("Skipping incomplete frame\n");
                }
            }
        }
        while (frameSync_) {
            /* Read the sensor measurements frame assuming that is alligned with the RX buffer */
            read(serial_port, (char*)&frame, sizeof(frame));
            /* Check if the frame is still alligned, otherwise exit */
            if (frame.header != frame_header) {
                frameSync_ = false;
                break;
            }
            // Read and check CRC 16-bit
            uint16_t crc_received = frame.crc16_ccitt;
            uint16_t crc_computed = calcCrc16X25((uint8_t*)&frame.data, sizeof(frame.data));
            if (crc_received != crc_computed) {
                printf("CRC missmatch received: 0x%04x calculated: 0x%04x\n", crc_received, crc_computed);
                break; //skip this measurements
            }
            // print measurements
            if (frame.data.status.app_took_too_long) {
                printf("Warning force sensor is skipping measurements, Increase sinc length\n");
            }
            if (frame.data.status.overrange) {
                printf("Warning measuring range exceeded\n");
            }
            if (frame.data.status.invalid_measurements) {
                printf("Warning measurements are invalid, Permanent damage may occur\n");
            }
            if (frame.data.status.raw_measurements) {
                printf("Warning raw force torque measurements from gages\n");
            }
            printf("Fx: %f\n",frame.data.forces[0]);
            printf("Fy: %f\n",frame.data.forces[1]);
            printf("Fz: %f\n",frame.data.forces[2]);
            printf("Tx: %f\n",frame.data.forces[3]);
            printf("Ty: %f\n",frame.data.forces[4]);
            printf("Tz: %f\n",frame.data.forces[5]);
            printf("temperature: %f\n", frame.data.temperature);
            printf("timestamp: %d\n", frame.data.timestamp);
        } // while synced
    } // while app run
    close(serial_port);
}
