#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/stat.h>

#define DEF_DEVICE      "/dev/ttyUSB0"
#define DEF_BAUDRATE    115200
#define DEF_DATABITS    8
#define DEF_STOPBITS    1
#define DEF_PARITY      0

#define DEF_LOADADDR    0x00000000

#define BUF_SIZE 512
#define WAIT_TIMEOUT 3

#define VERSION "0.1"

/* error message helper */
void sl_error(char *s)
{
    fprintf(stderr, "error: %s\n", s);
    exit(-1);
}

/* waits for special character in serial line */
int sl_wait(int fd, char c, long timeout)
{
    int retval, bytes_read;
    fd_set fds;
    struct timeval tv, *ptv;
    char data;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    if (timeout) {
        tv.tv_sec = timeout;
        tv.tv_usec = 0;
        ptv = &tv;
    } else
        ptv = NULL;

    retval = select(fd + 1, &fds, NULL, NULL, ptv);
    if (retval > 0) {
        bytes_read = read(fd, &data, 1);
        if (bytes_read == 1 && data == c)
            return 1;
        else
            return -1;
    } else
        return retval;
}

/* high level wrapper for sl_wait */
void waitchar(int fd, char c, long timeout)
{
    int r;

    printf("waiting for magic signature (timeout %lu sec)\n", timeout);
    r = sl_wait(fd, c, timeout);
    if (r == 0)
        sl_error("timeout");
    else if (r < 0)
        sl_error("wrong magic");
    printf(">> ok [got \'%c\']\n", c);
}

/* send string over serial interface */
void sl_send_string(int fd, char *s)
{
    int wrote;
    int left = strlen(s);
    do {
        wrote = write(fd, s, left);
        left -= wrote;
        s += wrote;
    } while (left > 0);
}

/* send 32bit word over serial interface */
void sl_send_32(int fd, unsigned long word)
{
    unsigned char c;

    c = (unsigned char)word;
    write(fd, &c, 1);
    c = (unsigned char)(word >>= 8);
    write(fd, &c, 1);
    c = (unsigned char)(word >>= 8);
    write(fd, &c, 1);
    c = (unsigned char)(word >>= 8);
    write(fd, &c, 1);
}

/* stat file for size in bytes */
unsigned long sl_image_size(char *filename)
{
    struct stat fs;

    if (stat(filename, &fs) == 0) {
        return fs.st_size;
    } else {
        perror("error");
        exit(-1);
    }
}

/* send file over serial interface in raw (binary) mode */
int sl_send_bin(int fd, char *filename)
{
    int f;
    struct stat st;
    unsigned long bytes, r, w;
    char buf[BUF_SIZE];

    f = open(filename, O_RDONLY);
    if (f < 0)
        return -1;

    if (fstat(f, &st) < 0)
        return -1;

    bytes = 0;
    do {
        r = read(f, buf, BUF_SIZE);
        if (r > 0) {
            w = write(fd, buf, r);
            bytes += w;
            printf("<< sending image: %lu / %lu bytes sent\r", bytes, st.st_size);
        }
    } while (r);
    printf("\n");
    close(f);
    return 0;
}

/* open and initialize serial interface */
int sl_serial_init(char *dev, unsigned int baudrate, int databits, int stopbits, int parity)
{
    struct termios tio;
    int fd;

    bzero(&tio, sizeof(tio));   
    tio.c_lflag = 0;
    tio.c_cflag |= CLOCAL;  // ignore modem control lines
    tio.c_cflag |= CREAD;   // enable reciever
    switch (baudrate) {
        case 300:
            tio.c_cflag |= B300;
            break;
        case 600:
            tio.c_cflag |= B600;
            break;
        case 1200:
            tio.c_cflag |= B1200;
            break;
        case 2400:
            tio.c_cflag |= B2400;
            break;
        case 4800:
            tio.c_cflag |= B4800;
            break;
        case 9600:
            tio.c_cflag |= B9600;
            break;
        case 19200:
            tio.c_cflag |= B19200;
            break;
        case 38400:
            tio.c_cflag |= B38400;
            break;
        case 57600:
            tio.c_cflag |= B57600;
            break;
        case 115200:
            tio.c_cflag |= B115200;
            break;
        case 230400:
            tio.c_cflag |= B230400;
            break;
        default:
            tio.c_cflag |= B115200;
    }

    switch (databits) {
        case 5:
            tio.c_cflag |= CS5;
            break;
        case 6:
            tio.c_cflag |= CS6;
            break;
        case 7:
            tio.c_cflag |= CS7;
            break;
        case 8:
            tio.c_cflag |= CS8;
            break;
        default:
            tio.c_cflag |= CS8;
    }

    if (stopbits == 2)
        tio.c_cflag |= CSTOPB;

    if (parity) {
        tio.c_cflag |= PARENB;
        if (parity % 2)
            tio.c_cflag |= PARODD;
    }

    // block read until at least 1 byte is appeared
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;

    fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd > 0)
        tcsetattr(fd, TCSANOW, &tio);

    return fd;
}

/* load image to IRAM using LPC3250 serial boot protocol */
void sl_load_image(int fd, char *filename, unsigned long loadaddr)
{
    int r;
    unsigned long size;
    
    size = sl_image_size(filename);
    if (size == 0) {
        perror("error");
        exit(-1);
    }

    printf("\nstarting serial loading procedure (please, reset board)\n");
    waitchar(fd, '5', 0);
    
    printf("<< sending \'A\'\n");
    sl_send_string(fd, "A");
    
    waitchar(fd, '5', WAIT_TIMEOUT);
    
    printf("<< sending \'U3\'\n");
    sl_send_string(fd, "U3");
    
    waitchar(fd, 'R', WAIT_TIMEOUT);
    
    printf("<< sending addr [0x%lx]\n", loadaddr);
    sl_send_32(fd, loadaddr);
    
    printf("<< sending image size [%lu bytes]\n", size);
    sl_send_32(fd, size);
    
    r = sl_send_bin(fd, filename);
    if (r != 0) {
        perror("error");
        exit(-1);
    }
}

/* start serial teminal mode */
void sl_terminal(int fd)
{
    char recv_buf[BUF_SIZE];
    char byte;
    int r;
    fd_set iset;
    char cr = '\r';

    printf("\nswitching to serial terminal mode (press Ctrl+D to exit)\n");
    while (1) {
        FD_ZERO(&iset);
        FD_SET(0, &iset);
        FD_SET(fd, &iset);

        if (select(fd + 1, &iset, NULL, NULL, NULL) < 0) {
            perror("error");
            exit(-1);
        }
        
        if (FD_ISSET(fd, &iset)) {
            r = read(fd, recv_buf, BUF_SIZE);
            if (r > 0)
                write(1, recv_buf, r);
            else if (r == 0) {
                printf("EOF\n");
                break;
            } else {
                perror("error");
                break;
            }
        }

        if (FD_ISSET(0, &iset)) {
            r = read(0, &byte, 1);
            if (r > 0) {
                if (byte == '\n')
                    write(fd, &cr, 1);
                write(fd, &byte, 1);
                read(fd, &byte, 1); // hack to skip written data when reading from device (FIXME how to do it right way?)

            }
            else if (r == 0) {
                printf("\n");
                break;
            } else {
                perror("error");
                break;
            }
        }
    }
    printf("\nserial terminal session closed\n");
}

void sl_print_help(char *execname)
{
    char help[] = "sterm-" VERSION "\n"
                  "\tIRAM image loader for NXP LPC3250 MCU serial boot protocol\n"
                  "\tand a simple serial terminal program.\n"
                  "\t2011 (c) Dmitry Nikolaev <borealis@permlug.org>\n\n"
                  "usage: %s [options] [image.bin]\n\n"
                  "-d device\t- serial device name (default: /dev/ttyUSB0)\n"
                  "-B baudrate\t- baudrate (default: 115200)\n"
                  "-b databits\t- amount of databits (default: 8)\n"
                  "-s stopbits\t- amount of stopbits (default: 1)\n"
                  "-p parity\t- parity, 0 - off, 1 - odd, 2 - even (default: 0)\n"
                  "-a addr\t\t- loading address of the image (default: 0x00000000)\n\n"
                  "If image file is not specified when program start in terminal mode.\n"
                  "Otherwise it tries to load image and then switches to terminal mode.\n";
    fprintf(stderr, help, execname);
    exit(-1);
}

int main(int argc, char **argv)
{    
    int fd;
    int opt;
    char *device, *image;
    int baudrate, databits, stopbits, parity;
    unsigned long loadaddr;

    device = DEF_DEVICE;
    baudrate = DEF_BAUDRATE;
    databits = DEF_DATABITS;
    stopbits = DEF_STOPBITS;
    parity = DEF_PARITY;
    loadaddr = DEF_LOADADDR;
    image = NULL;

    printf("\n");
    while ((opt = getopt(argc, argv, "hd:B:b:s:p:a:")) != -1) {
        switch (opt) {
            case 'h':
                sl_print_help(argv[0]);
            case 'd':
                device = optarg;
                break;
            case 'B':
                baudrate = atoi(optarg);
                break;
            case 'b':
                databits = atoi(optarg);
                break;
            case 's':
                stopbits = atoi(optarg);
                break;
            case 'p':
                parity = atoi(optarg);
                break;
            case 'a':
                loadaddr = strtol(optarg, NULL, 0);
                break;
            default:
                sl_print_help(argv[0]);
        }
    }

    if (optind < argc)
        image = argv[optind];

    printf("device: %s %d,%ds,%d,%d\n", device, baudrate, databits, stopbits, parity);
    if (image)
        printf("loading image: %s at addr: 0x%lx\n", image, loadaddr);

    fd = sl_serial_init(device, baudrate, databits, stopbits, parity);    
    if (fd <= 0) {
        perror("error");
        return -1;
    }

    if (image)
        sl_load_image(fd, image, loadaddr);

    sl_terminal(fd);

    close(fd);
    return 0;
}

