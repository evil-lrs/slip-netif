#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <termios.h>
#include <pthread.h>

#include "slip/slip.h"
#include "baudrates.h"


#define TUN_NAME        "tun0"
#define BUFFER_SIZE     1600



// File descriptors
static int tun_fd;
static int serial_fd;

static uint8_t          slip_buf[BUFFER_SIZE];
static slip_handler_s   slip;
static int              debug = 0;


// Callback function for received SLIP messages
void slip_recv_message(uint8_t *data, uint32_t size) {

    if (debug) {
        printf("[DBG] Received: ");
        for (uint32_t i = 0; i < size; ++i) printf("%02X ", data[i]);
        printf("\n");
    }

    ssize_t nwrite = write(tun_fd, data, size);
    if (nwrite < 0) {
        perror("Writing to TUN interface");
    }

    printf("Wrote %zd bytes to TUN interface\n", nwrite);
}

// Callback function to send a byte over the serial connection
uint8_t slip_write_byte(uint8_t byte) {
    if (write(serial_fd, &byte, 1) == 1) {
        return 1; // Success
    } else {
        perror("Serial write");
        return 0; // Failure
    }
}

int create_tun(char *dev) {
    struct ifreq ifr;
    int fd, err;

    if ((fd = open("/dev/net/tun", O_RDWR)) < 0) {
        perror("Opening /dev/net/tun");
        return -1;
    }

    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TUN | IFF_NO_PI;
    // ifr.ifr_flags &= ~IFF_MULTICAST;

    if (*dev) {
        strncpy(ifr.ifr_name, dev, IFNAMSIZ);
    }

    if ((err = ioctl(fd, TUNSETIFF, (void *)&ifr)) < 0) {
        perror("ioctl(TUNSETIFF)");
        close(fd);
        return -1;
    }

    strcpy(dev, ifr.ifr_name);
    return fd;
}


int open_uart(const char *device, speed_t baud_rate) {
    int fd;
    struct termios tty;

    fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Opening serial port");
        return -1;
    }

    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }

    // Set baud rates
    if (
        cfsetospeed(&tty, baud_rate) < 0 ||
        cfsetispeed(&tty, baud_rate) < 0
    ) {
        perror("Set UART baud rate");
        return -1;
    }

    for(int i = 0; i < NCCS; i++)                   // No spec chr
        tty.c_cc[i] = '\0';

    tty.c_cc[VMIN] = 1;                             // Read block
    tty.c_cc[VTIME] = 0;                            // Infinite timeout
    tty.c_lflag = 0;                                // No signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                                // No remapping, no delays
    tty.c_iflag = (IGNBRK | IGNPAR);                // Ignore break, parity

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_cflag |= (CLOCAL | CREAD );               // Ignore modem controls, enable reading
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // Shut off xon/xoff ctrl
    tty.c_cflag &= ~(PARENB | PARODD);              // Shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }
    return fd;
}

void *thread_tun_rx(void *arg) {

    uint8_t buffer[BUFFER_SIZE];

    while(1) {
        ssize_t nread = read(tun_fd, buffer, sizeof(buffer));
        if (nread < 0) {
            perror("Reading from TUN interface");
            return NULL;
        }
        printf("Read %zd bytes from TUN interface\n", nread);
        if (debug) {
            printf("[DBG] Send: ");
            for (uint32_t i = 0; i < nread; ++i) printf("%02X ", buffer[i]);
            printf("\n");
        }
        slip_send_message(&slip, buffer, nread);
    }
}

void *thread_uart_rx(void *arg) {
    uint8_t byte;

    while(1) {
        ssize_t nread = read(serial_fd, &byte, 1);
        if (nread < 0) {
            perror("Reading from UART port");
            return NULL;
        }
        slip_read_byte(&slip, byte);
    }
}

void die(int code) {
    printf("....\nExit\n");
    close(tun_fd);
    close(serial_fd);
    exit(code);
}

void print_usage(char *program_name) {
    printf("\nUsage: %s -s [serial speed] -l [serial device] [-d]\n\n", program_name);
}


int main(int argc, char *argv[]) {

    pthread_t t_tun, t_uart;
    int uart_speed = -1;
    char* uart_name = NULL;
    char tun_name[IFNAMSIZ] = TUN_NAME;

    // Setup signal handlers
    (void) signal(SIGHUP, die);
    (void) signal(SIGINT, die);
    (void) signal(SIGQUIT, die);
    (void) signal(SIGTERM, die);

    // Parse params
    int opt;
    while ((opt = getopt(argc, argv, "s:l:d")) != -1) {
        switch (opt) {
            case 's':;
                uart_speed = atoi(optarg);
                break;
            case 'l':
                uart_name = optarg;
                break;
            case 'd':
                debug = 1;
                break;
        }
    }

    if (!uart_name || uart_speed < 0 ) {
        print_usage(argv[0]);
        exit(EXIT_FAILURE);
    }


    // Alloc TUN interface
    tun_fd = create_tun(tun_name);
    if (tun_fd < 0) {
        die(EXIT_FAILURE);
    } else {
        printf("Interface \"%s\" created\n", tun_name);
    }

    // Init UART port
    serial_fd = open_uart(uart_name, get_baud_rate(uart_speed));
    if (serial_fd < 0) {
        die(EXIT_FAILURE);
    } else {
        printf("Serial \"%s\" opened\n", uart_name); // Added verbosity
    }

    // Init SLIP processor
    slip_descriptor_s slip_descriptor = {
        .buf = slip_buf,
        .buf_size = sizeof(slip_buf),
        .crc_seed = 0xFFFF,
        .recv_message = slip_recv_message,
        .write_byte = slip_write_byte,
    };
    slip_init(&slip, &slip_descriptor);

    // Create TUN RX thread
    if (pthread_create(&t_tun, NULL, thread_tun_rx, NULL) != 0) {
        perror("pthread_create");
        die(EXIT_FAILURE);
    }

    // Create UART RX thread
    if (pthread_create(&t_uart, NULL, thread_uart_rx, NULL) != 0) {
        perror("pthread_create");
        die(EXIT_FAILURE);
    }

    // Wait for threads to finish
    while (1) {
        if (pthread_tryjoin_np(t_tun, NULL) == 0)   break;
        if (pthread_tryjoin_np(t_uart, NULL) == 0)  break;
        sleep(1);
    }

    // Stop threads
    pthread_cancel(t_tun);
    pthread_cancel(t_uart);

    die(EXIT_SUCCESS);
}
