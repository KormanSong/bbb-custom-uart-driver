#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdint.h>
#include <ctype.h>

#define DEVICE_NODE "/dev/sw_uart_rx"
#define BUFFER_SIZE 8  // 64비트(8바이트) 데이터 수신

void print_binary(uint8_t* data, int size) {
    for (int i = 0; i < size; i++) {
        for (int j = 7; j >= 0; j--) {
            printf("%d", (data[i] >> j) & 1);
        }
        printf(" ");
    }
    printf("\n");
}

void print_ascii(uint8_t* data, int size) {
    printf("ASCII: \"");
    for (int i = 0; i < size; i++) {
        if (isprint(data[i])) {  // 출력 가능한 문자인 경우
            printf("%c", data[i]);
        } else {
            printf(".");  // 출력 불가능한 문자는 '.'으로 표시
        }
    }
    printf("\"\n");
}

int main() {
    int fd;
    uint8_t rx_buffer[BUFFER_SIZE];
    ssize_t bytes_read;

    fd = open(DEVICE_NODE, O_RDONLY);
    if (fd < 0) {
        printf("Failed to open device: %s\n", strerror(errno));
        return -1;
    }

    printf("Waiting for data...\n");

    while (1) {
        bytes_read = read(fd, rx_buffer, BUFFER_SIZE);
        
        if (bytes_read < 0) {
            if (errno == EIO) {
                printf("I/O error (CRC mismatch or uncorrectable ECC error)\n");
            } else {
                printf("Read error: %s\n", strerror(errno));
            }
            continue;
        }
        
        if (bytes_read == 0) {
            usleep(1000);  // 100ms 대기
            continue;
        }

        // 수신된 데이터 출력
        printf("\nReceived %zd bytes:\n", bytes_read);
        printf("Binary: ");
        print_binary(rx_buffer, bytes_read);
        
        printf("Hex: ");
        for (int i = 0; i < bytes_read; i++) {
            printf("%02X ", rx_buffer[i]);
        }
        printf("\n");
        
        print_ascii(rx_buffer, bytes_read);
        printf("\n");
    }

    close(fd);
    return 0;
}