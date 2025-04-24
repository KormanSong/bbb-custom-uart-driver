 #include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

#define DEVICE "/dev/sw_uart_tx"
#define BUFFER_SIZE 8  // 버퍼 크기 8바이트로 고정

void print_binary(const char* data, size_t len) {
    printf("Binary: ");
    for(size_t i = 0; i < len; i++) {
        for(int j = 7; j >= 0; j--) {
            printf("%d", (data[i] >> j) & 1);
        }
        printf(" ");
    }
    printf("\n");
}

void print_hex(const char* data, size_t len) {
    printf("Hex: ");
    for(size_t i = 0; i < len; i++) {
        printf("%02X ", (unsigned char)data[i]);
    }
    printf("\n");
}

int main() {
    int fd = open(DEVICE, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open device");
        return -1;
    }

    char buffer[BUFFER_SIZE];  // 8바이트 버퍼
    char input[1024];         // 충분히 큰 입력 버퍼
    uint64_t binary_value;
    
    while(1) {
        printf("\nEnter text ('q' to quit): ");
        if (fgets(input, sizeof(input), stdin) == NULL) {
            break;
        }
        
        // 종료 조건 체크
        if(input[0] == 'q' && (input[1] == '\n' || input[1] == '\0')) {
            break;
        }

        // 입력 데이터 처리
        size_t input_len = strlen(input);
        size_t bytes_to_send = (input_len > BUFFER_SIZE) ? BUFFER_SIZE : input_len;
        
        // 버퍼에 데이터 복사
        memcpy(buffer, input, bytes_to_send);

        printf("\nInput text: ");
        for(size_t i = 0; i < bytes_to_send; i++) {
            if(buffer[i] == '\n') {
                printf("\\n");
            } else {
                printf("%c", buffer[i]);
            }
        }
        printf(" (length: %zu)\n", bytes_to_send);

        printf("ASCII values: ");
        for(size_t i = 0; i < bytes_to_send; i++) {
            printf("%d ", buffer[i]);
        }
        printf("\n");
        
        print_hex(buffer, bytes_to_send);
        print_binary(buffer, bytes_to_send);

        // 64비트 정수로 변환된 값 출력
        binary_value = 0;
        memcpy(&binary_value, buffer, bytes_to_send);
        printf("64-bit integer: %lu\n", binary_value);
        
        // 디바이스에 쓰기
        if(write(fd, buffer, bytes_to_send) < 0) {
            perror("Write failed");
            break;
        }
        
        usleep(100000);  // 100ms 대기
    }

    close(fd);
    return 0;
}