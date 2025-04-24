#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/io.h>

#include <linux/interrupt.h> 
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>



#define BAUD_RATE 9600
#define BIT_DELAY_NS (1000000000 / BAUD_RATE)

#define GPIO_MAJOR   246
#define GPIO_MINOR   0
#define GPIO_DEVICE   "sw_uart_rx"
#define GPIO_UART_RX 13
#define RX_BUFFER_SIZE 256 

#define GPIO_BASE   0x4804C000
#define GPIO_END   0x4804CFFF
#define GPIO_SIZE   GPIO_END-GPIO_BASE

#define GPIO_DATAIN     0x138
#define GPIO_DATAOUT    0x13C
#define GPIO_OE_OFF      0x134
#define GPIO_SET_OFF   0x194
#define GPIO_CLEAR_OFF   0x190

#define GPIO_IN(g)      ((*(gpio+GPIO_OE_OFF/4)) |= (1<<g))
#define GPIO_OUT(g)      ((*(gpio+GPIO_OE_OFF/4)) &= ~(1<<g))
#define GPIO_SET(g)      ((*(gpio+GPIO_SET_OFF/4)) |= (1<<g))
#define GPIO_CLEAR(g)   ((*(gpio+GPIO_CLEAR_OFF/4)) |= (1<<g))
#define BUF 9
#define DATA_BITS 64
#define PARITY_BITS 7
#define TOTAL_BITS (DATA_BITS + PARITY_BITS)
#define RX_DATA_SIZE_BYTES 11
#define RX_PACKET_SIZE_BITS 88
#define RX_TOTAL_BITS (1 + RX_PACKET_SIZE_BITS + 1)

#define RX_BUFFER_SIZE 256


static int gpio_open(struct inode* inode, struct file* file);
static int gpio_release(struct inode* inode, struct file* file);
static ssize_t gpio_read(struct file* file, char* buf, size_t len, loff_t* off);
static ssize_t gpio_write(struct file* file, const char* buf, size_t len, loff_t* off);

static irqreturn_t gpio_irq_handler(int irq, void *dev_id);
static void buffer_push(const unsigned char* data);
static int buffer_pop(unsigned char* out_data);
enum hrtimer_restart rx_timer_callback(struct hrtimer *timer);

volatile unsigned* gpio;
struct cdev gpio_cdev;
static struct file_operations gpio_fop={
   .owner = THIS_MODULE,
   .open = gpio_open,
   .release = gpio_release,
   .read = gpio_read,
   .write = gpio_write,
};

static int irq_num; // GPIO 핀의 인터럽트 번호

typedef struct {
    uint8_t* data;
    uint16_t crc;
} PACKET;

typedef struct {
    uint8_t interleaved[9];  
} ecc_interleaved;

static unsigned char rx_data[RX_DATA_SIZE_BYTES];
static unsigned char rx_buffer[RX_BUFFER_SIZE][RX_DATA_SIZE_BYTES];
static int rx_buffer_head = 0;       
static int rx_buffer_tail = 0;    
static int rx_bit_count = 0;          
static unsigned char rx_data[RX_DATA_SIZE_BYTES]; 

static struct hrtimer rx_timer;  // 수신용 hrtimer
static bool timer_running = false; // 타이머 상태 확인

#define BUFFER_FULL ((rx_buffer_head + 1) % RX_BUFFER_SIZE == rx_buffer_tail)
#define BUFFER_EMPTY (rx_buffer_head == rx_buffer_tail)

static irqreturn_t gpio_irq_handler(int irq, void *dev_id) {
    if (!timer_running) {
        rx_bit_count = 0;
        memset(rx_data, 0, sizeof(rx_data));
        // Start bit 감지 후 1.5 비트 시간 후에 첫 데이터 비트 샘플링
    hrtimer_start(&rx_timer, ns_to_ktime(BIT_DELAY_NS * 0.15), HRTIMER_MODE_REL);
        timer_running = true;
    }
    return IRQ_HANDLED;
}

static const uint16_t ccitt_hash[] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78, 
};

uint16_t crc16_ccitt(uint8_t* buffer, size_t size)
{
    uint16_t crc = 0;
    while (size-- > 0)
    {
    	crc = (crc >> 8) ^ ccitt_hash[(crc ^ *(buffer++)) & 0x00FF];
    }
    return crc;
}
static inline int is_parity_position(int pos) {
    return (pos & (pos - 1)) == 0; 
}

uint64_t decode_interleaved(ecc_interleaved* encoded, int* error_type) {
    uint8_t syndrome = 0;
    int total_ones = 0;
    int i1, i2, p, i3, j, i4, i5;
    int error_weight;
    int matching_positions;
    int last_error_pos;
    int data_bit;
    int byte_pos, bit_pos;
    uint8_t bit_syndrome;
    uint8_t parity;
    uint64_t result;
    for (i1 = 0; i1 < TOTAL_BITS; i1++) {
        byte_pos = i1 / 8;
        bit_pos = (i1 % 8);
        if (encoded->interleaved[byte_pos] & (1 << bit_pos)) {
            total_ones++;
        }
    }
    for (p = 0; p < PARITY_BITS; p++) {
        parity = 0;
        for (i2 = 0; i2 < TOTAL_BITS; i2++) {
            if ((i2 + 1) & (1 << p)) {
                byte_pos = i2 / 8;
                bit_pos = (i2 % 8);
                parity ^= ((encoded->interleaved[byte_pos] >> bit_pos) & 1);
            }
        }
        if (parity) {
            syndrome |= (1 << p);
        }
    }
    if (syndrome == 0) {
        *error_type = 0; 
    } else {
        error_weight = 0;  
        for (i3 = 0; i3 < PARITY_BITS; i3++) {
            if (syndrome & (1 << i3)) {
                error_weight++;
            }
        }
        matching_positions = 0;
        last_error_pos = -1;
        for (i4 = 0; i4 < TOTAL_BITS; i4++) {
            bit_syndrome = 0;
            for (j = 0; j < PARITY_BITS; j++) {
                if ((i4 + 1) & (1 << j)) {
                    bit_syndrome |= (1 << j);
                }
            }
            if (bit_syndrome == syndrome) {
                matching_positions++;
                last_error_pos = i4;
            }
        }
        if (matching_positions == 1 && error_weight <= 3) {  
            *error_type = 1;
            byte_pos = last_error_pos / 8;
            bit_pos = 7 - (last_error_pos % 8);
            encoded->interleaved[byte_pos] ^= (1 << bit_pos);
        } else {  
            *error_type = 2;
            return 0; 
        }
    }
    result = 0;
    if (*error_type != 2) {
        data_bit = 0;
        for (i5 = 1; i5 <= TOTAL_BITS; i5++) {
            if (!is_parity_position(i5)) {
                byte_pos = (i5 - 1) / 8;
                bit_pos = ((i5 - 1) % 8);
                if (encoded->interleaved[byte_pos] & (1 << bit_pos)) {
                    result |= (1ULL << data_bit);
                }
                data_bit++;
            }
        }
    }
    return result;
}


static void buffer_push(const unsigned char* data)
{
    if (BUFFER_FULL) {
        // 가득 찼다면 오래된 데이터 덮어쓰기
        rx_buffer_tail = (rx_buffer_tail + 1) % RX_BUFFER_SIZE;
    }
    memcpy(rx_buffer[rx_buffer_head], data, RX_DATA_SIZE_BYTES);
    rx_buffer_head = (rx_buffer_head + 1) % RX_BUFFER_SIZE;
}


static int buffer_pop(unsigned char* out_data)
{
    if (BUFFER_EMPTY) {
        return 0; 
    }
    memcpy(out_data, rx_buffer[rx_buffer_tail], RX_DATA_SIZE_BYTES);
    memset(rx_buffer[rx_buffer_tail], 0, RX_DATA_SIZE_BYTES);
    rx_buffer_tail = (rx_buffer_tail + 1) % RX_BUFFER_SIZE;
    return 1; 
}

enum hrtimer_restart rx_timer_callback(struct hrtimer *timer)
{
    int gpio_value = ((*(gpio + GPIO_DATAIN / 4)) & (1 << GPIO_UART_RX)) ? 1 : 0;
    if (rx_bit_count == 0) {
       printk("Start bit = %d\n", gpio_value);
    }
    else if (rx_bit_count >= 1 && rx_bit_count <= RX_PACKET_SIZE_BITS) {
        int bitIndex = rx_bit_count - 1; // 0~87
        int byteIndex = bitIndex / 8;    // 0~10
        int bitPos = bitIndex % 8;       // 0~7
        if (gpio_value) {
            rx_data[byteIndex] |= (1 << bitPos);
        }
    }
    else if (rx_bit_count == RX_PACKET_SIZE_BITS + 1) {
        printk("Stop bit = %d\n", gpio_value);
        buffer_push(rx_data);
        timer_running = false;
        return HRTIMER_NORESTART;
    }

    rx_bit_count++;
    if (rx_bit_count < RX_TOTAL_BITS) {
        hrtimer_forward_now(timer, ns_to_ktime(BIT_DELAY_NS));
        return HRTIMER_RESTART;
    }

    return HRTIMER_NORESTART; 
}


int start_module(void){
   unsigned int cnt = 1;             // 관리할 장치의 개수를 1로 설정
   static void* map;                 // 메모리 매핑된 주소를 저장할 포인터 map을 선언
   int add;                          // cdev_add 함수의 반환 값을 저장할 변수 add
   dev_t devno;                      // 장치 번호를 저장할 변수 devno

   printk(KERN_INFO "START MODULE\n"); // 커널 로그에 "START MODULE" 메시지를 출력

   devno = MKDEV(GPIO_MAJOR, GPIO_MINOR); // MAJOR와 MINOR 번호를 사용해 장치 번호 devno 생성
   register_chrdev_region(devno, 1, GPIO_DEVICE); // 생성된 장치 번호를 커널에 등록하여 사용 가능하도록 설정

   cdev_init(&gpio_cdev, &gpio_fop); // 캐릭터 디바이스 구조체 gpio_cdev 초기화하고 file_operations 구조체 연결
   gpio_cdev.owner = THIS_MODULE;     // 이 캐릭터 디바이스가 현재 모듈에 속함을 지정

   add = cdev_add(&gpio_cdev, devno, cnt); // 장치 번호 devno로 gpio_cdev를 커널에 등록

   map = ioremap(GPIO_BASE, GPIO_SIZE);    // GPIO 레지스터의 물리 주소를 가상 주소로 매핑하여 map에 저장
   gpio = (volatile unsigned int*)map;     // map을 gpio로 캐스팅하여 GPIO 레지스터에 접근할 수 있게 설정

   GPIO_IN(GPIO_UART_RX); // GPIO_UART_RX 핀을 입력 모드로 설정
   GPIO_SET(GPIO_UART_RX);   // GPIO_UART_RX 핀을 기본 상태 HIGH로 설정

    irq_num = gpio_to_irq(45); // GPIO 핀에 대한 IRQ 번호 가져오기
    if (irq_num < 0) {
        printk(KERN_ERR "Failed to map GPIO to IRQ\n");
        return -1;
    }

    // 인터럽트 핸들러 등록
    if (request_irq(irq_num, gpio_irq_handler, IRQF_TRIGGER_FALLING, 
                    "gpio_irq_handler", NULL)) {
        printk(KERN_ERR "Failed to request IRQ\n");
        return -1;
    }

    hrtimer_init(&rx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    rx_timer.function = rx_timer_callback;

   return 0;           
}

void end_module(void){
   dev_t devno = MKDEV(GPIO_MAJOR, GPIO_MINOR);
   unregister_chrdev_region(devno, 1);
   cdev_del(&gpio_cdev);

    if(gpio){
        iounmap(gpio);
    }

    if (irq_num >= 0) {
        free_irq(irq_num, NULL); // 등록된 인터럽트를 해제
    }

    hrtimer_cancel(&rx_timer);

   printk(KERN_INFO "END MODUILE\n");
}

static int gpio_open(struct inode *inode, struct file *file){
	try_module_get(THIS_MODULE);
	printk("OPEN - gpio device\n");
	return 0;
}

static int gpio_release(struct inode* inode, struct file* file){
	module_put(THIS_MODULE);
	printk("CLOSE - gpio device\n");
	return 0;
}


static ssize_t gpio_read(struct file *file, char *buf, size_t len, loff_t *off)
{
    unsigned char temp_buf[RX_DATA_SIZE_BYTES];
    unsigned char decoded_buf[8];
    int bytes_to_copy = RX_DATA_SIZE_BYTES; 
    int error_type;
    uint16_t received_crc, calculated_crc;
    uint64_t decoded_data;
    ecc_interleaved received = {0};
    if (len < RX_DATA_SIZE_BYTES) {
        bytes_to_copy = len;
    }
    if (buffer_pop(temp_buf)) {
        received_crc = (temp_buf[9] << 8 ) | temp_buf[10];
        calculated_crc = crc16_ccitt(temp_buf, 9);
        if (received_crc != calculated_crc) {
            printk(KERN_WARNING "CRC mismatch: received=0x%04X, calculated=0x%04X\n", received_crc, calculated_crc);
            return -EIO;
        }
        printk("CRC MATCHED!\n");

        memcpy(received.interleaved, temp_buf, 9);
        decoded_data = decode_interleaved(&received, &error_type);
        switch(error_type) {
            case 0:
                printk(KERN_INFO "No ECC errors detected\n");
                break;
            case 1:
                printk(KERN_INFO "Single bit error corrected by ECC\n");
                break;
            case 2:
                printk(KERN_ERR "Multiple bit errors detected - uncorrectable\n");
                return -EIO;
        }
        memcpy(decoded_buf, &decoded_data, 8);
        if (copy_to_user(buf, decoded_buf, bytes_to_copy)) {
            return -EFAULT;
        }
        return bytes_to_copy;
    }
    return 0; 
}



static ssize_t gpio_write(struct file* file, const char* buf, size_t len, loff_t* off){
	return len;
}

MODULE_LICENSE("GPL");
module_init(start_module);
module_exit(end_module);