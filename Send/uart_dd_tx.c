#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/gpio.h>

#define BAUD_RATE       9600
#define BIT_DELAY_NS    ((1000000000 / BAUD_RATE)) // 나노초 단위
// Start(1) + Data(88) + Stop(1) = 총 90비트
#define DATA_BYTES      (88 / 8) // 88bit = 11byte

#define GPIO_MAJOR      244
#define GPIO_MINOR      0
#define GPIO_DEVICE     "sw_uart_tx"
#define GPIO_UART_TX    12

#define GPIO_BASE       0x4804C000
#define GPIO_END        0x4804CFFF
#define GPIO_SIZE       (GPIO_END - GPIO_BASE)

#define GPIO_DATAIN     0x138
#define GPIO_DATAOUT    0x13C
#define GPIO_OE_OFF     0x134
#define GPIO_SET_OFF    0x194
#define GPIO_CLEAR_OFF  0x190

#define GPIO_IN(g)      ((*(gpio+GPIO_OE_OFF/4)) |= (1<<(g)))
#define GPIO_OUT(g)     ((*(gpio+GPIO_OE_OFF/4)) &= ~(1<<(g)))
#define GPIO_SET(g)     ((*(gpio+GPIO_SET_OFF/4)) |= (1<<(g)))
#define GPIO_CLEAR(g)   ((*(gpio+GPIO_CLEAR_OFF/4)) |= (1<<(g)))

#define DATA_BITS 64
#define PARITY_BITS 7
#define TOTAL_BITS (DATA_BITS + PARITY_BITS)

static int gpio_open(struct inode* inode, struct file* file);
static int gpio_release(struct inode* inode, struct file* file);
static ssize_t gpio_read(struct file* file, char* buf, size_t len, loff_t* off);
static ssize_t gpio_write(struct file* file, const char* buf, size_t len, loff_t* off);


typedef struct {
    uint8_t data[9];
    uint16_t crc;
} PACKET;

typedef struct {
    uint8_t interleaved[9];  
} ecc_interleaved;

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


// GPIO 레지스터 맵핑
static volatile unsigned* gpio;

// 송신용 버퍼: 88bit -> 11바이트
static unsigned char tx_data[DATA_BYTES];

// 현재 전송 중인 비트 인덱스(-1이면 아직 Start 비트 전송 전)
static int bit_idx = -1;

// 전송 완료 플래그
static int stop_transmission = 0;

// hrtimer
static struct hrtimer tx_timer;

// cdev
static struct cdev gpio_cdev;
static struct file_operations gpio_fop = {
   .owner   = THIS_MODULE,
   .open    = gpio_open,
   .release = gpio_release,
   .read    = gpio_read,
   .write   = gpio_write,
};

/* 
 * hrtimer 콜백: bit_idx 순서대로 GPIO_UART_TX에 데이터 설정
 *   - bit_idx == -1: Start bit(LOW)
 *   - bit_idx in [0 .. 87]: tx_data에서 해당 비트 추출
 *   - bit_idx == 88: Stop bit(HIGH)
 */
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


ecc_interleaved encode_interleaved(uint64_t data) {
    ecc_interleaved result = {0};
    int data_bit = 0;
    int i, p, i1;
    int byte_pos, bit_pos;
    int parity_pos;
    uint8_t parity;
    for (i = 1; i <= TOTAL_BITS; i++) {
        if (!is_parity_position(i)) {
            if (data & (1ULL << data_bit)) {
                byte_pos = (i - 1) / 8;
                bit_pos = (i - 1) % 8;
                result.interleaved[byte_pos] |= (1 << bit_pos);
            }
            data_bit++;
        }
    }
    

    for (p = 0; p < PARITY_BITS; p++) {
        parity_pos = (1 << p) - 1;  
        parity = 0;
        

        for (i1 = 0; i1 < TOTAL_BITS; i1++) {
            if (i1 != parity_pos) {  // 패리티 비트 자신은 제외
                byte_pos = i1 / 8;
                bit_pos = (i1 % 8);
                if ((i1 + 1) & (1 << p)) {  // 이 비트가 현재 패리티에 영향을 주는지 확인
                    parity ^= ((result.interleaved[byte_pos] >> bit_pos) & 1);
                }
            }
        }
        
        // 계산된 패리티 비트 삽입
        byte_pos = parity_pos / 8;
        bit_pos = (parity_pos % 8);
        if (parity) {
            result.interleaved[byte_pos] |= (1 << bit_pos);
        }
    }
    
    return result;
}

PACKET create_full_packet(uint64_t input_data) {
    PACKET result;
    ecc_interleaved encoded;
    printk("Original data: 0x%016llX\n", input_data);
    
    encoded = encode_interleaved(input_data);
    printk("Interleaved data (with parity):\n");
    memcpy(result.data, encoded.interleaved, 9);
    result.crc = crc16_ccitt(result.data, 9);
    
    return result;
}


static enum hrtimer_restart tx_timer_callback(struct hrtimer *timer) 
{
    if (bit_idx == -1) {
        // Start bit -> LOW
        GPIO_CLEAR(GPIO_UART_TX);
    }
    else if (bit_idx >= 0 && bit_idx < 88) {
        // 어느 byte, 어느 bit인지 계산
        int bytePos = bit_idx / 8; 
        int bitPos  = bit_idx % 8;
        // 해당 비트가 1이면 HIGH, 아니면 LOW
        if (tx_data[bytePos] & (1 << bitPos))
            GPIO_SET(GPIO_UART_TX);  
        else
            GPIO_CLEAR(GPIO_UART_TX);
    }
    else if (bit_idx == 88) {
        // Stop bit -> HIGH
        GPIO_SET(GPIO_UART_TX);
    }
    else {
        // bit_idx > 88 => 전송 완료
        stop_transmission = 1; 
        return HRTIMER_NORESTART;
    }

    bit_idx++;
    hrtimer_forward_now(timer, ns_to_ktime(BIT_DELAY_NS));
    return HRTIMER_RESTART;
}

// 모듈 초기화
int start_module(void)
{
   dev_t devno;
   unsigned int cnt = 1;
   static void* map;
   int add;

   printk(KERN_INFO "START MODULE\n");

   // 장치 번호 할당 및 등록
   devno = MKDEV(GPIO_MAJOR, GPIO_MINOR);
   register_chrdev_region(devno, 1, GPIO_DEVICE);

   // cdev 초기화 및 등록
   cdev_init(&gpio_cdev, &gpio_fop);
   gpio_cdev.owner = THIS_MODULE;
   add = cdev_add(&gpio_cdev, devno, cnt);
   if (add < 0) {
       printk(KERN_ERR "cdev_add failed\n");
       return add;
   }

   // GPIO 레지스터 맵핑
   map = ioremap(GPIO_BASE, GPIO_SIZE);
   gpio = (volatile unsigned int*)map;

   // TX 핀 출력으로 설정, 초기 HIGH
   GPIO_OUT(GPIO_UART_TX);
   GPIO_SET(GPIO_UART_TX);

   // hrtimer 초기화
   hrtimer_init(&tx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
   tx_timer.function = &tx_timer_callback;

   return 0;
}

// 모듈 종료
void end_module(void)
{
   dev_t devno = MKDEV(GPIO_MAJOR, GPIO_MINOR);

   unregister_chrdev_region(devno, 1);
   cdev_del(&gpio_cdev);

   if (gpio) {
      iounmap(gpio);
   }

   hrtimer_cancel(&tx_timer);
   printk(KERN_INFO "END MODULE\n");
}

static int gpio_open(struct inode *inode, struct file *file)
{
   try_module_get(THIS_MODULE);
   printk("OPEN - gpio device\n");
   return 0;
}

static int gpio_release(struct inode* inode, struct file* file)
{
   module_put(THIS_MODULE);
   printk("CLOSE - gpio device\n");
   return 0;
}

static ssize_t gpio_read(struct file* file, char* buf, size_t len, loff_t* off)
{
   // 송신 전용 예시이므로, 특별히 읽어야 할 내용이 없다면 0 처리
   return 0;
}

/*
 * gpio_write:
 *   - 사용자로부터 임의 길이(len)만큼 데이터를 받는다.
 *   - 최대 88비트(11바이트)씩 끊어서 전송한다.
 *   - 만약 11바이트보다 적으면 나머지는 0으로 패딩하여 88비트를 맞춤.
 *   - 각 88비트 전송마다 busy waiting.
 *   - 전송할 때마다 실제 받은 비트 수와 패딩한 비트 수를 printk로 출력.
 */
static ssize_t gpio_write(struct file* file, const char* buf, size_t len, loff_t* off)
{
   uint64_t input_data;
   size_t total_sent = 0;
    ecc_interleaved encoded;
    uint16_t crc;
   while (total_sent < len) {
       // 남은 데이터 크기 계산
       size_t remaining = len - total_sent;
       size_t chunk_size = (remaining >= 8) ? 8 : remaining;
       
       // 64비트 버퍼 초기화 (패딩용)
       input_data = 0;

       // 데이터 복사 (chunk_size만큼만)
       if (copy_from_user(&input_data, buf + total_sent, chunk_size)) {
           return -EFAULT;
       }

       printk(KERN_INFO "[sw_uart_tx] Transmitting %zu bytes + padded %zu bytes",
              chunk_size, 8 - chunk_size);

       // ECC 인코딩 (64비트 -> 72비트)
       encoded = encode_interleaved(input_data);
       
       // CRC 계산 (72비트에 대해)
       crc = crc16_ccitt(encoded.interleaved, 9);

       // 최종 88비트 패킷 구성
       memset(tx_data, 0, DATA_BYTES);
       memcpy(tx_data, encoded.interleaved, 9);  // 72비트 ECC
       tx_data[9] = (crc >> 8) & 0xFF;   // CRC 상위 바이트
       tx_data[10] = crc & 0xFF;         // CRC 하위 바이트

       // 전송 준비 및 시작
       bit_idx = -1;
       stop_transmission = 0;
       hrtimer_start(&tx_timer, ns_to_ktime(BIT_DELAY_NS), HRTIMER_MODE_REL);

       // 전송 완료 대기
       while (!stop_transmission)
           cpu_relax();

       total_sent += chunk_size;
   }
   return total_sent;
}
MODULE_LICENSE("GPL");
module_init(start_module);
module_exit(end_module);