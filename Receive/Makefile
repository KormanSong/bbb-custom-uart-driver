obj-m += uart_dd_rx.o
ccflags-y += -I"/usr/src/linux-headers-4.19.94-ti-r42/include"

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	gcc -o main main.c
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

load:
	sudo insmod uart_dd_rx.ko
	sudo mknod /dev/sw_uart_rx c 246 0
	sudo chmod 666 /dev/sw_uart_rx

unload:
	sudo rmmod -f uart_dd_rx || true
	sudo rm /dev/sw_uart_rx || true
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

reload:
	sudo rmmod -f uart_dd_rx.ko
	sudo insmod uart_dd_rx.ko
	sudo rm /dev/sw_uart_rx
	sudo mknod /dev/sw_uart_rx c 246 0
	sudo chmod 666 /dev/sw_uart_rx
