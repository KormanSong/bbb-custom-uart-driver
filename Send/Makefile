obj-m += uart_dd_tx.o
ccflags-y += -I"/usr/src/linux-headers-4.19.94-ti-r42/include"

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	gcc -o send main.c
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

load:
	sudo insmod uart_dd_tx.ko
	sudo mknod /dev/sw_uart_tx c 244 0
	sudo chmod 666 /dev/sw_uart_tx
reload:
	sudo rmmod -f uart_dd_tx.ko
	sudo insmod uart_dd_tx.ko
	sudo rm /dev/sw_uart_tx
	sudo mknod /dev/sw_uart_tx c 244 0
	sudo chmod 666 /dev/sw_uart_tx
unload:
	sudo rmmod -f uart_dd_tx || true
	sudo rm /dev/sw_uart_tx || true
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
