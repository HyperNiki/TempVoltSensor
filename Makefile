obj-m += tempvoltsensor.o
all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	rm ./.*.cmd tempvoltsensor.mod* tempvoltsensor.o Module.* modules.*
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

test:
	sudo dmesg -C
	sudo insmod tempvoltsensor.ko
	sudo rmmod tempvoltsensor.ko
	dmesg