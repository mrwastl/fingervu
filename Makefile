obj-m = fingervu.o
KVERSION = $(shell uname -r)
all:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
clean:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
install:
	/bin/mkdir -p /lib/modules/$(KVERSION)/extra/ && /bin/cp -f fingervu.ko /lib/modules/$(KVERSION)/extra/ && depmod -a && sync
