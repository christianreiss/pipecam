# Out-of-tree build for the pipecam V4L2 driver.
obj-m += pipecam.o

# VB2 released q->lock through explicit wait callbacks up to Linux 6.12.
# Newer trees do it in the core and Linux 7.0 removed the callback fields.
# Probe the target headers instead of assuming an upstream version layout,
# since distributions backport media subsystem changes independently.
ifneq ($(KERNELRELEASE),)
ifneq ($(shell \
	grep -F -q 'void (*wait_prepare)(struct vb2_queue *q);' \
		$(srctree)/include/media/videobuf2-core.h 2>/dev/null && \
	grep -F -q 'void vb2_ops_wait_prepare(struct vb2_queue *vq);' \
		$(srctree)/include/media/videobuf2-v4l2.h 2>/dev/null && echo y),)
ccflags-y += -DPIPECAM_HAVE_VB2_WAIT_OPS
endif
endif

KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install: all
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a $(KVER)

.PHONY: all clean install
