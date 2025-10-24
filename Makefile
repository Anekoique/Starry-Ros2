IMG := debian-riscv64.img

init:
	mkdir -p rootfs
	sudo debootstrap --arch=riscv64 --foreign --variant=minbase \
		trixie ./rootfs \
		http://deb.debian.org/debian
	sudo cp /usr/bin/qemu-riscv64-static ./rootfs/usr/bin/
	sudo chroot ./rootfs /debootstrap/debootstrap --second-stage

root:
	sudo chroot ./rootfs /bin/bash

img:
	dd if=/dev/zero of=$(IMG) count=1024 bs=1M
	mkfs.ext4 -b 4096 -O ^metadata_csum $(IMG)
	mkdir -p mnt
	sudo mount $(IMG) mnt
	sudo cp -a rootfs/* mnt/
	sudo umount mnt

clean:
	rm -rf rootfs mnt

.PHONY: init root img clean

