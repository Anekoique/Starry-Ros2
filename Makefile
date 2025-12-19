ROOT_FS := rootfs
IMG := debian-riscv64.img
IMG_SIZE := 4G
TARFLAGS ?= -I 'xz -T0 -9e'

init:
	mkdir -p $(ROOT_FS)
	sudo debootstrap --arch=riscv64 --foreign --variant=minbase \
		trixie $(ROOT_FS) \
		http://deb.debian.org/debian
	sudo cp /usr/bin/qemu-riscv64-static $(ROOT_FS)/usr/bin/
	sudo chroot $(ROOT_FS) /debootstrap/debootstrap --second-stage

mount: 
	sudo mount -o bind /dev $(ROOT_FS)/dev
	sudo mount -t devpts devpts $(ROOT_FS)/dev/pts
	sudo mount -t proc proc $(ROOT_FS)/proc
	sudo mount -t sysfs sysfs $(ROOT_FS)/sys

root: mount
	sudo chroot $(ROOT_FS) /bin/bash
	@set -e; \
	for mp in dev/pts proc sys dev; do \
	  if mountpoint -q "$(ROOT_FS)/$$mp"; then \
	    echo "umount $(ROOT_FS)/$$mp"; \
	    sudo umount "$(ROOT_FS)/$$mp" || sudo umount -l "$(ROOT_FS)/$$mp"; \
	  fi; \
	done

img:
	truncate -s $(IMG_SIZE) $(IMG)
	mkfs.ext4 -b 4096 -O ^metadata_csum $(IMG)
	mkdir -p mnt
	sudo mount $(IMG) mnt
	sudo cp -a $(ROOT_FS)/* mnt/
	sudo umount mnt

pack:
	tar $(TARFLAGS) -cpf $(basename $(IMG)).tar.xz $(IMG)

clean:
	rm -rf $(ROOT_FS) mnt

.PHONY: init root img clean

