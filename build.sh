#! /bin/bash

sudo make
sudo make install
sudo depmod --quick
sudo modprobe dm-imrsim
if [ "$1" = "loop" ]; then
    dd if=/dev/zero of=/tmp/imrsim1 bs=4096 seek=$(((256*4+64)*1024*1024/4096-1)) count=1
    losetup /dev/loop1 /tmp/imrsim1
    echo "0 `imrsim_util/imr_format.sh -d /dev/loop1` imrsim /dev/loop1 0" | dmsetup create imrsim
else
    echo "0 2097152 imrsim /dev/sdb2 0" | dmsetup create imrsim
    dd if=/dev/zero of=/dev/mapper/imrsim bs=4096 seek=$((2097152)) count=32*1024 2> /dev/null 1> /dev/null
fi
