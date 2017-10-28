#!/bin/sh

cp -vf 6q_mq.config .config 
make -j4 zImage

make imx6q-mq.dtb 
make imx6q-mq-ldo.dtb

cp -vf arch/arm/boot/zImage /home/suifeng/XP-UBUNTU/zImage_mq
cp -vf arch/arm/boot/dts/imx6q-mq-ldo.dtb /home/suifeng/XP-UBUNTU/


#uImage的制作
#cat  ./arch/arm/boot/zImage ./arch/arm/boot/dts/imx6q-sz3d-dlp-v0.5.dtb > zImage+dtb
#mkimage -n imx6 -A arm -O linux -T kernel -C none -a 0x10008000 -e 0x10008000 -d zImage+dtb uImage+dtb
#cp uImage+dtb ~/tftpboot/imx6/
