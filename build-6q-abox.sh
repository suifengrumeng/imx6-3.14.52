#!/bin/sh

cp -vf 6q_abox.config .config 
make -j4 zImage

make imx6dl-abox-ldo.dtb 
make imx6q-abox-ldo.dtb 

cp -vf arch/arm/boot/zImage /home/suifeng/XP-UBUNTU/zImage_abox
cp -vf arch/arm/boot/dts/imx6q-abox-ldo.dtb /home/suifeng/XP-UBUNTU/
cp -vf arch/arm/boot/dts/imx6dl-abox-ldo.dtb /home/suifeng/XP-UBUNTU/


#uImage的制作
#cat  ./arch/arm/boot/zImage ./arch/arm/boot/dts/imx6q-sz3d-dlp-v0.5.dtb > zImage+dtb
#mkimage -n imx6 -A arm -O linux -T kernel -C none -a 0x10008000 -e 0x10008000 -d zImage+dtb uImage+dtb
#cp uImage+dtb ~/tftpboot/imx6/
