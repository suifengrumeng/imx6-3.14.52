#!/bin/sh

#make -j8 clean

#贺工让做的测试版本

rm -rf  arch/arm/boot/zImage uImage+dtb zImage+dtb 
rm -rf arch/arm/boot/dts/imx6q-sz3d-dlp-v0.5.dtb


cp -rf sz3d-dlp-gpio.config .config 
make -j8 zImage

#dtb
#make imx6q-sz3d-dlp.dtb
make imx6q-sz3d-dlp-ldo-gpio.dtb 


#uImage的制作
#cat  ./arch/arm/boot/zImage ./arch/arm/boot/dts/imx6q-sz3d-dlp-v0.5.dtb > zImage+dtb
#mkimage -n imx6 -A arm -O linux -T kernel -C none -a 0x10008000 -e 0x10008000 -d zImage+dtb uImage+dtb
#cp uImage+dtb ~/tftpboot/imx6/
