#!/bin/sh

#make -j8 clean

rm -rf  arch/arm/boot/zImage uImage+dtb zImage+dtb 
rm -rf arch/arm/boot/dts/imx6q-sz3d-dlp-v0.5.dtb


cp -rf sz3d-dlp.config .config 
make -j8 zImage

#dtb
#make imx6q-sz3d-dlp.dtb
make imx6q-sz3d-dlp-ldo.dtb 

make modules

cp -vf arch/arm/boot/zImage ../release/sz3d/
cp -vf arch/arm/boot/dts/imx6q-sz3d-dlp-ldo.dtb ../release/sz3d/

cp -vf drivers/usb/gadget/gadgetfs.ko ../release/sz3d/



#uImage的制作
#cat  ./arch/arm/boot/zImage ./arch/arm/boot/dts/imx6q-sz3d-dlp-v0.5.dtb > zImage+dtb
#mkimage -n imx6 -A arm -O linux -T kernel -C none -a 0x10008000 -e 0x10008000 -d zImage+dtb uImage+dtb
#cp uImage+dtb ~/tftpboot/imx6/


#wifi
cd ../../wifi/rtl8188EUS_linux_v4.3.24_16705.20160509/
make clean all
cp -vf ../../wifi/rtl8188EUS_linux_v4.3.24_16705.20160509/8188eu.ko ../release/sz3d/

