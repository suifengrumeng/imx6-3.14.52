#!/bin/sh

#参考的板级为mach-imx6q.c, dtb文件为imx6dl-sabresd-ldo.dts

cp -rf 6dl_shsz_emmc.config .config 
make -j8 zImage

#dtb
make imx6dl-shsz-emmc-ldo.dtb 
make imx6q-shsz-emmc.dtb 

cp -vf arch/arm/boot/zImage /home/suifeng/XP-UBUNTU/zImage_6dl_shsz_emmc
cp -vf arch/arm/boot/dts/imx6dl-shsz-emmc-ldo.dtb /home/suifeng/XP-UBUNTU/imx6dl-shsz-emmc-ldo.dtb

#uImage的制作
#cat  ./arch/arm/boot/zImage ./arch/arm/boot/dts/imx6q-sz3d-dlp-v0.5.dtb > zImage+dtb
#mkimage -n imx6 -A arm -O linux -T kernel -C none -a 0x10008000 -e 0x10008000 -d zImage+dtb uImage+dtb
#cp uImage+dtb ~/tftpboot/imx6/
