#!/bin/bash

# Makes a bootable card with Raspbian
# will automatically assign to WLAN SSID pulu with password pulupulu
# You can log in as pi, password raspberry
# Then run:
# passwd root
# su
# /home/pi/install.sh
# install.sh will create user pulu, download all relevant software
# You are responsible for creating strong passwords when running install.sh
# Default pulu/pulupulu network must also be changed as soon as possible.

set -x

sudo umount /dev/mmcblk0p1
sudo umount /dev/mmcblk0p2
set -e
sudo dd bs=4M if=raspbian.img of=/dev/mmcblk0 conv=fsync
sudo sync
sleep 1
sudo partprobe /dev/mmcblk0
set +e
sudo mkdir /mnt/tmpmnt_boot
sudo mkdir /mnt/tmpmnt_system
set -e
sudo mount /dev/mmcblk0p1 /mnt/tmpmnt_boot
sudo mount /dev/mmcblk0p2 /mnt/tmpmnt_system
sudo cp raspi_example_config.txt /mnt/tmpmnt_boot/config.txt
sudo cp raspi_rclocal_example /mnt/tmpmnt_system/etc/rc.local
sudo cp raspi_wpasupplicant_example /mnt/tmpmnt_system/etc/wpa_supplicant/wpa_supplicant.conf
sudo echo $1 > /mnt/tmpmnt_system/etc/hostname
sudo touch /mnt/tmpmnt_boot/ssh
sudo cp install.sh /mnt/tmpmnt_system/home/pi/
sudo chmod 777 /mnt/tmpmnt_system/home/pi/install.sh
sudo umount /dev/mmcblk0p1
sudo umount /dev/mmcblk0p2
sudo rmdir /mnt/tmpmnt_boot
sudo rmdir /mnt/tmpmnt_system
echo "Card done, please detach"


