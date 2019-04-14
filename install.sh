#!/bin/bash

set -e
set -x

sed '$s/$/ spidev.bufsiz=65536/' /boot/cmdline.txt > tmp.txt && mv -- tmp.txt /boot/cmdline.txt
chmod 777 /dev/spidev0.0
chmod 777 /dev/serial0
chmod 777 /dev/ttyS0

adduser pulu
adduser pulu dialout
adduser pulu tty
adduser pulu spi
adduser pulu sudo

systemctl enable ssh

apt-get update
apt-get -y install git
cd /home/pulu
sudo -u pulu git clone https://github.com/siwastaja/rn1-host
sudo -u pulu git clone https://github.com/siwastaja/rn1-brain
sudo -u pulu git clone https://github.com/siwastaja/rn1-tools
cd /home/pulu/rn1-tools
sudo -u pulu make spiprog
sudo -u pulu make prog
cd /home/pulu/rn1-host
sudo -u pulu make

