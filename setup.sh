#!/bin/bash

echo -e "\033[32m[INFO]\033[0m ===== Step 1: Install usbnet driver  ====="
sudo cp usbnet.ko /lib/modules/$(uname -r)/kernel/drivers/net/usb/
sudo depmod -a
sudo modprobe usbnet

echo -e "\033[32m[INFO]\033[0m ===== Step 2: Install rndis driver ====="
sudo cp rndis_host.ko /lib/modules/$(uname -r)/kernel/drivers/net/usb/
sudo depmod -a
sudo modprobe rndis_host

echo -e "\033[32m[INFO]\033[0m ===== Step 3: Install cdc_ether driver ====="
sudo cp cdc_ether.ko /lib/modules/$(uname -r)/kernel/drivers/net/usb/
sudo depmod -a
sudo modprobe cdc_ether

echo -e"\033[32m[INFO]\033[0m ===== Step 4: Install python packages  ====="
pip3 install -r requirements.txt

echo -e "\033[32m[INFO]\033[0m ===== Complete Setup ====="
