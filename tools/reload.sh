#!/bin/bash

sudo rmmod ath9k_htc
sudo rmmod ath9k
sudo rmmod ath9k_common
sudo rmmod ath9k_hw

#sudo insmod ../linux/drivers/net/wireless/ath/ath9k/
cd ../linux/drivers/net/wireless/ath/ath9k/

sudo insmod ath9k_hw.ko
sudo insmod ath9k_common.ko
sudo insmod ath9k.ko
sudo insmod ath9k_htc.ko

sudo chmod 777 /dev/CSI_dev 
