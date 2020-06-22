#!/bin/bash

cd ../linux/
sudo make -j4 -C . M=drivers/net/wireless/ath/ath9k modules_install