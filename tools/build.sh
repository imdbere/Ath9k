#!/bin/bash

cp *.* ../linux/drivers/net/wireless/ath/ath9k/
cd ../linux/
make -j4 -C . M=drivers/net/wireless/ath/ath9k modules