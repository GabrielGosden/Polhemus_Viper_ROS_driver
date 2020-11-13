#!/usr/bin/env bash

# This script installs libusb and the rules for the Viper

sudo apt-get install libusb-dev fxload
sudo cp 71-viper-rule.rules /etc/udev/rules.d/