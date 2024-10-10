#!/bin/bash

# Define the version
VERSION="10.3-2021.10"

# Create symbolic links
sudo ln -s /usr/share/gcc-arm-none-eabi-$VERSION/bin/arm-none-eabi-gcc /usr/bin/arm-none-eabi-gcc
sudo ln -s /usr/share/gcc-arm-none-eabi-$VERSION/bin/arm-none-eabi-g++ /usr/bin/arm-none-eabi-g++
sudo ln -s /usr/share/gcc-arm-none-eabi-$VERSION/bin/arm-none-eabi-gdb /usr/bin/arm-none-eabi-gdb
sudo ln -s /usr/share/gcc-arm-none-eabi-$VERSION/bin/arm-none-eabi-size /usr/bin/arm-none-eabi-size
sudo ln -s /usr/share/gcc-arm-none-eabi-$VERSION/bin/arm-none-eabi-objcopy /usr/bin/arm-none-eabi-objcopy

echo "Symbolic links created for gcc-arm-none-eabi-$VERSION."

