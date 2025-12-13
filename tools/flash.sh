#!/bin/bash

SCRIPT_DIR=$(dirname "$(realpath "$0")")      
BUILD_PATH=$(realpath "$SCRIPT_DIR/../build/") 
dfu-util -a 0 -s 0x08000000:leave -D ${BUILD_PATH}/prototype.bin