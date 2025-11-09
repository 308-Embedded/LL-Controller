# Current State

Dshot Motor Driver          ✅  
Bmi088 IMU Driver           ✅  
Serial Communication        ✅  
3d Math Library             ✅  
PWM Driver                  ✅ 

# Usage
install dependency:  
```
sudo apt-get install libncurses5-dev genromfs build-essential  
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi
sudo apt install kconfig-frontends  
 
sudo apt install dfu-util  
```
Init: `cmake ..`
Open menuconfig: `cmake --build . --target menuconfig`.  
Save defconfig: `cmake --build . --target saveconfig`. 
build: `make -j8`.  
Download: `dfu-util -a 0 -s 0x08000000:leave -D prototype.bin`  
Compile twice surprise.
