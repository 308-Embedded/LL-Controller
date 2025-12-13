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
Download: `dfu-util -a 0 -s 0x08000000:leave -D prototype.bin` or   
`../tools/flash.sh`  
Compile twice surprise.

# development guide 

### `src/modules`  
This path contains the source code of executables.
```
add_module(
  MODULE modules__hello_nuttx
  MAIN hello_nuttx
  SRCS
    hello_nuttx_main.cc
    hello_nuttx.cc
    hello_nuttx.h
  DEPENDS nuttx_arch
)
```
This will generate an executable binary `hello_nuttx` in nuttx shell.  

### `src/libraries`  
This path contains the source code of libraries.   

```
ll_add_library(
  LIB_NAME library__dshot
  SRCS
  empty.cc
  dshot_lower_drv.hpp
  INTERFACES
  interface
)
```
Note that by filling the `INTERFACES` here, interface headers will be automatically exposed to the linking modules.