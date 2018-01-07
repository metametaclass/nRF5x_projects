# nRF5x_projects
nordic semiconductor nRF5x series projects

* download gcc-arm embedded toolchain: https://launchpad.net/gcc-arm-embedded
* unpack gcc-arm-XXXX.zip somewhere
* download and install MSYS: http://www.mingw.org/wiki/MSYS http://downloads.sourceforge.net/mingw/MSYS-1.0.11.exe
* download and unpack openocd
* install ST-Link drivers

* run MSYS with PATH to python and gcc:
* create and run script run_msys_gcc_arm.cmd in MSYS folder (where msys.bat is located):

```
rem change path to installed gcc-arm and openocd 'bin' folder
set PATH=%PATH%;D:\Programm\gcc-arm\bin;D:\Programm\OpenOCD0.10\bin
call %~dp0msys.bat
```

* download and unpack nRF SDK 12.3 to folder nordic\nRF5_SDK_12.3.0\

* change folder to example projec

```
cd nRF51822\ble_app_beacon_12.2\
```

* connect st-link v2 to nRF51822 development board:
```
GND - GND
3.3v - VCC
SWDCLK - SWDCLK
SWDIO - SWDIO
```

* build and flash program
make flash_130

* use any BLe scanner app on Android or iOS and find running beacon
