# LuminaREACT firmware

## Building

1. Download the latest version of the gcc-arm-none-eabi toolchain from (ARM)[https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads]
2. Unpack it to `/usr/share`: 
```bash
sudo tar xjf gcc-arm-none-eabi-your-version.bz2 -C /usr/share/
```
3. Create links so that binaries are accessible system-wide:
```
sudo ln -s /usr/share/gcc-arm-none-eabi-your-version/bin/arm-none-eabi-gcc /usr/bin/arm-none-eabi-gcc 
sudo ln -s /usr/share/gcc-arm-none-eabi-your-version/bin/arm-none-eabi-g++ /usr/bin/arm-none-eabi-g++
sudo ln -s /usr/share/gcc-arm-none-eabi-your-version/bin/arm-none-eabi-gdb /usr/bin/arm-none-eabi-gdb
sudo ln -s /usr/share/gcc-arm-none-eabi-your-version/bin/arm-none-eabi-size /usr/bin/arm-none-eabi-size
```
