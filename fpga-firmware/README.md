# firmware-kr260-eagle-eye

`kr260-eagle-eye` is an FPGA firmware for the Eagle-Eye-AI application that runs on the Kria kr260 or kv260 board made by AMD. 

This file demonstrates how to package files into a .deb package for Ubuntu.

## The firmware-kr260-eagle-eye Directory Structure

- `DEBIAN/`: Contains packaging metadata.
- `/lib/firmware/xilinx/kr260-eagle-eye/`: Contains the fpga firmware files.

## Building the .deb Package

To build the .deb package, run the following command:

`dpkg-deb --build firmware-kr260-eagle-eye`

To install the `firmware-kr260-eagle-eye.deb` package, copy the file on 
the Kria board in /tmp folder. Then run the following command:

`sudo apt install /tmp/firmware-kr260-eagle-eye.deb`

To list the installed files, run:

`dpkg -L firmware-kr260-eagle-eye.deb`


## The Vivado `src` Directory Structure

- `block_design/`: Contains a Vivado TCL script that generates the block design for extensible platform.
- `xdc`: Contains the the Vivado constraints file for RS485 PMOD pins and other external signal.

## How to use the Vivado files

Get the Official Kria Vitis Platform repository

```
git clone --recursive --branch xlnx_rel_v2022.1 https://github.com/Xilinx/kria-vitis-platforms.git
cd kria-vitis-platforms/kv260
```

Copy generated block design tcl script from platform building over old one here:

```
cp block_design/config_bd.tcl  kria-vitis-platforms/kv260/platforms/vivado/kv260_ispMipiRx_vcu_DP/scripts/config_bd.tcl
```

Copy constraint file pin.xdc from platform ditectory over old one here

`cp xdc/pin.xdc kria-vitis-platforms/kv260/platforms/vivado/kv260_ispMipiRx_vcu_DP/xdc/pin.xdc`

Build the overlay - it takes abour 4 hours

```
cd kria-vitis-platforms/kv260

make clean
make overlay OVERLAY=smartcam
```

The generated bitfile and xclbin will be located at:

```
 overlays/examples/smartcam/binary_container_1/link/int/system.bit
 overlays/examples/smartcam/binary_container_1/dpu.xclbin
```

Convert system.bit to a .bit.bin file:

```
cd overlays/examples/smartcam/binary_container_1/link/int/

echo 'all:{system.bit}'>bootgen.bif
bootgen -w -arch zynqmp -process_bitstream bin -image bootgen.bif

ls -l system.bit.bin
cd -

cp overlays/examples/smartcam/binary_container_1/link/int/system.bit.bin ./kr260-eagle-eye.bit.bin
cp overlays/examples/smartcam/binary_container_1/dpu.xclbin ./kr260-eagle-eye.xclbin
```

## Generate Device Tree

Get the original smartcam dtsi

`wget https://raw.githubusercontent.com/Xilinx/kria-apps-firmware/xlnx_rel_v2022.1/boards/kv260/smartcam/kv260-smartcam.dtsi`

Open the `kv260-smartcam.dtsi` file and Add the content of the `update_dtsi.txt` file in the same place where the unused already commented out audio module is. Remove the commented out audio module.

`vi kv260-smartcam.dtsi`

Compile dtb with dtc

`dtc -@ -I dts -O dtb -o kr260-eagle-eye.dtbo kv260-smartcam.dtsi`

