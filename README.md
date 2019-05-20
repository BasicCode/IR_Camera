# Thermal Infrared Camera using AMG8833 board

A very simple demonstration of the AMG8833 board (from Adafruit) using a PIC 18F24J50 MPU and generic (eBay) SPI LCD display.<br>

* Uses the Adafruit AMG8833 breakbout board found here: https://www.adafruit.com/product/3538
* Uses my previous ST7735 library: https://github.com/BasicCode/ST7735-generic

## Installation
Pin connections are shown below, and I'll add KiCad files soon.

| AMG8833 Pin | PIC 18F24J50 Pin |
| :--------- | :---------- |
| **Vin** | **+5V** |
| **3Vo** | NC |
| **GND** | **GND** |
| **SDA** | **26** |
| **SCL** | **25** |
| **INT** | NC |
<br><br>
| LCD Pin | PIC 18F24J50 Pin |
| :--------- | :---------- |
| **VCC** | **+5V** |
| **GND** | **GND** |
| **CS** | **13**|
| **RESET** | **12** |
| **A0** | **11** |
| **SDA** | **2** |
| **SCK** | **3** |
| **LED** | **3.3V** |

## ... More to come
