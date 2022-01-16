# rust-shed-nano

Embedded rust for the duinotech nano 5v boad (purchased from jaycar).

Requirements:

- Controls 24ch PWM meanwell LDD-300 chips via adafruit 12bit 24ch board
  - Soft on. (Lights gently turn on, as to not blind you)
  - Soft switching of the main power relay for the LDD-300 chips (See non existant drawings)
  - Allow external control of lights via serial interface
- Controls Dust Extractor
  - Start/stop with 5 second hold to enable when VSD has been powered down
  - Automatic powerdown of VSD after extended periods of no use
  - Two speed control toggled with the start button (defaults to high speed at initial turn on)
  - Allow exernal control via serial interface

## Hardware description

The sheds lights are 21 320ma rated 4k (colour temp) 70mm down lights spaced as 3 strips of 7 lights running the length of a standard 20' shipping continer

Each light is logically connected to a seperate channel which consists of a Meanwell LDD-300 (step down DC-DC converter) which interfaces with the main controller (a Duiotech nano 5v board) via an Adafruit 24 channel PWM boad controlled via a SPI interface.

The human interface to the lights are a push button rotary encoder connected to the hardware interrupt on the arduino nano board.

The dust extractor is powered via an Aerotech N34 series fan controlled by a \_\_ VSD. The power supply to the VSD is controlled via a relay.

Physical Start and stop buttons exist for this system (connected to the arduino).

## Design considerations

The hardware supports 12bit resolution for the lighting level, however due to the some human/hardware limitations this is not realized. There are two main considerations in controlling the brightness of a given light. The first is the human perception of brightness is not linear, rather it is a curve descriped by the CIE colour standard, roughly speaking, it says that as a light source gets brighter we need a bigger change in lumens to see any difference.

The second consideration is related to how the hardware updates a brightness. One unfortunate serious limitation of the TLC5947 chip is that when a new grayscale (GS) setting is latched in, the outputs are turned off until the internal counter ticks over to a new 12bit window. There is an inverse relationship between the TLCs output and the LDD-300 output so the result of this is that when latching in new GS values the lights all flash full brightness (a truely painful and aweful thing to look at). The solution to this is to toggle the reset pin on the TLC chip at the same time as latching in new GS values. This dramaticly improves the flashing issue but creates another subtle effect in that during a smooth change in brightness, the actual percieved brightness is effected by how fast (often Hz) the GS values are updated. Ideally these effects can be accounted for by the software.

## Parts:

- Duinotech [nano](https://www.jaycar.com.au/duinotech-nano-board-arduino-compatible/p/XC4414)
- Adafruit 24-Channel 12-bit PWM LED [Driver](https://www.adafruit.com/product/1429)

## Setting up the developoment environment

These instructions are written for a windows 11 computer running WSL2 (kernal version 5.10.60.1) as required by [microsoft](https://devblogs.microsoft.com/commandline/connecting-usb-devices-to-wsl/)

However should work just fine for other Debian based distro's (Ubuntu) just with some steps skipped over/adapted.

<br/>

## Step 1

<br/>

Install [rust](https://www.rust-lang.org/tools/install) using the WLS2 option (windows remember)

###### The programming language

<br/>

Install [usbipid-win](https://github.com/dorssel/usbipd-win/releases)

###### Required to share usb devices with WSL2

<br/>

## Step 2

**INSTALL THE CARGO DEPENDENICES BEFORE DOWNGRADING THE TOOLCHAIN TO nightly-2021-01-07**

- [ravedude](https://github.com/Rahix/avr-hal/blob/main/ravedude)

- [avr-hal-template](https://github.com/Rahix/avr-hal-template)

Then continnue to follow the instruction as per [avr-hal](https://github.com/Rahix/avr-hal).

I also found this [documentation](https://rustduino.shivammalhotra.dev/install) (for some course work) quite useful. It outlines some basic principles but also describes setting up the rust toolchain

<br/>

## Step 3

Connect the arduino to a USB port and share the serial device with WSL2 via the
`usbipd wsl list` and `usbipd wsl attach --busid <busid>` as described in the [microsoft docs](https://devblogs.microsoft.com/commandline/connecting-usb-devices-to-wsl/).

(Use an admin powershell console)

In a WSL2 shell check that the usb device exists:

```bash
$ lsusb

# Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
# Bus 001 Device 002: ID 0403:6001 Future Technology Devices International, Ltd FT232 Serial (UART) IC <- THIS IS THE ARDUINO
# Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```

Then check that there is a USB device:

```bash
$ ls /dev/ttyUSB*

# /dev/ttyUSB0
```

And then enable permissions on the above listed usb device

```bash
sudo chmod +777 /dev/ttyUSB0
```

## Step 4

Specify this device when running `RAVEDUDE_PORT=/dev/ttyUSB0 cargo run`

###### Note: The ./cargo/config.toml  file specifes 'uno' as the ravedude run command, I think this is required because of the Duinotech nano using the same bootloader as found on the uno, the legitimate arduino nano uses a newer version.