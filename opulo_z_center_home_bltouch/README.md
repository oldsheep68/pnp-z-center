# opulo_z_center_home_bltouch
# `SW for opulo z-axis with a center HOME (G28) function`
# `the combined W and Z axis will always be in middle position after homing`
# `in addition a bltouch can be controlled and a touch event will stop the move`
# `after that, the Z or W position can be read by gcode command`


## This is a Rust repository
To compile it, you must install Rust first, follow this link:
https://www.rust-lang.org/tools/install

As the project is an embedded one, more additions are needed, at least those and probably some more:

rustup target install thumbv7m-none-eabi

cargo install probe-run

cargo install flip-link  (recomended)

The embedded book has lot's of good informations:
https://doc.rust-lang.org/stable/embedded-book/


## Build it
cargo build --release

## Download and run it
cargo run --release

## Connection for downloading
ST-Link V2 should be enough

## This software is intended to run on an old modified index PCBA (Mai 2021 "index-mobo-rev03"), with a STM32F103V mounted on it
Lots of IOs have moved on the mobo since then. Here I list the IO's used in this software.

//
// Pinnig for MOBO REV3 (Mai21) with STM32F103 and additional modifications necessary
// Z_DIR:     PB1
// Z_STEP:    PE7
// Z_EN:      PE9
// Z_Limit:   PD14  => used for light detection
// A_EN =>    PA3 wired to Z:MS2_CS_UART (0 Ohm resistor) [driver pin 12]
//               needs to be 3.3V for A49xx and tree-state for TMC2208/2209

// blTouch is connected to
// Servo1     PB6  => control pin
// A_Limit:   PC9  => used as IRQ, when the probe triggers

The STM32F103 needs an additional pull-up resistor, for the USB interface to work properly
=> please check out the Bluepill schematic. There are some further modifications on
PCB level, for the stm32f103 to work properly (C6 is 0Ohm Resistor and probably some traces need to be cut)
I used the F103, because it was difficult to get an STM32F407 (note, the software will not compile for a STM32F407V)

## G-Code Interface
is build as usb-serial interface

## developpmentstate
prof of idea

# openpnp configurations
Driver: ttyACMx (on linux)
Baud: 115200
parity: None
Data Bits: Eight
Stop Bits: One
Flow Control: Off

Gcode (driver for blTouch):
ACTUATE_COUBLE_COMMAND:  M280 P0 S{DoubleValue}
ACTUATE_BOOLEAN_COMMAND: M280 P0 {True: S10}{False: S160}
ACTUATE_STRING_COMMAND: {StringValue}  // most probably not needed
ACTUATOR_READ_COMMAND: M489
ACTUATOR_READ_REGEX: (?<Value>-?\d+)

Acutuator settings:
Value Type: Double
ON Value 1.000
OFF Value 16.000
// this is due to the fact, that openpnp-python interface always uses the double actuator, as of my experiance

Gcode (driver for motor control)
G28        Home to the center (light barrier position)
G0 Z{absolute position in mm} F{max speed in mm/sec}     
                  Speed settings stay till
                  next home or next rewrite
                  and are valied for W and Z axis
G0 W{absolute position in mm}                            
                  W and Z are independent logical axis
                  which are obviously connected with the belt
M203 W{offset in mm}
                 To adjust, so that both axes W and Z can same altitude
                 positions on the pick and place machine
M114             Read positions of W and Z

M115             Read FW version


# Binary image
not available, does not make to much sense, with this manual modified HW

License
-------

[0-clause BSD license](LICENSE-0BSD.txt).
