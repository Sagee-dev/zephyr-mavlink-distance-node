# mavlink-distance-node

A portable MAVLink v2 fake distance sensor node implemented on Zephyr RTOS. Simulates four directional distance sensors (front, right, back, left) and transmits readings to a flight controller using standard MAVLink DISTANCE_SENSOR messages over UART.

Designed to be platform-agnostic. Any board supported by Zephyr with a UART peripheral can run this node without modifying the application or MAVLink layers.

---

## Overview

The node simulates four distance sensors that independently oscillate between 50 cm and 200 cm at different speeds, producing a triangle wave pattern per sensor. A MAVLink HEARTBEAT is sent at 1 Hz and four DISTANCE_SENSOR messages are sent at 10 Hz. The flight controller receives and fuses the sensor data as if connected to real hardware.

Verified against Pixhawk 4 running both ArduPilot and PX4 firmware.

---

## Hardware

The reference hardware is the NUCLEO-F401RE but any Zephyr-supported board with UART works.

Board: NUCLEO-F401RE
Microcontroller: STM32F401RET6
CPU Core: ARM Cortex-M4 with FPU
System Clock: 16 MHz HSI internal oscillator
Flash: 512 KB
SRAM: 96 KB

---

## Communication Settings

UART Peripheral: USART2
TX Pin: PA2, CN10 connector, labelled UART2_TX
RX Pin: PA3, CN10 connector, labelled UART2_RX
Baud Rate: 57600
Frame Format: 8N1
Flow Control: None
Protocol: MAVLink v2

---

## Wiring to Flight Controller

Connect three wires between the Nucleo CN10 header and the Pixhawk 4 TELEM port. TX and RX must be crossed.

Nucleo CN10 UART2_TX connects to Pixhawk TELEM RX
Nucleo CN10 UART2_RX connects to Pixhawk TELEM TX
Nucleo CN10 GND connects to Pixhawk TELEM GND

The Pixhawk 4 TELEM port uses a 6-pin JST-GH connector. Only pins TX, RX, and GND are used. VCC and flow control pins are left unconnected.

---

## MAVLink Configuration

System ID: 1
Component ID: 196 (MAV_COMP_ID_OBSTACLE_AVOIDANCE)
MAVLink Version: v2, start byte 0xFD

HEARTBEAT is sent at 1 Hz.
DISTANCE_SENSOR is sent at 10 Hz for each of the four sensors.

Sensor mapping:

Sensor ID 0, Front, MAV_SENSOR_ROTATION_NONE, orientation value 0
Sensor ID 1, Right, MAV_SENSOR_ROTATION_YAW_90, orientation value 2
Sensor ID 2, Back, MAV_SENSOR_ROTATION_YAW_180, orientation value 4
Sensor ID 3, Left, MAV_SENSOR_ROTATION_YAW_270, orientation value 6

---

## Software Architecture

Three independent layers. Each layer has one responsibility and communicates only with the layer directly below it.

Application Layer, src/main.c
Scheduling, sensor simulation, and work item management. Uses Zephyr delayable work items on the system workqueue. No UART or MAVLink protocol details present in this layer.

MAVLink Layer, src/mavlinkLayer.c and src/mavlinkLayer.h
MAVLink message packing and serialisation using c_library_v2. Produces uart_packet_t instances and passes them to the UART layer via uart_queue_up(). No knowledge of UART hardware or Zephyr internals.

UART Layer, src/uartLayer.c and src/uartLayer.h
UART initialisation, message queue, and TX thread. The queue and TX thread are private to this layer. External callers use uart_init() and uart_queue_up() only. Uses Zephyr UART driver API exclusively with no register-level access.

---

## Scheduling Design

No mutex is used in this project. The Zephyr system workqueue is single-threaded by default, so the heartbeat and distance work items never execute concurrently. The UART message queue is thread-safe by design.

Heartbeat work item fires every 1000 ms and reschedules itself.
Distance work item fires every 100 ms and reschedules itself.
UART TX thread blocks on the queue and drains packets as they arrive.

---

## Sensor Simulation

Each sensor follows an independent triangle wave between 50 cm and 200 cm.

Sensor 0, Front, step size 2 cm per cycle, slow
Sensor 1, Right, step size 5 cm per cycle, medium
Sensor 2, Back, step size 8 cm per cycle, fast
Sensor 3, Left, step size 3 cm per cycle, medium slow

At 10 Hz update rate each sensor independently oscillates. The flight controller observes four distance channels changing at visibly different rates, confirming independent sensor operation.

---

## Project Structure

```
mavlink-distance-node/
    src/
        main.c
        mavlinkLayer.c
        mavlinkLayer.h
        uartLayer.c
        uartLayer.h
    c_library_v2/               (MAVLink submodule)
        common/
            mavlink.h
    boards/
        nucleo_f401re.overlay
    test/
        testscript.py           (pymavlink verification script)
    CMakeLists.txt
    prj.conf
    Doxyfile
    Doc/                        (Doxygen generated documentation)
    README.md
```

---

## Cloning

This repository uses MAVLink c_library_v2 as a Git submodule. You must initialise the submodule after cloning or the build will fail with missing headers.

Clone the repository and initialise the submodule in one command:
```
git clone --recurse-submodules https://github.com/yourrepo/mavlink-distance-node.git
```

If you already cloned without the submodule flag:
```
cd mavlink-distance-node
git submodule update --init --recursive
```

Verify the submodule was initialised correctly — the following directory must exist and must not be empty:
```
c_library_v2/common/mavlink.h
```

To update the MAVLink library to the latest version:
```
git submodule update --remote c_library_v2
git add c_library_v2
git commit -m "Update MAVLink c_library_v2 submodule"
```

---

## Building

Toolchain required: Zephyr SDK and west

Build for NUCLEO-F401RE:
```
west build -b nucleo_f401re
```

Flash:
```
west flash
```

To build for a different board, replace nucleo_f401re with your board name and update boards/your_board.overlay with the correct UART node label and speed.

---

## prj.conf

```
# enable uart
CONFIG_SERIAL=y
CONFIG_UART_INTERRUPT_DRIVEN=y

# disable debug console — keeps UART output clean for MAVLink
CONFIG_PRINTK=n
CONFIG_STDOUT_CONSOLE=n
```

printk and stdout console are disabled intentionally. Both share the same UART as MAVLink. Leaving them enabled corrupts the MAVLink byte stream with debug text.

---

## Board Overlay

```
&usart2 {
    status = "okay";
    current-speed = <57600>;
};
```

---

## CMakeLists.txt

```
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(fake_sensor_node)

target_include_directories(app PRIVATE c_library_v2/)

target_sources(app PRIVATE
    src/main.c
    src/uartLayer.c
    src/mavlinkLayer.c
)
```

---

## Timing Budget

Four DISTANCE_SENSOR packets per 100ms cycle at 57600 baud:

4 sensors x 26 bytes x 10 bits = 1040 bits
1040 divided by 57600 = 18.06 ms transmission time
100 ms window = 18 percent bandwidth utilisation

Queue depth of 100 packets provides headroom for any scheduling delay.

---

## Testing

A verification script is provided in the test/ folder.

Install pymavlink:
```
pip install pymavlink
```

Connect the Nucleo board via USB. Find the device port:
```
dmesg | tail -20
```

Run the test script:
```
python3 test/testscript.py
```

Expected output:

HEARTBEAT every 1000 ms with system_status 4 and mavlink_version 3.
Four DISTANCE_SENSOR messages every 100 ms with orientation values 0, 2, 4, 6
and independently changing distance values between 50 and 200.

The script connects to /dev/ttyACM0 at 57600 baud. If your device appears
on a different port, edit the connection string in test/testscript.py.

---

## Documentation

Generate Doxygen HTML documentation:
```
doxygen Doxyfile
```

Open Doc/html/index.html in a browser.

---

## Portability

To run on a different board:

1. Update boards/your_board.overlay with the correct UART node and speed
2. Change DT_NODELABEL(usart2) in src/uartLayer.c to match your UART node label
3. Run west build -b your_board_name
4. No changes needed in src/main.c or src/mavlinkLayer.c

---

## Author

W S G De Silva
M.Sc. Embedded Systems
Technische Universitat Chemnitz
Faculty of Electrical Engineering and Information Technology

---

## License

MIT License. See LICENSE file for details.

---

## References

MAVLink Developer Guide: https://mavlink.io/en/
MAVLink c_library_v2: https://github.com/mavlink/c_library_v2
DISTANCE_SENSOR message: https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
Zephyr RTOS Documentation: https://docs.zephyrproject.org
Zephyr Work Queue API: https://docs.zephyrproject.org/latest/kernel/services/scheduling/workqueue.html
STM32F401RE Reference Manual RM0368: https://www.st.com/resource/en/reference_manual/rm0368-stm32f401xbc-and-stm32f401xde-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
ArduPilot MAVLink Documentation: https://ardupilot.org/dev/docs/mavlink-basics.html
