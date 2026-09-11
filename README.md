# Local modifications relative to upstream

This working copy was compared against [`naubiomech/OpenExo`](https://github.com/naubiomech/OpenExo) at commit `523852aaa184e9b3cf6b67ffcb4316a07c09ba0a` (2026-06-11). Its local changes include:

- Reworked CAN receive handling for bilateral operation: motors register dedicated receive slots, frames are routed by motor ID and frame format, and CAN transmit/receive diagnostics are tracked. Motor enable-state handling and fault recovery were also corrected. (By Dr. Moon)
- Hardened BLE communication: the parser now validates input and output bounds, resets incomplete state after timeouts or reconnects, and the message queue is a 32-entry circular FIFO with safer synchronization. (By Dr. Moon)
- Hardened UART and real-time I2C framing: added payload validation, overflow and malformed-escape recovery, partial-frame handling, a Teensy UART receive buffer, and consistent I2C packet sizing and validation. (By Dr. Moon)
- Made timer/context management rollover-safe and deterministic, and updated joint error reporting to drain queued errors and clear latched motor errors outside trials. (By Dr. Moon)
- Added local SD-card capture material under `SDCard_asfound_20260825/` and retained `.bak` copies of several modified firmware source files in `ExoCode/src/`. (By Dr. Moon)
- Added debug logging (behind `SIMPLE_DEBUG` flag) which emits motor telemetry every 500 ms. This is what I used to find the wiring issue on the right motor. (By Tarryk)

The upstream README content follows.

# Teensy Nano Board

![Diagram](/Documentation/Figures/Code_Structure.png)

This code uses two microcontrollers to control wearable robots. In order to
build the and flash the codebase check [here](Documentation/BUILD_AND_FLASH.md) and [here](https://github.com/naubiomech/TeensyNanoExoCode/blob/main/Documentation/README.md#how-to-deploy). If you would like to
contribute check [here](CONTRIBUTING.md) for more details. 
 
Check out the [Documentation Folder](/Documentation) for more details and C++ info.

Documentation on the software can be found on our [read the docs page](https://theopenexo.readthedocs.io/en/latest/index.html).

Documentation on the hardware can be found on our [wiki](https://wiki.theopenexo.org).

Video walkthroughs on getting started and indepth looks at different aspects of the system can be found on the OpenExo [YouTube page](https://www.youtube.com/@TheOpenExo). 
