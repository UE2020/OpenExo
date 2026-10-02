## Controller-settings readback

The Python GUI's live controller values and CSV settings require **both Teensy
and Nano** firmware from the controller-readback revision. Bulk Apply still uses
the existing `f`/`a` parameter-update protocol; readback adds BLE `Q`/`p` and UART
`0x1C`/`0x1D`. Flashing only one board is insufficient. No SD controller-file
changes are required.

With Arduino CLI and the board packages installed, compile from the repository root:

```sh
arduino-cli lib install SD
arduino-cli compile --fqbn teensy:avr:teensy41 --libraries Libraries ExoCode
arduino-cli compile --fqbn arduino:mbed_nano:nano33ble --libraries Libraries ExoCode
```

The SD library is needed by the shared firmware headers when compiling the Nano.
Follow the flashing steps below for each board. Hardware validation should check
live readback after startup, a multi-property Apply (including both sides), and a
recorded CSV with small PID gains; builds and host smoke runs do not prove delivery
over the physical BLE/UART links.

## Downloads
1. Download Arduino IDE 1.8.19, which can be found [here](https://www.arduino.cc/en/software) or [here](https://drive.google.com/drive/folders/1IRxJFNm2gxUtCeU8Dcavg_Mv2ubANOHK?usp=drive_link)
2. Now install Teensyduino, which can be found [here](https://www.pjrc.com/teensy/td_download.html) or [here](https://drive.google.com/drive/folders/1IRxJFNm2gxUtCeU8Dcavg_Mv2ubANOHK?usp=drive_link). Be sure to install the 1.56 version for your system.
3. Install the Arduino Nano 33 BLE through the Arduino Boards Manager.  
3. Install the Arduino libraries according to [Libraries](#libraries).

## Flashing
Now that you have all of the tools installed, lets flash.
1. Connect your computer to the Teensy using a micro-USB cable. 
2. Change the Board to the Teensy 4.1
3. Change the Arduino port to use the Teensyduino port. The correct port will have the board name to the right of the COM (Windows Only) port.
4. Make sure that the Board Version is properly set in the Config.h
5. Press the Arrow button on the Arduino IDE to upload the code to the Teensy. 
6. Change the Board to the Arduino Nano 33 BLE.
7. Change the Arduino port in the same manner as step 3. 
8. Press the upload button. Note: The Nano will take much longer to build than the Teensy. 

## Libraries
All of the files in the libraries directory of this codebase should be copied to
C:\Users\\\[USER\]\Documents\Arduino\libraries\ or system equivalent. This will
'install' the libraries on your system. 
Details on the libraries can be found in the [Libraries Folder](/Libraries).

## Arduino Boards Manager
To open the boards manager press:
Tools -> Board -> Boards Manager

