# Morimoto-XBT-Controller
Morimoto XBT master CANbus controller

This is a master controller library used on Teensy 4.0 using FlexCAN_T4 library.
It communicates to the ESP32 slave device over CANbus to control the LED colors on the Morimoto XBT controllers (up to 4 can be connected to the ESP32 at same time).
All commands and requests are using single frames. For the state of connections or to detect new XBT controllers, the ESP32 will stream back the responses in ISOTP format.

All the XBT communications is done on the ESP32 end, while allowing Teensy to control the colors on all 3 ports of each XBT.

The led struct allows you to assign ports for the given color. If you are running rock lights or headlight mods, you can assign the struct to control 1 or more ports.
Writing the struct allows that port to change to the color of your choice. This gives better control over the colors and has automation in mind via vehicle data and code.


Bonus, compared to their phone app:

1) No phone needed
2) When powering up an XBT module, the 3 ports default to all on white. The ESP32 detects this POR after connecting to it when found and automatically shuts them off.
3) Color retention on ESP32 resets. The ESP32 downloads the current color data after reconnecting to the XBT modules.
4) Auto off. If you don't set colors or run keepAlive() within 3 seconds, the ESP32 automatically closes the led channels, to prevent drain.
5) Colors can be set to static (remote color), fade(builtin XBT), or strobe(builtin XBT), just like the phone app, via the struct, automated
6) LEDs controlled by CAN data! RPM's? Temperature? Ambient lights (doors)? VSS ?
7) Add pots to have physical color changing control on your ride! (no current rgbw controllers offer this on the market!)
8) FastLED can be used to sequence colors in a multi port setup!
9) Connect to found XBT controllers, control their colors! Phone pairing requires placing phone on XBT directly, the ESP32 can connect to it in visible BLE range, no contact!
10) MAC addresses of your XBT connections are stored on ESP32 SPIFFS, so reprogramming the ESP32 or rebooting it retains XBT connections and restores connections on boot!
11) yielding support for prioritizing color control. If two objects are writing to the same channel and one has yielding enabled, it will not change the channel color state until the other yielded color set times out --> https://youtu.be/lSP--44Ucbo Demonstrates 2 objects yielding with 3 writing. 3 and 2 second timeout, once both timeout the color control is returned to the global color changer :)
