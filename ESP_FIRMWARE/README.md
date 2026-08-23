This is the low-level driver file for the ESP32 module on the robot.

The Raspberry Pi is not a good real-time controller and it can freeze which can be dangerous as the robot might go out of control. Also, processing I2C transactions and other low-level work will consume a decently-sized portion of the resources, which will make it harder to run the high-level planning algorithms.

The ESP32 reads the sensors and prints data on the serial port, as defined in <a href="../rPi software/Commands.md">Commands</a>. This data can be relatively easily parsed by the Pi without much resources.

The ESP32 is a real-time controller and is very unlikely to freeze, and even if it does, it will reset, and then it will work fine.
