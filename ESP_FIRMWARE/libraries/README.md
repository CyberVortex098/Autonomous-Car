# Libraries for Compiling

This repo includes a `libraries/` folder containing all the dependencies needed to compile `ESP_FIRMWARE`. To set them up, copy the contents of that folder into your own libraries directory:

- **Arduino IDE:** the `libraries/` folder set in *File > Preferences > Sketchbook location*
- **PlatformIO:** the project's `lib/` folder

## Steps

1. Locate the `libraries/` folder in this repo.
2. Copy each library folder inside it into your Arduino/PlatformIO libraries directory.
3. Restart the IDE and verify each library appears under `Sketch > Include Library` (Arduino) or is picked up on next build (PlatformIO).

## Included Libraries

| Library | Used for |
|---|---|
| Adafruit Unified Sensor | Common sensor interface (dependency of all Adafruit drivers below) |
| Adafruit BusIO | I2C/SPI abstraction (dependency of all Adafruit drivers below) |
| Adafruit PWM Servo Driver Library | PCA9685 servo driver |
| Adafruit VL53L0X | Time-of-flight lidar distance sensors (x4, via TCA9548A mux) |
| Adafruit ADS1X15 | ADS1115 ADC |
| Adafruit TCS34725 | Color sensor |
| MPU6050 | IMU (accelerometer/gyroscope) |

> **Note on the TCA9548A mux:** no dedicated library is needed. Channel selection is handled by writing directly to the mux's I2C address over `Wire` before each sensor call.

Built-in libraries (`Wire.h`, `math.h`, `stdlib.h`, `string.h`) are part of the core and require no separate installation.
