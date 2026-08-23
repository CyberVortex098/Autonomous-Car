# Libraries for Compiling
This repo includes a libraries/ folder containing all the dependencies needed to compile Arduino Debug Console. To set them up, copy the contents of that folder into your own libraries directory:

- Arduino IDE: the libraries/ folder set in File > Preferences > Sketchbook location
- PlatformIO: the project's lib/ folder

## Steps
- Locate the libraries/ folder in this repo.
- Copy each library folder inside it into your Arduino/PlatformIO libraries directory.
- Restart the IDE and verify each library appears under Sketch > Include Library (Arduino) or is picked up on next build (PlatformIO).

## Use

This library is needed to use the 16x2 Liquid Crystal Display to display the serial message.
