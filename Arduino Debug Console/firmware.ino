#include <LiquidCrystal.h>

LiquidCrystal lcd_1(12, 11, 5, 4, 3, 2);

String inputBuffer = "";

void setup()
{
  lcd_1.begin(16, 2);
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(13,HIGH);
  analogWrite(9,0);
}

void displayWrapped(const String &msg)
{
  lcd_1.clear();
  lcd_1.setCursor(0, 0);
  lcd_1.print(msg.substring(0, 16));           // row 0: chars 0-15

  if (msg.length() > 16) {
    lcd_1.setCursor(0, 1);
    unsigned int end = min((unsigned int)msg.length(), 32u);
    lcd_1.print(msg.substring(16, end));       // row 1: chars 16-31
  }
}

void loop()
{
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        displayWrapped(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}
