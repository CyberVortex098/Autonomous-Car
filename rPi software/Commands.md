## Commands on serial for the ESP 

**Transport:** UART, 921600 baud, newline-terminated (`\n`), comma-separated fields

---

**Received messages**

| Message | Format | Fields | Limits |
|---|---|---|---|
| `$IMU` | `$IMU,gx,gy,gz,ax,ay,az,roll,pitch,yaw` | Gyro X/Y/Z, Accel X/Y/Z, Roll, Pitch, Yaw | — |
| `$DIST` | `$DIST,d0,d1,d2,d3` | Distance sensors 0–3 | ≥ 0 mm |
| `$COLOR` | `$COLOR,lux,r,g,b` | Lux, Red %, Green %, Blue % | Lux ≥ 0, RGB 0–100 % |
| `$ADC` | `$ADC,adc0,adc1,adc2` | ADC channels 0–2 | 0–100 % |
| `$BUTTON` | `$BUTTON,btn1,btn2` | Button 1, Button 2 | 0 or 1 |

---

**Commands**

| Command | Format | Example | Limits |
|---|---|---|---|
| LED | `$LED,<led1>,<led2>` | `$LED,1,0` | Each: 0 or 1 |
| Beep | `$BEEP,<freq>,<dur>` | `$BEEP,1000,200` | Freq: 50–15000 Hz, Dur: >0 ms |
| Servo | `$SERVO,<ch>,<angle>,<mode>,<speed>` | `$SERVO,2,135,1,100` | Ch: 0–15, Angle: 0–180°, Mode: 1/2/3, Speed: positive integer  |
| Move | `$MOVE,<x>,<y>,<rotate>` | `$MOVE,50,-30,0` | X, Y, rotate: -100 to +100 |

---

**Servo Modes**
- `1` — Non-blocking ramp-up
- `2` — Direct (instant jump to angle)
- `3` — Blocking ramp-up; in practice smoother than `1`
