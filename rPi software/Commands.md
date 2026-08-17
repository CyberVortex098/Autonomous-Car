## Commands on serial for the ESP 

**Transport:** UART, 921600 baud, newline-terminated (`\n`), comma-separated fields

---

**RECEIVED**

| Message | Format | Fields | Limits |
|---|---|---|---|
| `$IMU` | `$IMU,gx,gy,gz,ax,ay,az,roll,pitch,yaw` | Gyro X/Y/Z, Accel X/Y/Z, Roll, Pitch, Yaw | — |
| `$DIST` | `$DIST,d0,d1,d2,d3` | Distance sensors 0–3 | ≥ 0 mm |
| `$COLOR` | `$COLOR,lux,r,g,b` | Lux, Red %, Green %, Blue % | Lux ≥ 0, RGB 0–100 % |
| `$ADC` | `$ADC,adc0,adc1,adc2` | ADC channels 0–2 | 0–100 % |
| `$BUTTON` | `$BUTTON,btn1,btn2` | Button 1, Button 2 | 0 or 1 |

---

**SENT**

| Command | Format | Example | Limits |
|---|---|---|---|
| LED | `$LED,<led1>,<led2>` | `$LED,1,0` | Each: 0 or 1 |
| Beep | `$BEEP,<freq>,<dur>` | `$BEEP,1000,200` | Freq: 50–10000 Hz, Dur: 10–5000 ms |
| Servo | `$SERVO,<ch>,<angle>,<mode>,<speed>` | `$SERVO,2,135,1,100` | Ch: 0–15, Angle: 0–180°, Mode: 1/2/3, Speed: positive integer  |
| Move | `$MOVE,<x>,<y>` | `$MOVE,50,-30` | X, Y: -100 to +100 |
| Rotate | `$ROTATE,<speed>,0` | `$ROTATE,25,0` | Speed: -100 to +100, |

---

**Servo Modes**
- `1` — Blocking ramp-up; in practice smoother than `3`
- `2` — Direct (instant jump to angle)
- `3` — Non-blocking ramp-up
