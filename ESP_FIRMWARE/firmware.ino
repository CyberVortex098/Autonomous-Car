#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_PWMServoDriver.h>

#define BASE 0        // Actuator Channel on the driver
#define GRIPPER 1     // Actuator Channel on the driver
#define SHOULDER 2    // Actuator Channel on the driver
#define W_ROT 3       // Actuator Channel on the driver
#define W_PIT 4       // Actuator Channel on the driver
#define ELBOW 5       // Actuator Channel on the driver
#define CAM_YAW 6     // Actuator Channel on the driver
#define CAM_PIT 7     // Actuator Channel on the driver
#define MOTOR_BL 12   // Actuator Channel on the driver
#define MOTOR_BR 14   // Actuator Channel on the driver
#define MOTOR_FL 13   // Actuator Channel on the driver
#define MOTOR_FR 15   // Actuator Channel on the driver

#define BUTTON_PIN   4     // Should be color sensor interupt
#define BUTTON2_PIN  19    // reported over serial only, does NOT touch the LEDs
#define LED1_PIN     17    // LED on arm tip
#define LED2_PIN     0     // strapping pin, fine once booted, white led
#define BUZZER_PIN   18    // passive buzzer, driven via standard tone()

#define TCA9548A_ADDR   0x71   // A0 tied high -> 0x71
#define PCA9685_ADDR    0x40   // main bus, not muxed
#define ADS1115_ADDR    0x48   // 4-channel ADC I2C Address 

#define SERVO_COUNT   16  // Channels on PCA9685 (0-15, end inclusive)

const float threshold_lidar=3501;

const uint8_t VL_CHANNELS[4] = {7, 6, 5, 4};  // Channel for the LIDARs on the Mux
const uint8_t TCS_CHANNEL    = 3;             // Channel for the color sensor on the Mux 

// ================= Sensor / Driver Instances =================

Adafruit_MPU6050       mpu;                                                                               // Object
Adafruit_ADS1115       ads;                                                                               // Object
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA9685_ADDR);                                      // Object
Adafruit_VL53L0X       vl53[4];                                                                           // Object
Adafruit_TCS34725      tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_4X);         // Object

// ================= Servo config =================

struct ServoState {
  float current;   // deg
  float target;    // deg
  float speed;     // deg/sec, used only while sweeping
  bool  sweeping;  // boolean to activate sweeping
};

ServoState servos[SERVO_COUNT];

bool ledState1 = false;
bool ledState2 = false;
bool sensor_health=true;
bool vl53_ready[4] = {false, false, false, false};

const uint32_t LIDAR_PERIOD  = 28;   // ~35.5 Hz
const uint32_t COLOR_PERIOD  = 50;   // ~20 Hz
const uint32_t ADC_PERIOD    = 17;   // ~58.8 Hz
const uint32_t BUTTON_PERIOD = 20;   // 50 Hz
const uint32_t IMU_PERIOD    = 10;   // 100 Hz
const uint32_t SERVO_PERIOD  = 20;   // 50 Hz

uint32_t tLidar = 0, tColor = 0, tAdc = 0, tButton = 0, tImu = 0, tServo = 0;

void servoWriteDeg(uint8_t channel, float deg);

// ================= Multiplexer Helper =================
void muxSelect(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ================= MPU6050 Data =================
float roll = 0, pitch = 0, yaw = 0;
uint32_t lastMPUus = 0;

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// Rotation matrix that maps raw sensor-frame readings into the level body frame.
// Built once at boot from the measured gravity vector, so a tilted/skewed IMU
// mount is compensated for instead of just averaged out as a "bias".
float mountR[3][3] = {
  {1,0,0},
  {0,1,0},
  {0,0,1}
};

float lastGX = 0, lastGY = 0, lastGZ = 0;
float lastAX = 0, lastAY = 0, lastAZ = 0;

// Builds mountR such that mountR * a_measured (normalized) = (0,0,1).
// Assumes the robot is resting level (chassis-level, not necessarily IMU-level)
// at the moment mpu6050Calibrate() runs.
void computeMountRotation(float ax, float ay, float az) {
  float mag = sqrt(ax*ax + ay*ay + az*az);
  if (mag < 1e-6) mag = 1e-6;
  float from[3] = { ax/mag, ay/mag, az/mag };
  float to[3]   = { 0, 0, 1 };

  float e = from[0]*to[0] + from[1]*to[1] + from[2]*to[2];
  float f = fabs(e);

  if (f > 1.0 - 1e-4) {
    // from and to are (near) parallel or antiparallel -- the standard
    // cross-product formula is singular here (this is exactly the case
    // for a Z-inverted mount). Use a robust double-Householder-reflection
    // construction instead, which has no singularity in this regime.
    float x[3];
    float afx = fabs(from[0]), afy = fabs(from[1]), afz = fabs(from[2]);
    if (afx < afy) {
      if (afx < afz) { x[0]=1; x[1]=0; x[2]=0; }
      else           { x[0]=0; x[1]=0; x[2]=1; }
    } else {
      if (afy < afz) { x[0]=0; x[1]=1; x[2]=0; }
      else           { x[0]=0; x[1]=0; x[2]=1; }
    }

    float u[3] = { x[0]-from[0], x[1]-from[1], x[2]-from[2] };
    float v[3] = { x[0]-to[0],   x[1]-to[1],   x[2]-to[2]   };

    float uu = u[0]*u[0]+u[1]*u[1]+u[2]*u[2];
    float vv = v[0]*v[0]+v[1]*v[1]+v[2]*v[2];
    float uv = u[0]*v[0]+u[1]*v[1]+u[2]*v[2];

    float c1 = (uu > 1e-8) ? 2.0/uu : 0.0;
    float c2 = (vv > 1e-8) ? 2.0/vv : 0.0;
    float c3 = c1*c2*uv;

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        mountR[i][j] = (i==j ? 1.0 : 0.0) - c1*u[i]*u[j] - c2*v[i]*v[j] + c3*v[i]*u[j];

  } else {
    // Well-conditioned case: standard cross-product formula.
    float v[3] = {
      from[1]*to[2] - from[2]*to[1],
      from[2]*to[0] - from[0]*to[2],
      from[0]*to[1] - from[1]*to[0]
    };
    float h = 1.0 / (1.0 + e);

    mountR[0][0] = e + h*v[0]*v[0];
    mountR[0][1] = h*v[0]*v[1] - v[2];
    mountR[0][2] = h*v[0]*v[2] + v[1];

    mountR[1][0] = h*v[0]*v[1] + v[2];
    mountR[1][1] = e + h*v[1]*v[1];
    mountR[1][2] = h*v[1]*v[2] - v[0];

    mountR[2][0] = h*v[0]*v[2] - v[1];
    mountR[2][1] = h*v[1]*v[2] + v[0];
    mountR[2][2] = e + h*v[2]*v[2];
  }
}

void rotateVec(const float in[3], float out[3]) {
  out[0] = mountR[0][0]*in[0] + mountR[0][1]*in[1] + mountR[0][2]*in[2];
  out[1] = mountR[1][0]*in[0] + mountR[1][1]*in[1] + mountR[1][2]*in[2];
  out[2] = mountR[2][0]*in[0] + mountR[2][1]*in[1] + mountR[2][2]*in[2];
}

void mpu6050Calibrate() {
  const int N = 200;
  double sax = 0, say = 0, saz = 0, sgx = 0, sgy = 0, sgz = 0;
  sensors_event_t a, g, temp;

  for (int i = 0; i < N; i++) {
    mpu.getEvent(&a, &g, &temp);
    sax += a.acceleration.x / SENSORS_GRAVITY_STANDARD;
    say += a.acceleration.y / SENSORS_GRAVITY_STANDARD;
    saz += a.acceleration.z / SENSORS_GRAVITY_STANDARD;
    sgx += g.gyro.x * 180.0 / PI;
    sgy += g.gyro.y * 180.0 / PI;
    sgz += g.gyro.z * 180.0 / PI;
  }

  float gravX = sax / N, gravY = say / N, gravZ = saz / N;
  gyroBiasX  = sgx / N; gyroBiasY  = sgy / N; gyroBiasZ  = sgz / N;

  // Derive the mount-tilt correction from the measured gravity direction.
  computeMountRotation(gravX, gravY, gravZ);
}

void mpu6050Update() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float axRaw = a.acceleration.x / SENSORS_GRAVITY_STANDARD;
  float ayRaw = a.acceleration.y / SENSORS_GRAVITY_STANDARD;
  float azRaw = a.acceleration.z / SENSORS_GRAVITY_STANDARD;

  float gxRaw = (g.gyro.x * 180.0 / PI) - gyroBiasX;
  float gyRaw = (g.gyro.y * 180.0 / PI) - gyroBiasY;
  float gzRaw = (g.gyro.z * 180.0 / PI) - gyroBiasZ;

  // Transform sensor-frame readings into the level body frame.
  float accIn[3]  = { axRaw, ayRaw, azRaw };
  float accOut[3];
  rotateVec(accIn, accOut);

  float gyroIn[3] = { gxRaw, gyRaw, gzRaw };
  float gyroOut[3];
  rotateVec(gyroIn, gyroOut);

  float axg = accOut[0], ayg = accOut[1], azg = accOut[2];
  float gxds = gyroOut[0], gyds = gyroOut[1], gzds = gyroOut[2];

  lastAX = axg; lastAY = ayg; lastAZ = azg;
  lastGX = gxds; lastGY = gyds; lastGZ = gzds;

  float accelRoll  = atan2(ayg, azg) * 180.0 / PI;
  float accelPitch = atan2(axg, sqrt(ayg * ayg + azg * azg)) * 180.0 / PI;

  uint32_t now = micros();
  float dt = (lastMPUus == 0) ? 0.01 : (now - lastMPUus) / 1000000.0;
  lastMPUus = now;

  roll  = 0.94 * (roll  + gxds * dt) + 0.06 * accelRoll;
  pitch = 0.94 * (pitch + gyds * dt) + 0.06 * accelPitch;
  yaw  += gzds * dt;

  // NOTE: IMU is no longer used for drive feedback. $MOVE / $ROTATE now set
  // motor speeds directly and hold them until the next command -- yaw is
  // still tracked here for telemetry ($IMU line) but nothing closes the
  // loop on it anymore.
}

// ================= Buttons =================
bool lastButtonState = HIGH;
bool lastButton2State = HIGH;

void buttonPoll() {
  static uint32_t lastDebounce = 0;
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState && millis() - lastDebounce > 50) {
    lastDebounce = millis();
    lastButtonState = reading;
  }

  static uint32_t lastDebounce2 = 0;
  bool reading2 = digitalRead(BUTTON2_PIN);
  if (reading2 != lastButton2State && millis() - lastDebounce2 > 50) {
    lastDebounce2 = millis();
    lastButton2State = reading2;
  }
}

// ================= ADS1115 (PGA: +-4.096V) =================
float adsScaled(uint8_t channel) {
  int16_t raw = ads.readADC_SingleEnded(channel);
  // Full scale range is +-4.096V (32767 count = 4.096V)
  // Scaling relative to 3.3V max input voltage:
  float voltage = ads.computeVolts(raw);
  float pct = (voltage / 3.3f) * 100.0f;
  return constrain(pct, 0.0f, 100.0f);
}

// ================= Servo Control =================
void servoWriteDeg(uint8_t channel, float deg) {
  deg = constrain(deg, 0.0, 180.0);
  if(channel==3){
    pca.setPWM(channel, 0, map(deg,0,180,150,570));
  } else {
    pca.setPWM(channel, 0, map(deg,0,180,140,510));
  }
}

void servosUpdate(float dt) {
  for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
    if (!servos[ch].sweeping) continue;
    float maxStep = servos[ch].speed * dt;
    float diff = servos[ch].target - servos[ch].current;
    if (fabs(diff) <= maxStep) {
      servos[ch].current = servos[ch].target;
      servos[ch].sweeping = false;
    } else {
      servos[ch].current += (diff > 0 ? maxStep : -maxStep);
    }
    servoWriteDeg(ch, servos[ch].current);
  }
}

// ================= Drive Motor Control =================

void driveMecanum(float vx, float vy) {
  vx = map(constrain(vx, -100.0, 100.0),-100,100,-90,90);
  vy = map(constrain(vy, -100.0, 100.0),-100,100,-90,90);

  float tmag=fabsf(vx)+fabsf(vy);

  if(tmag>90){
    float magnitude=90.0f/tmag;
    vx*=magnitude;
    vy*=magnitude;
  }

  float fl = 90 + (vy + vx);
  float fr = 90 - (vy - vx);
  float bl = 90 + (vy - vx);
  float br = 90 - (vy + vx);

  servoWriteDeg(MOTOR_FL,fl);
  servoWriteDeg(MOTOR_FR,fr);
  servoWriteDeg(MOTOR_BL,bl);
  servoWriteDeg(MOTOR_BR,br);
}

void driveRotate(float speed) {
  speed = constrain(speed, -100.0, 100.0);
  servoWriteDeg(MOTOR_FL,(90+map(speed,-100,100,-90,90)));
  servoWriteDeg(MOTOR_FR,(90+map(speed,-100,100,-90,90)));
  servoWriteDeg(MOTOR_BL,(90+map(speed,-100,100,-90,90)));
  servoWriteDeg(MOTOR_BR,(90+map(speed,-100,100,-90,90)));
}

// ================= Serial Command Parsing =================
char serialBuf[64];
uint8_t serialIdx = 0;

void processCommand(char *line) {
  char *tok = strtok(line, ",");
  if (!tok) return;

  if (strcmp(tok, "$SERVO") == 0) {
    char *chStr     = strtok(NULL, ",");
    char *targetStr = strtok(NULL, ",");
    char *sweepStr  = strtok(NULL, ",");
    char *speedStr  = strtok(NULL, ",");
    if (!chStr || !targetStr || !sweepStr || !speedStr) return;

    int ch = atoi(chStr);
    if (ch < 0 || ch >= SERVO_COUNT) return;
    float target = constrain((float)atof(targetStr), 0.0, 180.0);
    bool sweep = (strcmp(sweepStr, "true") == 0);
    float speed = atof(speedStr);

    if (sweep) {
      servos[ch].target = target;
      servos[ch].speed = (speed > 0) ? speed : 60.0;
      servos[ch].sweeping = true;
    } else {
      servos[ch].current = target;
      servos[ch].target = target;
      servos[ch].sweeping = false;
      servoWriteDeg(ch, target);
    }
  } else if (strcmp(tok, "$LED") == 0) {
    char *l1 = strtok(NULL, ",");
    char *l2 = strtok(NULL, ",");
    if (!l1 || !l2) return;
    ledState1 = atoi(l1) ? true : false;
    ledState2 = atoi(l2) ? true : false;
    digitalWrite(LED1_PIN, ledState1);
    digitalWrite(LED2_PIN, ledState2);
  } else if (strcmp(tok, "$BEEP") == 0) {
    char *freqStr = strtok(NULL, ",");
    char *durStr  = strtok(NULL, ",");
    if (!freqStr || !durStr) return;

    float freq = atof(freqStr);
    uint32_t dur = (uint32_t)atol(durStr);
    if (freq <= 0 || dur == 0) return;

    tone(BUZZER_PIN, (unsigned int)freq, dur);
  } else if (strcmp(tok, "$MOVE") == 0) {
    // $MOVE,xSpeed,ySpeed -- both -100..100.
    // xSpeed: strafe, +ve = right, -ve = left
    // ySpeed: forward/back, +ve = forward, -ve = back
    char *xStr = strtok(NULL, ",");
    char *yStr = strtok(NULL, ",");
    if (!xStr || !yStr) return;

    float vx = constrain((float)atof(xStr), -100.0, 100.0);
    float vy = constrain((float)atof(yStr), -100.0, 100.0);

    driveMecanum(vx, vy);
  } else if (strcmp(tok, "$ROTATE") == 0) {
    // $ROTATE,speed -- -100..100, +ve = CW viewed from above, -ve = CCW.
    // Keeps rotating at this speed until the next $ROTATE/$MOVE.
    char *speedStr = strtok(NULL, ",");
    if (!speedStr) return;

    float speed = constrain((float)atof(speedStr), -100.0, 100.0);

    driveRotate(speed);
  }
}

void serialPoll() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialIdx > 0) {
        serialBuf[serialIdx] = '\0';
        processCommand(serialBuf);
        serialIdx = 0;
      }
    } else if (serialIdx < sizeof(serialBuf) - 1) {
      serialBuf[serialIdx++] = c;
    }
  }
}

// ================= Setup & Loop =================
void setup() {
  Serial.begin(921600);
  Wire.begin();

  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  noTone(BUZZER_PIN);

  // Let I2C rails / sensor regulators (VL53L0X, TCS34725 breakouts especially)
  // settle before the first transaction. The ESP32 boots fast enough that
  // hitting begin() immediately after Wire.begin() can catch a sensor mid
  // power-up, which reports a false failure even on healthy hardware. Each
  // sensor below also gets a few retries as a second layer of defense.
  delay(300);

  // MPU6050
  {
    bool ok = false;
    for (uint8_t attempt = 0; attempt < 3 && !ok; attempt++) {
      ok = mpu.begin();
      if (!ok) delay(50);
    }
    if (ok) {
      mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
      mpu.setGyroRange(MPU6050_RANGE_250_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      mpu6050Calibrate();
    }
  }

  // ADS1115 -> Set gain to GAIN_ONE (+-4.096V range)
  {
    bool ok = false;
    for (uint8_t attempt = 0; attempt < 3 && !ok; attempt++) {
      ok = ads.begin(ADS1115_ADDR);
      if (!ok) delay(50);
    }
    if (ok) {
      ads.setGain(GAIN_ONE);
    } 
  }

  // VL53L0X
  for (uint8_t i = 0; i < 4; i++) {
    bool ok = false;
    for (uint8_t attempt = 0; attempt < 3 && !ok; attempt++) {
      muxSelect(VL_CHANNELS[i]);
      ok = vl53[i].begin();
      if (!ok) delay(50);
    }
    if (ok) {
      vl53[i].startRangeContinuous();
      vl53_ready[i] = true;
    } else {
      sensor_health = false;
    }
  }

  // TCS34725
  {
    bool ok = false;
    for (uint8_t attempt = 0; attempt < 3 && !ok; attempt++) {
      muxSelect(TCS_CHANNEL);
      ok = tcs.begin();
      if (!ok) delay(50);
    }
  }

  // PCA9685 Servos
  pca.begin();
  pca.setPWMFreq(50.0);
  for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
    servos[ch].current = 90;
    servos[ch].target = 90;
    servos[ch].speed = 60;
    servos[ch].sweeping = false;
    servoWriteDeg(ch, 90); // Set to active 90 deg position on boot (also = motor stop)
  }
  for(int i=0;i<91;i++){
    servoWriteDeg(3,(90+i));
  }

  if(sensor_health==false){
    for(int i=0;i<8;i++){
      tone(BUZZER_PIN,400,400);
      delay(200);
    }
  } else {
    // O4 a, O5 d c — repeated 3x, L8 (125ms)
    tone(BUZZER_PIN, 440, 125); delay(125); // A4
    tone(BUZZER_PIN, 587, 125); delay(125); // D5
    tone(BUZZER_PIN, 523, 125); delay(125); // C5

    tone(BUZZER_PIN, 440, 125); delay(125); // A4
    tone(BUZZER_PIN, 587, 125); delay(125); // D5
    tone(BUZZER_PIN, 523, 125); delay(125); // C5

    tone(BUZZER_PIN, 440, 125); delay(125); // A4
    tone(BUZZER_PIN, 587, 125); delay(125); // D5
    tone(BUZZER_PIN, 523, 125); delay(125); // C5

    // L16 dcdcdcdc (63ms each), octave still 5
    tone(BUZZER_PIN, 587, 63); delay(63); // D5
    tone(BUZZER_PIN, 523, 63); delay(63); // C5
    tone(BUZZER_PIN, 587, 63); delay(63); // D5
    tone(BUZZER_PIN, 523, 63); delay(63); // C5
    tone(BUZZER_PIN, 587, 63); delay(63); // D5
    tone(BUZZER_PIN, 523, 63); delay(63); // C5
    tone(BUZZER_PIN, 587, 63); delay(63); // D5
    tone(BUZZER_PIN, 523, 63); delay(63); // C5
  }
}

void loop() {
  serialPoll();

  uint32_t now = millis();

  // IMU Loop
  if (now - tImu >= IMU_PERIOD) {
    tImu = now;
    mpu6050Update();
    Serial.print("$IMU,");
    Serial.print(lastGX, 2); Serial.print(",");
    Serial.print(lastGY, 2); Serial.print(",");
    Serial.print(lastGZ, 2); Serial.print(",");
    Serial.print(lastAX, 3); Serial.print(",");
    Serial.print(lastAY, 3); Serial.print(",");
    Serial.print(lastAZ, 3); Serial.print(",");
    Serial.print(roll, 1);   Serial.print(",");
    Serial.print(pitch, 1);  Serial.print(",");
    Serial.println(yaw, 1);
  }

  // Servo Loop
  if (now - tServo >= SERVO_PERIOD) {
    float dt = (now - tServo) / 1000.0;
    tServo = now;
    servosUpdate(dt);
  }

  // Button Loop
  if (now - tButton >= BUTTON_PERIOD) {
    tButton = now;
    buttonPoll();
    Serial.print("$BUTTON,");
    Serial.print(digitalRead(BUTTON_PIN));
    Serial.print(",");
    Serial.println(digitalRead(BUTTON2_PIN));
  }

  // ADC Loop
  if (now - tAdc >= ADC_PERIOD) {
    tAdc = now;
    Serial.print("$ADC,");
    Serial.print(adsScaled(0), 1); Serial.print(",");
    Serial.print(adsScaled(1), 1); Serial.print(",");
    Serial.print(adsScaled(2), 1); Serial.print(",");
    Serial.println(adsScaled(3), 1);
  }

  // Distance (VL53L0X) Loop
  if (now - tLidar >= LIDAR_PERIOD) {
    tLidar = now;
    uint16_t d[4] = {0, 0, 0, 0};
    for (uint8_t i = 0; i < 4; i++) {
      if (vl53_ready[i]) {
        muxSelect(VL_CHANNELS[i]);
        if (vl53[i].isRangeComplete()) {
          int val=0;
          val = vl53[i].readRangeResult();
          if(val<threshold_lidar){
            d[i]=val;
          } else {
            d[i]=threshold_lidar;
          }
        }
      }
    }
    Serial.print("$DIST,");
    Serial.print(d[0]); Serial.print(",");
    Serial.print(d[1]); Serial.print(",");
    Serial.print(d[2]); Serial.print(",");
    Serial.println(d[3]);
  }

  // Color (TCS34725) Loop
  if (now - tColor >= COLOR_PERIOD) {
    tColor = now;
    muxSelect(TCS_CHANNEL);
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    uint16_t lux = tcs.calculateLux(r, g, b);
    float rPct = (c > 0) ? constrain(r * 100.0f / c, 0, 100) : 0;
    float gPct = (c > 0) ? constrain(g * 100.0f / c, 0, 100) : 0;
    float bPct = (c > 0) ? constrain(b * 100.0f / c, 0, 100) : 0;
    Serial.print("$COLOR,");
    Serial.print(lux);      Serial.print(",");
    Serial.print(rPct, 1); Serial.print(",");
    Serial.print(gPct, 1); Serial.print(",");
    Serial.println(bPct, 1);
  }
}
