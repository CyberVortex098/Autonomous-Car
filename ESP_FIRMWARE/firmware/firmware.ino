#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_PWMServoDriver.h>

#define BASE 0                 // Actuator Channel on the driver
#define GRIPPER 1              // Actuator Channel on the driver
#define SHOULDER 2             // Actuator Channel on the driver
#define W_ROT 3                // Actuator Channel on the driver
#define W_PIT 4                // Actuator Channel on the driver
#define ELBOW 5                // Actuator Channel on the driver
#define CAM_YAW 6              // Actuator Channel on the driver
#define CAM_PIT 7              // Actuator Channel on the driver
#define MOTOR_BL 12            // Actuator Channel on the driver
#define MOTOR_BR 14            // Actuator Channel on the driver
#define MOTOR_FL 13            // Actuator Channel on the driver
#define MOTOR_FR 15            // Actuator Channel on the driver
#define BUTTON_PIN   4         // Should be color sensor interupt
#define BUTTON2_PIN  19        // reported over serial only, does NOT touch the LEDs
#define LED1_PIN     17        // LED on arm tip
#define LED2_PIN     0         // strapping pin, fine once booted, white led
#define BUZZER_PIN   18        // passive buzzer, driven via standard tone()
#define TCA9548A_ADDR   0x71   // A0 tied high -> 0x71
#define PCA9685_ADDR    0x40   // fast bus (GPIO 25/26), not muxed
#define ADS1115_ADDR    0x48   // 4-channel ADC I2C Address, slow bus (GPIO 21/22)
#define MPU_ADDR 0x68          //  MPU6050 Address, fast bus (GPIO 25/26)
#define SERVO_COUNT   16       // Channels on PCA9685 (0-15, end inclusive)
#define AVOIDER_ENABLED 1
#define FORWARD_LIDAR 1
#define BACKWARD_LIDAR 2
#define RIGHT_LIDAR 3
#define LEFT_LIDAR 0

const float threshold_lidar=3501;             // to limit false readings or out-of-range 8192 reading
const float ACC_SENS = 16384.0;
const float GYRO_SENS = 131.0;
const uint8_t VL_CHANNELS[4] = {7, 6, 5, 4};  // Channel for the LIDARs on the Mux
const uint8_t TCS_CHANNEL    = 3;             // Channel for the color sensor on the Mux 
const uint32_t LIDAR_PERIOD  = 28;            // 35.5 Hz    to boost, running far below, library seems to not use continious mode but instead blocking
const uint32_t COLOR_PERIOD  = 105;           // 9.5 Hz     
const uint32_t ADC_PERIOD    = 17;            // 58.8 Hz
const uint32_t BUTTON_PERIOD = 20;            // 50 Hz
const uint32_t IMU_PERIOD    = 4;            // 250hz
const uint32_t SERVO_PERIOD  = 1;             // 1000 Hz
uint32_t tLidar = 0, tColor = 0, tAdc = 0, tButton = 0, tImu = 0, tServo = 0;
uint32_t lastMPUus = 0;
uint8_t serialIdx = 0;
unsigned long lastTime;
float roll = 0, pitch = 0, yaw = 0;
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
float ALPHA = 0.98;
float lastGX = 0, lastGY = 0, lastGZ = 0;
float lastAX = 0, lastAY = 0, lastAZ = 0;
float rollOffset = 0;
float pitchOffset = 0;
volatile float VX=0;   // only ever touched from core 1 (fastwork/driveMecanum) in this split
volatile float VY=0;   // no cross-core mutex needed, but kept volatile for safety
volatile float SPEED=0;
float STOP_THRES = 30.0;
bool ledState1 = false;
bool ledState2 = false;
bool sensor_health=true;
volatile bool vl53_ready[4] = {false, false, false, false}; // write-once at setup(), read-only after -> safe unprotected
volatile uint16_t d[4] = {0,0,0,0}; // move to global scope -- WRITTEN on core 0 (slowwork), READ on core 1 (fastwork) -> protect with dMux
bool lastButtonState = HIGH;
bool lastButton2State = HIGH;                
char serialBuf[64];                          

// ===== I2C buses =====
// Wire    (GPIO 21/22, default) : slow bus  -- ADS1115, TCA9548A mux -> VL53L0X x4, TCS34725
// I2C_FAST(GPIO 25/26)          : fast bus  -- MPU6050, PCA9685
TwoWire I2C_FAST = TwoWire(1);

Adafruit_ADS1115       ads;                                                                               // Object, Wire (slow bus)
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA9685_ADDR, I2C_FAST);                            // Object, I2C_FAST (fast bus)
Adafruit_VL53L0X       vl53[4];                                                                           // Object, Wire (slow bus, muxed)
Adafruit_TCS34725      tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_4X);         // Object, Wire (slow bus, muxed)

struct ServoState {
  float current;   // deg
  float target;    // deg
  float speed;     // deg/sec, used only while sweeping
  bool  sweeping;  // boolean to activate sweeping
};

// ===== Dual-core sync primitives =====
portMUX_TYPE dMux = portMUX_INITIALIZER_UNLOCKED;   // protects d[] (written core0, read core1)
SemaphoreHandle_t serialMux;                        // protects Serial.print() from interleaving across cores

TaskHandle_t fastTaskHandle = NULL;
TaskHandle_t slowTaskHandle = NULL;

ServoState servos[SERVO_COUNT];

void servoWriteDeg(uint8_t channel, float deg);

void muxSelect(uint8_t channel) {
  // TCA9548A lives on the slow bus (Wire), only ever called from slowworkTask (core 0)
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void mpuWrite(uint8_t reg, uint8_t val){
    I2C_FAST.beginTransmission(MPU_ADDR);
    I2C_FAST.write(reg);
    I2C_FAST.write(val);
    I2C_FAST.endTransmission();
}

void mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz){
  I2C_FAST.beginTransmission(MPU_ADDR);
  I2C_FAST.write(0x3B);
  I2C_FAST.endTransmission(false);
  I2C_FAST.requestFrom(MPU_ADDR, 14, true);

  ax = I2C_FAST.read() << 8 | I2C_FAST.read();
  ay = I2C_FAST.read() << 8 | I2C_FAST.read();
  az = I2C_FAST.read() << 8 | I2C_FAST.read();
  I2C_FAST.read(); I2C_FAST.read(); // temp, skip
  gx = I2C_FAST.read() << 8 | I2C_FAST.read();
  gy = I2C_FAST.read() << 8 | I2C_FAST.read();
  gz = I2C_FAST.read() << 8 | I2C_FAST.read();
}

void mpu6050Calibrate() {
  // runs only during setup(), single-threaded
  Serial.println("Calibrating gyro, keep still...");
  long sx = 0, sy = 0, sz = 0;
  const int N = 500;
  int16_t ax, ay, az, gx, gy, gz;
  double sAccRoll = 0, sAccPitch = 0;

  for(int i = 0; i < N; i++){
      mpuReadRaw(ax, ay, az, gx, gy, gz);
      sx += gx; sy += gy; sz += gz;

      float axg = ax / ACC_SENS;
      float ayg = ay / ACC_SENS;
      float azg = az / ACC_SENS;

      sAccRoll  += atan2(ayg, azg) * 180.0 / PI;
      sAccPitch += atan2(-axg, sqrt(ayg*ayg + azg*azg)) * 180.0 / PI;

      delay(2);
  }
  gyroXoffset = sx / (float)N;
  gyroYoffset = sy / (float)N;
  gyroZoffset = sz / (float)N;
  rollOffset  = sAccRoll  / N;
  pitchOffset = sAccPitch / N;

  Serial.println("Calibration done.");
}

void mpu6050Update() {
  // called at runtime from fastwork (core 1) -- MPU is on I2C_FAST, which core 0
  // never touches, so no mutex needed here anymore.
  int16_t ax, ay, az, gx, gy, gz;
  mpuReadRaw(ax, ay, az, gx, gy, gz);

  lastAX = ax; lastAY = ay; lastAZ = az;
  lastGX = gx; lastGY = gy; lastGZ = gz;

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  float axg = lastAX / ACC_SENS;
  float ayg = lastAY / ACC_SENS;
  float azg = lastAZ / ACC_SENS;

  float gxds = (lastGX - gyroXoffset) / GYRO_SENS;
  float gyds = (lastGY - gyroYoffset) / GYRO_SENS;
  float gzds = (lastGZ - gyroZoffset) / GYRO_SENS;

  float accRoll  = atan2(ayg, azg) * 180.0 / PI - rollOffset;
  if (accRoll > 180.0)  accRoll -= 360.0;
  if (accRoll < -180.0) accRoll += 360.0;

  float accPitch = atan2(-axg, sqrt(ayg*ayg + azg*azg)) * 180.0 / PI - pitchOffset;
  if (accPitch > 180.0)  accPitch -= 360.0;
  if (accPitch < -180.0) accPitch += 360.0;

  roll  = ALPHA * (roll  + gxds * dt) + (1 - ALPHA) * accRoll;
  pitch = ALPHA * (pitch + gyds * dt) + (1 - ALPHA) * accPitch;
  yaw   += gzds * dt;
}

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

float adsScaled(uint8_t channel) {
  // called from slowwork (core 0) -- ADS1115 is on Wire, only touched by core 0
  int16_t raw = ads.readADC_SingleEnded(channel);
  float voltage = ads.computeVolts(raw);
  // Full scale range is +-4.096V (32767 count = 4.096V)
  // Scaling relative to 3.3V max input voltage:
  float pct = (voltage / 3.3f) * 100.0f;
  return constrain(pct, 0.0f, 100.0f);
}

void servoWriteDeg(uint8_t channel, float deg) {
  // PCA9685 is only ever called from core 1 (servosUpdate, driveMecanum,
  // processCommand $SERVO) and lives on I2C_FAST, which core 0 never touches.
  deg = constrain(deg, 0.0, 180.0);
  if(channel==3){
    pca.setPWM(channel, 0, map(deg,0,180,140,555));
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

void driveMecanum(float vx, float vy, float speed) {
  VX=vx;
  VY=vy;
  SPEED=speed;
  vx = map(constrain(vx, -100.0, 100.0),-100,100,-90,90);
  vy = map(constrain(vy, -100.0, 100.0),-100,100,-90,90);
  speed = map(constrain(speed, -100.0, 100.0),-100,100,-90,90);

  float tmag=fabsf(vx)+fabsf(vy)+fabsf(speed);
  if(tmag>90){
    float magnitude=90.0f/tmag;
    vx*=magnitude;
    vy*=magnitude;
    speed*=magnitude;
  }

  float fl = 90 + (vy + vx) + speed;
  float fr = 90 - (vy - vx) + speed;
  float bl = 90 + (vy - vx) + speed;
  float br = 90 - (vy + vx) + speed;

  servoWriteDeg(MOTOR_FL,fl);
  servoWriteDeg(MOTOR_FR,fr);
  servoWriteDeg(MOTOR_BL,bl);
  servoWriteDeg(MOTOR_BR,br);
}

void initServosToBoot(int ch,int deg){
  servos[ch].current = deg;
  servos[ch].target = deg;
  servos[ch].speed = 60;
  servos[ch].sweeping = false;
  servoWriteDeg(ch, deg); // Set to active 90 deg position on boot (also = motor stop)
}

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
    int sweep = atof(sweepStr);
    float speed = atof(speedStr);

    if (sweep==1) {
      servos[ch].target = target;
      servos[ch].speed = (speed > 0) ? speed : 60.0;
      servos[ch].sweeping = true;
    } else if(sweep==2){
      servos[ch].current = target;
      servos[ch].target = target;
      servos[ch].sweeping = false;
      servoWriteDeg(ch, target);
    } else if(sweep==3){
      servos[ch].target = target;
      servos[ch].sweeping = true;
      int step = (target >= servos[ch].current) ? 1 : -1;
      for(int i = servos[ch].current; i != target + step; i += step){
          servos[ch].current = i;
          servoWriteDeg(ch, i);
          delay(speed);
      }
      servos[ch].sweeping = false;
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
    char *speedStr = strtok(NULL, ",");
    if (!xStr || !yStr) return;
    
    float vx = constrain((float)atof(xStr), -100.0, 100.0);
    float vy = constrain((float)atof(yStr), -100.0, 100.0);
    float rot_speed = speedStr ? constrain((float)atof(speedStr), -100.0, 100.0) : 0.0f;

    driveMecanum(vx, vy,rot_speed);
  } else if (strcmp(tok, "$STOPTHRES") == 0) {
    char *STR = strtok(NULL, ",");
    if (!STR) return;
    STOP_THRES = constrain((float)atof(STR), 0.0, 3500.0);
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

// ================= Task bodies =================

void fastworkTask(void *pvParameters) {
  for (;;) {
    serialPoll();

    uint32_t now = millis();

    // IMU Loop
    if (now - tImu >= IMU_PERIOD) {
      tImu = now;
      mpu6050Update();

      xSemaphoreTake(serialMux, portMAX_DELAY);
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
      xSemaphoreGive(serialMux);
    }

    // Servo Loop
    if (now - tServo >= SERVO_PERIOD) {
      float dt = (now - tServo) / 1000.0;
      tServo = now;
      servosUpdate(dt);
    }

    // Obstacle avoidance -- snapshot d[] under dMux before comparing
    if(AVOIDER_ENABLED){
      uint16_t dLocal[4];
      portENTER_CRITICAL(&dMux);
      dLocal[0]=d[0]; dLocal[1]=d[1]; dLocal[2]=d[2]; dLocal[3]=d[3];
      portEXIT_CRITICAL(&dMux);

      if(VX > 0 && vl53_ready[RIGHT_LIDAR]  && dLocal[RIGHT_LIDAR]  <= STOP_THRES){
        driveMecanum(0, VY, 0);
        xSemaphoreTake(serialMux, portMAX_DELAY);
        Serial.print("$OBSTACLE,"); Serial.println(RIGHT_LIDAR);
        xSemaphoreGive(serialMux);
      }
      if(VX < 0 && vl53_ready[LEFT_LIDAR]   && dLocal[LEFT_LIDAR]   <= STOP_THRES){
        driveMecanum(0, VY, 0);
        xSemaphoreTake(serialMux, portMAX_DELAY);
        Serial.print("$OBSTACLE,"); Serial.println(LEFT_LIDAR);
        xSemaphoreGive(serialMux);
      }
      if(VY > 0 && vl53_ready[FORWARD_LIDAR]&& dLocal[FORWARD_LIDAR]<= STOP_THRES){
        driveMecanum(VX, 0, 0);
        xSemaphoreTake(serialMux, portMAX_DELAY);
        Serial.print("$OBSTACLE,"); Serial.println(FORWARD_LIDAR);
        xSemaphoreGive(serialMux);
      }
      if(VY < 0 && vl53_ready[BACKWARD_LIDAR]&&dLocal[BACKWARD_LIDAR]<=STOP_THRES){
        driveMecanum(VX, 0, 0);
        xSemaphoreTake(serialMux, portMAX_DELAY);
        Serial.print("$OBSTACLE,"); Serial.println(BACKWARD_LIDAR);
        xSemaphoreGive(serialMux);
      }
    }

    vTaskDelay(1); // yield so the core 1 idle task / watchdog gets serviced
  }
}

void slowworkTask(void *pvParameters) {
  for (;;) {
    uint32_t now = millis();

    // Button Loop
    if (now - tButton >= BUTTON_PERIOD) {
      tButton = now;
      buttonPoll();
      xSemaphoreTake(serialMux, portMAX_DELAY);
      Serial.print("$BUTTON,");
      Serial.print(digitalRead(BUTTON_PIN));
      Serial.print(",");
      Serial.println(digitalRead(BUTTON2_PIN));
      xSemaphoreGive(serialMux);
    }

    // ADC Loop
    if (now - tAdc >= ADC_PERIOD) {
      tAdc = now;
      float a0 = adsScaled(0);
      float a1 = adsScaled(1);
      float a2 = adsScaled(2);
      float a3 = adsScaled(3);
      xSemaphoreTake(serialMux, portMAX_DELAY);
      Serial.print("$ADC,");
      Serial.print(a0, 1); Serial.print(",");
      Serial.print(a1, 1); Serial.print(",");
      Serial.print(a2, 1); Serial.print(",");
      Serial.println(a3, 1);
      xSemaphoreGive(serialMux);
    }

    // Distance (VL53L0X) Loop
    if (now - tLidar >= LIDAR_PERIOD) {
      tLidar = now;
      uint16_t dNew[4];
      for (uint8_t i = 0; i < 4; i++) {
        dNew[i] = d[i]; // keep current value unless we get a fresh reading below
        if (vl53_ready[i]) {
          muxSelect(VL_CHANNELS[i]);
          if (vl53[i].isRangeComplete()) {
            int val = vl53[i].readRangeResult();
            dNew[i] = (val < threshold_lidar) ? val : (uint16_t)threshold_lidar;
          }
        }
      }

      portENTER_CRITICAL(&dMux);
      d[0]=dNew[0]; d[1]=dNew[1]; d[2]=dNew[2]; d[3]=dNew[3];
      portEXIT_CRITICAL(&dMux);

      xSemaphoreTake(serialMux, portMAX_DELAY);
      Serial.print("$DIST,");
      Serial.print(dNew[0]); Serial.print(",");
      Serial.print(dNew[1]); Serial.print(",");
      Serial.print(dNew[2]); Serial.print(",");
      Serial.println(dNew[3]);
      xSemaphoreGive(serialMux);
    }

    // Color (TCS34725) Loop
    if (now - tColor >= COLOR_PERIOD) {
      tColor = now;
      uint16_t r, g, b, c;
      muxSelect(TCS_CHANNEL);
      tcs.getRawData(&r, &g, &b, &c);

      uint16_t lux = tcs.calculateLux(r, g, b);
      float rPct = (c > 0) ? constrain(r * 100.0f / c, 0, 100) : 0;
      float gPct = (c > 0) ? constrain(g * 100.0f / c, 0, 100) : 0;
      float bPct = (c > 0) ? constrain(b * 100.0f / c, 0, 100) : 0;

      xSemaphoreTake(serialMux, portMAX_DELAY);
      Serial.print("$COLOR,");
      Serial.print(lux);      Serial.print(",");
      Serial.print(rPct, 1); Serial.print(",");
      Serial.print(gPct, 1); Serial.print(",");
      Serial.println(bPct, 1);
      xSemaphoreGive(serialMux);
    }

    vTaskDelay(1); // yield so core 0's idle task / watchdog gets serviced
  }
}

// ================= Setup & Loop =================
void setup() {
  Serial.begin(921600);

  Wire.begin(21, 22);         // slow bus: ADS1115, TCA9548A mux -> VL53L0X x4, TCS34725
  Wire.setClock(400000);

  I2C_FAST.begin(25, 26);     // fast bus: MPU6050, PCA9685
  I2C_FAST.setClock(400000);

  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  noTone(BUZZER_PIN);

  delay(300);

  serialMux = xSemaphoreCreateMutex();

  mpuWrite(0x6B, 0x00); // wake up
  mpuWrite(0x1A, 0x01); // DLPF ~184Hz
  mpuWrite(0x1B, 0x00); // gyro ±250dps
  mpuWrite(0x1C, 0x00); // accel ±2g
  delay(100);
  mpu6050Calibrate();

  ALPHA=0.02;              // initial settling
  for(int i=0;i<=20;i++){
    mpu6050Update();
  }
  ALPHA=0.98;

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

  initServosToBoot(12, 90);
  initServosToBoot(13, 90);
  initServosToBoot(14, 90);
  initServosToBoot(15, 90);

  initServosToBoot(6,110);
  initServosToBoot(7,90);

  initServosToBoot(0, 73);
  initServosToBoot(1, 120);
  initServosToBoot(2, 90);
  initServosToBoot(3, 180);
  initServosToBoot(4, 112);
  initServosToBoot(5, 160);

  if(sensor_health==false){
    for(int i=0;i<8;i++){
      tone(BUZZER_PIN,400,400);
      delay(200);
    }
  } else {
    noTone(BUZZER_PIN); delay(4);
    tone(BUZZER_PIN, 440); delay(125); noTone(BUZZER_PIN); delay(4); // A4
    tone(BUZZER_PIN, 587); delay(125); noTone(BUZZER_PIN); delay(4);// D5
    tone(BUZZER_PIN, 523); delay(125); noTone(BUZZER_PIN); delay(4); // C5

    tone(BUZZER_PIN, 440); delay(125); noTone(BUZZER_PIN); delay(4); // A4
    tone(BUZZER_PIN, 587); delay(125); noTone(BUZZER_PIN); delay(4); // D5
    tone(BUZZER_PIN, 523); delay(125); noTone(BUZZER_PIN); delay(4); // C5

    tone(BUZZER_PIN, 440); delay(125); noTone(BUZZER_PIN); delay(4); // A4
    tone(BUZZER_PIN, 587); delay(125); noTone(BUZZER_PIN); delay(4); // D5
    tone(BUZZER_PIN, 523); delay(125); noTone(BUZZER_PIN); delay(4); // C5

    tone(BUZZER_PIN, 587); delay(63); noTone(BUZZER_PIN); delay(4); // D5
    tone(BUZZER_PIN, 523); delay(63); noTone(BUZZER_PIN); delay(4); // C5
    tone(BUZZER_PIN, 587); delay(63); noTone(BUZZER_PIN); delay(4); // D5
    tone(BUZZER_PIN, 523); delay(63); noTone(BUZZER_PIN); delay(4); // C5
    tone(BUZZER_PIN, 587); delay(63); noTone(BUZZER_PIN); delay(4); // D5
    tone(BUZZER_PIN, 523); delay(63); noTone(BUZZER_PIN); delay(4); // C5
    tone(BUZZER_PIN, 587); delay(63); noTone(BUZZER_PIN); delay(4); // D5
    tone(BUZZER_PIN, 523); delay(63); noTone(BUZZER_PIN); delay(4); // C5
  }

  lastTime = micros();

  // Launch the two tasks pinned to their cores
  xTaskCreatePinnedToCore(
    fastworkTask, "FastWork", 4096, NULL, 2, &fastTaskHandle, 1 // core 1
  );

  xTaskCreatePinnedToCore(
    slowworkTask, "SlowWork", 4096, NULL, 1, &slowTaskHandle, 0 // core 0
  );
}

void loop() {
  // All work now happens in fastworkTask (core 1) and slowworkTask (core 0).
  // Arduino's loopTask already runs pinned to core 1 -- delete it so it
  // doesn't compete with FastWork for core 1 time.
  vTaskDelete(NULL);
}
