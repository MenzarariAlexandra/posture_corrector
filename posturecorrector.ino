#include <Wire.h>
#include <Servo.h>
#include <math.h>

// ============ PINI ============
#define MPU_ADDR      0x68
#define SERVO_PIN     7      // MOTOR pe pin 7 (în loc de buzzer pe D8)
#define BTN_CALIB     2      // Buton calibrare
#define BTN_POWER     4      // Buton on/off

// Accelerometre analogice HW-142
#define LEFT_X        A0
#define LEFT_Y        A1
#define LEFT_Z        A2
#define RIGHT_X       A3
#define RIGHT_Y       A6
// RIGHT_Z pe A7 opțional

// ============ PRAGURI ============
#define DEVIATION_THRESHOLD  15.0   // 15% deviație
#define CALIB_HOLD_TIME      7000   // 7 secunde hold
#define CALIB_SAMPLES        50     // Număr sample-uri calibrare

// ============ VARIABILE GLOBALE ============
Servo myServo;

// Stare sistem
bool systemON = false;
bool isCalibrated = false;

// Butoane
unsigned long btn1PressStart = 0;
bool btn1WasPressed = false;

// Baseline-uri (valori calibrate)
struct Baseline {
  float mpuRoll;
  float mpuPitch;
  float leftX, leftY, leftZ;
  float rightX, rightY, rightZ;
} baseline;

// Valori live
float roll = 0, pitch = 0;
float alpha = 0.96;  // Filtru complementar
unsigned long lastTime = 0;

// Servo vibrație
int servoPos = 90;
unsigned long lastServoToggle = 0;
bool alertActive = false;

// ============ SETUP ============
void setup() {
  Serial.begin(9600);
  
  // Inițializare pini
  pinMode(BTN_CALIB, INPUT);
  pinMode(BTN_POWER, INPUT);
  
  myServo.attach(SERVO_PIN);
  myServo.write(90);  // Poziție neutră
  
  Wire.begin();
  
  // Wake up MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);
  
  lastTime = millis();
  
  // Stabilizare filtru MPU
  for (int i = 0; i < 30; i++) {
    readMPU();
    delay(20);
  }
  roll = 0;
  pitch = 0;
  
  Serial.println("=== SISTEM GATA ===");
  Serial.println("Buton ALBASTRU (D4) = ON/OFF");
  Serial.println("Buton ROȘU (D2) = CALIBRARE (hold 7 sec)");
  
  // Beep scurt confirmare pornire
  servoBeep(1, 200);
}

// ============ LOOP PRINCIPAL ============
void loop() {
  handleButtons();
  
  if (!systemON) {
    // Sistem oprit
    if (alertActive) {
      myServo.write(90);
      alertActive = false;
    }
    delay(100);
    return;
  }
  
  if (!isCalibrated) {
    // Nu e calibrat
    delay(100);
    return;
  }
  
  // Citire senzori live
  readMPU();
  float leftVals[3], rightVals[3];
  readAnalogSensors(leftVals, rightVals);
  
  // Calcul deviații
  float mpuDev = computeMPUDeviation();
  float scapulaDev = computeScapulaDeviation(leftVals, rightVals);
  
  float maxDeviation = max(mpuDev, scapulaDev);
  
  // Decizie alertă
  if (maxDeviation > DEVIATION_THRESHOLD) {
    if (!alertActive) {
      Serial.print("ALERTĂ! Deviație: ");
      Serial.print(maxDeviation);
      Serial.println("%");
      alertActive = true;
    }
    vibrateServo(3);  // Vibrații puternice
  } else {
    if (alertActive) {
      Serial.println("Postură corectată!");
      myServo.write(90);
      alertActive = false;
    }
  }
  
  delay(50);
}
// ============ GESTIONARE BUTOANE ============
void handleButtons() {
  // Buton POWER (D4) - Toggle ON/OFF
  static bool lastPowerState = LOW;
  bool powerState = digitalRead(BTN_POWER);
  
  if (powerState == HIGH && lastPowerState == LOW) {
    systemON = !systemON;
    Serial.print("Sistem: ");
    Serial.println(systemON ? "ON" : "OFF");
    
    if (systemON) {
      servoBeep(2, 150);  // 2 beep-uri scurte
    } else {
      servoBeep(1, 300);  // 1 beep lung
    }
    delay(200);  // Debounce
  }
  lastPowerState = powerState;
  
  // Buton CALIBRARE (D2) - Hold 7 secunde
  bool calibState = digitalRead(BTN_CALIB);
  
  if (calibState == HIGH) {
    if (!btn1WasPressed) {
      btn1PressStart = millis();
      btn1WasPressed = true;
      Serial.println("Menține apăsat 7 sec...");
    }
    
    unsigned long holdTime = millis() - btn1PressStart;
    
    if (holdTime >= CALIB_HOLD_TIME && !isCalibrated) {
      // START CALIBRARE
      Serial.println("\n>>> CALIBRARE PORNITĂ <<<");
      servoBeep(1, 200);  // Beep scurt
      
      performCalibration();
      
      Serial.println(">>> CALIBRARE COMPLETĂ <<<\n");
      servoBeep(1, 2000);  // Beep lung 2 sec
      
      btn1WasPressed = false;
    }
  } else {
    btn1WasPressed = false;
  }
}

// ============ CALIBRARE ============
void performCalibration() {
  float sumMpuRoll = 0, sumMpuPitch = 0;
  float sumLX = 0, sumLY = 0, sumLZ = 0;
  float sumRX = 0, sumRY = 0, sumRZ = 0;
  
  Serial.println("Stai drept și relaxat...");
  delay(1000);
  
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    readMPU();
    sumMpuRoll += roll;
    sumMpuPitch += pitch;
    
    float lv[3], rv[3];
    readAnalogSensors(lv, rv);
    sumLX += lv[0];
    sumLY += lv[1];
    sumLZ += lv[2];
    sumRX += rv[0];
    sumRY += rv[1];
    sumRZ += rv[2];
    
    delay(50);
    
    if (i % 10 == 0) {
      Serial.print(".");
    }
  }
  Serial.println();
  
  // Salvare baseline
  baseline.mpuRoll = sumMpuRoll / CALIB_SAMPLES;
  baseline.mpuPitch = sumMpuPitch / CALIB_SAMPLES;
  baseline.leftX = sumLX / CALIB_SAMPLES;
  baseline.leftY = sumLY / CALIB_SAMPLES;
  baseline.leftZ = sumLZ / CALIB_SAMPLES;
  baseline.rightX = sumRX / CALIB_SAMPLES;
  baseline.rightY = sumRY / CALIB_SAMPLES;
  baseline.rightZ = sumRZ / CALIB_SAMPLES;
  
  isCalibrated = true;
  
  Serial.println("Baseline salvat:");
  Serial.print("  MPU Roll: "); Serial.println(baseline.mpuRoll);
  Serial.print("  MPU Pitch: "); Serial.println(baseline.mpuPitch);
}

// ============ CITIRE MPU-6050 ============
void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);
  
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // Skip temp
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // Skip gz
  
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  
  float aR = atan2((float)ay, (float)az) * 57.2958;
  float aP = atan2(-(float)ax, sqrt((float)ay * ay + (float)az * az)) * 57.2958;
  
  roll = alpha * (roll + gx / 131.0 * dt) + (1 - alpha) * aR;
  pitch = alpha * (pitch + gy / 131.0 * dt) + (1 - alpha) * aP;
}

// ============ CITIRE ACCELEROMETRE ANALOGICE ============
void readAnalogSensors(float left[3], float right[3]) {
  left[0] = analogRead(LEFT_X);
  left[1] = analogRead(LEFT_Y);
  left[2] = analogRead(LEFT_Z);
  
  right[0] = analogRead(RIGHT_X);
  right[1] = analogRead(RIGHT_Y);
  right[2] = 0;  // RIGHT_Z opțional pe A7
}
[2/13/2026 12:31 AM] Nicu: // ============ CALCUL DEVIAȚIE MPU ============
float computeMPUDeviation() {
  float deltaRoll = abs(roll - baseline.mpuRoll);
  float deltaPitch = abs(pitch - baseline.mpuPitch);
  
  float baselineMag = sqrt(baseline.mpuRoll * baseline.mpuRoll + 
                           baseline.mpuPitch * baseline.mpuPitch);
  
  if (baselineMag < 0.1) baselineMag = 1.0;  // Evită diviziune cu 0
  
  float deviation = sqrt(deltaRoll * deltaRoll + deltaPitch * deltaPitch);
  return (deviation / baselineMag) * 100.0;
}

// ============ CALCUL DEVIAȚIE SCAPULĂ ============
float computeScapulaDeviation(float left[3], float right[3]) {
  // Deviație stânga
  float deltaLX = abs(left[0] - baseline.leftX);
  float deltaLY = abs(left[1] - baseline.leftY);
  float deltaLZ = abs(left[2] - baseline.leftZ);
  
  float baselineLeftMag = sqrt(baseline.leftX * baseline.leftX + 
                               baseline.leftY * baseline.leftY + 
                               baseline.leftZ * baseline.leftZ);
  
  if (baselineLeftMag < 1.0) baselineLeftMag = 1.0;
  
  float leftDev = sqrt(deltaLX*deltaLX + deltaLY*deltaLY + deltaLZ*deltaLZ);
  float leftPercent = (leftDev / baselineLeftMag) * 100.0;
  
  // Deviație dreapta
  float deltaRX = abs(right[0] - baseline.rightX);
  float deltaRY = abs(right[1] - baseline.rightY);
  
  float baselineRightMag = sqrt(baseline.rightX * baseline.rightX + 
                                baseline.rightY * baseline.rightY);
  
  if (baselineRightMag < 1.0) baselineRightMag = 1.0;
  
  float rightDev = sqrt(deltaRX*deltaRX + deltaRY*deltaRY);
  float rightPercent = (rightDev / baselineRightMag) * 100.0;
  
  // Asimetrie stânga-dreapta
  float asymmetry = abs(leftPercent - rightPercent);
  
  return max(max(leftPercent, rightPercent), asymmetry);
}

// ============ VIBRAȚII SERVO ============
void vibrateServo(int intensity) {
  unsigned long now = millis();
  int amplitude, speed;
  
  switch (intensity) {
    case 1:
      amplitude = 25;
      speed = 100;
      break;
    case 2:
      amplitude = 45;
      speed = 60;
      break;
    case 3:
      amplitude = 70;
      speed = 35;
      break;
    default:
      return;
  }
  
  if (now - lastServoToggle > (unsigned long)speed) {
    servoPos = (servoPos == 90 + amplitude) ? 90 - amplitude : 90 + amplitude;
    myServo.write(servoPos);
    lastServoToggle = now;
  }
}

// ============ BEEP SERVO (simulare buzzer) ============
void servoBeep(int count, int duration) {
  for (int i = 0; i < count; i++) {
    myServo.write(60);
    delay(duration / 2);
    myServo.write(120);
    delay(duration / 2);
  }
  myServo.write(90);
}
