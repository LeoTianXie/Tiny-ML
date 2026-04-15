/*
 * Board:   Arduino Nano 33 BLE Sense Rev2
 * Libraries required:
 *   - PDM          (built-in with Mbed OS Nano board package)
 *   - Arduino_APDS9960
 *   - Arduino_BMI270_BMM150
 *
 * Threshold rationale:
 *   MIC_THRESHOLD     100   — speech/clap easily exceeds 100.
 *   CLEAR_DARK_THRESH 50   — my desktop's ambient light is around 67,
 *                            so 50 is dark.
 *   MOTION_THRESHOLD  0.15  — detects shaking of the board.
 *   PROX_NEAR_THRESH  200    — hand within ~10 cm reliably exceeds 200.
 */

#include <PDM.h>
#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>
#include <math.h>

//  Thresholds 
static const int   MIC_THRESHOLD     = 100;
static const int   CLEAR_DARK_THRESH = 50;
static const float MOTION_THRESHOLD  = 0.15f;
static const int   PROX_NEAR_THRESH  = 200;

//  PDM microphone globals 
static short     sampleBuffer[256];
volatile int     samplesRead = 0;

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

//  Setup 
void setup() {
  Serial.begin(115200);
  delay(1500);

  // Microphone
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {
    Serial.println("ERROR: Failed to start PDM microphone.");
    while (1);
  }

  // IMU
  if (!IMU.begin()) {
    Serial.println("ERROR: Failed to initialize IMU.");
    while (1);
  }

  // APDS9960 (light + proximity)
  if (!APDS.begin()) {
    Serial.println("ERROR: Failed to initialize APDS9960.");
    while (1);
  }

  Serial.println("Workspace classifier ready. (115200 baud)");
}

//  Loop 
void loop() {

  //  1. Microphone: compute average absolute amplitude 
  static int micLevel = 0;
  if (samplesRead > 0) {
    long sum = 0;
    for (int i = 0; i < samplesRead; i++) {
      sum += abs(sampleBuffer[i]);
    }
    micLevel = (int)(sum / samplesRead);
    samplesRead = 0;
  }

  //  2. Ambient brightness: APDS9960 clear channel 
  static int clearVal = 0;
  if (APDS.colorAvailable()) {
    int r, g, b, c;
    APDS.readColor(r, g, b, c);
    clearVal = c;
  }

  //  3. Motion: deviation of acceleration magnitude from 1 g 
  float motionMetric = 0.0f;
  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    float mag = sqrt(ax * ax + ay * ay + az * az);
    motionMetric = fabs(mag - 1.0f);
  }

  //  4. Proximity: APDS9960 (0 = close, 255 = far) 
  static int proxVal = 0;
  if (APDS.proximityAvailable()) {
    proxVal = APDS.readProximity();
  }

  //  Binary decisions 
  int sound  = (micLevel    > MIC_THRESHOLD)     ? 1 : 0;
  int dark   = (clearVal    < CLEAR_DARK_THRESH) ? 1 : 0;
  int moving = (motionMetric > MOTION_THRESHOLD) ? 1 : 0;
  int near   = (proxVal     < PROX_NEAR_THRESH)  ? 1 : 0;

  //  Rule-based situation classification 
  const char* label;

  if (sound && !dark && moving && near) {
    label = "NOISY_BRIGHT_MOVING_NEAR";
  } else if (!sound && dark && !moving && near) {
    label = "QUIET_DARK_STEADY_NEAR";
  } else if (sound && !dark && !moving && !near) {
    label = "NOISY_BRIGHT_STEADY_FAR";
  } else {
    label = "QUIET_BRIGHT_STEADY_FAR";
  }

  // Serial output
  Serial.print("raw,mic=");    Serial.print(micLevel);
  Serial.print(",clear=");     Serial.print(clearVal);
  Serial.print(",motion=");    Serial.print(motionMetric, 3);
  Serial.print(",prox=");      Serial.println(proxVal);

  Serial.print("flags,sound="); Serial.print(sound);
  Serial.print(",dark=");        Serial.print(dark);
  Serial.print(",moving=");      Serial.print(moving);
  Serial.print(",near=");        Serial.println(near);

  Serial.print("state,");      Serial.println(label);

  delay(500);
}
