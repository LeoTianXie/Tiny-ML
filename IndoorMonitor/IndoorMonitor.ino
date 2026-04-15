/*
 * Board:     Arduino Nano 33 BLE Sense Rev2
 * Libraries: Arduino_HS300x, Arduino_BMI270_BMM150, Arduino_APDS9960
 *
 * Thresholds:
 *   HUMID_JUMP_THRESH  3.0 %    breath raises RH ~2-5% above ambient
 *   TEMP_RISE_THRESH   0.5 °C   warm air raises temp noticeably above baseline
 *   MAG_SHIFT_THRESH   30  µT   phone or metal object shifts field by this much
 *   LIGHT_CHANGE_FRAC  0.20     >20% change in clear channel from baseline
 *   COOLDOWN_MS        5000     hold event label 5s before re-evaluating
 *   EMA_ALPHA          0.97     slow baseline tracker; ~33 samples ≈ 16s time const
 *                               baseline only updates during BASELINE_NORMAL so
 *                               anomalous readings don't corrupt it
 */

#include <Arduino_HS300x.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_APDS9960.h>
#include <math.h>

static const float         HUMID_JUMP_THRESH = 3.0f;
static const float         TEMP_RISE_THRESH  = 0.5f;
static const float         MAG_SHIFT_THRESH  = 30.0f;
static const float         LIGHT_CHANGE_FRAC = 0.20f;
static const unsigned long COOLDOWN_MS       = 5000;
static const float         EMA_ALPHA         = 0.97f;

// EMA baselines — initialized from first valid set of readings
static float baseRH    = 0, baseTemp = 0, baseMag = 0, baseClear = 0;
static bool  baseInit  = false;

// Cooldown state
static unsigned long cooldownStart = 0;
static bool          inCooldown    = false;
static const char*   activeLabel   = "BASELINE_NORMAL";

void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!HS300x.begin()) {
    Serial.println("ERROR: HS300x init failed.");
    while (1);
  }
  if (!IMU.begin()) {
    Serial.println("ERROR: IMU init failed.");
    while (1);
  }
  if (!APDS.begin()) {
    Serial.println("ERROR: APDS9960 init failed.");
    while (1);
  }

  Serial.println("Indoor monitor ready. (115200 baud)");
}

void loop() {
  // 1. Humidity & temperature
  float rh   = HS300x.readHumidity();
  float temp = HS300x.readTemperature();

  // 2. Magnetometer magnitude (µT)
  static float magVal = 0;
  if (IMU.magneticFieldAvailable()) {
    float mx, my, mz;
    IMU.readMagneticField(mx, my, mz);
    magVal = sqrt(mx*mx + my*my + mz*mz);
  }

  // 3. APDS9960 color channels
  static int r = 0, g = 0, b = 0, clearVal = 0;
  if (APDS.colorAvailable()) {
    APDS.readColor(r, g, b, clearVal);
  }

  // Initialize baselines on the first cycle where all sensors give valid data
  if (!baseInit && rh > 0 && temp > 0 && magVal > 0 && clearVal > 0) {
    baseRH    = rh;
    baseTemp  = temp;
    baseMag   = magVal;
    baseClear = (float)clearVal;
    baseInit  = true;
  }

  // Update EMA baselines — only during BASELINE_NORMAL so events don't
  // bleed into the baseline and self-cancel
  if (baseInit && strcmp(activeLabel, "BASELINE_NORMAL") == 0) {
    baseRH    = EMA_ALPHA * baseRH    + (1.0f - EMA_ALPHA) * rh;
    baseTemp  = EMA_ALPHA * baseTemp  + (1.0f - EMA_ALPHA) * temp;
    baseMag   = EMA_ALPHA * baseMag   + (1.0f - EMA_ALPHA) * magVal;
    baseClear = EMA_ALPHA * baseClear + (1.0f - EMA_ALPHA) * (float)clearVal;
  }

  // Binary event flags
  int humid_jump           = (rh   - baseRH  > HUMID_JUMP_THRESH)                          ? 1 : 0;
  int temp_rise            = (temp - baseTemp > TEMP_RISE_THRESH)                           ? 1 : 0;
  int mag_shift            = (fabs(magVal - baseMag) > MAG_SHIFT_THRESH)                    ? 1 : 0;
  int light_or_color_change = (baseClear > 0 &&
    fabs((float)clearVal - baseClear) / baseClear > LIGHT_CHANGE_FRAC)                      ? 1 : 0;

  // Candidate event (priority: breath > magnetic > light > baseline)
  const char* candidate;
  if (humid_jump || temp_rise) {
    candidate = "BREATH_OR_WARM_AIR_EVENT";
  } else if (mag_shift) {
    candidate = "MAGNETIC_DISTURBANCE_EVENT";
  } else if (light_or_color_change) {
    candidate = "LIGHT_OR_COLOR_CHANGE_EVENT";
  } else {
    candidate = "BASELINE_NORMAL";
  }

  // Cooldown: once an event fires, hold that label for COOLDOWN_MS before
  // re-evaluating so the output doesn't spam the same event every 500ms
  if (inCooldown) {
    if (millis() - cooldownStart >= COOLDOWN_MS) {
      inCooldown  = false;
      activeLabel = candidate;
      if (strcmp(activeLabel, "BASELINE_NORMAL") != 0) {
        inCooldown    = true;
        cooldownStart = millis();
      }
    }
    // else: keep activeLabel as the held event label
  } else {
    activeLabel = candidate;
    if (strcmp(activeLabel, "BASELINE_NORMAL") != 0) {
      inCooldown    = true;
      cooldownStart = millis();
    }
  }

  // Serial output — three lines per cycle
  Serial.print("raw,rh=");   Serial.print(rh, 1);
  Serial.print(",temp=");    Serial.print(temp, 2);
  Serial.print(",mag=");     Serial.print(magVal, 1);
  Serial.print(",r=");       Serial.print(r);
  Serial.print(",g=");       Serial.print(g);
  Serial.print(",b=");       Serial.print(b);
  Serial.print(",clear=");   Serial.println(clearVal);

  Serial.print("flags,humid_jump=");       Serial.print(humid_jump);
  Serial.print(",temp_rise=");             Serial.print(temp_rise);
  Serial.print(",mag_shift=");             Serial.print(mag_shift);
  Serial.print(",light_or_color_change="); Serial.println(light_or_color_change);

  Serial.print("event,"); Serial.println(activeLabel);

  delay(500);
}
