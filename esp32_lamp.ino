#include "Adafruit_VL53L1X.h"
#include <Adafruit_DotStar.h>
#include "TCA9548.h"

#define NUMPIXELS         59
#define DATAPIN           18
#define CLOCKPIN          5
#define NUM_SENSORS       8
#define MAX_DISTANCE      2500
#define BRIGHT_MAX        1500    // distance mm where brightness starts
#define MIN_DISTANCE      50      // distance mm for full brightness
#define ACTIVATE_DIST     1500    // a zone activates when something is closer than this
#define MIN_VISIBLE       20.0    // brightness floor when a zone first activates
#define SAMPLES           2
#define TIMEOUT_MS        10000
#define STALE_MS          1500
#define FILL_STEP_MS      45      // ms per LED added during the fill sweep

// Zone mapping for 59 LEDs — Zone 6 wraps around the end of the strip.
// Specific to how the strip physically lands on the ring.
const int ZONE_STARTS[NUM_SENSORS] = {12, 20, 27, 34, 41, 48, 55, 4};
const int ZONE_SIZES[NUM_SENSORS]  = {8,  7,  7,  7,  7,  7,  8,  8};
const int SENSOR_TO_POSITION[NUM_SENSORS] = {0, 1, 2, 3, 4, 5, 6, 7};

PCA9548 tca(0x70);
Adafruit_VL53L1X sensors[NUM_SENSORS];
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

int16_t distanceBuffers[NUM_SENSORS][SAMPLES];
int bufferIndexes[NUM_SENSORS] = {0};
float smoothBrightness[NUM_SENSORS] = {0};
float targetBrightnessArr[NUM_SENSORS] = {0};
int16_t latestDistance[NUM_SENSORS];
int16_t smoothedDist[NUM_SENSORS];
unsigned long lastDetected[NUM_SENSORS] = {0};
unsigned long lastReadingTime[NUM_SENSORS] = {0};
bool timedOut[NUM_SENSORS] = {false};
unsigned long lastSerialSend = 0;
int currentSensor = 0;

// Fill state, per zone indexed by position
bool zoneActive[NUM_SENSORS] = {false};
int  fillCount[NUM_SENSORS] = {0};
int  fillDir[NUM_SENSORS] = {0};    // 0=middle-out, 1=from low side, 2=from high side, 3=both-in
unsigned long lastFillStep[NUM_SENSORS] = {0};
bool perLED[NUMPIXELS] = {false};

int neighborLow(int pos)  { return (pos + NUM_SENSORS - 1) % NUM_SENSORS; }
int neighborHigh(int pos) { return (pos + 1) % NUM_SENSORS; }

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  strip.begin();
  strip.setBrightness(255);
  strip.clear();
  strip.show();

  Wire.begin();
  if (!tca.begin()) {
    Serial.println("PCA9548 not found!");
    while (1) delay(10);
  }
  Serial.println("PCA9548 found!");

  unsigned long nowInit = millis();
  for (int i = 0; i < NUM_SENSORS; i++) {
    for (int j = 0; j < SAMPLES; j++) distanceBuffers[i][j] = MAX_DISTANCE;
    latestDistance[i] = MAX_DISTANCE;
    smoothedDist[i] = MAX_DISTANCE;
    lastDetected[i] = nowInit;
    lastReadingTime[i] = nowInit;

    tca.selectChannel(i);
    delay(10);

    if (!sensors[i].begin(0x29, &Wire)) {
      Serial.print("Sensor "); Serial.print(i); Serial.println(" FAILED");
      continue;
    }
    sensors[i].startRanging();
    sensors[i].setTimingBudget(50);
    Serial.print("Sensor "); Serial.print(i); Serial.println(" OK");
  }

  Serial.println("Setup complete");
}

void updateSensor(int s) {
  unsigned long now = millis();
  tca.selectChannel(s);

  if (sensors[s].dataReady()) {
    int16_t distance = sensors[s].distance();
    sensors[s].clearInterrupt();

    if (distance != -1 && distance >= 10) {
      latestDistance[s] = distance;
      lastReadingTime[s] = now;

      // Clamp far reads to a uniform "far" value so all zones fade evenly
      int16_t bufVal = (distance >= BRIGHT_MAX) ? MAX_DISTANCE : distance;

      if (distance < ACTIVATE_DIST) {
        lastDetected[s] = now;
        timedOut[s] = false;
      }
      if (now - lastDetected[s] > TIMEOUT_MS) timedOut[s] = true;

      distanceBuffers[s][bufferIndexes[s]] = timedOut[s] ? MAX_DISTANCE : bufVal;
      bufferIndexes[s] = (bufferIndexes[s] + 1) % SAMPLES;

      int32_t sum = 0;
      for (int j = 0; j < SAMPLES; j++) sum += distanceBuffers[s][j];
      int16_t smoothDistance = sum / SAMPLES;
      smoothedDist[s] = smoothDistance;

      // Set TARGET brightness only, smoothing happens every loop in smoothAllZones()
      if (smoothDistance >= BRIGHT_MAX) {
        targetBrightnessArr[s] = 0.0;
      } else {
        float t = (float)(smoothDistance - MIN_DISTANCE) / (BRIGHT_MAX - MIN_DISTANCE);
        t = constrain(t, 0.0, 1.0);
        float closeness = 1.0 - t;
        targetBrightnessArr[s] = MIN_VISIBLE + (255.0 - MIN_VISIBLE) * pow(closeness, 2.0);
      }
    }
  }
}

void smoothAllZones() {
  // Uniform fade for all zones every loop, independent of sensor read rate.
  for (int s = 0; s < NUM_SENSORS; s++) {
    float target = targetBrightnessArr[s];
    float lerpFactor;
    if (target > smoothBrightness[s]) {
      lerpFactor = (smoothBrightness[s] < 80) ? 0.012 : 0.03;   // fade in
    } else {
      lerpFactor = 0.008;                                        // slow fade out
    }
    smoothBrightness[s] += (target - smoothBrightness[s]) * lerpFactor;
    // Only snap to 0 when fading toward off
    if (target < 1.0 && smoothBrightness[s] <= 2) smoothBrightness[s] = 0;
  }
}

void fadeStaleSensors() {
  unsigned long now = millis();
  for (int s = 0; s < NUM_SENSORS; s++) {
    if (now - lastReadingTime[s] > STALE_MS) {
      targetBrightnessArr[s] = 0.0;
      latestDistance[s] = MAX_DISTANCE;
      smoothedDist[s] = MAX_DISTANCE;
      for (int j = 0; j < SAMPLES; j++) distanceBuffers[s][j] = MAX_DISTANCE;
    }
  }
}

// Fill direction based on which neighbors are still lit
int decideFillDir(int pos) {
  bool lowLit  = smoothBrightness[neighborLow(pos)]  > 2.0;
  bool highLit = smoothBrightness[neighborHigh(pos)] > 2.0;
  if (lowLit && highLit) return 3;
  if (lowLit)  return 1;
  if (highLit) return 2;
  return 0;
}

void applyFill(int pos, int count, int dir) {
  int zoneStart = ZONE_STARTS[pos];
  int zoneSize  = ZONE_SIZES[pos];
  for (int k = 0; k < zoneSize; k++) {
    int p = (zoneStart + k) % NUMPIXELS;
    perLED[p] = false;
  }
  count = constrain(count, 0, zoneSize);

  if (dir == 1) {
    for (int k = 0; k < count; k++)
      perLED[(zoneStart + k) % NUMPIXELS] = true;
  } else if (dir == 2) {
    for (int k = zoneSize - count; k < zoneSize; k++)
      perLED[(zoneStart + k) % NUMPIXELS] = true;
  } else if (dir == 3) {
    int half = count / 2, extra = count % 2;
    for (int k = 0; k < half + extra; k++)
      perLED[(zoneStart + k) % NUMPIXELS] = true;
    for (int k = zoneSize - half; k < zoneSize; k++)
      perLED[(zoneStart + k) % NUMPIXELS] = true;
  } else {
    int center = zoneSize / 2, half = count / 2, extra = count % 2;
    int lo = constrain(center - half, 0, zoneSize - 1);
    int hi = constrain(center + half + extra - 1, 0, zoneSize - 1);
    for (int k = lo; k <= hi; k++)
      perLED[(zoneStart + k) % NUMPIXELS] = true;
  }
}

void updateFills() {
  unsigned long now = millis();
  for (int pos = 0; pos < NUM_SENSORS; pos++) {
    bool wantLit = smoothBrightness[pos] > 2.0;
    int zoneSize = ZONE_SIZES[pos];

    if (wantLit && !zoneActive[pos]) {
      zoneActive[pos] = true;
      fillCount[pos] = 1;
      fillDir[pos] = decideFillDir(pos);
      lastFillStep[pos] = now;
      applyFill(pos, fillCount[pos], fillDir[pos]);
    } else if (wantLit && zoneActive[pos]) {
      if (fillCount[pos] < zoneSize && now - lastFillStep[pos] >= FILL_STEP_MS) {
        fillCount[pos]++;
        lastFillStep[pos] = now;
        applyFill(pos, fillCount[pos], fillDir[pos]);
      }
    } else if (!wantLit && zoneActive[pos]) {
      if (smoothBrightness[pos] <= 2.0) {
        zoneActive[pos] = false;
        fillCount[pos] = 0;
        int zoneStart = ZONE_STARTS[pos];
        for (int k = 0; k < zoneSize; k++)
          perLED[(zoneStart + k) % NUMPIXELS] = false;
      } else {
        applyFill(pos, zoneSize, fillDir[pos]);
      }
    }
  }
}

void drawNormal() {
  for (int p = 0; p < NUMPIXELS; p++) strip.setPixelColor(p, 0, 0, 0);
  for (int pos = 0; pos < NUM_SENSORS; pos++) {
    uint8_t b = (uint8_t)smoothBrightness[pos];
    if (b == 0) continue;
    int zoneStart = ZONE_STARTS[pos];
    int zoneSize  = ZONE_SIZES[pos];
    for (int k = 0; k < zoneSize; k++) {
      int p = (zoneStart + k) % NUMPIXELS;
      if (perLED[p]) strip.setPixelColor(p, b, b, b);
    }
  }
}

void loop() {
  unsigned long now = millis();

  updateSensor(currentSensor);
  currentSensor = (currentSensor + 1) % NUM_SENSORS;

  fadeStaleSensors();
  smoothAllZones();
  updateFills();

  drawNormal();
  strip.show();

  if (now - lastSerialSend >= 20) {
    Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d\n",
      smoothedDist[0], smoothedDist[1], smoothedDist[2], smoothedDist[3],
      smoothedDist[4], smoothedDist[5], smoothedDist[6], smoothedDist[7]);
    lastSerialSend = now;
  }
}
