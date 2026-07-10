/*
  Local Edge AI Object-Tracking Camera

  Hardware:
  - Grove Vision AI V2
  - Seeed Studio XIAO ESP32-S3
  - PCA9685 servo driver
  - 2 x MG90S servos
  - External 5 V servo power supply

  Libraries:
  - Seeed Arduino SSCMA
  - Adafruit PWM Servo Driver Library

  Wiring used in this project:
  Grove Vision AI V2:
    SDA -> XIAO D4 / GPIO 5
    SCL -> XIAO D5 / GPIO 6

  PCA9685:
    SDA -> XIAO GPIO 9
    SCL -> XIAO GPIO 7

  Important:
  Power the servos from an external 5 V supply and connect
  the external supply ground to the XIAO ground.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Seeed_Arduino_SSCMA.h>

#define AI_SDA 5
#define AI_SCL 6
#define PCA_SDA 9
#define PCA_SCL 7
#define PCA9685_ADDR 0x40

TwoWire AIWire = TwoWire(0);
TwoWire ServoWire = TwoWire(1);

SSCMA AI;
Adafruit_PWMServoDriver pwm(PCA9685_ADDR, ServoWire);

constexpr float FRAME_WIDTH = 240.0f;
constexpr float FRAME_HEIGHT = 240.0f;
constexpr float FRAME_CENTER_X = FRAME_WIDTH / 2.0f;
constexpr float FRAME_CENTER_Y = FRAME_HEIGHT / 2.0f;

constexpr int PAN_CHANNEL = 0;
constexpr int TILT_CHANNEL = 1;
constexpr int SERVO_MIN_PULSE = 110;
constexpr int SERVO_MAX_PULSE = 500;
constexpr int SERVO_TICK_MS = 20;

constexpr int MIN_DETECTION_SCORE = 50;
constexpr float MAX_Y_JUMP_PX = 75.0f;
constexpr int STRONG_Y_JUMP_SCORE = 65;
constexpr unsigned long TARGET_STALE_MS = 420;
constexpr float Y_LOCK_PAN_ERROR_PX = 22.0f;
constexpr float Y_LOCK_MAX_Y_ERROR_PX = 18.0f;
constexpr float FILTER_ALPHA_X = 0.50f;
constexpr float FILTER_ALPHA_Y = 0.34f;
constexpr float TILT_AIM_OFFSET_Y_PX = 0.0f;

struct AxisController {
  int channel;
  float minAngle;
  float centerAngle;
  float maxAngle;
  int direction;
  float deadzonePx;
  float kp;
  float maxVelocityDps;
  float minVelocityDps;
  float accelerationDps2;
  float currentAngle;
  float currentVelocityDps;
  int lastPulse;
};

struct TargetTracker {
  float filteredX;
  float filteredY;
  float errorX;
  float errorY;
  bool filterReady;
  bool targetVisible;
  unsigned long lastSeenMs;
};

AxisController panAxis = {
  PAN_CHANNEL,
  35.0f,
  90.0f,
  145.0f,
  -1,
  10.0f,
  1.35f,
  120.0f,
  2.5f,
  520.0f,
  90.0f,
  0.0f,
  -1
};

AxisController tiltAxis = {
  TILT_CHANNEL,
  95.0f,
  125.0f,
  140.0f,
  -1,
  14.0f,
  2.10f,
  150.0f,
  5.0f,
  780.0f,
  125.0f,
  0.0f,
  -1
};

TargetTracker tracker = {
  FRAME_CENTER_X,
  FRAME_CENTER_Y,
  0.0f,
  0.0f,
  false,
  false,
  0
};

SemaphoreHandle_t stateMutex;

bool aiOK = false;
bool trackingEnabled = false;

unsigned long lastInvokeMs = 0;
unsigned long invokeOkCount = 0;
unsigned long invokeFailCount = 0;
unsigned long lastTimingPrintMs = 0;

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

float absFloat(float value) {
  return value < 0.0f ? -value : value;
}

float signFloat(float value) {
  if (value > 0.0f) return 1.0f;
  if (value < 0.0f) return -1.0f;
  return 0.0f;
}

float moveToward(float current, float target, float maxChange) {
  float difference = target - current;
  if (absFloat(difference) <= maxChange) return target;
  return current + signFloat(difference) * maxChange;
}

float easeOutCubic(float value) {
  value = clampFloat(value, 0.0f, 1.0f);
  float remaining = 1.0f - value;
  return 1.0f - remaining * remaining * remaining;
}

int angleToPulse(float angle) {
  angle = clampFloat(angle, 0.0f, 180.0f);
  float normalized = angle / 180.0f;
  return static_cast<int>(
    SERVO_MIN_PULSE +
    normalized * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) +
    0.5f
  );
}

void writeAxis(AxisController &axis) {
  axis.currentAngle = clampFloat(
    axis.currentAngle,
    axis.minAngle,
    axis.maxAngle
  );

  int pulse = angleToPulse(axis.currentAngle);
  if (pulse == axis.lastPulse) return;

  axis.lastPulse = pulse;
  pwm.setPWM(axis.channel, 0, pulse);
}

void writeServos() {
  writeAxis(panAxis);
  writeAxis(tiltAxis);
}

float calculateTargetVelocity(float error, const AxisController &axis) {
  float magnitude = absFloat(error);
  if (magnitude <= axis.deadzonePx) return 0.0f;

  float effectiveError = magnitude - axis.deadzonePx;
  float velocity = axis.kp * effectiveError;

  if (velocity > 0.0f && velocity < axis.minVelocityDps) {
    velocity = axis.minVelocityDps;
  }

  velocity = clampFloat(velocity, 0.0f, axis.maxVelocityDps);
  return signFloat(error) * velocity;
}

void resetTrackingNoLock() {
  tracker.filteredX = FRAME_CENTER_X;
  tracker.filteredY = FRAME_CENTER_Y + TILT_AIM_OFFSET_Y_PX;
  tracker.errorX = 0.0f;
  tracker.errorY = 0.0f;
  tracker.filterReady = false;
  tracker.targetVisible = false;
  tracker.lastSeenMs = 0;
  panAxis.currentVelocityDps = 0.0f;
  tiltAxis.currentVelocityDps = 0.0f;
}

void centerSmooth() {
  float startPan = panAxis.currentAngle;
  float startTilt = tiltAxis.currentAngle;

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    resetTrackingNoLock();
    xSemaphoreGive(stateMutex);
  }

  constexpr int durationMs = 800;
  constexpr int intervalMs = 20;
  constexpr int steps = durationMs / intervalMs;

  panAxis.lastPulse = -1;
  tiltAxis.lastPulse = -1;

  for (int step = 1; step <= steps; step++) {
    float progress = static_cast<float>(step) / static_cast<float>(steps);
    float eased = easeOutCubic(progress);

    panAxis.currentAngle =
      startPan + (panAxis.centerAngle - startPan) * eased;

    tiltAxis.currentAngle =
      startTilt + (tiltAxis.centerAngle - startTilt) * eased;

    writeServos();
    delay(intervalMs);
  }

  panAxis.currentAngle = panAxis.centerAngle;
  tiltAxis.currentAngle = tiltAxis.centerAngle;
  panAxis.currentVelocityDps = 0.0f;
  tiltAxis.currentVelocityDps = 0.0f;
  writeServos();
}

bool beginAI() {
  bool success = AI.begin(&AIWire);
  Serial.println(success ? "AI BEGIN OK" : "AI BEGIN FAIL");
  return success;
}

bool readBestBox(
  int &x,
  int &y,
  int &width,
  int &height,
  int &score,
  int &target
) {
  int count = AI.boxes().size();
  if (count == 0) return false;

  int bestIndex = -1;
  long bestValue = -1;

  for (int index = 0; index < count; index++) {
    int currentScore = AI.boxes()[index].score;
    if (currentScore < MIN_DETECTION_SCORE) continue;

    int boxWidth = AI.boxes()[index].w;
    int boxHeight = AI.boxes()[index].h;
    long area = static_cast<long>(boxWidth) * boxHeight;
    long value = static_cast<long>(currentScore) * 1000L + area;

    if (value > bestValue) {
      bestValue = value;
      bestIndex = index;
    }
  }

  if (bestIndex < 0) return false;

  auto box = AI.boxes()[bestIndex];
  x = box.x;
  y = box.y;
  width = box.w;
  height = box.h;
  score = box.score;
  target = box.target;
  return true;
}

void updateTarget(int detectedX, int detectedY, int score) {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

  tracker.lastSeenMs = millis();
  tracker.targetVisible = true;
  bool allowYUpdate = true;

  if (!tracker.filterReady) {
    tracker.filteredX = detectedX;
    tracker.filteredY = detectedY;
    tracker.filterReady = true;
  } else {
    tracker.filteredX =
      tracker.filteredX * (1.0f - FILTER_ALPHA_X) +
      detectedX * FILTER_ALPHA_X;

    float yJump = absFloat(detectedY - tracker.filteredY);

    if (yJump > MAX_Y_JUMP_PX && score < STRONG_Y_JUMP_SCORE) {
      allowYUpdate = false;
    }

    if (allowYUpdate) {
      tracker.filteredY =
        tracker.filteredY * (1.0f - FILTER_ALPHA_Y) +
        detectedY * FILTER_ALPHA_Y;
    }
  }

  float aimY = FRAME_CENTER_Y + TILT_AIM_OFFSET_Y_PX;
  float errorX = tracker.filteredX - FRAME_CENTER_X;
  float errorY = tracker.filteredY - aimY;

  if (!allowYUpdate) errorY = 0.0f;
  if (absFloat(errorX) <= panAxis.deadzonePx) errorX = 0.0f;
  if (absFloat(errorY) <= tiltAxis.deadzonePx) errorY = 0.0f;

  if (
    absFloat(errorX) > Y_LOCK_PAN_ERROR_PX &&
    absFloat(errorY) < Y_LOCK_MAX_Y_ERROR_PX
  ) {
    errorY = 0.0f;
  }

  tracker.errorX = errorX;
  tracker.errorY = errorY;
  xSemaphoreGive(stateMutex);
}

void markNoFreshTarget() {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(3)) != pdTRUE) return;

  if (millis() - tracker.lastSeenMs > TARGET_STALE_MS) {
    tracker.errorX = 0.0f;
    tracker.errorY = 0.0f;
    tracker.targetVisible = false;
    panAxis.currentVelocityDps = 0.0f;
    tiltAxis.currentVelocityDps = 0.0f;
  }

  xSemaphoreGive(stateMutex);
}

void aiTask(void *parameter) {
  while (true) {
    if (!trackingEnabled || !aiOK) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    unsigned long invokeStartMs = millis();
    int result = AI.invoke(1, false, false);
    lastInvokeMs = millis() - invokeStartMs;

    int x;
    int y;
    int width;
    int height;
    int score;
    int target;

    bool hasBox = readBestBox(x, y, width, height, score, target);

    if (result != 0) {
      invokeFailCount++;
      markNoFreshTarget();
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    invokeOkCount++;

    if (!hasBox) {
      markNoFreshTarget();
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    updateTarget(x, y, score);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void updateAxis(AxisController &axis, float error, float deltaTime) {
  float targetVelocity = calculateTargetVelocity(error, axis);
  float maxVelocityChange = axis.accelerationDps2 * deltaTime;

  axis.currentVelocityDps = moveToward(
    axis.currentVelocityDps,
    targetVelocity,
    maxVelocityChange
  );

  axis.currentAngle +=
    axis.direction * axis.currentVelocityDps * deltaTime;

  if (axis.currentAngle <= axis.minAngle) {
    axis.currentAngle = axis.minAngle;
    axis.currentVelocityDps = 0.0f;
  }

  if (axis.currentAngle >= axis.maxAngle) {
    axis.currentAngle = axis.maxAngle;
    axis.currentVelocityDps = 0.0f;
  }
}

void servoTask(void *parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  constexpr float deltaTime = SERVO_TICK_MS / 1000.0f;

  while (true) {
    float errorX = 0.0f;
    float errorY = 0.0f;
    unsigned long lastSeenMs = 0;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      errorX = tracker.errorX;
      errorY = tracker.errorY;
      lastSeenMs = tracker.lastSeenMs;
      xSemaphoreGive(stateMutex);
    }

    bool targetStale = millis() - lastSeenMs > TARGET_STALE_MS;

    if (!trackingEnabled || targetStale) {
      errorX = 0.0f;
      errorY = 0.0f;
    }

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      updateAxis(panAxis, errorX, deltaTime);
      updateAxis(tiltAxis, errorY, deltaTime);
      xSemaphoreGive(stateMutex);
    }

    writeServos();

    vTaskDelayUntil(
      &lastWakeTime,
      pdMS_TO_TICKS(SERVO_TICK_MS)
    );
  }
}

void printTimingOncePerSecond() {
  unsigned long now = millis();
  if (now - lastTimingPrintMs < 1000) return;

  lastTimingPrintMs = now;

  Serial.print("invokeMs=");
  Serial.print(lastInvokeMs);
  Serial.print(" ok=");
  Serial.print(invokeOkCount);
  Serial.print(" fail=");
  Serial.println(invokeFailCount);
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  Serial.println();
  Serial.println("BOOT: Local Edge AI Object Tracker");

  stateMutex = xSemaphoreCreateMutex();

  if (stateMutex == nullptr) {
    Serial.println("ERROR: Could not create mutex");
    while (true) delay(1000);
  }

  AIWire.begin(AI_SDA, AI_SCL);
  AIWire.setClock(400000);

  ServoWire.begin(PCA_SDA, PCA_SCL);
  ServoWire.setClock(400000);

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(200);

  centerSmooth();

  aiOK = beginAI();
  trackingEnabled = aiOK;

  xTaskCreatePinnedToCore(
    servoTask,
    "ServoTask",
    4096,
    nullptr,
    3,
    nullptr,
    1
  );

  xTaskCreatePinnedToCore(
    aiTask,
    "AITask",
    8192,
    nullptr,
    1,
    nullptr,
    0
  );
}

void loop() {
  printTimingOncePerSecond();
  delay(20);
}
