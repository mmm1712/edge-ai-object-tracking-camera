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

constexpr unsigned long TARGET_STALE_MS = 550;

constexpr float FILTER_ALPHA_X = 0.62f;

constexpr float FILTER_ALPHA_Y = 0.30f;
constexpr float BASE_MAX_Y_JUMP_PX = 30.0f;
constexpr float BOX_HEIGHT_JUMP_SCALE = 0.45f;
constexpr int STRONG_Y_JUMP_SCORE = 82;
constexpr int Y_JUMP_CONFIRM_FRAMES = 2;
constexpr float Y_JUMP_CONFIRM_TOLERANCE_PX = 16.0f;

constexpr float Y_LOCK_PAN_ERROR_PX = 22.0f;
constexpr float Y_LOCK_MAX_Y_ERROR_PX = 18.0f;
constexpr float TILT_AIM_OFFSET_Y_PX = 0.0f;

constexpr uint16_t AI_I2C_ADDRESS = 0x62;
constexpr uint32_t AI_I2C_WAIT_MS = 1;
constexpr uint32_t AI_I2C_CLOCK_HZ = 400000;

constexpr int INVOKE_FAILURE_BACKOFF_MS = 20;

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

  float pendingY;
  int pendingYFrames;

  int lastRawY;
  int lastScore;
  bool lastYRejected;

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
  8.0f,
  1.65f,
  145.0f,
  3.5f,
  700.0f,
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
  1.85f,   
  125.0f,   
  4.0f,
  650.0f,  
  125.0f,
  0.0f,
  -1
};

TargetTracker tracker = {
  FRAME_CENTER_X,
  FRAME_CENTER_Y,
  0.0f,
  0.0f,
  FRAME_CENTER_Y,
  0,
  0,
  0,
  false,
  false,
  false,
  0
};

SemaphoreHandle_t stateMutex;

bool aiOK = false;
bool trackingEnabled = false;

portMUX_TYPE statsMux = portMUX_INITIALIZER_UNLOCKED;

uint32_t totalInvokeOK = 0;
uint32_t totalInvokeFail = 0;

uint32_t windowInvokeOK = 0;
uint32_t windowInvokeFail = 0;
uint32_t windowDetectionFrames = 0;

uint64_t windowInvokeTimeUs = 0;
uint32_t lastInvokeUs = 0;

int lastPreprocessMs = 0;
int lastInferenceMs = 0;
int lastPostprocessMs = 0;

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

  tracker.pendingY = FRAME_CENTER_Y;
  tracker.pendingYFrames = 0;
  tracker.lastRawY = 0;
  tracker.lastScore = 0;
  tracker.lastYRejected = false;

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
  bool success = AI.begin(
    &AIWire,
    -1,
    AI_I2C_ADDRESS,
    AI_I2C_WAIT_MS,
    AI_I2C_CLOCK_HZ
  );

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

void updateTarget(
  int detectedX,
  int detectedY,
  int boxWidth,
  int boxHeight,
  int score
) {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

  tracker.lastSeenMs = millis();
  tracker.targetVisible = true;
  tracker.lastRawY = detectedY;
  tracker.lastScore = score;
  tracker.lastYRejected = false;

  if (!tracker.filterReady) {
    tracker.filteredX = detectedX;
    tracker.filteredY = detectedY;
    tracker.pendingY = detectedY;
    tracker.pendingYFrames = 0;
    tracker.filterReady = true;
  } else {
    tracker.filteredX =
      tracker.filteredX * (1.0f - FILTER_ALPHA_X) +
      detectedX * FILTER_ALPHA_X;

    float yJump =
      absFloat(detectedY - tracker.filteredY);

    float adaptiveJumpLimit =
      boxHeight * BOX_HEIGHT_JUMP_SCALE;

    if (adaptiveJumpLimit < BASE_MAX_Y_JUMP_PX) {
      adaptiveJumpLimit = BASE_MAX_Y_JUMP_PX;
    }

    bool acceptY =
      yJump <= adaptiveJumpLimit ||
      score >= STRONG_Y_JUMP_SCORE;

    if (!acceptY) {
      // A single detector glitch is ignored. A real large movement normally
      // appears in two consecutive detections near the same new Y position.
      if (
        tracker.pendingYFrames > 0 &&
        absFloat(detectedY - tracker.pendingY) <=
          Y_JUMP_CONFIRM_TOLERANCE_PX
      ) {
        tracker.pendingYFrames++;
      } else {
        tracker.pendingY = detectedY;
        tracker.pendingYFrames = 1;
      }

      if (tracker.pendingYFrames >= Y_JUMP_CONFIRM_FRAMES) {
        acceptY = true;
      } else {
        tracker.lastYRejected = true;
      }
    }

    if (acceptY) {
      tracker.filteredY =
        tracker.filteredY * (1.0f - FILTER_ALPHA_Y) +
        detectedY * FILTER_ALPHA_Y;

      tracker.pendingY = detectedY;
      tracker.pendingYFrames = 0;
    }
  }

  float aimY = FRAME_CENTER_Y + TILT_AIM_OFFSET_Y_PX;
  float errorX = tracker.filteredX - FRAME_CENTER_X;
  float errorY = tracker.filteredY - aimY;

  if (absFloat(errorX) <= panAxis.deadzonePx) errorX = 0.0f;
  if (absFloat(errorY) <= tiltAxis.deadzonePx) errorY = 0.0f;

  // Ignore tiny vertical box movement while the target is mainly travelling
  // horizontally.
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
    tracker.filterReady = false;
    tracker.pendingYFrames = 0;
    tracker.lastYRejected = false;
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

    uint32_t invokeStartUs = micros();
    int result = AI.invoke(1, false, false);
    uint32_t invokeElapsedUs = micros() - invokeStartUs;

    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    int score = 0;
    int target = -1;

    bool hasBox = false;
    int preprocessMs = 0;
    int inferenceMs = 0;
    int postprocessMs = 0;

    if (result == 0) {
      hasBox = readBestBox(
        x,
        y,
        width,
        height,
        score,
        target
      );

      preprocessMs = AI.perf().prepocess;
      inferenceMs = AI.perf().inference;
      postprocessMs = AI.perf().postprocess;
    }

    portENTER_CRITICAL(&statsMux);

    lastInvokeUs = invokeElapsedUs;

    if (result == 0) {
      totalInvokeOK++;
      windowInvokeOK++;
      windowInvokeTimeUs += invokeElapsedUs;

      lastPreprocessMs = preprocessMs;
      lastInferenceMs = inferenceMs;
      lastPostprocessMs = postprocessMs;

      if (hasBox) {
        windowDetectionFrames++;
      }
    } else {
      totalInvokeFail++;
      windowInvokeFail++;
    }

    portEXIT_CRITICAL(&statsMux);

    if (result != 0) {
      markNoFreshTarget();

      vTaskDelay(pdMS_TO_TICKS(INVOKE_FAILURE_BACKOFF_MS));
      continue;
    }

    if (!hasBox) {
      markNoFreshTarget();
      taskYIELD();
      continue;
    }

    updateTarget(
      x,
      y,
      width,
      height,
      score
    );
    taskYIELD();
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
  unsigned long elapsedMs = now - lastTimingPrintMs;

  if (elapsedMs < 1000) return;
  lastTimingPrintMs = now;

  uint32_t okFrames;
  uint32_t failedFrames;
  uint32_t detectionFrames;
  uint64_t invokeTimeUs;
  uint32_t latestInvokeUs;
  uint32_t totalOK;
  uint32_t totalFail;

  int preprocessMs;
  int inferenceMs;
  int postprocessMs;

  portENTER_CRITICAL(&statsMux);

  okFrames = windowInvokeOK;
  failedFrames = windowInvokeFail;
  detectionFrames = windowDetectionFrames;
  invokeTimeUs = windowInvokeTimeUs;
  latestInvokeUs = lastInvokeUs;

  totalOK = totalInvokeOK;
  totalFail = totalInvokeFail;

  preprocessMs = lastPreprocessMs;
  inferenceMs = lastInferenceMs;
  postprocessMs = lastPostprocessMs;

  windowInvokeOK = 0;
  windowInvokeFail = 0;
  windowDetectionFrames = 0;
  windowInvokeTimeUs = 0;

  portEXIT_CRITICAL(&statsMux);

  float actualFps =
    elapsedMs > 0
      ? okFrames * 1000.0f / elapsedMs
      : 0.0f;

  float attemptFps =
    elapsedMs > 0
      ? (okFrames + failedFrames) * 1000.0f / elapsedMs
      : 0.0f;

  float detectionFps =
    elapsedMs > 0
      ? detectionFrames * 1000.0f / elapsedMs
      : 0.0f;

  float averageInvokeMs =
    okFrames > 0
      ? (invokeTimeUs / 1000.0f) / okFrames
      : 0.0f;

  int modelPipelineMs =
    preprocessMs + inferenceMs + postprocessMs;

  float modelPipelineFps =
    modelPipelineMs > 0
      ? 1000.0f / modelPipelineMs
      : 0.0f;

  bool visible = false;
  bool yRejected = false;
  int rawY = 0;
  int score = 0;
  float filteredY = 0.0f;
  float tiltError = 0.0f;
  float panAngle = 0.0f;
  float tiltAngle = 0.0f;

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
    visible = tracker.targetVisible;
    yRejected = tracker.lastYRejected;
    rawY = tracker.lastRawY;
    score = tracker.lastScore;
    filteredY = tracker.filteredY;
    tiltError = tracker.errorY;
    panAngle = panAxis.currentAngle;
    tiltAngle = tiltAxis.currentAngle;
    xSemaphoreGive(stateMutex);
  }

  Serial.print("FPS=");
  Serial.print(actualFps, 2);

  Serial.print(" detectionFPS=");
  Serial.print(detectionFps, 2);

  Serial.print(" attempts=");
  Serial.print(attemptFps, 2);

  Serial.print(" avgInvokeMs=");
  Serial.print(averageInvokeMs, 1);

  Serial.print(" lastInvokeMs=");
  Serial.print(latestInvokeUs / 1000.0f, 1);

  Serial.print(" perf[");
  Serial.print(preprocessMs);
  Serial.print("+");
  Serial.print(inferenceMs);
  Serial.print("+");
  Serial.print(postprocessMs);
  Serial.print("ms]");

  Serial.print(" modelFPS=");
  Serial.print(modelPipelineFps, 2);

  Serial.print(" target=");
  Serial.print(visible ? "YES" : "NO");

  Serial.print(" y=");
  Serial.print(rawY);
  Serial.print("/");
  Serial.print(filteredY, 1);

  Serial.print(" yErr=");
  Serial.print(tiltError, 1);

  Serial.print(" yReject=");
  Serial.print(yRejected ? "YES" : "NO");

  Serial.print(" score=");
  Serial.print(score);

  Serial.print(" servo=");
  Serial.print(panAngle, 1);
  Serial.print(",");
  Serial.print(tiltAngle, 1);

  Serial.print(" ok=");
  Serial.print(totalOK);

  Serial.print(" fail=");
  Serial.println(totalFail);
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
  lastTimingPrintMs = millis();

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
