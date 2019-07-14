#define ENV_DETECTION_DIST_CM 100;
#define ENV_MIN_LENGTH 10;
#define ENV_MAX_CONSECUTIVE_NEGATIVES 35;
#define ENV_DETECTION_DELAY_MS 6

// 0 - none, 1 - entries, 2 - entities, 3 - entity attempts, 4 - detections
#define DEBUG_LEVEL 2

#include <stdio.h>
#include <string.h>

struct Entity {
  int64_t finishTime;
  long len;
  int source;
  Entity(int64_t finishTime, long len, int source) : finishTime(finishTime), len(len), source(source) {}
  void log() const {
    char buff[55];
    sprintf(buff, "Time: %10ld; Length: %4ld; Source: %2d", static_cast<long>(finishTime), len, source);
    Serial.println(buff);
  }
};

class EntityProcessor {
 private:
  int64_t loopIndex;
 public:
  EntityProcessor() : loopIndex(0) {}
  void nextLoop() {
    ++loopIndex;
  }
  void pushEntity(long len, int sensorIndex) {
    if (DEBUG_LEVEL == 2) {
      Entity(loopIndex, len, sensorIndex).log();
    }
  }
  void endLoop() {
  }
};

class Sensor {
 private:
  static const long DETECTION_DIST_CM = ENV_DETECTION_DIST_CM;
  static const long MIN_LENGTH = ENV_MIN_LENGTH;
  static const long MAX_CONSECUTIVE_NEGATIVES = ENV_MAX_CONSECUTIVE_NEGATIVES;
  static int instanceCounter;
  EntityProcessor* processor;
  int sensorIndex;
  int trigPin, echoPin;
  long entityLength;
  long negatives;
  long getDuration() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    return pulseIn(echoPin, HIGH);
  }
  long getDistanceCm() {
    long d = getDuration() * 0.034 / 2;
    if (DEBUG_LEVEL == 4) {
      char buff[30];
      sprintf(buff, "Sensor %2d: Dist: %5ld", sensorIndex, d);
      Serial.println(buff);
    }
    return d;
  }
  bool checkObject() {
    return getDistanceCm() < DETECTION_DIST_CM;
  }
  void endEntity() {
    entityLength -= MAX_CONSECUTIVE_NEGATIVES;
    if (entityLength > MIN_LENGTH) {
      processor->pushEntity(entityLength, sensorIndex);
    }
    if (DEBUG_LEVEL == 3) {
      char buff[30];
      sprintf(buff, "Sensor %2d: Length: %4ld", sensorIndex, entityLength);
      Serial.println(buff);
    }
    entityLength = 0;
    negatives = 0;
  }
 public:
  Sensor(int trig, int echo, EntityProcessor* proc) : trigPin(trig), echoPin(echo), processor(proc), sensorIndex(++instanceCounter), entityLength(0), negatives(0) {}
  void init() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }
  void pulse() {
    if (checkObject()) {
      ++entityLength;
      negatives = 0;
    } else if (entityLength > 0) {
      ++entityLength;
      ++negatives;
      if (negatives > MAX_CONSECUTIVE_NEGATIVES) {
        endEntity();
      }
    }
  }
};
int Sensor::instanceCounter = 0;

EntityProcessor proc;
Sensor sensor(9, 8, &proc);

void setup() {
  sensor.init();
  Serial.begin(9600);
}

void loop() {
  proc.nextLoop();
  sensor.pulse();
  delayMicroseconds(ENV_DETECTION_DELAY_MS * 1000);
  proc.endLoop();
}
