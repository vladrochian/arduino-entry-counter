#define ENV_DETECTION_DIST_CM 100;
#define ENV_MIN_LENGTH 10;
#define ENV_MAX_CONSECUTIVE_NEGATIVES 35;
#define ENV_DETECTION_DELAY_MS 6
#define ENV_ENTITY_VALIDITY 150

// 0 - entrances, 1 - column detections, 2 - entities, 3 - entity attempts, 4 - detections
#define DEBUG_LEVEL 1

#include <stdio.h>
#include <string.h>

struct Entity {
  int64_t finishTime;
  long len;
  int source;
  Entity() = default;
  Entity(int64_t finishTime, long len, int source) : finishTime(finishTime), len(len), source(source) {}
  static Entity nullEntity() {
    return Entity(-1, -1, -1);
  }
  void log() const {
    char buff[55];
    sprintf(buff, "Time: %10ld; Length: %4ld; Source: %2d", static_cast<long>(finishTime), len, source);
    Serial.println(buff);
  }
  bool isNull() const {
    return finishTime == -1 && len == -1 && source == -1;
  }
  int64_t getStartTime() const {
    return finishTime - len;
  }
  bool operator<(const Entity& other) const {
    return getStartTime() < other.getStartTime() && finishTime < other.finishTime;
  }
};

class EntityProcessor {
 private:
  int64_t loopIndex;
  Entity entities[16];
 public:
  EntityProcessor() : loopIndex(0) {
    for (auto& entity : entities) {
      entity = Entity::nullEntity();
    }
  }
  void nextLoop() {
    ++loopIndex;
  }
  void addColumnDetection(int col, int d, int64_t startTime, int64_t finishTime) {
    if (DEBUG_LEVEL == 1) {
      char buff[70];
      sprintf(buff, "Column: %d; Direction: %d; Start: %10ld; Finish: %10ld", col, d, static_cast<long>(startTime), static_cast<long>(finishTime));
      Serial.println(buff);
    }
  }
  void processColumn(int col) {
    int first = col << 1, second = first | 1;
    if (entities[first] < entities[second]) {
      addColumnDetection(col, 0, entities[first].getStartTime(), entities[second].finishTime);
    } else if (entities[second] < entities[first]) {
      addColumnDetection(col, 1, entities[second].getStartTime(), entities[first].finishTime);
    }
  }
  void pushEntity(long len, int sensorIndex) {
    auto q = Entity(loopIndex, len, sensorIndex);
    if (DEBUG_LEVEL == 2) {
      q.log();
    }
    int qs = q.source;
    entities[qs] = q;
    if (!entities[qs ^ 1].isNull()) {
      processColumn(qs >> 1);
      entities[qs] = Entity::nullEntity();
      entities[qs ^ 1] = Entity::nullEntity();
    }
  }
  void endLoop() {
    for (auto& entity : entities) {
      if (loopIndex - entity.finishTime > ENV_ENTITY_VALIDITY) {
        entity = Entity::nullEntity();
      }
    }
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
  Sensor(int trig, int echo, EntityProcessor* proc) : trigPin(trig), echoPin(echo), processor(proc), sensorIndex(instanceCounter++), entityLength(0), negatives(0) {}
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
Sensor sensors[] = {Sensor(9, 8, &proc), Sensor(11, 10, &proc)};

void setup() {
  for (auto& sensor : sensors) {
    sensor.init();
  }
  Serial.begin(9600);
}

void loop() {
  proc.nextLoop();
  for (auto& sensor : sensors) {
    sensor.pulse();
  }
  delayMicroseconds(ENV_DETECTION_DELAY_MS * 1000);
  proc.endLoop();
}
