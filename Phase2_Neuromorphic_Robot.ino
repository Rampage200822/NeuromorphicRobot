/*
  Phase 2 — integrated SNN robot + topological mapping + ESP32 phone dashboard
  Target: classic ESP32 Dev Module + L298N + 3 digital IR line sensors
          + MPU6050 + VL53L0X ToF.

  Libraries (Arduino Library Manager):
    Adafruit MPU6050, Adafruit Unified Sensor, Pololu VL53L0X

  Session-only: learned weights, concept prototypes, traces and experience
  are volatile RAM only and are erased at reboot. No Blynk/RFID/encoder sensors or PC required. The phone GUI is served by the ESP32.

  IMPORTANT:
  - This is a research prototype, not a certified autonomous safety system.
  - Test with wheels lifted first. Confirm motor polarity and sensor logic.
  - IR sensors are assumed LOW on black tape; change IR_BLACK_LOW if needed.
  - GPIO16/17 may be unavailable on some ESP32-WROVER boards (PSRAM).
  - Without wheel encoder sensors, actual wheel speed/distance is not measured.
  - Safety checks are deterministic and always override learned outputs.
*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>

// ---------------------------- Hardware ------------------------------------
static const int PIN_ENA = 25, PIN_IN1 = 26, PIN_IN2 = 27;
static const int PIN_ENB = 23, PIN_IN3 = 18, PIN_IN4 = 19;
static const int PIN_IR_L = 32, PIN_IR_C = 33, PIN_IR_R = 34;
static const bool IR_BLACK_LOW = true;
static const int PIN_MPU_SDA = 21, PIN_MPU_SCL = 22;
static const int PIN_TOF_SDA = 16, PIN_TOF_SCL = 17;
static const uint32_t LOOP_MS = 20;
static const float SAFE_STOP_CM = 12.0f;
static const float CAUTION_CM = 30.0f;
static const float FAST_CLOSE_CM_S = 25.0f;
static const int MAX_PWM = 180;
static const int MIN_PWM = 65;

TwoWire MPUWire(0);
TwoWire ToFWire(1);
Adafruit_MPU6050 mpu;
VL53L0X tof;
bool mpuOK = false, tofOK = false;

// ------------------------ SNN bounded resources ---------------------------
// Fixed resource budget; no unbounded allocation and no dynamic malloc.
static const int N_IN = 48;       // sensory/context/action/error spike channels
static const int N_ASSOC = 24;    // competitive, plastic LIF association cells
static const int N_PRED = N_IN;     // predicted sensory feature cells
static const int N_NEURONS = N_IN + N_ASSOC + N_PRED;
static const int MAX_SYNAPSES = 1800;
static const float DT_S = 0.020f;
static const float V_THRESH = 1.0f;
static const float V_RESET = 0.0f;
static const float TAU_MEM = 0.060f;
static const float TAU_PRE = 0.080f;
static const float TAU_POST = 0.080f;
static const float TAU_ASSOC_TRACE = 0.45f;
static const float STDP_A_PLUS = 0.006f;
static const float STDP_A_MINUS = 0.0065f;
static const float W_MIN = 0.0f, W_MAX = 0.65f;
static const float PRED_LEARN_RATE = 0.012f;
static const float ASSOC_LEARN_RATE = 0.004f;
static const int CONCEPT_SLOTS = 12;
static const float NOVELTY_THRESHOLD = 0.72f;
static const uint32_t CONCEPT_MIN_GAP_MS = 1200;

struct Synapse {
  uint8_t pre, post;
  float w, preTrace, postTrace;
  bool enabled;
};
Synapse synapses[MAX_SYNAPSES];
int synapseCount = 0;
float membrane[N_NEURONS];
bool spikes[N_NEURONS], prevSpikes[N_NEURONS];
float inputCurrent[N_NEURONS];
float assocTrace[N_ASSOC];
float predTrace[N_PRED];
float inputTrace[N_IN];
uint32_t lastSpikeMs[N_NEURONS];
uint32_t spikeCount[N_NEURONS];

struct Concept {
  bool used;
  float prototype[N_IN];
  uint32_t hits, lastSeen;
  float reliability;
  uint8_t dominantCell;
};
Concept concepts[CONCEPT_SLOTS];
int activeConcept = -1;
uint32_t lastConceptChange = 0;
float lastNovelty = 1.0f;
float lastPredictionError = 1.0f;
float predictionConfidence = 0.0f;
float predExpected[N_IN];
bool encoded[N_IN];
bool predicted[N_IN];
float sensory[N_IN];

// Context and motion estimates are uncertain; heading is relative gyro integration.
float relativeHeadingDeg = 0.0f;
float gyroBiasZ = 0.0f;
float headingUncertaintyDeg = 0.0f;
float lastDistanceCm = NAN;
float closingSpeedCmS = 0.0f;
float vibrationG = 0.0f;
float vibrationEnergy = 0.0f;
float previousVibrationEnergy = 0.0f;
uint32_t lastVibrationEvent = 0;
uint32_t previousVibrationEvent = 0;
uint32_t vibrationIntervalMs = 0;
uint8_t vibrationRhythmBin = 0;
uint8_t irMask = 0;
uint8_t previousIRMask = 0;
uint8_t motorAction = 0; // 0 stop, 1 forward, 2 left, 3 right, 4 reverse
uint8_t previousMotorAction = 0;
uint8_t contextBits = 0; // 0 straight/unknown, 1 left-mark, 2 center-mark, 3 right-mark, 4 all-mark
uint8_t previousContextBits = 0;
uint32_t lastLoopMs = 0;
uint32_t lastPrintMs = 0;
uint32_t lastSensorMs = 0;
uint32_t lastTurnMs = 0;
float lastGyroZ = 0.0f;
bool safetyLatched = true;
bool sensorsReady = false;

bool driveEnabled = false;

// ---------------- Phase 2: on-device web UI and learned topological map ----
// The graph is session-only. The phone browser can retain exported trip reports.
WebServer web(80);
static const char* AP_SSID = "Neuromorphic-Robot";
static const char* AP_PASSWORD = "RobotMap8266"; // 8+ characters
static const int MAX_MAP_NODES = 32;
static const int MAX_MAP_EDGES = 48;
static const int MAX_DESTINATIONS = 6;
static const int MAX_TRIP_EVENTS = 64;
enum RobotMode : uint8_t { MODE_EXPLORE=0, MODE_DESTINATION=1, MODE_MANUAL=2 };
RobotMode robotMode = MODE_EXPLORE;
bool webDriveRequested = false;
bool markerLatched = false;
bool junctionLatched = false;
bool lineLost = false;
bool mapNeedsUpdate = false;
uint32_t allBlackSince = 0, nonBlackSince = 0, lastMapEventMs = 0;
uint32_t turnUntilMs = 0;
int pendingTurn = 0; // -1 left, +1 right
int manualCommand = 0; // 0 stop, 1 forward, 2 reverse, 3 left, 4 right
int currentNode = -1, previousNode = -1, currentEdge = -1;
int destinationTarget = -1;
uint8_t exploreTurnCounter = 0;
float roadVibSum = 0, roadVibPeak = 0;
uint32_t roadSamples = 0, roadObstacleSamples = 0, roadElapsedMs = 0;
uint32_t lastMapSampleMs = 0;
uint32_t tripCount = 0, successfulTrips = 0, failedTrips = 0;
uint32_t lastTripStartMs = 0;
char statusMessage[96] = "Connect to robot Wi-Fi; motors disabled";

struct MapNode {
  bool used;
  uint8_t id;
  float x, y;                 // relative, estimated display coordinates (not metric)
  uint16_t visits;
  uint8_t confidence;
  int8_t destinationIndex;
  uint32_t lastSeen;
};
struct MapEdge {
  bool used;
  uint8_t a, b;
  uint16_t traversals;
  bool blocked;
  float vibMean, vibPeak, obstacleProbability, reliability;
  float estimatedCost;
  uint32_t lastSeen;
  char roadType[12];
};
struct Destination { bool used; char name[20]; int8_t nodeId; uint16_t hits; };
struct TripEvent { uint32_t t; uint8_t node; uint8_t eventType; float predErr; float vib; float tof; };
MapNode mapNodes[MAX_MAP_NODES];
MapEdge mapEdges[MAX_MAP_EDGES];
Destination destinations[MAX_DESTINATIONS];
TripEvent tripEvents[MAX_TRIP_EVENTS];
uint8_t tripEventHead=0, tripEventCount=0;
uint16_t junctionCount=0;
float lastMapX=0, lastMapY=0;

// ---------------------------- Motor control -------------------------------
void motorStop() {
  analogWrite(PIN_ENA, 0); analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
  motorAction = 0;
}
void motorForward(int leftPWM, int rightPWM) {
  leftPWM = constrain(leftPWM, 0, MAX_PWM); rightPWM = constrain(rightPWM, 0, MAX_PWM);
  digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, leftPWM); analogWrite(PIN_ENB, rightPWM);
  motorAction = (abs(leftPWM-rightPWM) < 12) ? 1 : (leftPWM < rightPWM ? 2 : 3);
}
void motorReverse(int pwm) {
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_ENA, constrain(pwm, 0, MAX_PWM));
  analogWrite(PIN_ENB, constrain(pwm, 0, MAX_PWM));
  motorAction = 4;
}

// ----------------------- Neural network construction ----------------------
float randWeight(float lo, float hi) {
  return lo + (hi-lo) * (float)random(0, 10001) / 10000.0f;
}
void addSynapse(int pre, int post, float weight) {
  if (synapseCount >= MAX_SYNAPSES) return;
  if (pre == post || pre < 0 || post < 0 || pre >= N_NEURONS || post >= N_NEURONS) return;
  // Prevent duplicate directed connections.
  for (int i=0; i<synapseCount; ++i)
    if (synapses[i].enabled && synapses[i].pre==pre && synapses[i].post==post) return;
  Synapse &s = synapses[synapseCount++];
  s.pre = (uint8_t)pre; s.post = (uint8_t)post;
  s.w = constrain(weight, W_MIN, W_MAX); s.preTrace = 0; s.postTrace = 0; s.enabled = true;
}
void buildNetwork() {
  synapseCount = 0;
  // Sparse sensory -> associative projection, with distinct random initial weights.
  for (int a=0; a<N_ASSOC; ++a) {
    int post = N_IN + a;
    for (int k=0; k<16; ++k) {
      int in = random(0, N_IN);
      addSynapse(in, post, randWeight(0.04f, 0.18f));
    }
    // Sparse lateral competition and recurrent temporal context.
    for (int b=0; b<N_ASSOC; ++b) if (a != b && random(0,100) < 18)
      addSynapse(N_IN+b, post, randWeight(0.02f, 0.10f));
    // Each associative cell predicts a sparse subset of sensory channels.
    for (int k=0; k<12; ++k) {
      int p = N_IN + N_ASSOC + random(0, N_PRED);
      addSynapse(post, p, randWeight(0.03f, 0.14f));
    }
  }
}
void clearSessionLearning() {
  for (int i=0; i<MAX_SYNAPSES; ++i) {
    synapses[i].preTrace = synapses[i].postTrace = 0;
  }
  for (int i=0; i<N_NEURONS; ++i) {
    membrane[i]=0; spikes[i]=prevSpikes[i]=false; inputCurrent[i]=0;
    lastSpikeMs[i]=0; spikeCount[i]=0;
  }
  for (int i=0; i<N_ASSOC; ++i) assocTrace[i]=0;
  for (int i=0; i<N_PRED; ++i) predTrace[i]=0;
  for (int i=0; i<N_IN; ++i) inputTrace[i]=0;
  for (int c=0; c<CONCEPT_SLOTS; ++c) {
    concepts[c].used=false; concepts[c].hits=0; concepts[c].lastSeen=0;
    concepts[c].reliability=0.5f; concepts[c].dominantCell=0;
    for (int i=0;i<N_IN;++i) concepts[c].prototype[i]=0;
  }
  activeConcept=-1; lastConceptChange=0; predictionConfidence=0;
}

// ------------------------ Sensor acquisition ------------------------------
bool readIR(int pin) {
  int v = digitalRead(pin);
  return IR_BLACK_LOW ? (v == LOW) : (v == HIGH);
}
void calibrateGyro() {
  if (!mpuOK) return;
  float sum=0; int n=0;
  for (int i=0; i<120; ++i) {
    sensors_event_t a,g,t;
    if (mpu.getEvent(&a,&g,&t)) { sum += g.gyro.z; n++; }
    delay(5);
  }
  if (n>0) gyroBiasZ=sum/n;
}
void readSensors() {
  previousIRMask = irMask;
  irMask = (readIR(PIN_IR_L)?1:0) | (readIR(PIN_IR_C)?2:0) | (readIR(PIN_IR_R)?4:0);
  previousContextBits = contextBits;
  // These are line-mark patterns, not guaranteed junction identities.
  contextBits = irMask;
  float distance = NAN;
  if (tofOK) {
    uint16_t mm = tof.readRangeContinuousMillimeters();
    if (!tof.timeoutOccurred() && mm > 0 && mm < 4000) distance = mm / 10.0f;
  }
  uint32_t now=millis();
  if (isfinite(distance) && isfinite(lastDistanceCm) && now>lastSensorMs) {
    float dt=(now-lastSensorMs)/1000.0f;
    if (dt>0.005f && dt<1.0f) {
      float closing=(lastDistanceCm-distance)/dt;
      closingSpeedCmS = 0.7f*closingSpeedCmS + 0.3f*closing;
    }
  } else if (!isfinite(distance)) closingSpeedCmS=0;
  lastDistanceCm=distance;
  if (mpuOK) {
    sensors_event_t a,g,t;
    if (mpu.getEvent(&a,&g,&t)) {
      float gx=g.gyro.x, gy=g.gyro.y, gz=g.gyro.z-gyroBiasZ;
      relativeHeadingDeg += gz * (now-lastSensorMs)/1000.0f * 57.29578f;
      if (relativeHeadingDeg > 180) relativeHeadingDeg -= 360;
      if (relativeHeadingDeg < -180) relativeHeadingDeg += 360;
      headingUncertaintyDeg = min(180.0f, headingUncertaintyDeg + fabs(gz)*0.02f + 0.08f);
      float dynamic = sqrtf(a.acceleration.x*a.acceleration.x + a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z);
      vibrationG = fabsf(dynamic - 9.80665f) / 9.80665f;
      // Smooth energy is a feature, not a calibrated vibration measurement.
      vibrationEnergy = 0.85f*vibrationEnergy + 0.15f*vibrationG;
      if (vibrationEnergy > 0.12f && previousVibrationEnergy <= 0.12f) {
        previousVibrationEvent=lastVibrationEvent; lastVibrationEvent=now;
        if (previousVibrationEvent && lastVibrationEvent>previousVibrationEvent) {
          vibrationIntervalMs=lastVibrationEvent-previousVibrationEvent;
          vibrationRhythmBin = (vibrationIntervalMs<100)?1:(vibrationIntervalMs<220)?2:(vibrationIntervalMs<450)?3:4;
        }
      }
      previousVibrationEnergy=vibrationEnergy;
      lastGyroZ=gz;
    }
  }
  lastSensorMs=now;
}

// ------------------------ Spike encoding -----------------------------------
void setFeature(int idx, float intensity) {
  if (idx<0 || idx>=N_IN) return;
  sensory[idx]=constrain(intensity,0.0f,1.0f);
  // Rate/population style encoding: intensity changes probability of a spike.
  encoded[idx] = random(0,1000) < (int)(sensory[idx]*1000.0f);
}
void encodeSensors() {
  for (int i=0;i<N_IN;++i) { sensory[i]=0; encoded[i]=false; }
  // IR pattern one-hot-ish channels 0..7, and transition channels 8..15.
  setFeature(irMask, 1.0f);
  setFeature(8+irMask, (irMask!=previousIRMask)?1.0f:0.0f);
  setFeature(16, (irMask&1)?1:0); setFeature(17,(irMask&2)?1:0); setFeature(18,(irMask&4)?1:0);
  setFeature(19, (irMask==7)?1:0); // ambiguous all-black mark, not assumed to be a junction
  // ToF range population code: channels 20..25.
  if (isfinite(lastDistanceCm)) {
    const float centers[6]={8,15,25,40,70,120};
    for(int i=0;i<6;++i) {
      float d=fabsf(lastDistanceCm-centers[i]);
      setFeature(20+i, expf(-d/12.0f));
    }
  }
  // Closing-speed population code 26..29.
  const float closeCenters[4]={-20,0,20,50};
  for(int i=0;i<4;++i) setFeature(26+i, expf(-fabsf(closingSpeedCmS-closeCenters[i])/18.0f));
  // Vibration energy and rhythm bins 30..35.
  setFeature(30, vibrationEnergy>0.04f); setFeature(31, vibrationEnergy>0.12f);
  setFeature(32, vibrationEnergy>0.25f); setFeature(33+min((int)vibrationRhythmBin,2), vibrationRhythmBin>0);
  // Relative heading and turn dynamics 36..39.
  float headingWrapped=fmodf(fabsf(relativeHeadingDeg),180.0f);
  setFeature(36, headingWrapped<30); setFeature(37, headingWrapped>=30 && headingWrapped<75);
  setFeature(38, headingWrapped>=75); setFeature(39, fabsf(lastGyroZ)>0.35f);
  // Action context and context transitions 40..43.
  setFeature(40+min((int)motorAction,3), 1.0f);
  setFeature(44, 0); setFeature(45,0); // filled by prediction error / novelty after prediction
  setFeature(46, (irMask!=previousIRMask)?1.0f:0.0f);
  setFeature(47, headingUncertaintyDeg>35.0f?1.0f:0.0f);
}

// ----------------------- Prediction and plasticity -------------------------
void computePredictions() {
  for (int i=0;i<N_IN;++i) { predExpected[i]=0; predicted[i]=false; }
  float total=0;
  for(int a=0;a<N_ASSOC;++a) {
    float activity=assocTrace[a];
    if(activity<0.015f) continue;
    int pre=N_IN+a;
    for(int s=0;s<synapseCount;++s) {
      Synapse &sy=synapses[s];
      if(sy.enabled && sy.pre==pre && sy.post>=N_IN+N_ASSOC) {
        int f=sy.post-(N_IN+N_ASSOC);
        if(f<N_IN) { predExpected[f] += activity*sy.w; total += activity*sy.w; }
      }
    }
  }
  for(int i=0;i<N_IN;++i) {
    predExpected[i]=constrain(predExpected[i]*1.5f,0.0f,1.0f);
    predicted[i]=predExpected[i]>0.12f;
  }
  // Feature-wise surprise; predictions arise from learned association spikes and weights.
  float err=0, norm=0;
  for(int i=0;i<N_IN;++i) {
    float actual=encoded[i]?1.0f:0.0f;
    err += fabsf(actual-predExpected[i]); norm += 1.0f;
  }
  lastPredictionError = norm>0 ? err/norm : 1.0f;
  predictionConfidence = 0.92f*predictionConfidence + 0.08f*(1.0f-lastPredictionError);
  lastNovelty = lastPredictionError;
}
void runLIFAndSTDP() {
  // Input neurons are genuine spike sources driven by the encoded sensor events.
  for (int i=0; i<N_IN; ++i) spikes[i] = encoded[i];
  // Decay traces and neuron states. Inputs are stochastic spikes; associative and
  // prediction populations integrate synaptic currents from the previous time step.
  for(int i=0;i<N_IN;++i) {
    inputTrace[i] *= expf(-DT_S/TAU_PRE);
    if(encoded[i]) inputTrace[i]=1.0f;
  }
  for(int i=0;i<N_ASSOC;++i) assocTrace[i] *= expf(-DT_S/TAU_ASSOC_TRACE);
  for(int i=0;i<N_PRED;++i) predTrace[i] *= expf(-DT_S/TAU_ASSOC_TRACE);
  for(int i=0;i<N_NEURONS;++i) { inputCurrent[i]=0; }
  // Input spikes enter as external current. Prediction neurons receive a weak
  // teaching drive from the actual current feature; plastic prediction synapses
  // learn which features tend to follow the active associative ensemble.
  for(int i=0;i<N_IN;++i) if(encoded[i]) inputCurrent[i]=0.65f;
  for(int s=0;s<synapseCount;++s) {
    Synapse &sy=synapses[s]; if(!sy.enabled) continue;
    if(prevSpikes[sy.pre]) inputCurrent[sy.post] += sy.w;
  }
  // Lateral inhibition implements competition among associative cells.
  int firedAssoc=0;
  for(int i=0;i<N_ASSOC;++i) {
    int n=N_IN+i;
    float current=inputCurrent[n];
    // Mild homeostatic bias prevents permanently silent/overactive cells.
    float homeo = constrain(0.10f - spikeCount[n]*0.00002f, -0.05f, 0.10f);
    float decay=expf(-DT_S/TAU_MEM);
    membrane[n]=membrane[n]*decay + current*0.42f + homeo;
    if(membrane[n]>=V_THRESH) { spikes[n]=true; membrane[n]=V_RESET; firedAssoc++; }
    else spikes[n]=false;
  }
  // Winner-take-most: retain the strongest spiking associative cell per step.
  if(firedAssoc>1) {
    int winner=-1; float best=-1;
    for(int i=0;i<N_ASSOC;++i) if(spikes[N_IN+i] && membrane[N_IN+i]>best) { winner=i; best=membrane[N_IN+i]; }
    // Since fired cells reset, choose one randomly among active cells to preserve competition.
    if(winner<0) {
      int seen=0, chosen=random(0,firedAssoc);
      for(int i=0;i<N_ASSOC;++i) if(spikes[N_IN+i] && seen++==chosen) { winner=i; break; }
    }
    for(int i=0;i<N_ASSOC;++i) if(i!=winner) spikes[N_IN+i]=false;
  }
  // Prediction population integrates learned prediction synapses and a weak actual-feature teacher.
  for(int i=0;i<N_PRED;++i) {
    int n=N_IN+N_ASSOC+i;
    float teach=(i<N_IN && encoded[i])?0.38f:0.0f;
    membrane[n]=membrane[n]*expf(-DT_S/TAU_MEM) + inputCurrent[n]*0.32f + teach;
    if(membrane[n]>=V_THRESH) { spikes[n]=true; membrane[n]=V_RESET; }
    else spikes[n]=false;
  }
  // Spike-timing-dependent plasticity: pre/post traces encode relative timing.
  for(int s=0;s<synapseCount;++s) {
    Synapse &sy=synapses[s]; if(!sy.enabled) continue;
    bool pre=prevSpikes[sy.pre], post=spikes[sy.post];
    if(post) sy.w -= STDP_A_MINUS*sy.preTrace;
    if(pre) sy.w += STDP_A_PLUS*sy.postTrace;
    sy.w=constrain(sy.w,W_MIN,W_MAX);
    sy.preTrace *= expf(-DT_S/TAU_PRE);
    sy.postTrace *= expf(-DT_S/TAU_POST);
    if(pre) sy.preTrace += 1.0f;
    if(post) sy.postTrace += 1.0f;
  }
  for(int i=0;i<N_NEURONS;++i) {
    if(spikes[i]) { lastSpikeMs[i]=millis(); spikeCount[i]++; }
    prevSpikes[i]=spikes[i];
  }
  for(int i=0;i<N_ASSOC;++i) if(spikes[N_IN+i]) assocTrace[i] += 1.0f;
  for(int i=0;i<N_PRED;++i) if(spikes[N_IN+N_ASSOC+i]) predTrace[i] += 1.0f;
}
void errorModulatedLearning() {
  // Prediction error gates extra learning: surprising transitions receive more plasticity.
  float gate=constrain(0.25f + lastPredictionError,0.25f,1.0f);
  for(int s=0;s<synapseCount;++s) {
    Synapse &sy=synapses[s]; if(!sy.enabled) continue;
    if(sy.pre>=N_IN && sy.pre<N_IN+N_ASSOC && sy.post>=N_IN+N_ASSOC) {
      int feature=sy.post-(N_IN+N_ASSOC);
      if(feature<N_IN && encoded[feature]) {
        sy.w=constrain(sy.w + PRED_LEARN_RATE*gate*assocTrace[sy.pre-N_IN]*0.01f,W_MIN,W_MAX);
      }
    }
  }
}

// ---------------------- Novelty and concept formation ----------------------
float prototypeDistance(const Concept &c) {
  // Normalize by active pattern magnitude so sparse IR-pattern changes matter;
  // plain mean error over 48 channels would hide meaningful sparse changes.
  float diff=0.0f, magnitude=0.0f;
  for(int i=0;i<N_IN;++i) {
    diff += fabsf(sensory[i]-c.prototype[i]);
    magnitude += max(sensory[i], c.prototype[i]);
  }
  return diff / max(0.25f, magnitude);
}
int findConceptSlot() {
  for(int i=0;i<CONCEPT_SLOTS;++i) if(!concepts[i].used) return i;
  // Bounded forgetting: replace the least recently used low-reliability concept.
  int victim=0; float worst=1e9f; uint32_t now=millis();
  for(int i=0;i<CONCEPT_SLOTS;++i) {
    float age=(now-concepts[i].lastSeen)/1000.0f;
    float score=concepts[i].reliability*2.0f - age*0.001f + concepts[i].hits*0.0001f;
    if(score<worst) { worst=score; victim=i; }
  }
  return victim;
}
void updateConcepts() {
  int best=-1; float bestD=10.0f;
  for(int c=0;c<CONCEPT_SLOTS;++c) if(concepts[c].used) {
    float d=prototypeDistance(concepts[c]); if(d<bestD) {bestD=d;best=c;}
  }
  uint32_t now=millis();
  if(best<0 || bestD>NOVELTY_THRESHOLD) {
    if(now-lastConceptChange>=CONCEPT_MIN_GAP_MS) {
      int slot=findConceptSlot(); concepts[slot].used=true; concepts[slot].hits=1;
      concepts[slot].lastSeen=now; concepts[slot].reliability=0.5f;
      for(int i=0;i<N_IN;++i) concepts[slot].prototype[i]=sensory[i];
      int dominant=0; for(int a=1;a<N_ASSOC;++a) if(spikeCount[N_IN+a]>spikeCount[N_IN+dominant]) dominant=a;
      concepts[slot].dominantCell=dominant; activeConcept=slot; lastConceptChange=now;
    }
  } else {
    activeConcept=best; Concept &c=concepts[best]; c.hits++; c.lastSeen=now;
    // Prototype reconsolidation: familiar patterns adapt slowly; surprise adapts faster.
    float lr=0.015f + 0.08f*lastPredictionError;
    for(int i=0;i<N_IN;++i) c.prototype[i]=(1.0f-lr)*c.prototype[i]+lr*sensory[i];
    c.reliability=constrain(0.96f*c.reliability+0.04f*(1.0f-bestD),0.0f,1.0f);
  }
  // Encode novelty and prediction error as actual input spikes for the next SNN step.
  sensory[44]=lastPredictionError; encoded[44]=random(0,1000)<(int)(lastPredictionError*1000.0f);
  sensory[45]=lastNovelty; encoded[45]=random(0,1000)<(int)(lastNovelty*1000.0f);
}

// ----------------------- Action/outcome association -------------------------
void encodeActionOutcome() {
  // Action and immediate next-step observation co-activate through input/association
  // timing and STDP. This learns correlations, not proof of causality.
  int actionFeature=40+min((int)previousMotorAction,3);
  if(actionFeature>=40 && actionFeature<44) { sensory[actionFeature]=1; encoded[actionFeature]=true; }
  if(previousMotorAction!=motorAction && millis()-lastTurnMs>150) lastTurnMs=millis();
}

// ----------------------------- Safety layer --------------------------------
bool safetyCheck() {
  // Conservative policy: if either safety-critical sensor failed, do not drive.
  if(!mpuOK || !tofOK || !isfinite(lastDistanceCm)) return false;
  // Intentional marker approach exception: only the all-three-IR marker pattern
  // permits a low-speed creep between 5 and 12 cm. At <=5 cm marker handling stops.
  bool markerApproach = (irMask==7 && lastDistanceCm>5.0f && lastDistanceCm<=SAFE_STOP_CM);
  if(lastDistanceCm<=SAFE_STOP_CM && !markerApproach) return false;
  if(lastDistanceCm<CAUTION_CM && closingSpeedCmS>FAST_CLOSE_CM_S) return false;
  return true;
}
void lineFollowingController() {
  if(!driveEnabled) { motorStop(); safetyLatched=true; return; }
  if(!safetyCheck()) { motorStop(); safetyLatched=true; return; }
  safetyLatched=false;
  // The SNN modulates speed based on surprise and vibration; deterministic IR steering
  // is retained as a low-level controller, not misrepresented as learned navigation.
  int base=MAX_PWM - (int)(lastPredictionError*55.0f) - (int)(vibrationEnergy*90.0f);
  base=constrain(base,MIN_PWM,MAX_PWM);
  int left=base, right=base;
  switch(irMask) {
    case 0b010: break; // center on line
    case 0b011: left=base-35; right=base+15; break;
    case 0b001: left=base-65; right=base+25; break;
    case 0b110: left=base+15; right=base-35; break;
    case 0b100: left=base+25; right=base-65; break;
    case 0b000: // line lost: slow crawl; do not pretend location is known
      left=MIN_PWM; right=MIN_PWM; break;
    case 0b111: // ambiguous broad mark: cautious crawl, not a confirmed junction
      left=MIN_PWM; right=MIN_PWM; break;
    default: left=MIN_PWM; right=MIN_PWM; break;
  }
  if(lastDistanceCm<CAUTION_CM) { left=min(left,MIN_PWM+25); right=min(right,MIN_PWM+25); }
  motorForward(left,right);
}

// ---------------- Phase 2: map, route learning, and phone web app ----------
const char DASHBOARD_HTML[] PROGMEM = R"ROBOTHTML(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no"><meta charset="utf-8"><title>Neuromorphic Robot</title><style>
:root{color-scheme:dark;--bg:#0b1220;--panel:#131e30;--line:#26364e;--text:#e7eefb;--muted:#9fb0c8;--accent:#63d7b0;--warn:#ffca72;--red:#ff7777}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px system-ui,Arial}header{padding:14px 16px;background:#101a2b;position:sticky;top:0;z-index:5;border-bottom:1px solid var(--line)}h1{font-size:18px;margin:0}small,.muted{color:var(--muted)}main{padding:12px;display:grid;gap:12px;max-width:1100px;margin:auto}.panel{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:12px;min-width:0}.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.metric{background:#0d1727;border:1px solid var(--line);border-radius:10px;padding:10px}.metric b{font-size:20px;display:block;margin-top:4px}.btn,select,input{border:1px solid #3a4b65;background:#1a2940;color:var(--text);border-radius:9px;padding:10px;font:inherit}.btn{cursor:pointer;touch-action:manipulation}.btn:active{background:#34506e}.primary{border-color:#329c7e;background:#164b43}.stop{background:#582c36;border-color:#a94c5b}.wide{flex:1}.hidden{display:none}canvas{width:100%;height:390px;background:#0a1423;border:1px solid var(--line);border-radius:10px;touch-action:none}.tabs{display:flex;gap:6px;overflow:auto}.pill{border-radius:999px;padding:6px 9px;background:#202f45;color:var(--muted);font-size:12px}.ok{color:var(--accent)}.warn{color:var(--warn)}.bad{color:var(--red)}textarea{width:100%;min-height:70px;background:#0d1727;color:var(--text);border:1px solid var(--line);border-radius:8px;padding:8px}#events{max-height:150px;overflow:auto;white-space:pre-wrap;font:12px ui-monospace,monospace;color:#c1d2e9}@media(min-width:800px){main{grid-template-columns:1.5fr 1fr}.mapPanel{grid-row:span 2}.widePanel{grid-column:1/-1}.grid{grid-template-columns:repeat(3,minmax(0,1fr))}}
</style></head><body><header><div class="row" style="justify-content:space-between"><div><h1>🧠 Smart Neuromorphic Robot</h1><small>ESP32-hosted · No PC/cloud required</small></div><span id="conn" class="pill">Connecting…</span></div></header><main>
<section class="panel mapPanel"><div class="row" style="justify-content:space-between"><div><b>Internal topological map</b><div class="muted">Drag nodes to edit the display layout. Coordinates are estimates, not centimetres.</div></div><button class="btn" onclick="centerMap()">Center</button></div><canvas id="map" width="900" height="520"></canvas><div class="row"><span class="pill">● current robot</span><span class="pill">● junction</span><span class="pill">● destination</span><span class="pill">━ blocked road</span></div><div class="muted" id="mapinfo">Waiting for map…</div></section>
<section class="panel"><b>Operating mode</b><div class="row" style="margin-top:10px"><button class="btn wide" onclick="cmd('mode_explore')">Explore</button><button class="btn wide" onclick="cmd('mode_dest')">Destination</button><button class="btn wide" onclick="cmd('mode_manual')">Manual</button></div><div class="row" style="margin-top:8px"><button class="btn primary wide" onclick="cmd('enable')">Enable drive</button><button class="btn stop wide" onclick="cmd('stop')">STOP</button></div><div class="row" style="margin-top:8px"><select id="destSelect" class="wide"><option value="-1">Choose destination</option></select><button class="btn" onclick="setDest()">Go</button></div><div class="row" style="margin-top:8px"><input id="newName" class="wide" maxlength="19" value="Hospital" placeholder="Destination name"><button class="btn" onclick="assignDest()">Name detected marker</button></div><small id="markerInfo">Destination marker not detected.</small></section>
<section class="panel"><b>Live telemetry</b><div class="grid" style="margin-top:8px"><div class="metric">Mode<b id="mode">—</b></div><div class="metric">ToF distance<b id="tof">—</b></div><div class="metric">Vibration proxy<b id="vib">—</b></div><div class="metric">Speed command<b id="speed">—</b></div><div class="metric">Relative heading<b id="heading">—</b></div><div class="metric">Prediction error<b id="pred">—</b></div></div><p id="safety" class="warn">Safety status: waiting</p><div class="muted" id="sensorStatus">Sensors —</div></section>
<section class="panel"><b>Manual controls</b><div class="muted">Hold a direction button to move; release to stop. Safety interlocks remain active.</div><div style="display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:12px auto;max-width:300px"><span></span><button class="btn" onpointerdown="manual('forward')" onpointerup="manual('stop')" onpointerleave="manual('stop')">▲</button><span></span><button class="btn" onpointerdown="manual('left')" onpointerup="manual('stop')" onpointerleave="manual('stop')">◀</button><button class="btn stop" onclick="cmd('stop')">STOP</button><button class="btn" onpointerdown="manual('right')" onpointerup="manual('stop')" onpointerleave="manual('stop')">▶</button><span></span><button class="btn" onpointerdown="manual('reverse')" onpointerup="manual('stop')" onpointerleave="manual('stop')">▼</button><span></span></div><div class="row"><button class="btn" onclick="cmd('reset_map')">Reset session map</button><button class="btn" onclick="cmd('heading_reset')">Reset heading reference</button></div></section>
<section class="panel"><b>Learning / map state</b><div id="learning" class="muted" style="margin-top:8px">Waiting…</div><div class="row" style="margin-top:8px"><button class="btn" onclick="cmd('mark_success')">Record trip success</button><button class="btn" onclick="cmd('mark_failure')">Record trip failure</button><button class="btn" onclick="downloadReports()">Export phone trip log</button></div><div id="events"></div></section>
<section class="panel widePanel"><b>Map editing / notes</b><p class="muted">Drag a node to adjust its visual position. This edits only the map layout coordinates, not the physical world or the robot's learned topology.</p><div class="row"><span>Selected node:</span><b id="selectedNode">none</b><button class="btn" onclick="clearSelection()">Clear selection</button></div><p class="muted">Trip reports are saved in this phone browser's local storage. Clearing browser data can erase them. ESP32 learned map and SNN weights are session-only and reset after reboot.</p></section>
</main><script>
const $=id=>document.getElementById(id);let state=null,selected=null,drag=null,lastLog='',lastSnapshot=0;const canvas=$('map'),ctx=canvas.getContext('2d');
async function cmd(c,extra={}){try{let q=new URLSearchParams({cmd:c,...extra});await fetch('/cmd?'+q.toString(),{cache:'no-store'});if(c==='stop')log('Manual stop');}catch(e){$('conn').textContent='Offline';}}
function manual(c){cmd('manual_'+c)}function setDest(){let v=$('destSelect').value;if(v!=='-1')cmd('go',{target:v})}function assignDest(){cmd('assign',{name:$('newName').value})}function clearSelection(){selected=null;$('selectedNode').textContent='none';drawMap()}
function log(s){let arr=JSON.parse(localStorage.getItem('robotTrips')||'[]');arr.push({time:new Date().toISOString(),note:s,mode:state?.mode,node:state?.currentNode,tof:state?.tof,vibration:state?.vibration,predictionError:state?.predictionError});if(arr.length>2000)arr=arr.slice(-2000);localStorage.setItem('robotTrips',JSON.stringify(arr));$('events').textContent=arr.slice(-8).map(x=>x.time+' '+x.note).reverse().join('\n')}
function downloadReports(){let a=document.createElement('a'),blob=new Blob([localStorage.getItem('robotTrips')||'[]'],{type:'application/json'});a.href=URL.createObjectURL(blob);a.download='robot-trip-history.json';a.click();URL.revokeObjectURL(a.href)}
function centerMap(){if(!state)return;drawMap(true)}function resizeCanvas(){let d=window.devicePixelRatio||1,r=canvas.getBoundingClientRect();canvas.width=Math.max(320,r.width*d);canvas.height=Math.max(260,r.height*d);drawMap()}window.addEventListener('resize',resizeCanvas);
function pos(n,fit=false){let nodes=state.nodes.filter(x=>x.used);let xs=nodes.map(n=>n.x),ys=nodes.map(n=>n.y),minX=Math.min(0,...xs),maxX=Math.max(1,...xs),minY=Math.min(0,...ys),maxY=Math.max(1,...ys);let pad=45,s=Math.min((canvas.width-2*pad)/(maxX-minX+1),(canvas.height-2*pad)/(maxY-minY+1));if(!isFinite(s))s=35;return {x:pad+(n.x-minX+0.5)*s,y:pad+(n.y-minY+0.5)*s}}
function drawMap(){if(!state)return;ctx.clearRect(0,0,canvas.width,canvas.height);let nodes=state.nodes.filter(n=>n.used),byId={};nodes.forEach(n=>byId[n.id]=n);ctx.lineWidth=3;for(let e of state.edges.filter(e=>e.used)){let a=byId[e.a],b=byId[e.b];if(!a||!b)continue;let p=pos(a),q=pos(b);ctx.beginPath();ctx.moveTo(p.x,p.y);ctx.lineTo(q.x,q.y);ctx.strokeStyle=e.blocked?'#ff6b79':e.roadType==='rough'?'#e4a45c':e.roadType==='medium'?'#8d9eb8':'#4dd4a8';ctx.setLineDash(e.blocked?[7,5]:[]);ctx.stroke();ctx.setLineDash([]);ctx.fillStyle='#b8c6dc';ctx.font='12px sans-serif';ctx.fillText(e.roadType+' '+Math.round(e.reliability*100)+'%',(p.x+q.x)/2,(p.y+q.y)/2-5)}for(let n of nodes){let p=pos(n);ctx.beginPath();ctx.arc(p.x,p.y,n.id===state.currentNode?10:8,0,Math.PI*2);ctx.fillStyle=n.id===state.currentNode?'#63d7b0':n.destinationIndex>=0?'#ffca72':selected===n.id?'#9c8cff':'#5c91d9';ctx.fill();ctx.strokeStyle='#e7eefb';ctx.lineWidth=selected===n.id?3:1;ctx.stroke();ctx.fillStyle='#e7eefb';ctx.font='bold 13px sans-serif';ctx.fillText(n.destinationIndex>=0?(state.destinations[n.destinationIndex]?.name||('D'+n.id)):'J'+n.id,p.x+12,p.y-10)}$('mapinfo').textContent=`${nodes.length} discovered locations · ${state.edges.filter(e=>e.used).length} connections · ${state.destinations.filter(d=>d.used).length} named destinations · selected node can be dragged`}
function pointer(e){let r=canvas.getBoundingClientRect();return {x:(e.clientX-r.left)*canvas.width/r.width,y:(e.clientY-r.top)*canvas.height/r.height}}canvas.addEventListener('pointerdown',e=>{let p=pointer(e);if(!state)return;let found=null;for(let n of state.nodes.filter(n=>n.used)){let q=pos(n);if(Math.hypot(q.x-p.x,q.y-p.y)<20){found=n;break}}if(found){selected=found.id;drag=found.id;$('selectedNode').textContent='J'+found.id;canvas.setPointerCapture(e.pointerId);drawMap()}});canvas.addEventListener('pointermove',e=>{if(drag===null||!state)return;let p=pointer(e),n=state.nodes.find(n=>n.id===drag);if(!n)return;let nodes=state.nodes.filter(x=>x.used),r=canvas.getBoundingClientRect();let sx=(canvas.width-90)/Math.max(2,Math.max(...nodes.map(n=>n.x))-Math.min(...nodes.map(n=>n.x))+1),sy=(canvas.height-90)/Math.max(2,Math.max(...nodes.map(n=>n.y))-Math.min(...nodes.map(n=>n.y))+1);let minX=Math.min(0,...nodes.map(n=>n.x)),minY=Math.min(0,...nodes.map(n=>n.y));n.x=Math.round((p.x-45)/Math.max(10,sx)+minX-.5);n.y=Math.round((p.y-45)/Math.max(10,sy)+minY-.5);drawMap()});canvas.addEventListener('pointerup',async()=>{if(drag!==null){let n=state.nodes.find(n=>n.id===drag);if(n)cmd('move_node',{id:n.id,x:n.x,y:n.y});drag=null}});
async function poll(){try{let r=await fetch('/api',{cache:'no-store'});state=await r.json();$('conn').textContent='Connected';$('conn').className='pill ok';$('mode').textContent=state.mode;$('tof').textContent=state.tofValid?state.tof.toFixed(1)+' cm':'INVALID';$('vib').textContent=state.vibration.toFixed(3)+' g*';$('speed').textContent=state.pwm+' / 255 PWM';$('heading').textContent=state.heading.toFixed(1)+'° ±'+state.headingUncertainty.toFixed(0)+'°';$('pred').textContent=state.predictionError.toFixed(3);$('safety').textContent=state.safety?'Safety: clear':'Safety: STOP / interlock';$('safety').className=state.safety?'ok':'bad';$('sensorStatus').textContent=`MPU ${state.mpuOK?'OK':'FAULT'} · ToF ${state.tofOK?'OK':'FAULT'} · IR mask ${state.irMask} · closing estimate ${state.closingSpeed.toFixed(1)} cm/s`;$('markerInfo').textContent=state.marker?'Destination marker confirmed — select a name and tap Name detected marker.':'Destination marker not detected.';$('learning').textContent=`SNN synapses ${state.synapses} · active concepts ${state.concepts} · confidence proxy ${state.confidence.toFixed(2)} · map uncertainty ${state.mapUncertainty} · trips ${state.trips} (success ${state.successes}, failure ${state.failures}) · ${state.message}`;let sel=$('destSelect'),old=sel.value;sel.innerHTML='<option value="-1">Choose destination</option>';state.destinations.forEach((d,i)=>{if(d.used){let o=document.createElement('option');o.value=d.slot;o.textContent=d.name+' (J'+d.nodeId+')';sel.appendChild(o)}});if([...sel.options].some(o=>o.value===old))sel.value=old;drawMap();let key=state.currentNode+'|'+state.mode+'|'+state.marker+'|'+state.edges.filter(e=>e.used).length;if(key!==lastLog){if(lastLog)log('State: '+key);lastLog=key}if(Date.now()-lastSnapshot>10000){lastSnapshot=Date.now();log('Telemetry snapshot')}}catch(e){$('conn').textContent='Disconnected';$('conn').className='pill bad'}setTimeout(poll,350)}poll();resizeCanvas();
</script></body></html>
)ROBOTHTML";

void addTripEvent(uint8_t type) {
  TripEvent &e=tripEvents[tripEventHead]; e.t=millis(); e.node=(currentNode>=0?currentNode:0); e.eventType=type;
  e.predErr=lastPredictionError; e.vib=vibrationEnergy; e.tof=lastDistanceCm;
  tripEventHead=(tripEventHead+1)%MAX_TRIP_EVENTS; if(tripEventCount<MAX_TRIP_EVENTS) tripEventCount++;
}
void setMessage(const char *s) { strncpy(statusMessage,s,sizeof(statusMessage)-1); statusMessage[sizeof(statusMessage)-1]='\0'; }
const char* modeName() { return robotMode==MODE_EXPLORE?"Exploration":robotMode==MODE_DESTINATION?"Destination":"Manual"; }
void classifyRoad(MapEdge &e) {
  if(e.vibMean < 0.055f) strncpy(e.roadType,"smooth",sizeof(e.roadType));
  else if(e.vibMean < 0.16f) strncpy(e.roadType,"medium",sizeof(e.roadType));
  else strncpy(e.roadType,"rough",sizeof(e.roadType));
  e.roadType[sizeof(e.roadType)-1]='\0';
}
int findFreeNode() { for(int i=0;i<MAX_MAP_NODES;i++) if(!mapNodes[i].used) return i; return -1; }
int findEdge(int a,int b) { for(int i=0;i<MAX_MAP_EDGES;i++) if(mapEdges[i].used && ((mapEdges[i].a==a&&mapEdges[i].b==b)||(mapEdges[i].a==b&&mapEdges[i].b==a))) return i; return -1; }
int connectNodes(int a,int b) {
  if(a<0||b<0||a==b) return -1;
  int e=findEdge(a,b); if(e>=0) { mapEdges[e].traversals++; mapEdges[e].lastSeen=millis(); mapEdges[e].blocked=false; return e; }
  for(int i=0;i<MAX_MAP_EDGES;i++) if(!mapEdges[i].used) { MapEdge &m=mapEdges[i]; m.used=true;m.a=a;m.b=b;m.traversals=1;m.blocked=false;m.vibMean=0;m.vibPeak=0;m.obstacleProbability=0;m.reliability=.5f;m.estimatedCost=1;m.lastSeen=millis();strncpy(m.roadType,"unknown",sizeof(m.roadType));return i; }
  return -1;
}
int nearestNode(float x,float y) {
  int best=-1; float bd=0.60f;
  for(int i=0;i<MAX_MAP_NODES;i++) if(mapNodes[i].used) { float dx=mapNodes[i].x-x,dy=mapNodes[i].y-y,d=sqrtf(dx*dx+dy*dy); if(d<bd) {bd=d;best=i;} }
  return best;
}
void updateEdgeProfile() {
  if(currentEdge<0 || currentEdge>=MAX_MAP_EDGES || !mapEdges[currentEdge].used || roadSamples==0) return;
  MapEdge &e=mapEdges[currentEdge]; float observed=roadVibSum/roadSamples;
  e.vibMean=e.traversals<=1?observed:(0.75f*e.vibMean+0.25f*observed);
  e.vibPeak=max(e.vibPeak,roadVibPeak);
  float op=(float)roadObstacleSamples/max(1UL,roadSamples);
  e.obstacleProbability=0.8f*e.obstacleProbability+0.2f*op;
  e.reliability=constrain(0.92f*e.reliability+0.08f*(e.blocked?0.0f:1.0f),0.0f,1.0f);
  e.estimatedCost=1.0f+e.vibMean*3.0f+e.obstacleProbability*4.0f+(1.0f-e.reliability)*2.0f;
  e.lastSeen=millis(); classifyRoad(e);
}
void addMapNodeAt(float x,float y,bool destinationMarker) {
  int newId=nearestNode(x,y);
  if(newId<0) {
    newId=findFreeNode(); if(newId<0) {setMessage("Map node limit reached; map is bounded");return;}
    MapNode &n=mapNodes[newId]; n.used=true;n.id=newId;n.x=x;n.y=y;n.visits=0;n.confidence=35;n.destinationIndex=-1;n.lastSeen=millis();junctionCount++;
  }
  MapNode &n=mapNodes[newId]; n.visits++; n.lastSeen=millis(); n.confidence=(uint8_t)min(100,(int)n.confidence+4);
  if(currentNode>=0 && currentNode!=newId) {
    updateEdgeProfile();
    previousNode=currentNode; currentNode=newId; currentEdge=connectNodes(previousNode,currentNode);
  } else if(currentNode<0) currentNode=newId;
  lastMapX=n.x;lastMapY=n.y;lastMapEventMs=millis();
  roadVibSum=0;roadVibPeak=0;roadSamples=0;roadObstacleSamples=0;roadElapsedMs=0;
  addTripEvent(destinationMarker?2:1); mapNeedsUpdate=true;
  if(destinationMarker) {markerLatched=true;driveEnabled=false;webDriveRequested=false;motorStop();setMessage("Destination marker detected; assign its name in the phone GUI");}
}
void observeMap() {
  uint32_t now=millis();
  if(lastMapSampleMs==0) lastMapSampleMs=now;
  uint32_t dt=now-lastMapSampleMs;lastMapSampleMs=now;
  if(motorAction!=0) roadElapsedMs+=dt;
  roadVibSum+=vibrationEnergy;roadSamples++;roadVibPeak=max(roadVibPeak,vibrationEnergy);
  bool markerPattern=(irMask==7 && tofOK && isfinite(lastDistanceCm));
  // A marker can be approached slowly only when the all-IR marker pattern is present.
  if(markerPattern && lastDistanceCm<=5.0f && !markerLatched) {
    // If a junction was just registered at this marker, associate the marker with
    // that same node instead of manufacturing a duplicate destination node.
    if(currentNode>=0 && junctionLatched && now-lastMapEventMs<1800){markerLatched=true;driveEnabled=false;webDriveRequested=false;motorStop();setMessage("Destination marker detected at current junction; assign its name in the phone GUI");addTripEvent(2);return;}
    float x=0,y=0;if(currentNode>=0){x=mapNodes[currentNode].x+sin(relativeHeadingDeg*0.0174533f)*1.0f;y=mapNodes[currentNode].y-cos(relativeHeadingDeg*0.0174533f)*1.0f;}
    addMapNodeAt(x,y,true); return;
  }
  if(isfinite(lastDistanceCm) && lastDistanceCm<30.0f) roadObstacleSamples++;
  if(markerPattern && lastDistanceCm<=12.0f && lastDistanceCm>5.0f) {
    // Creep to the marker at low speed. At <=5 cm the marker is latched and stopped.
    if(driveEnabled && robotMode!=MODE_MANUAL) motorForward(MIN_PWM,MIN_PWM);
    return;
  }
  if(irMask==7) {
    if(allBlackSince==0) allBlackSince=now;
    nonBlackSince=0;
    if(!junctionLatched && now-allBlackSince>=280 && now-lastMapEventMs>700) {
      float step=1.5f;float rad=relativeHeadingDeg*0.0174533f;
      float x=(currentNode>=0?mapNodes[currentNode].x:0)+sin(rad)*step;
      float y=(currentNode>=0?mapNodes[currentNode].y:0)-cos(rad)*step;
      addMapNodeAt(x,y,false);junctionLatched=true;
      if(robotMode==MODE_EXPLORE && driveEnabled) {
        // Simple exploration policy: vary turns at newly observed junctions.
        exploreTurnCounter++; pendingTurn=(exploreTurnCounter%3==1)?-1:(exploreTurnCounter%3==2)?1:0;
        if(pendingTurn!=0) turnUntilMs=now+320;
      }
    }
  } else {
    allBlackSince=0;
    if(nonBlackSince==0) nonBlackSince=now;
    if(now-nonBlackSince>100) {junctionLatched=false;if(irMask!=0)markerLatched=false;}
  }
  // Repeated close-obstacle evidence makes the current edge temporarily unavailable.
  if(currentEdge>=0 && currentEdge<MAX_MAP_EDGES && mapEdges[currentEdge].used && isfinite(lastDistanceCm) && lastDistanceCm<=SAFE_STOP_CM && irMask!=7) {
    if(roadObstacleSamples>4) {mapEdges[currentEdge].blocked=true;mapEdges[currentEdge].reliability=max(0.0f,mapEdges[currentEdge].reliability-0.15f);setMessage("Obstacle detected; current road marked blocked");mapNeedsUpdate=true;}
  }
  updateEdgeProfile();
}
int nextHop(int from,int target) {
  if(from<0||target<0||from>=MAX_MAP_NODES||target>=MAX_MAP_NODES||!mapNodes[from].used||!mapNodes[target].used) return -1;
  float dist[MAX_MAP_NODES];int prev[MAX_MAP_NODES];bool used[MAX_MAP_NODES];
  for(int i=0;i<MAX_MAP_NODES;i++){dist[i]=1e9f;prev[i]=-1;used[i]=false;}dist[from]=0;
  for(int k=0;k<MAX_MAP_NODES;k++) {int u=-1;float best=1e9f;for(int i=0;i<MAX_MAP_NODES;i++)if(mapNodes[i].used&&!used[i]&&dist[i]<best){best=dist[i];u=i;}if(u<0)break;if(u==target)break;used[u]=true;
    for(int e=0;e<MAX_MAP_EDGES;e++)if(mapEdges[e].used){MapEdge &m=mapEdges[e];int v=(m.a==u)?m.b:(m.b==u)?m.a:-1;if(v<0||m.blocked)continue;float cost=max(0.2f,m.estimatedCost)+(1.0f-m.reliability)*2.0f;if(dist[u]+cost<dist[v]){dist[v]=dist[u]+cost;prev[v]=u;}}
  }
  if(prev[target]<0)return -1;int n=target;while(prev[n]!=from&&prev[n]>=0)n=prev[n];return prev[n]==from?n:-1;
}
void scheduleDestinationTurn() {
  if(robotMode!=MODE_DESTINATION||destinationTarget<0||destinationTarget>=MAX_DESTINATIONS||!destinations[destinationTarget].used||currentNode<0)return;
  int target=destinations[destinationTarget].nodeId;int hop=nextHop(currentNode,target);if(hop<0){setMessage("No known unblocked route; switch to exploration");return;}if(hop==previousNode)return;
  float dx=mapNodes[hop].x-mapNodes[currentNode].x,dy=mapNodes[currentNode].y-mapNodes[hop].y;float bearing=atan2f(dx,dy)*57.29578f;float diff=bearing-relativeHeadingDeg;while(diff>180)diff-=360;while(diff<-180)diff+=360;
  if(diff>35){pendingTurn=1;turnUntilMs=millis()+350;}else if(diff<-35){pendingTurn=-1;turnUntilMs=millis()+350;}else pendingTurn=0;
}
void jsonString(String &out,const char *s) { out+='"';for(const char*p=s;*p;p++){if(*p=='"'||*p=='\\')out+='\\';if((uint8_t)*p>=32)out+=*p;}out+='"'; }
void sendApi() {
  String j; j.reserve(7000);j="{";
  j+="\"mode\":";jsonString(j,modeName());j+=",\"currentNode\":"+String(currentNode)+",\"tofValid\":"+(isfinite(lastDistanceCm)?"true":"false")+",\"tof\":"+String(isfinite(lastDistanceCm)?lastDistanceCm:0,1);
  j+=",\"vibration\":"+String(vibrationEnergy,3)+",\"pwm\":"+String(max(abs((int)0),max(MIN_PWM,MAX_PWM-(int)(lastPredictionError*55)-(int)(vibrationEnergy*90))))+",\"heading\":"+String(relativeHeadingDeg,1)+",\"headingUncertainty\":"+String(headingUncertaintyDeg,1);
  j+=",\"predictionError\":"+String(lastPredictionError,3)+",\"confidence\":"+String(predictionConfidence,3)+",\"synapses\":"+String(synapseCount)+",\"concepts\":";int cc=0;for(auto &c:concepts)if(c.used)cc++;j+=String(cc);
  j+=",\"tofOK\":";j+=(tofOK?"true":"false");j+=",\"mpuOK\":";j+=(mpuOK?"true":"false");j+=",\"irMask\":"+String(irMask)+",\"closingSpeed\":"+String(closingSpeedCmS,1)+",\"safety\":";j+=((safetyCheck()||(!driveEnabled))?"true":"false");j+=",\"marker\":";j+=(markerLatched?"true":"false");j+=",\"message\":";jsonString(j,statusMessage);
  j+=",\"mapUncertainty\":"+String(headingUncertaintyDeg>60?"high":headingUncertaintyDeg>25?"medium":"lower");j+=",\"trips\":"+String(tripCount)+",\"successes\":"+String(successfulTrips)+",\"failures\":"+String(failedTrips);
  j+=",\"nodes\":[";bool first=true;for(int i=0;i<MAX_MAP_NODES;i++)if(mapNodes[i].used){if(!first)j+=',';first=false;j+="{\"used\":true,\"id\":"+String(mapNodes[i].id)+",\"x\":"+String(mapNodes[i].x,2)+",\"y\":"+String(mapNodes[i].y,2)+",\"visits\":"+String(mapNodes[i].visits)+",\"confidence\":"+String(mapNodes[i].confidence)+",\"destinationIndex\":"+String(mapNodes[i].destinationIndex)+"}";}j+="]";
  j+=",\"edges\":[";first=true;for(int i=0;i<MAX_MAP_EDGES;i++)if(mapEdges[i].used){if(!first)j+=',';first=false;j+="{\"used\":true,\"a\":"+String(mapEdges[i].a)+",\"b\":"+String(mapEdges[i].b)+",\"blocked\":"+(mapEdges[i].blocked?"true":"false")+",\"reliability\":"+String(mapEdges[i].reliability,2)+",\"roadType\":";jsonString(j,mapEdges[i].roadType);j+="}";}j+="]";
  j+=",\"destinations\":[";first=true;for(int i=0;i<MAX_DESTINATIONS;i++)if(destinations[i].used){if(!first)j+=',';first=false;j+="{\"used\":true,\"slot\":"+String(i)+",\"name\":";jsonString(j,destinations[i].name);j+=",\"nodeId\":"+String(destinations[i].nodeId)+"}";}j+="]}";
  web.send(200,"application/json",j);
}
void assignDestination(const String &name) {
  if(!markerLatched||currentNode<0){setMessage("No confirmed marker. Reach marker first.");return;}
  int slot=-1;for(int i=0;i<MAX_DESTINATIONS;i++)if(destinations[i].used&&name.equalsIgnoreCase(destinations[i].name)){slot=i;break;}
  if(slot<0)for(int i=0;i<MAX_DESTINATIONS;i++)if(!destinations[i].used){slot=i;break;}
  if(slot<0){setMessage("Six destination slots used; rename/reassign an existing destination");return;}
  if(destinations[slot].used&&destinations[slot].nodeId>=0&&destinations[slot].nodeId<MAX_MAP_NODES)mapNodes[destinations[slot].nodeId].destinationIndex=-1;
  destinations[slot].used=true;name.substring(0,19).toCharArray(destinations[slot].name,20);destinations[slot].nodeId=currentNode;destinations[slot].hits++;mapNodes[currentNode].destinationIndex=slot;markerLatched=false;setMessage("Destination name associated with current marker, junction and road context");addTripEvent(3);mapNeedsUpdate=true;
}
void handleCommand() {
  String c=web.arg("cmd");
  if(c=="stop"){driveEnabled=false;webDriveRequested=false;manualCommand=0;motorStop();setMessage("Drive disabled by stop command");}
  else if(c=="enable"){if(!mpuOK||!tofOK){setMessage("Cannot enable: sensor fault");}else{driveEnabled=true;webDriveRequested=true;if(lastTripStartMs==0){lastTripStartMs=millis();tripCount++;}setMessage("Drive enabled; safety interlocks active");}}
  else if(c=="mode_explore"){robotMode=MODE_EXPLORE;destinationTarget=-1;manualCommand=0;setMessage("Exploration mode selected");}
  else if(c=="mode_dest"){robotMode=MODE_DESTINATION;manualCommand=0;setMessage("Destination mode selected; choose a learned destination");}
  else if(c=="mode_manual"){robotMode=MODE_MANUAL;manualCommand=0;motorStop();setMessage("Manual mode selected; hold a direction button");}
  else if(c=="manual_forward"){if(robotMode==MODE_MANUAL&&driveEnabled)manualCommand=1;}
  else if(c=="manual_reverse"){if(robotMode==MODE_MANUAL&&driveEnabled)manualCommand=2;}
  else if(c=="manual_left"){if(robotMode==MODE_MANUAL&&driveEnabled)manualCommand=3;}
  else if(c=="manual_right"){if(robotMode==MODE_MANUAL&&driveEnabled)manualCommand=4;}
  else if(c=="manual_stop"){manualCommand=0;motorStop();}
  else if(c=="go"){int t=web.arg("target").toInt();if(t>=0&&t<MAX_DESTINATIONS&&destinations[t].used){destinationTarget=t;robotMode=MODE_DESTINATION;setMessage("Destination route requested");}else setMessage("Invalid destination selection");}
  else if(c=="assign"){assignDestination(web.arg("name"));}
  else if(c=="heading_reset"){relativeHeadingDeg=0;headingUncertaintyDeg=0;setMessage("Relative heading reference reset; not an absolute compass");}
  else if(c=="reset_map"){motorStop();driveEnabled=false;webDriveRequested=false;memset(mapNodes,0,sizeof(mapNodes));memset(mapEdges,0,sizeof(mapEdges));memset(destinations,0,sizeof(destinations));currentNode=previousNode=currentEdge=-1;junctionCount=0;markerLatched=false;junctionLatched=false;tripEventCount=tripEventHead=0;setMessage("Session map and destinations reset");}
  else if(c=="reset_memory"){
    motorStop();driveEnabled=false;webDriveRequested=false;manualCommand=0;destinationTarget=-1;
    memset(mapNodes,0,sizeof(mapNodes));memset(mapEdges,0,sizeof(mapEdges));memset(destinations,0,sizeof(destinations));memset(tripEvents,0,sizeof(tripEvents));
    currentNode=previousNode=currentEdge=-1;junctionCount=0;markerLatched=false;junctionLatched=false;lineLost=false;
    tripEventCount=tripEventHead=0;tripCount=successfulTrips=failedTrips=0;lastTripStartMs=0;
    lastMapX=lastMapY=0;roadVibSum=roadVibPeak=0;roadSamples=roadObstacleSamples=roadElapsedMs=0;
    clearSessionLearning();buildNetwork();lastNovelty=1.0f;lastPredictionError=1.0f;predictionConfidence=0.0f;
    relativeHeadingDeg=0;headingUncertaintyDeg=0;setMessage("All robot session memory erased; fresh SNN and map initialized");
  }
  else if(c=="mark_success"){successfulTrips++;if(currentEdge>=0&&currentEdge<MAX_MAP_EDGES&&mapEdges[currentEdge].used)mapEdges[currentEdge].reliability=constrain(mapEdges[currentEdge].reliability+0.15f,0.0f,1.0f);lastTripStartMs=0;addTripEvent(4);setMessage("Trip marked successful; route reliability updated");}
  else if(c=="mark_failure"){failedTrips++;if(currentEdge>=0&&currentEdge<MAX_MAP_EDGES&&mapEdges[currentEdge].used){mapEdges[currentEdge].reliability=max(0.0f,mapEdges[currentEdge].reliability-0.2f);mapEdges[currentEdge].blocked=true;}lastTripStartMs=0;addTripEvent(5);setMessage("Trip marked failed; current route reliability reduced");}
  else if(c=="move_node"){int id=web.arg("id").toInt();float x=web.arg("x").toFloat(),y=web.arg("y").toFloat();if(id>=0&&id<MAX_MAP_NODES&&mapNodes[id].used){mapNodes[id].x=constrain(x,-50.0f,50.0f);mapNodes[id].y=constrain(y,-50.0f,50.0f);setMessage("Map display layout updated");}}
  web.send(200,"text/plain","OK");
}
void setupWebUI() {
  WiFi.mode(WIFI_AP);WiFi.softAP(AP_SSID,AP_PASSWORD);delay(100);
  web.on("/",HTTP_GET,[]{web.send_P(200,"text/html; charset=utf-8",DASHBOARD_HTML);});
  web.on("/api",HTTP_GET,sendApi);
  web.on("/cmd",HTTP_GET,handleCommand);
  web.onNotFound([](){web.send(404,"text/plain","Not found");});web.begin();
  IPAddress ip=WiFi.softAPIP();Serial.print("Phone GUI Wi-Fi: ");Serial.println(AP_SSID);Serial.print("Password: ");Serial.println(AP_PASSWORD);Serial.print("Open http://");Serial.println(ip);
}
void serviceManualAndTurns() {
  if(!driveEnabled){motorStop();return;}
  // Controlled manual reverse is allowed as a retreat from a forward-facing ToF obstacle,
  // but only while both safety sensors are valid.
  if(robotMode==MODE_MANUAL && manualCommand==2 && mpuOK && tofOK && isfinite(lastDistanceCm)){motorReverse(80);safetyLatched=false;return;}
  if(!safetyCheck()){motorStop();safetyLatched=true;return;}
  safetyLatched=false;
  if(robotMode==MODE_MANUAL){switch(manualCommand){case 1:motorForward(115,115);break;case 2:motorReverse(95);break;case 3:digitalWrite(PIN_IN1,LOW);digitalWrite(PIN_IN2,HIGH);digitalWrite(PIN_IN3,HIGH);digitalWrite(PIN_IN4,LOW);analogWrite(PIN_ENA,95);analogWrite(PIN_ENB,95);motorAction=2;break;case 4:digitalWrite(PIN_IN1,HIGH);digitalWrite(PIN_IN2,LOW);digitalWrite(PIN_IN3,LOW);digitalWrite(PIN_IN4,HIGH);analogWrite(PIN_ENA,95);analogWrite(PIN_ENB,95);motorAction=3;break;default:motorStop();}return;}
  if(turnUntilMs>millis()&&pendingTurn!=0){if(pendingTurn<0){digitalWrite(PIN_IN1,LOW);digitalWrite(PIN_IN2,HIGH);digitalWrite(PIN_IN3,HIGH);digitalWrite(PIN_IN4,LOW);}else{digitalWrite(PIN_IN1,HIGH);digitalWrite(PIN_IN2,LOW);digitalWrite(PIN_IN3,LOW);digitalWrite(PIN_IN4,HIGH);}analogWrite(PIN_ENA,90);analogWrite(PIN_ENB,90);motorAction=pendingTurn<0?2:3;return;}else if(turnUntilMs!=0){turnUntilMs=0;pendingTurn=0;}
  lineFollowingController();
}

// ---------------------------- Diagnostics ----------------------------------
void printStatus() {
  Serial.print("IR="); Serial.print(irMask,BIN);
  Serial.print(" ToF_cm="); if(isfinite(lastDistanceCm)) Serial.print(lastDistanceCm,1); else Serial.print("INVALID");
  Serial.print(" close_cm_s="); Serial.print(closingSpeedCmS,1);
  Serial.print(" vib_g_proxy="); Serial.print(vibrationEnergy,3);
  Serial.print(" heading_deg="); Serial.print(relativeHeadingDeg,1);
  Serial.print(" heading_unc_deg="); Serial.print(headingUncertaintyDeg,1);
  Serial.print(" pred_err="); Serial.print(lastPredictionError,3);
  Serial.print(" pred_conf="); Serial.print(predictionConfidence,3);
  Serial.print(" concept="); Serial.print(activeConcept);
  Serial.print(" concept_hits="); if(activeConcept>=0) Serial.print(concepts[activeConcept].hits); else Serial.print(0);
  Serial.print(" neurons_assoc_spikes="); int n=0; for(int i=0;i<N_ASSOC;++i) if(spikes[N_IN+i]) n++;
  Serial.print(n); Serial.print(" synapses="); Serial.print(synapseCount);
  Serial.print(" action="); Serial.print(motorAction);
  Serial.print(" safety="); Serial.println(safetyLatched?"STOP":"OK");
}
void printLearningSummary() {
  Serial.println("--- SESSION-ONLY SNN SUMMARY ---");
  Serial.print("synapses="); Serial.println(synapseCount);
  Serial.print("concepts="); int count=0; for(int c=0;c<CONCEPT_SLOTS;++c) if(concepts[c].used) count++;
  Serial.println(count);
  Serial.print("prediction_error="); Serial.println(lastPredictionError,4);
  Serial.print("confidence_proxy="); Serial.println(predictionConfidence,4);
  Serial.println("No learned state is saved across reboot.");
}

// -------------------------------- Setup ------------------------------------
void setup() {
  Serial.begin(115200); delay(300);
  pinMode(PIN_ENA,OUTPUT); pinMode(PIN_IN1,OUTPUT); pinMode(PIN_IN2,OUTPUT);
  pinMode(PIN_ENB,OUTPUT); pinMode(PIN_IN3,OUTPUT); pinMode(PIN_IN4,OUTPUT);
  pinMode(PIN_IR_L,INPUT); pinMode(PIN_IR_C,INPUT); pinMode(PIN_IR_R,INPUT);
  motorStop();
  randomSeed((uint32_t)micros());
  MPUWire.begin(PIN_MPU_SDA,PIN_MPU_SCL,400000);
  ToFWire.begin(PIN_TOF_SDA,PIN_TOF_SCL,400000);
  mpuOK=mpu.begin(0x68,&MPUWire);
  if(mpuOK) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    calibrateGyro();
  }
  tof.setBus(&ToFWire);
  tof.setTimeout(80);
  tofOK=tof.init();
  if(tofOK) { tof.setMeasurementTimingBudget(33000); tof.startContinuous(50); }
  clearSessionLearning(); buildNetwork();
  sensorsReady=mpuOK && tofOK;
  Serial.println("Feature 1 SNN prototype started. Motors remain stopped until valid sensors.");
  Serial.print("MPU6050="); Serial.print(mpuOK?"OK":"FAIL");
  Serial.print(" VL53L0X="); Serial.println(tofOK?"OK":"FAIL");
  Serial.println("Commands: s=disable/stop, g=enable line-following, r=learning summary, h=heading reset");
  motorStop(); safetyLatched=true;
  memset(mapNodes,0,sizeof(mapNodes)); memset(mapEdges,0,sizeof(mapEdges)); memset(destinations,0,sizeof(destinations));
  setupWebUI();
  setMessage("Ready. Connect phone to robot Wi-Fi; motors disabled");
  lastLoopMs=millis(); lastSensorMs=millis(); lastMapSampleMs=millis();
}

// -------------------------------- Loop -------------------------------------
void loop() {
  web.handleClient();
  if(Serial.available()) {
    char ch=(char)Serial.read();
    if(ch=='s'||ch=='S'){driveEnabled=false;manualCommand=0;motorStop();safetyLatched=true;setMessage("Stopped from serial");Serial.println("STOP: drive disabled");}
    else if(ch=='g'||ch=='G'){if(mpuOK&&tofOK){driveEnabled=true;setMessage("Drive enabled from serial");}else Serial.println("Cannot enable: sensor fault");}
    else if(ch=='r'||ch=='R')printLearningSummary();
    else if(ch=='h'||ch=='H'){relativeHeadingDeg=0;headingUncertaintyDeg=0;setMessage("Relative heading reset");}
  }
  uint32_t now=millis(); if(now-lastLoopMs<LOOP_MS)return;lastLoopMs=now;
  readSensors();
  encodeSensors();computePredictions();updateConcepts();encodeActionOutcome();runLIFAndSTDP();errorModulatedLearning();
  if(!mpuOK||!tofOK){motorStop();safetyLatched=true;driveEnabled=false;setMessage("Sensor fault: motors disabled");}
  observeMap();
  if(robotMode==MODE_DESTINATION && destinationTarget>=0 && destinationTarget<MAX_DESTINATIONS && destinations[destinationTarget].used && currentNode==destinations[destinationTarget].nodeId && !markerLatched){driveEnabled=false;motorStop();if(lastTripStartMs!=0){successfulTrips++;lastTripStartMs=0;addTripEvent(4);}setMessage("Destination reached according to estimated map location");}
  if(markerLatched){motorStop();driveEnabled=false;manualCommand=0;}
  else if(driveEnabled) serviceManualAndTurns(); else {motorStop();safetyLatched=true;}
  // If destination mode is active and a junction has just been registered, choose the next known hop.
  if(robotMode==MODE_DESTINATION && junctionLatched && now-lastMapEventMs<80 && driveEnabled) scheduleDestinationTurn();
  previousMotorAction=motorAction;
  if(now-lastPrintMs>=500){lastPrintMs=now;printStatus();}
}
