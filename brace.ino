// =====================================================================
//  Patient device / brace (ESP32) — FINAL
// =====================================================================
//
//  SOUND GUIDE (brace) — passive buzzer, uses pitch
//  ---------------------------------------------------------------------
//  Start/end position saved .... one 250 ms mid tone (1200 Hz)
//  Calibration rejected ........ two short mid tones (1200 Hz)
//  Good rep .................... short high chirp (2800 Hz)
//  Bad rep / timeout ........... long low tone (500 Hz)
//  Too fast .................... glide UP (800 -> 1400 Hz)
//  Too slow .................... glide DOWN (1400 -> 800 Hz)
//  Pause ....................... falling two-tone (1500 -> 1000 Hz)
//  Resume ...................... rising two-tone (1000 -> 1500 Hz)
//  Reset ....................... one 500 ms tone (900 Hz)
//  Reconnect attempt ........... one short high tick (1800 Hz)
//  Alert sent / instructor call  three quick high chirps (2500 Hz)
//  Instructor acknowledged ..... two high chirps (2800 Hz)
//  Pager connected ............. quick rising run (1600 -> 2100 -> 2800 Hz)
//  No pager / no answer ........ two low beeps (400 Hz)
//
//  BUTTONS
//  ---------------------------------------------------------------------
//  Set (GPIO19):   1st tap = save start, 2nd tap = save end,
//                  then tap = pause/resume, hold 3 s = call instructor (any time)
//  Reset (GPIO18): if the pager link is down -> reconnect only (keeps positions)
//                  if the pager link is up   -> clear positions + counter
//                                               (and quietly re-say hello)
//
//  PACE
//  ---------------------------------------------------------------------
//  First 3 good reps set the baseline (median). Faster than 0.5x baseline
//  = too fast, slower than 2x baseline = too slow.
//
//  NOISE HANDLING
//  ---------------------------------------------------------------------
//  The pot shares the 3.3 V rail with the buzzer and radio. Rep tracking
//  pauses while the buzzer sounds or the radio switches/transmits, plus
//  SETTLE_MS afterwards, so those never create false bad reps.
//
//  Radio is off except at power-on, on a reconnect/reset hello,
//  and while an alert/call is being handled.
// =====================================================================
//  ESP32 Arduino core 3.x (ESP-NOW receive callback with esp_now_recv_info_t, LEDC ledcAttach API).

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---------- Pins ----------
const int POT_PIN    = 34;   // ADC1 input-only pin, pot wiper
const int SET_PIN    = 19;   // 10k pull-down, HIGH = pressed
const int RESET_PIN  = 18;   // 10k pull-down, HIGH = pressed
const int BUZZER_PIN = 26;   // PNP driver: PWM LOW = on, HIGH = off

// ---------- Link settings (must match the pager) ----------
const uint8_t MAGIC        = 0xA5;
const uint8_t PAIR_ID      = 1;        // give each brace/pager pair its own ID
const int     WIFI_CHANNEL = 1;

// ---------- Tuning ----------
const float ZONE_PERCENT             = 0.07;   // zone = 7% of range...
const float ZONE_FLOOR_DEG           = 4.0;    // ...but never less than this
const float ZONE_TOLERANCE_DEG       = 2.0;    // extra noise tolerance added to each zone
const float EXIT_MARGIN_DEG          = 2.0;    // hysteresis: must go this far past the zone to leave it
const float MIN_RANGE_DEG            = 16.0;   // must leave a real middle section between the zones
const float TURN_THRESH_DEG          = 3.0;    // movement needed to confirm a direction switch
const float STILL_DEG                = 1.0;    // movement smaller than this counts as still
const unsigned long STILL_MS         = 800;    // stopped this long mid-range = bad rep
const unsigned long REP_TIMEOUT_MS   = 8000;   // max time away from both zones before a bad rep
const int ALERT_THRESHOLD            = 3;      // bad reps in a row before the instructor is paged
const unsigned long RESEND_MS        = 5000;   // resend unconfirmed pager messages this often...
const unsigned long RESEND_WINDOW_MS = 60000;  // ...for this long
const unsigned long HELLO_RETRY_MS   = 1000;   // hello: retry this often...
const unsigned long HELLO_WINDOW_MS  = 5000;   // ...for this long
const unsigned long BREAK_RESET_MS   = 10000;  // user pause longer than this resets the counter
const unsigned long DEBOUNCE_MS      = 40;
const unsigned long LONG_PRESS_MS    = 3000;   // hold set button this long to call the instructor
const float PACE_LOW                 = 0.50;   // faster than half the baseline time = too fast
const float PACE_HIGH                = 2.00;   // more than twice the baseline time = too slow

// ---------- Pot filtering / noise handling ----------
const int ADC_SAMPLES                = 15;     // samples per reading, spread over time
const unsigned long ADC_GAP_US       = 300;    // gap between samples (~4.5 ms per reading)
const float SPIKE_DEG                = 15.0;   // a jump bigger than this between loops is noise...
const int SPIKE_MAX_REJECT           = 3;      // ...unless it repeats this many times in a row
const unsigned long SETTLE_MS        = 150;    // quiet time after buzzer/radio activity

// ---------- Debug ----------
const bool DEBUG_REP = true;
const unsigned long DEBUG_MS = 200;

// ---------- Types ----------
struct CalPoint   { int counts; float degrees; };
struct Button     { int pin; bool stable; bool lastReading; unsigned long lastChange; };
enum ButtonEvent  { NO_EVENT, PRESSED_EV, RELEASED_EV };
struct ToneStep   { int freq; unsigned long ms; };   // freq 0 = silence
struct Sweep      { int fromHz; int toHz; unsigned long ms; };
enum Zone         { Z_NONE, Z_START, Z_END };

enum MsgType : uint8_t {
  MSG_ALERT   = 1,   // patient -> pager: bad reps, session auto-paused
  MSG_RESUMED = 2,   // patient -> pager: patient resumed on their own (or reset)
  MSG_CALL    = 3,   // patient -> pager: patient held the button to call
  MSG_CLEARED = 4,   // pager -> patient: instructor cleared the alert
  MSG_ACK     = 5,   // either way: confirms a message by sequence number
  MSG_HELLO   = 6    // patient -> pager: handshake (power-on / reconnect / reset)
};

struct __attribute__((packed)) Packet {
  uint8_t  magic;
  uint8_t  pairId;
  uint8_t  type;
  uint16_t seq;
};

// ---------- Noise window ----------
unsigned long quietUntil = 0;

void markNoisy() {
  quietUntil = millis() + SETTLE_MS;
}

// ---------- Calibration: counts -> degrees (averaged up/down sweeps), sorted by counts ----------
CalPoint cal[] = {
  {  514,   0.0 },
  {  840,  22.5 },
  { 1199,  45.0 },
  { 1509,  67.5 },
  { 1815,  90.0 },
  { 2134, 112.5 },
  { 2474, 135.0 },
  { 2700, 150.0 },
};
const int CAL_N = sizeof(cal) / sizeof(cal[0]);

float countsToDegrees(int c) {
  for (int i = 0; i < CAL_N - 1; i++) {
    int a = cal[i].counts;
    int b = cal[i + 1].counts;
    if (a != b && c >= min(a, b) && c <= max(a, b)) {
      float t = (float)(c - a) / (float)(b - a);
      return cal[i].degrees + t * (cal[i + 1].degrees - cal[i].degrees);
    }
  }
  return (abs(c - cal[0].counts) < abs(c - cal[CAL_N - 1].counts))
           ? cal[0].degrees
           : cal[CAL_N - 1].degrees;
}

// Samples spread over time, sorted, middle third averaged
int readCountsFiltered() {
  int s[ADC_SAMPLES];
  for (int i = 0; i < ADC_SAMPLES; i++) {
    s[i] = analogRead(POT_PIN);
    delayMicroseconds(ADC_GAP_US);
  }
  for (int i = 1; i < ADC_SAMPLES; i++) {        // insertion sort
    int v = s[i];
    int j = i - 1;
    while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; }
    s[j + 1] = v;
  }
  int lo = ADC_SAMPLES / 3;
  int hi = ADC_SAMPLES - ADC_SAMPLES / 3;
  long sum = 0;
  for (int i = lo; i < hi; i++) sum += s[i];
  return (int)(sum / (hi - lo));
}

float readAngleDeg() {
  return countsToDegrees(readCountsFiltered());
}

// Spike rejection for rep tracking: ignore physically impossible jumps
bool haveAngle = false;
float lastGoodAngle = 0;
int spikeRejects = 0;

float trackedAngle() {
  float a = readAngleDeg();
  if (haveAngle && fabs(a - lastGoodAngle) > SPIKE_DEG && spikeRejects < SPIKE_MAX_REJECT) {
    spikeRejects++;
    if (DEBUG_REP) Serial.printf("spike ignored: %.1f (last %.1f)\n", a, lastGoodAngle);
    return lastGoodAngle;
  }
  spikeRejects = 0;
  lastGoodAngle = a;
  haveAngle = true;
  return a;
}

// ---------- Buttons ----------
Button btnSet   = {SET_PIN,   false, false, 0};
Button btnReset = {RESET_PIN, false, false, 0};

ButtonEvent buttonEvent(Button &b) {
  bool reading = digitalRead(b.pin);
  if (reading != b.lastReading) {
    b.lastReading = reading;
    b.lastChange = millis();
  }
  if (millis() - b.lastChange > DEBOUNCE_MS && reading != b.stable) {
    b.stable = reading;
    return reading ? PRESSED_EV : RELEASED_EV;
  }
  return NO_EVENT;
}

bool buttonPressed(Button &b) {
  return buttonEvent(b) == PRESSED_EV;
}

// ---------- Sounds ----------
const int DUTY_ON  = 128;   // ~50%
const int DUTY_OFF = 256;   // fully high at 8-bit resolution = PNP off

const ToneStep SND_GOOD[]      = {{2800, 120}};
const ToneStep SND_BAD[]       = {{500, 450}};
const ToneStep SND_SET[]       = {{1200, 250}};
const ToneStep SND_CALREJ[]    = {{1200, 120}, {0, 120}, {1200, 120}};
const ToneStep SND_RESET[]     = {{900, 500}};
const ToneStep SND_SEARCH[]    = {{1800, 40}};                         // reconnect attempt
const ToneStep SND_PAUSE[]     = {{1500, 150}, {0, 50}, {1000, 150}};
const ToneStep SND_RESUME[]    = {{1000, 150}, {0, 50}, {1500, 150}};
const ToneStep SND_ALERT[]     = {{2500, 90}, {0, 70}, {2500, 90}, {0, 70}, {2500, 90}};
const ToneStep SND_INSTR_ACK[] = {{2800, 80}, {0, 80}, {2800, 80}};   // instructor acknowledged
const ToneStep SND_NO_PAGER[]  = {{400, 250}, {0, 150}, {400, 250}};  // pager not found / no answer
const ToneStep SND_CONNECTED[] = {{1600, 60}, {2100, 60}, {2800, 90}};

#define PLAY(s) play(s, sizeof(s) / sizeof(s[0]))

const ToneStep* snd = nullptr;
int sndLen = 0;
int sndIdx = 0;
unsigned long sndUntil = 0;

Sweep sw;
bool sweeping = false;
unsigned long swStart = 0;
unsigned long lastSweepStep = 0;

void stepSound() {
  ToneStep t = snd[sndIdx];
  if (t.freq > 0) {
    ledcChangeFrequency(BUZZER_PIN, t.freq, 8);
    ledcWrite(BUZZER_PIN, DUTY_ON);
  } else {
    ledcWrite(BUZZER_PIN, DUTY_OFF);
  }
  sndUntil = millis() + t.ms;
}

void play(const ToneStep* s, int n) {
  sweeping = false;               // a tone interrupts any glide
  snd = s;
  sndLen = n;
  sndIdx = 0;
  stepSound();
}

void updateSound() {
  if (snd == nullptr || millis() < sndUntil) return;
  sndIdx++;
  if (sndIdx >= sndLen) {
    snd = nullptr;
    ledcWrite(BUZZER_PIN, DUTY_OFF);
    markNoisy();                  // let the rail settle before reading the pot
    return;
  }
  stepSound();
}

void playSweep(int fromHz, int toHz, unsigned long ms) {
  snd = nullptr;                  // a glide interrupts any tone sequence
  sw = {fromHz, toHz, ms};
  swStart = millis();
  lastSweepStep = 0;
  sweeping = true;
  ledcChangeFrequency(BUZZER_PIN, fromHz, 8);
  ledcWrite(BUZZER_PIN, DUTY_ON);
}

void updateSweep() {
  if (!sweeping) return;
  unsigned long el = millis() - swStart;
  if (el >= sw.ms) {
    sweeping = false;
    ledcWrite(BUZZER_PIN, DUTY_OFF);
    markNoisy();
    return;
  }
  if (millis() - lastSweepStep >= 15) {
    lastSweepStep = millis();
    float t = (float)el / (float)sw.ms;
    int f = sw.fromHz + (int)((sw.toHz - sw.fromHz) * t);
    ledcChangeFrequency(BUZZER_PIN, f, 8);
  }
}

// Pot readings are only trusted when the buzzer is silent and the rail has settled
bool trackingQuiet() {
  return snd == nullptr && !sweeping && (long)(millis() - quietUntil) >= 0;
}

// ---------- Radio (ESP-NOW), only on when needed ----------
uint8_t BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
QueueHandle_t rxQueue;
bool radioIsOn = false;

// One counter for every outgoing message number, so they never collide
uint16_t seqCounter = 0;
uint16_t nextSeq() { return ++seqCounter; }

// Link status as far as the brace knows
bool linkUp = false;

// Outgoing message waiting for the pager to confirm
bool outActive = false;
uint8_t outType = 0;
uint16_t outSeq = 0;
unsigned long outFirstMs = 0;
unsigned long outLastMs = 0;

// An alert or call is open until the instructor clears it (radio keeps listening)
bool awaitingInstructor = false;

// Handshake (power-on, reconnect, or quiet hello on reset)
bool handshaking = false;
bool handshakeQuiet = false;   // quiet: no connected chirp if the link was already up
uint16_t helloSeq = 0;
unsigned long helloFirstMs = 0;
unsigned long helloLastMs = 0;

// Last message handled from the pager (to ignore repeats)
bool haveRxSeq = false;
uint16_t lastRxSeq = 0;

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(Packet)) return;
  Packet p;
  memcpy(&p, data, sizeof(p));
  xQueueSend(rxQueue, &p, 0);   // handled in loop(), not in the radio task
}

bool radioOn() {
  if (radioIsOn) return true;
  markNoisy();
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    WiFi.mode(WIFI_OFF);
    markNoisy();
    return false;
  }
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_ADDR, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    esp_now_deinit();
    WiFi.mode(WIFI_OFF);
    markNoisy();
    return false;
  }
  radioIsOn = true;
  markNoisy();
  Serial.println("Radio on");
  return true;
}

void radioOff() {
  if (!radioIsOn) return;
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);
  xQueueReset(rxQueue);
  radioIsOn = false;
  markNoisy();
  Serial.println("Radio off");
}

void transmit(uint8_t type, uint16_t seq) {
  if (!radioIsOn) return;
  Packet p = {MAGIC, PAIR_ID, type, seq};
  esp_now_send(BROADCAST_ADDR, (uint8_t *)&p, sizeof(p));
  markNoisy();
}

// Turns the radio on, sends, and keeps resending until the pager confirms or the window ends
void sendToPager(uint8_t type) {
  if (!radioOn()) {
    PLAY(SND_NO_PAGER);
    Serial.println("Radio failed to start");
    return;
  }
  outSeq = nextSeq();
  outType = type;
  outActive = true;
  outFirstMs = outLastMs = millis();
  transmit(type, outSeq);
  Serial.printf("-> pager: %d (seq %u)\n", type, outSeq);
}

void updateOutbox() {
  if (!outActive) return;
  unsigned long now = millis();
  if (now - outFirstMs > RESEND_WINDOW_MS) {
    outActive = false;
    awaitingInstructor = false;   // the pager never got it, so nothing to wait for
    linkUp = false;
    PLAY(SND_NO_PAGER);
    Serial.println("Pager did not answer; giving up (session stays as it is)");
    return;
  }
  if (now - outLastMs >= RESEND_MS) {
    transmit(outType, outSeq);
    outLastMs = now;
    Serial.printf("-> pager: resend %d (seq %u)\n", outType, outSeq);
  }
}

void startHandshake(bool quiet) {
  if (handshaking) return;
  if (!radioOn()) {
    linkUp = false;
    PLAY(SND_NO_PAGER);
    Serial.println("Radio failed to start");
    return;
  }
  handshaking = true;
  handshakeQuiet = quiet;
  helloSeq = nextSeq();
  helloFirstMs = helloLastMs = millis();
  transmit(MSG_HELLO, helloSeq);
  Serial.println("Looking for pager...");
}

void updateHandshake() {
  if (!handshaking) return;
  unsigned long now = millis();
  if (now - helloFirstMs > HELLO_WINDOW_MS) {
    handshaking = false;
    linkUp = false;
    PLAY(SND_NO_PAGER);
    Serial.println("No pager found");
    return;
  }
  if (now - helloLastMs >= HELLO_RETRY_MS) {
    transmit(MSG_HELLO, helloSeq);
    helloLastMs = now;
  }
}

// Radio goes off as soon as nothing needs it
void updateRadioPower() {
  if (radioIsOn && !outActive && !awaitingInstructor && !handshaking) radioOff();
}

void onPagerCleared();   // defined below

void processRadio() {
  if (!radioIsOn) return;
  Packet p;
  while (xQueueReceive(rxQueue, &p, 0) == pdTRUE) {
    if (p.magic != MAGIC || p.pairId != PAIR_ID) continue;

    bool wasUp = linkUp;
    linkUp = true;   // anything from the pager means the link is up

    if (p.type == MSG_ACK) {
      if (handshaking && p.seq == helloSeq) {
        handshaking = false;
        if (!handshakeQuiet || !wasUp) PLAY(SND_CONNECTED);
        Serial.println("Pager connected");
      } else if (outActive && p.seq == outSeq) {
        outActive = false;
        Serial.printf("Pager confirmed message %d\n", outType);
      }
      continue;
    }

    if (p.type == MSG_CLEARED) {
      transmit(MSG_ACK, p.seq);                       // always confirm
      if (haveRxSeq && p.seq == lastRxSeq) continue;  // repeat, already handled
      haveRxSeq = true;
      lastRxSeq = p.seq;
      onPagerCleared();
    }
  }
}

// ---------- Session state ----------
bool startSet = false;
bool endSet   = false;
float startDeg = 0;
float endDeg   = 0;
float rangeDeg = 0;
float band     = 0;   // zone half-width including tolerance

// Zone membership with hysteresis
bool inStart = false;
bool inEnd   = false;

// Direction-switch tracking
float tpAngle = 0;          // last confirmed direction switch
float pivot   = 0;          // most extreme angle since the last switch
bool pivotZoned = false;    // was the arm inside a zone when the pivot was recorded
int dir       = 0;          // +1 rising, -1 falling, 0 unknown

// Stillness tracking
float stillAngle = 0;
unsigned long stillSinceMs = 0;

// Rep tracking
Zone lastZone = Z_NONE;          // last zone the arm was in
unsigned long lastZoneMs = 0;    // last time the arm was inside a zone
unsigned long leaveStartMs = 0;  // when the arm last left the start zone (rep timer start)
bool segPenalized = false;       // the current excursion already has a bad rep counted
int midTurns = 0;                // mid-range switches since the last zone entry
bool curlGood = false;           // start -> end was clean

bool running    = false;
bool paused     = false;
bool autoPaused = false;
unsigned long userPauseStart = 0;

int badInRow = 0;

unsigned long paceSamples[3];
int paceCount = 0;
unsigned long paceBaseline = 0;
bool hasBaseline = false;

unsigned long setPressStart = 0;
bool setLongFired = false;

unsigned long lastDebug = 0;

// ---------- Geometry ----------
// Progress along the rep: 0 at start, rangeDeg at end. Works whichever way the pot turns.
float progress(float a) {
  float s = (endDeg >= startDeg) ? 1.0 : -1.0;
  return (a - startDeg) * s;
}

// ---------- Direction-switch detection ----------
void resetTurning(float a) {
  tpAngle = a;
  pivot = a;
  pivotZoned = inStart || inEnd;
  dir = 0;
  stillAngle = a;
  stillSinceMs = millis();
}

// Returns true when the arm has clearly switched direction.
// tpZonedOut says whether the arm was inside a zone at the turning point.
bool updateTurning(float a, float &tpOut, bool &tpZonedOut) {
  bool zonedNow = inStart || inEnd;

  if (dir == 0) {
    if (a - tpAngle > TURN_THRESH_DEG)      { dir = +1; pivot = a; pivotZoned = zonedNow; }
    else if (tpAngle - a > TURN_THRESH_DEG) { dir = -1; pivot = a; pivotZoned = zonedNow; }
    return false;
  }

  bool extended = (dir == +1) ? (a > pivot) : (a < pivot);
  if (extended) {
    pivot = a;
    pivotZoned = zonedNow;
    return false;
  }

  float back = (dir == +1) ? (pivot - a) : (a - pivot);
  if (back > TURN_THRESH_DEG) {
    tpOut = pivot;
    tpZonedOut = pivotZoned;
    tpAngle = pivot;
    dir = -dir;
    pivot = a;
    pivotZoned = zonedNow;
    return true;
  }
  return false;
}

// ---------- Helpers ----------
void resetRepState() {
  inStart = inEnd = false;
  lastZone = Z_NONE;
  lastZoneMs = millis();
  leaveStartMs = millis();
  segPenalized = false;
  midTurns = 0;
  curlGood = false;
  haveAngle = false;
}

// Set zone membership from the current angle without firing any events
void syncZones(float a) {
  unsigned long now = millis();
  float p = progress(a);
  inStart = (p <= band);
  inEnd   = (p >= rangeDeg - band);
  lastZone = inStart ? Z_START : (inEnd ? Z_END : Z_NONE);
  lastZoneMs = now;
  leaveStartMs = now;
  segPenalized = false;
  midTurns = 0;
  curlGood = false;
  haveAngle = true;          // seed spike rejection with this angle
  lastGoodAngle = a;
  spikeRejects = 0;
  resetTurning(a);
}

void clearPositions() {
  startSet = endSet = false;
  running = paused = autoPaused = false;
  badInRow = 0;
  paceCount = 0;
  hasBaseline = false;
  resetRepState();
}

// Restart tracking cleanly; the first reading is taken once the rail is quiet
bool needResync = false;

void prepareResume() {
  needResync = true;
}

void startAlert() {
  autoPaused = true;
  paused = true;
  awaitingInstructor = true;
  PLAY(SND_ALERT);
  sendToPager(MSG_ALERT);
  Serial.println("ALERT: session auto-paused");
}

void badRep(const char *why) {
  badInRow++;
  Serial.printf("Bad rep (%d in a row): %s\n", badInRow, why);
  if (badInRow >= ALERT_THRESHOLD) {
    startAlert();
  } else {
    PLAY(SND_BAD);
  }
}

// ---------- Pace ----------
void evaluatePace(unsigned long dur) {
  if (!hasBaseline) {
    paceSamples[paceCount++] = dur;
    if (paceCount == 3) {
      unsigned long a = paceSamples[0], b = paceSamples[1], c = paceSamples[2];
      paceBaseline = max(min(a, b), min(max(a, b), c));   // median
      hasBaseline = true;
      Serial.printf("Pace baseline set: %lu ms\n", paceBaseline);
    }
    PLAY(SND_GOOD);
    return;
  }

  if ((float)dur < PACE_LOW * (float)paceBaseline) {
    playSweep(800, 1400, 700);     // too fast: glide up
    Serial.println("Too fast");
  } else if ((float)dur > PACE_HIGH * (float)paceBaseline) {
    playSweep(1400, 800, 700);     // too slow: glide down
    Serial.println("Too slow");
  } else {
    PLAY(SND_GOOD);
    Serial.println("Good rep");
  }
}

void goodRep(unsigned long dur) {
  badInRow = 0;
  Serial.printf("Full rep, range OK, %lu ms\n", dur);
  evaluatePace(dur);
}

// ---------- Zone events ----------
void onExitZone(Zone z, unsigned long now) {
  if (z == Z_START) leaveStartMs = now;   // the rep timer starts when the arm leaves start
  if (DEBUG_REP) Serial.printf("Left zone %d\n", (int)z);
}

void onEnterZone(Zone z, unsigned long now) {
  midTurns = 0;
  Zone prev = lastZone;
  if (DEBUG_REP) Serial.printf("Entered zone %d\n", (int)z);

  if (prev == Z_NONE) {
    lastZone = z;
    segPenalized = false;
    curlGood = false;
    return;
  }

  if (z == prev) {
    // Came back to the zone it left without reaching the other end.
    // Any bad rep for that excursion is already counted; start fresh from here.
    if (segPenalized) curlGood = false;
    segPenalized = false;
    return;
  }

  if (z == Z_END) {
    curlGood = !segPenalized;              // clean curl from start to end
  } else {
    // Back at start from the end: score the rep immediately
    if (curlGood && !segPenalized) {
      goodRep(now - leaveStartMs);
    }
    curlGood = false;
  }

  lastZone = z;
  segPenalized = false;
}

void updateMembership(float a, unsigned long now) {
  float p = progress(a);
  bool wasStart = inStart;
  bool wasEnd   = inEnd;

  // Overshooting past either end still counts as being in that zone
  inStart = wasStart ? (p <= band + EXIT_MARGIN_DEG) : (p <= band);
  inEnd   = wasEnd   ? (p >= rangeDeg - band - EXIT_MARGIN_DEG) : (p >= rangeDeg - band);

  if (wasStart && !inStart) onExitZone(Z_START, now);
  if (wasEnd && !inEnd)     onExitZone(Z_END, now);
  if (!wasStart && inStart) onEnterZone(Z_START, now);
  if (!wasEnd && inEnd)     onEnterZone(Z_END, now);

  if (inStart || inEnd) lastZoneMs = now;
}

// A direction switch while outside both zones: every other one is a bad rep
void onMiddleTurn() {
  if (lastZone == Z_NONE) return;   // no reference zone yet
  segPenalized = true;
  midTurns++;
  if (midTurns % 2 == 1) {
    badRep("switched direction mid-range");
  }
}

// Stopping mid-range, or staying away from both zones too long
void checkMidRange(float a, unsigned long now) {
  if (fabs(a - stillAngle) > STILL_DEG) {
    stillAngle = a;
    stillSinceMs = now;
  }

  if (lastZone == Z_NONE || inStart || inEnd) return;

  if (!segPenalized && now - stillSinceMs > STILL_MS) {
    segPenalized = true;
    badRep("stopped mid-range");
  }

  if (now - lastZoneMs > REP_TIMEOUT_MS) {
    segPenalized = true;
    lastZoneMs = now;
    badRep("too long away from both zones");
  }
}

// ---------- Button handlers ----------
void onSetPressed() {
  if (!startSet) {
    startDeg = readAngleDeg();
    startSet = true;
    PLAY(SND_SET);
    Serial.printf("Start saved: %.1f deg\n", startDeg);
    return;
  }

  if (!endSet) {
    endDeg = readAngleDeg();
    rangeDeg = fabs(endDeg - startDeg);
    if (rangeDeg < MIN_RANGE_DEG) {
      clearPositions();
      PLAY(SND_CALREJ);
      Serial.println("Calibration rejected: range too small");
      return;
    }
    endSet = true;
    running = true;
    band = max(ZONE_PERCENT * rangeDeg, ZONE_FLOOR_DEG) + ZONE_TOLERANCE_DEG;
    badInRow = 0;
    paceCount = 0;
    hasBaseline = false;
    syncZones(endDeg);
    PLAY(SND_SET);
    Serial.printf("End saved: %.1f deg, range %.1f, zone +/-%.1f\n", endDeg, rangeDeg, band);
    return;
  }

  // Positions are set: the set button pauses and resumes
  if (autoPaused) {
    // Resume after an alert without the instructor. Counter is kept.
    autoPaused = false;
    paused = false;
    awaitingInstructor = false;
    PLAY(SND_RESUME);
    sendToPager(MSG_RESUMED);   // stop the pager alarming, then the radio turns off
    prepareResume();
    Serial.println("Resumed after alert");
    return;
  }

  if (!paused) {
    paused = true;
    userPauseStart = millis();
    PLAY(SND_PAUSE);
    Serial.println("Paused");
    return;
  }

  if (millis() - userPauseStart > BREAK_RESET_MS) {
    badInRow = 0;
    Serial.println("Break over 10s: counter reset");
  }
  paused = false;
  prepareResume();
  PLAY(SND_RESUME);
  Serial.println("Resumed");
}

// Hold 3 s: call the instructor at any time (positions never change on a hold)
void onLongPress() {
  PLAY(SND_ALERT);
  if (outActive && outType == MSG_ALERT) {
    Serial.println("Alert already on its way to the pager");
    return;   // don't replace a pending alert with a call
  }
  awaitingInstructor = true;
  sendToPager(MSG_CALL);
  Serial.println("Instructor called");
}

void updateSetButton() {
  ButtonEvent e = buttonEvent(btnSet);

  if (e == PRESSED_EV) {
    setPressStart = millis();
    setLongFired = false;
  }

  if (btnSet.stable && !setLongFired && millis() - setPressStart >= LONG_PRESS_MS) {
    setLongFired = true;
    onLongPress();
  }

  if (e == RELEASED_EV && !setLongFired) {
    onSetPressed();
  }
}

// Reset button: reconnect if the link is down, otherwise a normal reset
void onResetPressed() {
  if (handshaking) return;   // already looking for the pager

  if (!linkUp) {
    // Reconnect only: positions and counter are kept
    PLAY(SND_SEARCH);
    startHandshake(false);
    Serial.println("Reconnecting to pager (positions kept)");
    return;
  }

  PLAY(SND_RESET);
  if (autoPaused) {
    awaitingInstructor = false;
    sendToPager(MSG_RESUMED);   // stop the pager alarming
  }
  clearPositions();
  startHandshake(true);         // quiet hello, in case the pager was rebooted
  Serial.println("Reset: positions and counter cleared");
}

// The instructor cleared the pager: they came over, so the counter resets
void onPagerCleared() {
  outActive = false;
  awaitingInstructor = false;   // radio turns off after this
  badInRow = 0;
  if (autoPaused) {
    autoPaused = false;
    paused = false;
    prepareResume();
    PLAY(SND_RESUME);
    Serial.println("Instructor cleared: session resumed, counter reset");
  } else {
    PLAY(SND_INSTR_ACK);
    Serial.println("Instructor acknowledged: counter reset");
  }
}

// ---------- Rep detection ----------
void updateRep() {
  // After a resume, take the first reading only now that the rail is quiet
  if (needResync) {
    needResync = false;
    syncZones(readAngleDeg());
    if (DEBUG_REP) Serial.println("Tracking resynced");
    return;
  }

  float a = trackedAngle();
  unsigned long now = millis();

  updateMembership(a, now);

  float tp;
  bool tpZoned;
  if (updateTurning(a, tp, tpZoned)) {
    if (DEBUG_REP) Serial.printf("switch at %.1f deg (%s)\n", tp, tpZoned ? "in zone" : "mid-range");
    if (!tpZoned) onMiddleTurn();
  }

  checkMidRange(a, now);

  if (DEBUG_REP && now - lastDebug >= DEBUG_MS) {
    lastDebug = now;
    Serial.printf("angle=%.1f inS=%d inE=%d lastZone=%d penalized=%d curlGood=%d\n",
                  a, inStart, inEnd, (int)lastZone, segPenalized, curlGood);
  }
}

// ---------- Arduino entry points ----------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(SET_PIN, INPUT);       // pull-downs are external
  pinMode(RESET_PIN, INPUT);
  ledcAttach(BUZZER_PIN, 2000, 8);
  ledcWrite(BUZZER_PIN, DUTY_OFF);   // buzzer off at boot

  rxQueue = xQueueCreate(8, sizeof(Packet));
  seqCounter = (uint16_t)esp_random();   // random start so a reboot never looks like a repeat

  Serial.printf("Patient device ready, pair ID %d\n", PAIR_ID);
  startHandshake(false);                 // power-on hello
}

void loop() {
  updateSetButton();
  if (buttonPressed(btnReset)) onResetPressed();

  processRadio();
  updateHandshake();
  updateOutbox();
  updateRadioPower();

  // Rep tracking only runs while the buzzer is silent and the radio isn't switching
  if (running && !paused && trackingQuiet()) updateRep();

  updateSound();
  updateSweep();
}
