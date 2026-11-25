// =====================================================================
//  Instructor pager (ESP32) — FINAL — ACTIVE buzzer version (fixed pitch)
// =====================================================================
//
//  SOUND GUIDE (pager) — all one pitch, told apart by rhythm
//  ---------------------------------------------------------------------
//  Boot ........................ one 400 ms beep
//  Brace connected ............. two quick blips
//  Brace disconnected .......... one long 1-second beep
//                                (brace didn't confirm a clear within 15 s)
//  BAD-REP ALERT ............... five rapid buzzes, repeated every
//                                ALERT_REPEAT_MS, LED solid ON.
//                                No timeout. Stops only when the patient
//                                resumes/resets, or the instructor clears.
//  Patient CALL (3 s hold) ..... two slow beeps, repeated every
//                                CALL_REPEAT_MS, LED BLINKING.
//                                Stops only when the instructor clears.
//  Patient resumed on own ...... short beep then long beep, LED off
//  Instructor cleared (button) . one short beep, LED off
//  Button with nothing active .. tiny click (pager is alive)
//  Radio failed at boot ........ three 300 ms beeps
//
//  LED: solid = bad-rep alert, blinking = call, off = nothing active
//
//  WIRING: LED GPIO25 -> 220 ohm -> LED -> GND
//          Button 3V3 -> button -> GPIO18, 10k GPIO18 -> GND
//          Buzzer + -> 5V, buzzer - -> BC547 collector,
//          BC547 emitter -> GND, base -> 1k -> GPIO26
// =====================================================================
//  ESP32 Arduino core 3.x (ESP-NOW receive callback with esp_now_recv_info_t).

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---------- Pins ----------
const int LED_PIN    = 25;   // LED through 220 ohm to GND
const int BUTTON_PIN = 18;   // 10k pull-down, HIGH = pressed
const int BUZZER_PIN = 26;   // BC547 base through 1k

// false: NPN low-side driver like the BC547 (HIGH = on). true: PNP high-side driver (LOW = on).
const bool BUZZER_ACTIVE_LOW = false;

// ---------- Link settings (must match the patient device) ----------
const uint8_t MAGIC        = 0xA5;
const uint8_t PAIR_ID      = 1;        // give each brace/pager pair its own ID
const int     WIFI_CHANNEL = 1;

// ---------- Timing ----------
const unsigned long ALERT_REPEAT_MS = 3000;   // five buzzes this often until resolved
const unsigned long CALL_REPEAT_MS  = 6000;   // call beeps this often until cleared
const unsigned long BLINK_MS        = 500;    // LED blink for a call
const unsigned long CLEAR_RETRY_MS  = 1000;   // resend "cleared" this often...
const unsigned long CLEAR_WINDOW_MS = 15000;  // ...for this long; no answer = disconnected
const unsigned long DEBOUNCE_MS     = 40;

// ---------- Messages ----------
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

enum PagerState { P_IDLE, P_ALERT, P_CALL };   // up here so the IDE's auto-prototypes compile

uint8_t BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
QueueHandle_t rxQueue;
bool radioOk = false;

// ---------- Beep engine (on/off rhythms) ----------
struct Beep { bool on; unsigned long ms; };

const Beep SND_BOOT[]         = {{true, 400}};
const Beep SND_CONNECTED[]    = {{true, 60}, {false, 60}, {true, 60}};
const Beep SND_DISCONNECTED[] = {{true, 1000}};
const Beep SND_ALERT[]        = {{true, 80}, {false, 60}, {true, 80}, {false, 60},
                                 {true, 80}, {false, 60}, {true, 80}, {false, 60},
                                 {true, 80}};                       // five buzzes
const Beep SND_CALL[]         = {{true, 300}, {false, 300}, {true, 300}};
const Beep SND_RESUMED[]      = {{true, 100}, {false, 150}, {true, 500}};
const Beep SND_CLEAR[]        = {{true, 120}};
const Beep SND_TICK[]         = {{true, 30}};
const Beep SND_RADIO_FAIL[]   = {{true, 300}, {false, 200}, {true, 300}, {false, 200},
                                 {true, 300}};

#define PLAY(s) play(s, sizeof(s) / sizeof(s[0]))

const Beep* snd = nullptr;
int sndLen = 0;
int sndIdx = 0;
unsigned long sndUntil = 0;

void buzzer(bool on) {
  digitalWrite(BUZZER_PIN, (on != BUZZER_ACTIVE_LOW) ? HIGH : LOW);
}

void stepSound() {
  buzzer(snd[sndIdx].on);
  sndUntil = millis() + snd[sndIdx].ms;
}

void play(const Beep* s, int n) {
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
    buzzer(false);
    return;
  }
  stepSound();
}

// ---------- Button ----------
bool btnStable = false;
bool btnLastReading = false;
unsigned long btnLastChange = 0;

bool buttonPressed() {
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != btnLastReading) {
    btnLastReading = reading;
    btnLastChange = millis();
  }
  if (millis() - btnLastChange > DEBOUNCE_MS && reading != btnStable) {
    btnStable = reading;
    return reading;   // true only on press
  }
  return false;
}

// ---------- State ----------
PagerState state = P_IDLE;
unsigned long nextRepeatMs = 0;

bool haveRxSeq = false;
uint16_t lastRxSeq = 0;

bool clearPending = false;
uint16_t clearSeq = 0;
unsigned long clearFirstMs = 0;
unsigned long clearLastMs = 0;

// Link status: false until the brace is first heard
bool linkUp = false;

// ---------- Radio ----------
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(Packet)) return;
  Packet p;
  memcpy(&p, data, sizeof(p));
  xQueueSend(rxQueue, &p, 0);   // handled in loop(), not in the radio task
}

void transmit(uint8_t type, uint16_t seq) {
  if (!radioOk) return;
  Packet p = {MAGIC, PAIR_ID, type, seq};
  esp_now_send(BROADCAST_ADDR, (uint8_t *)&p, sizeof(p));
}

bool setupRadio() {
  rxQueue = xQueueCreate(8, sizeof(Packet));
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_ADDR, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

// ---------- Link status ----------
// Any message from the brace means it's connected
void markConnected() {
  if (!linkUp) {
    linkUp = true;
    PLAY(SND_CONNECTED);
    Serial.println("Patient device connected");
  }
}

// The brace stopped answering
void markDisconnected() {
  if (linkUp) {
    linkUp = false;
    Serial.println("Patient device disconnected");
  }
  PLAY(SND_DISCONNECTED);
}

// ---------- Actions ----------
void setState(PagerState s) {
  state = s;
  nextRepeatMs = millis();   // play the new pattern right away
  if (s == P_IDLE) digitalWrite(LED_PIN, LOW);
}

void sendCleared() {
  clearSeq++;
  clearPending = true;
  clearFirstMs = clearLastMs = millis();
  transmit(MSG_CLEARED, clearSeq);
  Serial.printf("-> patient: CLEARED (seq %u)\n", clearSeq);
}

void handlePacket(const Packet &p) {
  if (p.magic != MAGIC || p.pairId != PAIR_ID) return;

  markConnected();

  if (p.type == MSG_ACK) {
    if (clearPending && p.seq == clearSeq) {
      clearPending = false;
      Serial.println("Patient confirmed the clear");
    }
    return;
  }

  // Always confirm, even repeats, so the patient stops resending
  transmit(MSG_ACK, p.seq);
  if (haveRxSeq && p.seq == lastRxSeq) return;   // repeat of a message already handled
  haveRxSeq = true;
  lastRxSeq = p.seq;

  switch (p.type) {
    case MSG_HELLO:
      break;   // handshake; markConnected() already handled it

    case MSG_ALERT:
      Serial.println("ALERT: patient has repeated bad reps");
      setState(P_ALERT);   // repeats until the patient resumes or the instructor clears
      break;

    case MSG_CALL:
      Serial.println("CALL: patient asked for help");
      if (state != P_ALERT) setState(P_CALL);   // an alert outranks a call
      break;

    case MSG_RESUMED:
      if (state == P_ALERT) {
        Serial.println("Patient resumed on their own: alert stopped");
        setState(P_IDLE);
        PLAY(SND_RESUMED);
      }
      break;
  }
}

void processRadio() {
  Packet p;
  while (xQueueReceive(rxQueue, &p, 0) == pdTRUE) handlePacket(p);
}

void updateClearRetry() {
  if (!clearPending) return;
  unsigned long now = millis();
  if (now - clearFirstMs > CLEAR_WINDOW_MS) {
    clearPending = false;
    Serial.println("Patient device did not confirm the clear");
    markDisconnected();
    return;
  }
  if (now - clearLastMs >= CLEAR_RETRY_MS) {
    transmit(MSG_CLEARED, clearSeq);
    clearLastMs = now;
  }
}

// Repeats the alert/call pattern with no time limit until the state goes back to idle
void updateAlarm() {
  unsigned long now = millis();

  if (state == P_ALERT) {
    digitalWrite(LED_PIN, HIGH);
    if ((long)(now - nextRepeatMs) >= 0) {
      PLAY(SND_ALERT);
      nextRepeatMs = now + ALERT_REPEAT_MS;
    }
  } else if (state == P_CALL) {
    digitalWrite(LED_PIN, ((now / BLINK_MS) % 2) ? HIGH : LOW);
    if ((long)(now - nextRepeatMs) >= 0) {
      PLAY(SND_CALL);
      nextRepeatMs = now + CALL_REPEAT_MS;
    }
  }
}

void onButton() {
  if (state == P_IDLE) {
    PLAY(SND_TICK);
    return;
  }
  setState(P_IDLE);
  PLAY(SND_CLEAR);
  sendCleared();
  Serial.println("Instructor cleared the alert");
}

// ---------- Arduino entry points ----------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT);   // pull-down is external

  pinMode(BUZZER_PIN, OUTPUT);
  buzzer(false);

  clearSeq = (uint16_t)esp_random();   // random start so a reboot never looks like a repeat

  radioOk = setupRadio();
  if (!radioOk) {
    Serial.println("ESP-NOW setup failed");
    PLAY(SND_RADIO_FAIL);
  } else {
    Serial.printf("Pager ready, pair ID %d\n", PAIR_ID);
    PLAY(SND_BOOT);
  }
}

void loop() {
  if (buttonPressed()) onButton();
  processRadio();
  updateClearRetry();
  updateAlarm();
  updateSound();
}
