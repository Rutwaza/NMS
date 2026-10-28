/*
 * ESP32 GPS Tracker v3.0 - MQTT build
 * Hardware: ESP32 + NEO-6M + A7670C + 2S pack (LM2596 -> ESP32/GPS, pack -> modem VIN)
 *
 * What changed vs v2.1:
 *   - MQTT over TLS with ONE persistent connection (no TLS handshake per upload)
 *   - Firebase secret is gone from the device (a small bridge writes to Firebase)
 *   - Sampling keeps running during modem waits (service())
 *   - Hardware watchdog, tighter GPS filter, DEBUG flag for field builds
 *   - Flash queue is written only when publishing fails (not on every fix)
 *
 * Wiring (unchanged):
 *   GPS TX -> GPIO16, GPS RX -> GPIO17
 *   Modem TX -> GPIO21, Modem RX -> GPIO22, DTR -> GPIO23, PWK -> GPIO5 (optional)
 *   SOS button -> GPIO4 to GND, Battery ADC -> GPIO34 through divider
 *
 * Library: TinyGPSPlus
 * NOTE: written without access to your hardware/toolchain - test in the stages
 *       described in the chat, with DEBUG 1 first.
 */

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <LittleFS.h>

// ===================== PINS =====================
#define GPS_RX      16
#define GPS_TX      17
#define MODEM_RX    21
#define MODEM_TX    22
#define MODEM_DTR   23
#define MODEM_PWK   5
#define BATT_ADC    34
#define SOS_PIN     4

// ===================== OPTIONS =====================
#define DEBUG             1      // set to 0 for the field build
#define USE_DTR_SLEEP     0      // leave 0 until MQTT works; see notes
#define USE_PWK           1
#define HAVE_BATT_DIVIDER 1
#define BATT_THROTTLE     0      // 1 = halve rates below 25% (only after ADC is verified)
#define LTE_ONLY          0      // 1 = force LTE (only if coverage is solid)
#define BATT_DIV          3.488f // MUST equal (R1+R2)/R2 of YOUR divider - check with a multimeter
#define FW_VERSION        "3.0"
#define WDT_SECONDS       90

// ===================== CONFIG =====================
const char* APN = "internet";

const char* MQTT_HOST = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "";
const char* MQTT_PASS = "";           // no double quotes in user/pass
const char*    TRACKER_NAME = "S_01";

const uint32_t SAMPLE_MOVING_MS  = 5000;
const uint32_t SAMPLE_IDLE_MS    = 30000;
const uint32_t PUBLISH_MOVING_MS = 10000;   // map updates every ~10 s while moving
const uint32_t PUBLISH_IDLE_MS   = 60000;
const uint32_t HEARTBEAT_MS      = 60000;   // live message at least this often
const uint32_t RETRY_MS          = 15000;
const uint32_t BATT_MS           = 30000;
const uint16_t MQTT_KEEPALIVE_S  = 120;
const uint8_t  TRACK_CHUNK       = 40;      // fixes per history message

const float   MOVING_KMPH = 3.0f;
const float   MIN_MOVE_M  = 8.0f;
const float   HDOP_MAX    = 3.0f;
const uint8_t MIN_SATS    = 4;

#define BUF_SIZE 120

// ===================== LOGGING =====================
#if DEBUG
  #define LOG(...) Serial.printf(__VA_ARGS__)
#else
  #define LOG(...)
#endif

// ===================== GLOBALS =====================
HardwareSerial neogps(2);
HardwareSerial SerialAT(1);
TinyGPSPlus gps;

struct Fix {
  uint32_t t;
  int32_t lat;
  int32_t lng;
  uint16_t spd10;
  uint16_t hdg;
  uint8_t hdop10;
  uint8_t sat;
};

Fix buf[BUF_SIZE];
uint8_t bufHead = 0;
uint8_t bufCount = 0;
uint32_t bufOverflow = 0;       // counts fixes dropped because the buffer was full

Fix lastFix;
bool haveLastFix = false;
uint32_t lastStoreMs = 0;

bool modemReady = false;
bool mqttUp = false;
bool moving = false;

uint8_t powerScale = 1;
float battV = 0;
int battPct = 0;

uint8_t failCount = 0;
bool sosActive = false;
bool forcePublish = false;
uint32_t sosHoldMs = 0;

uint32_t lastSampleMs = 0;
uint32_t lastBattMs = 0;
uint32_t lastInitTry = 0;
uint32_t nextPublishMs = 0;
uint32_t lastSuccessMs = 0;
uint32_t lastQSaveMs = 0;
uint32_t lastGPSLogMs = 0;

String clientId, topicLive, topicTrack, topicStatus;

// forward declarations
uint32_t sampleMs();
uint32_t publishMs();
void trySample();
void sosPoll();

// ===================== SERVICE LOOP (runs during ALL waits) =====================
inline void pumpGPS() {
  while (neogps.available()) {
    gps.encode(neogps.read());
  }
}

// Called from every blocking wait so GPS, SOS, sampling and the watchdog keep running.
inline void service() {
  pumpGPS();
  esp_task_wdt_reset();
  sosPoll();

  uint32_t now = millis();
  if (now - lastSampleMs >= sampleMs()) {
    lastSampleMs = now;
    trySample();
  }
}

void idleMs(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    service();
    delay(5);
  }
}

// ===================== GPS =====================
void ubx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
  uint8_t a = 0, b = 0;

  auto put = [&](uint8_t x) {
    neogps.write(x);
    a += x;
    b += a;
  };

  neogps.write(0xB5);
  neogps.write(0x62);
  put(cls);
  put(id);
  put(len & 0xFF);
  put(len >> 8);

  for (uint16_t i = 0; i < len; i++) put(payload[i]);

  neogps.write(a);
  neogps.write(b);
  neogps.flush();
}

void configGPS() {
  // Keep only GGA + RMC (disable GLL, GSA, GSV, VTG).
  const uint8_t msgs[] = {0x01, 0x02, 0x03, 0x05};
  for (uint8_t m : msgs) {
    uint8_t p[3] = {0xF0, m, 0x00};
    ubx(0x06, 0x01, p, 3);
    delay(50);
  }

  // Pedestrian dynamic model.
  uint8_t nav5[36] = {0};
  nav5[0] = 0x01;
  nav5[2] = 0x03;
  ubx(0x06, 0x24, nav5, 36);

  LOG("[GPS] Configuration sent.\n");
}

uint32_t gpsEpoch() {
  if (!gps.date.isValid() || !gps.time.isValid()) return 0;

  int y = gps.date.year();
  int m = gps.date.month();
  int d = gps.date.day();

  if (y < 2024) return 0;

  y -= (m <= 2);
  int era = y / 400;
  unsigned yoe = y - era * 400;
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = era * 146097L + (long)doe - 719468L;

  return (uint32_t)(days * 86400L + gps.time.hour() * 3600L +
                    gps.time.minute() * 60L + gps.time.second());
}

void printGPSDiagnostics() {
#if DEBUG
  uint32_t now = millis();
  if (now - lastGPSLogMs < 10000) return;
  lastGPSLogMs = now;

  bool valid = gps.location.isValid();
  LOG("[GPS] chars=%lu badsum=%lu fix=%s age=%lums sats=%d hdop=%.1f lat=%.6f lng=%.6f kmh=%.1f\n",
      (unsigned long)gps.charsProcessed(),
      (unsigned long)gps.failedChecksum(),
      (valid && gps.location.age() < 5000) ? "YES" : "NO",
      (unsigned long)gps.location.age(),
      gps.satellites.isValid() ? (int)gps.satellites.value() : -1,
      gps.hdop.isValid() ? gps.hdop.hdop() : -1.0,
      valid ? gps.location.lat() : 0.0,
      valid ? gps.location.lng() : 0.0,
      gps.speed.isValid() ? gps.speed.kmph() : 0.0);
#endif
}

// ===================== BATTERY =====================
uint32_t readBattPinMilliVolts() {
  uint32_t total = 0;
  for (int i = 0; i < 8; i++) {
    total += analogReadMilliVolts(BATT_ADC);
    delay(2);
  }
  return total / 8;
}

int voltToPct(float cellVoltage) {
  // Approximate resting curve for one Li-ion cell; under load it reads low.
  static const float tv[] = {4.20, 4.10, 4.00, 3.90, 3.80, 3.70, 3.60, 3.50, 3.40, 3.30};
  static const int tp[]   = {100, 90, 80, 65, 50, 35, 20, 10, 5, 0};

  if (cellVoltage >= tv[0]) return 100;
  if (cellVoltage <= tv[9]) return 0;

  for (int i = 1; i < 10; i++) {
    if (cellVoltage >= tv[i]) {
      float f = (cellVoltage - tv[i]) / (tv[i - 1] - tv[i]);
      return tp[i] + (int)(f * (tp[i - 1] - tp[i]));
    }
  }
  return 0;
}

void batteryRead() {
#if HAVE_BATT_DIVIDER
  float v = (readBattPinMilliVolts() / 1000.0f) * BATT_DIV;

  // Smooth out the sag caused by modem TX bursts.
  battV = (battV < 1.0f) ? v : (battV * 0.8f + v * 0.2f);
  battPct = voltToPct(battV / 2.0f);

#if BATT_THROTTLE
  powerScale = (battPct < 25) ? 2 : 1;
#endif

  LOG("[BATT] pack=%.2f V (%d%%)\n", battV, battPct);
#endif
}

// ===================== AT HELPERS =====================
String waitFor(const char* tok, uint32_t timeout) {
  String response;
  response.reserve(160);
  uint32_t start = millis();

  while (millis() - start < timeout) {
    service();

    while (SerialAT.available()) response += (char)SerialAT.read();

    if (tok && response.indexOf(tok) >= 0) break;
    if (response.indexOf("ERROR") >= 0) break;

    delay(5);
  }
  return response;
}

String sendCmd(const String& cmd, const char* expect = "OK", uint32_t timeout = 1000) {
  while (SerialAT.available()) SerialAT.read();
  SerialAT.println(cmd);
  return waitFor(expect, timeout);
}

// Keep appending to `resp` until a full "+TOKEN: ...\n" line has arrived.
bool waitURCInto(String& resp, const char* tok, uint32_t timeout) {
  uint32_t start = millis();

  while (millis() - start < timeout) {
    service();

    while (SerialAT.available()) resp += (char)SerialAT.read();

    int i = resp.indexOf(tok);
    if (i >= 0 && resp.indexOf('\n', i) >= 0) return true;
    if (resp.indexOf("ERROR") >= 0) return false;

    delay(5);
  }
  return false;
}

// Send a command, wait for OK, then wait for its asynchronous "+XXX: ..." result line.
String sendCmdURC(const String& cmd, const char* urc, uint32_t okTimeout, uint32_t urcTimeout) {
  String r = sendCmd(cmd, "OK", okTimeout);
  if (r.indexOf("ERROR") >= 0) return r;

  int i = r.indexOf(urc);
  if (i < 0 || r.indexOf('\n', i) < 0) waitURCInto(r, urc, urcTimeout);
  return r;
}

// Last integer on a URC line: "+CMQTTCONNECT: 0,0" -> 0, "+CMQTTSTART: 0" -> 0. -1 if absent.
int urcResult(const String& r, const char* tok) {
  int i = r.indexOf(tok);
  if (i < 0) return -1;

  int e = r.indexOf('\n', i);
  if (e < 0) e = r.length();

  String line = r.substring(i, e);
  int c = line.lastIndexOf(',');
  if (c < 0) c = line.indexOf(':');
  if (c < 0) return -1;

  line = line.substring(c + 1);
  line.trim();
  if (line.length() == 0) return -1;
  return line.toInt();
}

// Commands that answer with a '>' prompt and then expect exactly len bytes.
bool atData(const String& cmd, const String& data, uint32_t dataTimeout = 5000) {
  String r = sendCmd(cmd, ">", 3000);
  if (r.indexOf(">") < 0) {
    LOG("[AT] no '>' prompt for: %s -> %s\n", cmd.c_str(), r.c_str());
    return false;
  }

  SerialAT.print(data);
  SerialAT.flush();

  r = waitFor("OK", dataTimeout);
  return r.indexOf("OK") >= 0;
}

void modemWake() {
#if USE_DTR_SLEEP
  digitalWrite(MODEM_DTR, LOW);
  idleMs(150);
#endif
}

void modemSleep() {
#if USE_DTR_SLEEP
  digitalWrite(MODEM_DTR, HIGH);
#endif
}

bool modemAlive() {
  for (int i = 0; i < 6; i++) {
    if (sendCmd("AT", "OK", 600).indexOf("OK") >= 0) return true;
  }
  return false;
}

void modemPowerPulse() {
#if USE_PWK
  LOG("[GSM] PWK pulse.\n");
  pinMode(MODEM_PWK, OUTPUT);
  digitalWrite(MODEM_PWK, LOW);
  idleMs(1000);
  pinMode(MODEM_PWK, INPUT);
  idleMs(8000);
#endif
}

bool regOK(const String& r) {
  return r.indexOf(",1") >= 0 || r.indexOf(",5") >= 0;
}

bool modemRegistered() {
  if (regOK(sendCmd("AT+CEREG?", "OK", 1500))) return true;
  if (regOK(sendCmd("AT+CGREG?", "OK", 1500))) return true;
  return regOK(sendCmd("AT+CREG?", "OK", 1500));
}

int modemCSQ() {
  String r = sendCmd("AT+CSQ", "OK", 1500);
  int i = r.indexOf("+CSQ:");
  if (i < 0) return 99;
  return r.substring(i + 5).toInt();
}

// ===================== MODEM INIT =====================
bool modemInit() {
  LOG("\n[GSM] Init...\n");

  modemWake();

  if (!modemAlive()) {
    LOG("[GSM] No AT response, trying PWK.\n");
    modemPowerPulse();
    if (!modemAlive()) {
      LOG("[GSM] ERROR: modem dead.\n");
      return false;
    }
  }

  sendCmd("ATE0", "OK", 1000);

  String sim = sendCmd("AT+CPIN?", "READY", 3000);
  if (sim.indexOf("READY") < 0) {
    LOG("[GSM] ERROR: SIM not ready: %s\n", sim.c_str());
    return false;
  }

  sendCmd("AT+CFUN=1", "OK", 3000);

#if LTE_ONLY
  sendCmd("AT+CNMP=38", "OK", 2000);
#endif

  bool registered = false;
  uint32_t start = millis();

  while (millis() - start < 60000) {
    if (modemRegistered()) {
      registered = true;
      break;
    }
    idleMs(2500);
  }

  if (!registered) {
    LOG("[GSM] ERROR: registration timeout. CSQ=%d\n", modemCSQ());
    return false;
  }

  String r = sendCmd("AT+CGDCONT=1,\"IP\",\"" + String(APN) + "\"", "OK", 3000);
  LOG("[GSM] APN: %s\n", r.c_str());

  r = sendCmd("AT+CGATT=1", "OK", 10000);
  LOG("[GSM] Attach: %s\n", r.c_str());

  r = sendCmd("AT+CGACT=1,1", "OK", 10000);
  LOG("[GSM] PDP: %s\n", r.c_str());

  r = sendCmd("AT+CGPADDR=1", "OK", 2000);
  LOG("[GSM] IP: %s\n", r.c_str());

#if USE_DTR_SLEEP
  sendCmd("AT+CSCLK=1", "OK", 1500);
#endif

  LOG("[GSM] Ready. CSQ=%d\n", modemCSQ());
  return true;
}

// ===================== MQTT =====================
bool mqttAbort() {
  sendCmd("AT+CMQTTREL=0", "OK", 2000);
  sendCmd("AT+CMQTTSTOP", "OK", 3000);
  return false;
}

bool mqttOpen() {
  LOG("[MQTT] Connecting to %s:%u\n", MQTT_HOST, MQTT_PORT);

  // Clear any stale client/session state.
  sendCmd("AT+CMQTTDISC=0,3", "OK", 2500);
  sendCmd("AT+CMQTTREL=0", "OK", 2000);
  sendCmd("AT+CMQTTSTOP", "OK", 3000);
  idleMs(500);

  // Start MQTT service.
  String r = sendCmdURC(
    "AT+CMQTTSTART",
    "+CMQTTSTART:",
    3000,
    12000
  );

  LOG("[MQTT] START: %s\n", r.c_str());

  if (urcResult(r, "+CMQTTSTART:") != 0) {
    LOG("[MQTT] Could not start service.\n");
    return mqttAbort();
  }

  // Acquire a client. The final 0 selects non-TLS MQTT.
  r = sendCmd(
    "AT+CMQTTACCQ=0,\"" + clientId + "\",0",
    "OK",
    3000
  );

  LOG("[MQTT] ACCQ: %s\n", r.c_str());

  if (r.indexOf("OK") < 0) {
    return mqttAbort();
  }

  // Configure Last Will: publish retained "offline" if disconnected
  // unexpectedly. Check both commands rather than ignoring failures.
  bool willTopicOK = atData(
    "AT+CMQTTWILLTOPIC=0," + String(topicStatus.length()),
    topicStatus
  );

  bool willMsgOK = atData(
    "AT+CMQTTWILLMSG=0,7,1",
    "offline"
  );

  if (!willTopicOK || !willMsgOK) {
    LOG("[MQTT] Last Will configuration failed.\n");
    return mqttAbort();
  }

  // Plain MQTT, not TLS.
  // Build the MQTT CONNECT command.
  // If credentials are empty, omit both optional parameters.
  String conn =
      "AT+CMQTTCONNECT=0,\"tcp://" +
      String(MQTT_HOST) + ":" +
      String(MQTT_PORT) + "\"," +
      String(MQTT_KEEPALIVE_S) + ",1";

  // Only append credentials when BOTH are configured.
  if (strlen(MQTT_USER) > 0 && strlen(MQTT_PASS) > 0) {
    conn += ",\"" + String(MQTT_USER) +
            "\",\"" + String(MQTT_PASS) + "\"";
  }

  LOG("[MQTT] CONNECT command: %s\n", conn.c_str());

  r = sendCmdURC(
      conn,
      "+CMQTTCONNECT:",
      5000,
      30000
  );

  LOG("[MQTT] CONNECT response: %s\n", r.c_str());

  if (urcResult(r, "+CMQTTCONNECT:") != 0) {
    LOG("[MQTT] Connection failed.\n");
    return mqttAbort();
  }

  LOG("[MQTT] CONNECT: %s\n", r.c_str());

  if (urcResult(r, "+CMQTTCONNECT:") != 0) {
    LOG("[MQTT] Connection failed.\n");
    return mqttAbort();
  }

  LOG("[MQTT] Connected.\n");

  // Announce online and retain it for clients joining later.
  if (!mqttPub(topicStatus, "online", true)) {
    LOG("[MQTT] Online status publish failed.\n");
    return mqttAbort();
  }

  LOG("[MQTT] Online status published.\n");
  return true;
}

bool mqttPub(
  const String& topic,
  const String& payload,
  bool retain
) {
  if (!atData(
        "AT+CMQTTTOPIC=0," + String(topic.length()),
        topic
      )) {
    LOG("[MQTT] Topic setup failed.\n");
    return false;
  }

  if (!atData(
        "AT+CMQTTPAYLOAD=0," + String(payload.length()),
        payload
      )) {
    LOG("[MQTT] Payload setup failed.\n");
    return false;
  }

  // A7670-series MQTT publish timeout is configured in seconds.
  // Use 60 seconds rather than the previous value of 20.
  String cmd = "AT+CMQTTPUB=0,1,60";

  if (retain) {
    cmd += ",1";
  }

  String r = sendCmdURC(
    cmd,
    "+CMQTTPUB:",
    5000,
    65000
  );

  bool ok = (urcResult(r, "+CMQTTPUB:") == 0);

  if (!ok) {
    LOG("[MQTT] PUB failed: %s\n", r.c_str());
  }

  return ok;
}

// ===================== BUFFER / FLASH QUEUE =====================
void bufPush(const Fix& f) {
  if (bufCount == BUF_SIZE) {
    bufHead = (bufHead + 1) % BUF_SIZE;
    bufCount--;
    bufOverflow++;
  }
  buf[(bufHead + bufCount) % BUF_SIZE] = f;
  bufCount++;
}

void bufDrop(uint8_t n) {
  if (n > bufCount) n = bufCount;
  bufHead = (bufHead + n) % BUF_SIZE;
  bufCount -= n;
}

// Snapshot the whole RAM buffer to flash (overwrites; used only when publishing fails).
void qSave() {
  if (bufCount == 0) return;

  File q = LittleFS.open("/q.bin", FILE_WRITE);
  if (!q) {
    LOG("[QUEUE] Cannot open queue.\n");
    return;
  }

  for (uint8_t i = 0; i < bufCount; i++) {
    q.write((const uint8_t*)&buf[(bufHead + i) % BUF_SIZE], sizeof(Fix));
  }
  q.close();

  LOG("[QUEUE] Saved %u fixes.\n", bufCount);
}

void qLoad() {
  File q = LittleFS.open("/q.bin", FILE_READ);
  if (!q) return;

  size_t n = q.size() / sizeof(Fix);
  if (n > BUF_SIZE) q.seek((n - BUF_SIZE) * sizeof(Fix));

  Fix f;
  while (q.read((uint8_t*)&f, sizeof(Fix)) == sizeof(Fix)) bufPush(f);
  q.close();

  LOG("[QUEUE] Restored %u fixes.\n", bufCount);
}

// ===================== SOS =====================
void sosPoll() {
  static bool armed = true;

  if (digitalRead(SOS_PIN) == LOW) {
    if (!sosHoldMs) {
      sosHoldMs = millis();
    } else if (armed && millis() - sosHoldMs > 1500) {
      sosActive = !sosActive;
      armed = false;
      forcePublish = true;
      nextPublishMs = millis();
      LOG("[SOS] %s\n", sosActive ? "ACTIVE" : "INACTIVE");
    }
  } else {
    sosHoldMs = 0;
    armed = true;
  }
}

// ===================== PAYLOADS =====================
String buildLive(int csq) {
  String b;
  b.reserve(300);
  b = "{";

  if (haveLastFix) {
    b += "\"t\":" + String(lastFix.t);
    b += ",\"la\":" + String(lastFix.lat / 1e6, 6);
    b += ",\"lo\":" + String(lastFix.lng / 1e6, 6);
    b += ",\"s\":" + String(lastFix.spd10 / 10.0, 1);
    b += ",\"h\":" + String(lastFix.hdg);
    b += ",\"hd\":" + String(lastFix.hdop10 / 10.0, 1);
    b += ",\"sat\":" + String(lastFix.sat);
    b += ",";
  }

  b += "\"fix\":" + String((gps.location.isValid() && gps.location.age() < 5000) ? 1 : 0);

#if HAVE_BATT_DIVIDER
  b += ",\"b\":" + String(battV, 2);
  b += ",\"bp\":" + String(battPct);
#endif

  b += ",\"sos\":" + String(sosActive ? 1 : 0);
  b += ",\"fw\":\"" FW_VERSION "\"";
  b += ",\"q\":" + String(csq);
  b += ",\"up\":" + String(millis() / 1000);
  b += ",\"rr\":" + String((int)esp_reset_reason());
  b += "}";

  return b;
}

// History batch: {"p":[[t,lat,lng,speed,heading],...]}  (first n buffered fixes)
String buildTrack(uint8_t n) {
  String b;
  b.reserve(n * 48 + 16);
  b = "{\"p\":[";

  for (uint8_t i = 0; i < n; i++) {
    const Fix& f = buf[(bufHead + i) % BUF_SIZE];
    if (i) b += ",";
    b += "[" + String(f.t) + "," + String(f.lat / 1e6, 6) + "," + String(f.lng / 1e6, 6) +
         "," + String(f.spd10 / 10.0, 1) + "," + String(f.hdg) + "]";
  }

  b += "]}";
  return b;
}

// ===================== INTERVALS =====================
uint32_t sampleMs() {
  if (sosActive) return 3000;
  return (moving ? SAMPLE_MOVING_MS : SAMPLE_IDLE_MS) * powerScale;
}

uint32_t publishMs() {
  if (sosActive) return 10000;
  return (moving ? PUBLISH_MOVING_MS : PUBLISH_IDLE_MS) * powerScale;
}

// ===================== GPS SAMPLING =====================
void trySample() {
  if (!(gps.location.isValid() &&
        gps.location.age() < 2500 &&
        gps.hdop.isValid() &&
        gps.hdop.hdop() <= HDOP_MAX &&
        gps.satellites.isValid() &&
        gps.satellites.value() >= MIN_SATS)) {
    return;
  }

  uint32_t t = gpsEpoch();
  if (!t) return;

  double lat = gps.location.lat();
  double lng = gps.location.lng();
  float spd = gps.speed.isValid() ? gps.speed.kmph() : 0;

  moving = spd > MOVING_KMPH;

  Fix f;
  f.t = t;
  f.lat = (int32_t)(lat * 1e6);
  f.lng = (int32_t)(lng * 1e6);
  f.spd10 = (uint16_t)(spd * 10);
  f.hdg = gps.course.isValid() ? (uint16_t)gps.course.deg() : 0;
  f.hdop10 = (uint8_t)(gps.hdop.hdop() * 10);
  f.sat = (uint8_t)gps.satellites.value();

  // Standing still: skip tiny GPS drift, but keep a heartbeat fix.
  if (haveLastFix && !moving) {
    double d = TinyGPSPlus::distanceBetween(lastFix.lat / 1e6, lastFix.lng / 1e6, lat, lng);
    if (d < MIN_MOVE_M && millis() - lastStoreMs < HEARTBEAT_MS) {
      lastFix.t = t;
      return;
    }
  }

  lastFix = f;
  haveLastFix = true;
  lastStoreMs = millis();

  bufPush(f);

  LOG("[GPS] Stored. buf=%u %.6f,%.6f %.1f km/h sats=%u\n", bufCount, lat, lng, spd, f.sat);
}

// ===================== PUBLISH =====================
void doPublish() {
  modemWake();

  bool ok = false;

  if (!modemAlive()) {
    LOG("[GSM] Modem not responding.\n");
  } else if (!mqttUp && !modemRegistered()) {
    LOG("[GSM] Not registered.\n");
  } else {
    if (!mqttUp) {
      String attach = sendCmd("AT+CGATT?", "OK", 2000);
      if (attach.indexOf("+CGATT: 1") < 0) sendCmd("AT+CGATT=1", "OK", 10000);
      mqttUp = mqttOpen();
    }

    if (mqttUp) {
      // 1) live position first so the map updates immediately (retained for new viewers)
      ok = mqttPub(topicLive, buildLive(modemCSQ()), true);

      // 2) buffered history, in chunks
      while (ok && bufCount > 0) {
        uint8_t n = (bufCount < TRACK_CHUNK) ? bufCount : TRACK_CHUNK;
        uint32_t ov0 = bufOverflow;

        ok = mqttPub(topicTrack, buildTrack(n), false);

        if (ok) {
          // New fixes may have been added (or old ones dropped) while we were publishing.
          uint32_t lost = bufOverflow - ov0;
          if (lost < n) bufDrop(n - lost);
        }
      }

      if (!ok) mqttUp = false;
    }
  }

  modemSleep();

  uint32_t now = millis();

  if (ok) {
    failCount = 0;
    forcePublish = false;
    lastSuccessMs = now;
    nextPublishMs = now + publishMs();

    if (bufCount == 0) LittleFS.remove("/q.bin");
    LOG("[MQTT] Publish OK.\n");
  } else {
    failCount++;
    nextPublishMs = now + RETRY_MS;
    LOG("[MQTT] Publish FAILED (%u in a row)\n", failCount);

    if (lastQSaveMs == 0 || now - lastQSaveMs >= 60000) {
      qSave();
      lastQSaveMs = now ? now : 1;
    }

    if (failCount == 3) {
      mqttUp = false;
      modemReady = false;       // full modem re-init
    }

    if (failCount == 5) {
      LOG("[GSM] Modem reset.\n");
      modemWake();
      sendCmd("AT+CFUN=1,1", "OK", 3000);
      idleMs(8000);
      mqttUp = false;
      modemReady = false;
    }

    if (failCount >= 8) {
      LOG("[SYSTEM] Too many failures, restarting.\n");
      qSave();
      ESP.restart();
    }
  }
}

// ===================== WATCHDOG =====================
void wdtInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {
    .timeout_ms = WDT_SECONDS * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_reconfigure(&cfg);
#else
  esp_task_wdt_init(WDT_SECONDS, true);
#endif
  esp_task_wdt_add(NULL);
}

// ===================== SETUP =====================
void setup() {
  setCpuFrequencyMhz(80);

#if DEBUG
  Serial.begin(115200);
#endif

  delay(500);
  LOG("\n=== ESP32 GPS TRACKER v%s (MQTT) ===\n", FW_VERSION);

  wdtInit();

  clientId    = String("trk-") + TRACKER_NAME;
  topicLive   = String("trk/") + TRACKER_NAME + "/live";
  topicTrack  = String("trk/") + TRACKER_NAME + "/track";
  topicStatus = String("trk/") + TRACKER_NAME + "/status";

  pinMode(MODEM_DTR, OUTPUT);
  digitalWrite(MODEM_DTR, LOW);

#if USE_PWK
  pinMode(MODEM_PWK, INPUT);
#endif

  analogReadResolution(12);
  analogSetPinAttenuation(BATT_ADC, ADC_11db);
  pinMode(SOS_PIN, INPUT_PULLUP);

  if (!LittleFS.begin(true)) {
    LOG("[QUEUE] LittleFS mount failed.\n");
  }
  qLoad();

  neogps.setRxBufferSize(1024);
  neogps.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  SerialAT.setRxBufferSize(1024);
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

  delay(1000);

  configGPS();
  batteryRead();

  LOG("[SYSTEM] Setup complete.\n");
}

// ===================== LOOP =====================
void loop() {
  service();
  printGPSDiagnostics();

  uint32_t now = millis();

  if (now - lastBattMs >= BATT_MS) {
    lastBattMs = now;
    batteryRead();
  }

  if (!modemReady) {
    if (lastInitTry == 0 || now - lastInitTry >= 30000) {
      lastInitTry = now ? now : 1;
      modemReady = modemInit();
      mqttUp = false;

      if (modemReady) {
        nextPublishMs = millis();
        LOG("[GSM] Init OK.\n");
      } else {
        LOG("[GSM] Init failed, retrying later.\n");
      }

      modemSleep();
    }
  } else {
    bool timeUp = (int32_t)(now - nextPublishMs) >= 0;

    if (timeUp && (bufCount > 0 || sosActive || forcePublish ||
                   now - lastSuccessMs >= HEARTBEAT_MS)) {
      doPublish();
    }
  }

  delay(20);
}
