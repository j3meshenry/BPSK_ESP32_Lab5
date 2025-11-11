// ===== Your original wiring (unchanged) =====
#define TXD2 23
#define RXD2 22

#include <Arduino.h>

// ===== Lab packet params =====
#define PKT_SYNC        0xAA
#define PKT_MAX_PAYLOAD 8

// ===== Optional OK/ERR LEDs (set to -1 to disable) =====
const int LED_OK  = 18;   // green  (use -1 if not used)
const int LED_ERR = 19;   // red    (use -1 if not used)

// ===== Fault injection (receiver-side simulation only) =====
enum FaultMode { F_NONE, F_CKSUM };
volatile FaultMode g_fault = F_NONE;

// ===== Helpers =====
static inline void led_init() {
  if (LED_OK  >= 0) pinMode(LED_OK, OUTPUT);
  if (LED_ERR >= 0) pinMode(LED_ERR, OUTPUT);
  if (LED_OK  >= 0) digitalWrite(LED_OK, LOW);
  if (LED_ERR >= 0) digitalWrite(LED_ERR, LOW);
}
static inline void led_ok()  { if (LED_OK  >= 0) digitalWrite(LED_OK, HIGH); if (LED_ERR >= 0) digitalWrite(LED_ERR, LOW); }
static inline void led_err() { if (LED_OK  >= 0) digitalWrite(LED_OK, LOW);  if (LED_ERR >= 0) digitalWrite(LED_ERR, HIGH); }
static inline void led_off() { if (LED_OK  >= 0) digitalWrite(LED_OK, LOW);  if (LED_ERR >= 0) digitalWrite(LED_ERR, LOW); }

static uint8_t xor_ck(const uint8_t* p, uint8_t n) {
  uint8_t x = 0; for (uint8_t i=0;i<n;i++) x ^= p[i]; return x;
}

// Parse a line like: "FRAME: AA 05 48 45 4C 4C 4F 0D"
bool parse_hex_frame(const String& line, uint8_t* out, size_t* out_len) {
  *out_len = 0;
  int start = line.indexOf(':');
  if (start < 0) return false;
  String bytes = line.substring(start+1);
  bytes.trim();
  // token by spaces
  int idx = 0;
  while (idx < (int)bytes.length()) {
    // skip spaces
    while (idx < (int)bytes.length() && isspace(bytes[idx])) idx++;
    if (idx >= (int)bytes.length()) break;
    // read token until space
    int j = idx;
    while (j < (int)bytes.length() && !isspace(bytes[j])) j++;
    String tok = bytes.substring(idx, j);
    tok.trim();
    if (tok.length() == 0) break;
    char* endptr = nullptr;
    long v = strtol(tok.c_str(), &endptr, 16);
    if (*out_len >= 64 || v < 0 || v > 255) return false; // simple guard
    out[(*out_len)++] = (uint8_t)v;
    idx = j;
  }
  return (*out_len > 0);
}

// Validate one full frame buffer (already bytes)
void consume_frame_and_report(const uint8_t* frame, size_t n) {
  // Expect: SYNC, LEN, DATA[LEN], CKSUM
  if (n < 3) { Serial.println("[ERR] frame too short"); led_err(); return; }
  if (frame[0] != PKT_SYNC) { Serial.println("[ERR] bad SYNC"); led_err(); return; }

  uint8_t L = frame[1];
  if (L > PKT_MAX_PAYLOAD) { Serial.println("[ERR] length > 8"); led_err(); return; }
  if (n != (size_t)(3 + L - (L==0 ? 1 : 0))) {
    // For L bytes data, total should be 3+L-? Wait—normal is SYNC(1)+LEN(1)+DATA(L)+CK(1)= L+3
    if (n != (size_t)(L + 3)) { Serial.println("[ERR] size mismatch"); led_err(); return; }
  }

  const uint8_t* data = &frame[2];
  uint8_t rx_cksum   = frame[2 + L];

  // Receiver-side fault: pretend checksum is wrong if mode is set
  uint8_t calc = xor_ck(data, L);
  if (g_fault == F_CKSUM) calc ^= 0x01;

  if (rx_cksum == calc) {
    // Print payload as ASCII-friendly
    Serial.print("PAYLOAD: ");
    for (uint8_t i=0;i<L;i++) Serial.print((char)data[i]);
    Serial.println("  [OK]");
    led_ok();
  } else {
    Serial.println("[ERR] checksum mismatch");
    led_err();
  }
}

// Simulate: given a raw ASCII message, chunk into ≤8 and “verify” each
void simulate_receive_plaintext(const String& msg) {
  const uint8_t* p = (const uint8_t*)msg.c_str();
  size_t left = msg.length();
  while (left > 0) {
    uint8_t L = (left > PKT_MAX_PAYLOAD) ? PKT_MAX_PAYLOAD : (uint8_t)left;
    // build a local frame (SYNC LEN DATA CK)
    uint8_t frame[1 + 1 + PKT_MAX_PAYLOAD + 1];
    size_t n = 0;
    frame[n++] = PKT_SYNC;
    frame[n++] = L;
    for (uint8_t i=0;i<L;i++) frame[n++] = p[i];
    uint8_t ck = xor_ck(&frame[2], L);
    frame[n++] = ck;
    consume_frame_and_report(frame, n);
    p    += L;
    left -= L;
  }
}

// Optional: help text
void print_help() {
  Serial.println("UART RX BPSK-sim commands:");
  Serial.println("  MSG:<text>            (simulate frames from text)");
  Serial.println("  FRAME:<hex bytes>     (parse hex frame, e.g. FRAME: AA 05 48 45 4C 4C 4F 0D)");
  Serial.println("  FAULT:NONE | CKSUM    (force checksum error on next validation if CKSUM)");
  Serial.println();
}

// ===== Your original structure with minimal changes =====
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("ESP32 #2 - Receiver Ready (UART-sim BPSK packets)");
  led_init();
  print_help();
}

void loop() {
  if (Serial2.available()) {
    String incoming = Serial2.readStringUntil('\n');
    incoming.trim(); // remove stray newlines/spaces

    if (incoming.length() > 0) {
      // Keep your original prints/echo so nothing breaks
      Serial.println("Received: " + incoming);
      Serial2.println("Echo: " + incoming);

      // --- New: interpret the line as a simulated packet input ---
      if (incoming.startsWith("MSG:")) {
        String msg = incoming.substring(4);
        simulate_receive_plaintext(msg);
        g_fault = F_NONE; // auto-clear fault after a simulated run
      } else if (incoming.startsWith("FRAME:")) {
        uint8_t buf[64]; size_t n=0;
        if (parse_hex_frame(incoming, buf, &n)) {
          consume_frame_and_report(buf, n);
          g_fault = F_NONE;
        } else {
          Serial.println("[ERR] could not parse FRAME line");
          led_err();
        }
      } else if (incoming.startsWith("FAULT:")) {
        String m = incoming.substring(6);
        m.trim(); m.toUpperCase();
        if (m == "NONE")  { g_fault = F_NONE;  Serial.println("[RX] Fault=NONE");  }
        else if (m == "CKSUM") { g_fault = F_CKSUM; Serial.println("[RX] Fault=CKSUM (next check inverted)"); }
        else { Serial.println("[RX] Unknown FAULT"); }
      } else if (incoming == "HELP") {
        print_help();
      } else {
        // If it’s just text, treat as MSG for convenience
        simulate_receive_plaintext(incoming);
        g_fault = F_NONE;
      }
    }
  }
}
