// ESP32 #1 — Transmitter (UART "BPSK" Simulation with Packet Framing & Timing GPIO)
// Keeps your UART echo workflow, adds packet framing and a timing pin for LA.

#define TXD2 23
#define RXD2 22
#define LA_PIN 21            // Logic Analyzer timing pin (per lab table)
#define SYNC_BYTE 0xAA       // 1010 1010
#define MAX_DATA 8           // <= 8 bytes per frame as specified

// "baud-like" pacing between frames (us). Tune for your timing study if needed.
#define FRAME_GAP_US 5000    // 5 ms between frames
// Optional per-byte pacing (us) to emulate symbol pacing. Set to 0 to disable.
#define BYTE_GAP_US 0

// If you want to send each message multiple times for RX/LA capture, bump this up.
#define REPEAT_FRAMES 1

// ----------------- Helpers -----------------
uint8_t computeChecksum(const uint8_t* data, uint8_t len, uint8_t lengthField) {
  // CHECKSUM = XOR of LENGTH + DATA bytes (SYNC excluded)
  uint8_t cs = 0x00;
  cs ^= lengthField;
  for (uint8_t i = 0; i < len; i++) cs ^= data[i];
  return cs;
}

void hexdumpToSerial(const uint8_t* buf, size_t n) {
  for (size_t i = 0; i < n; i++) {
    if (buf[i] < 16) Serial.print('0');
    Serial.print(buf[i], HEX);
    if (i + 1 < n) Serial.print(' ');
  }
}

// Build and send one framed packet: [SYNC][LENGTH][DATA...][CHECKSUM]
void sendFrame(const uint8_t* data, uint8_t len) {
  if (len == 0 || len > MAX_DATA) return;

  uint8_t frame[2 + MAX_DATA + 1];   // SYNC + LENGTH + DATA(<=8) + CHECKSUM
  size_t idx = 0;
  frame[idx++] = SYNC_BYTE;
  frame[idx++] = len;
  for (uint8_t i = 0; i < len; i++) frame[idx++] = data[i];
  frame[idx++] = computeChecksum(data, len, len);

  // Timing marker for LA + timestamps for your report
  uint64_t t0 = esp_timer_get_time();
  digitalWrite(LA_PIN, HIGH);

  // Send raw binary frame over UART2
  for (size_t i = 0; i < idx; i++) {
    Serial2.write(frame[i]);
    if (BYTE_GAP_US) delayMicroseconds(BYTE_GAP_US);
  }

  // Append newline purely so your existing echo path (on the other ESP32) can line-buffer.
  Serial2.write('\n');

  digitalWrite(LA_PIN, LOW);
  uint64_t t1 = esp_timer_get_time();

// --- Pretty-print formatted frame info ---
Serial.print("[SYNC][Length][Data0-7][CheckSum] = ");
Serial.print("[");
Serial.print(frame[0], HEX);
Serial.print("][");
Serial.print(frame[1], HEX);
Serial.print("][");

// Print data bytes dynamically (up to 8)
for (uint8_t i = 0; i < len; i++) {
  Serial.print(frame[2 + i], HEX);
  if (i < len - 1) Serial.print(" ");
}

Serial.print("][");
Serial.print(frame[2 + len], HEX);
Serial.println("]");

// Timing info for verification
Serial.print("t_start(us)="); Serial.print((uint32_t)t0);
Serial.print(" t_end(us)=");  Serial.print((uint32_t)t1);
Serial.print(" duration(us)="); Serial.println((uint32_t)(t1 - t0));
}

// Chunk an arbitrary message into <=8B frames and send
void sendMessageAsFrames(const String& msg) {
  const uint8_t* p = reinterpret_cast<const uint8_t*>(msg.c_str());
  size_t remaining = msg.length();

  for (int rep = 0; rep < REPEAT_FRAMES; rep++) {
    size_t offset = 0;
    while (remaining - offset > 0) {
      uint8_t chunk = (remaining - offset > MAX_DATA) ? MAX_DATA : (uint8_t)(remaining - offset);
      sendFrame(p + offset, chunk);
      offset += chunk;

      if (FRAME_GAP_US) delayMicroseconds(FRAME_GAP_US);
    }
    // Optional gap between repeats
    if (REPEAT_FRAMES > 1 && FRAME_GAP_US) delayMicroseconds(3 * FRAME_GAP_US);
  }
}

// ----------------- Your original skeleton, lightly extended -----------------
void setup() {
  pinMode(LA_PIN, OUTPUT);
  digitalWrite(LA_PIN, LOW);

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  Serial.println("ESP32 #1 - Transmitter Ready");
  Serial.println("Type a message and press Enter to send:");
}

void loop() {
  // Send when you type something in the Serial Monitor
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    message.trim();

    if (message.length() > 0) {
      // Build lab-style frames and send over UART2 (simulating BPSK link via UART)
      sendMessageAsFrames(message);

      // User feedback (original behavior)
      Serial.println("Message queued as framed TX: \"" + message + "\"");
    }
  }

  // Keep your original echo monitor.
  // If the RX echoes printable text (e.g., because of the newline we append), you'll see it here.
  if (Serial2.available()) {
    String incoming = Serial2.readStringUntil('\n');
    incoming.trim();
    if (incoming.length() > 0) {
      Serial.println("Received echo: " + incoming);
    }
  }
}
