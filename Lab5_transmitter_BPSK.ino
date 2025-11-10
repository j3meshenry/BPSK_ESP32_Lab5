/*
 * CECS 460 – Lab 5
 * Part 1 – DBPSK Transmitter (ESP32, 1 kbps, near-continuous carrier)
 * ---------------------------------------------------------------
 * - 64-entry LUT sine via dacWrite(GPIO25).
 * - DBPSK: bit=1 keep phase, bit=0 flip phase (no absolute reference needed).
 * - Framing: [0xAA x3] + 0x55 + LEN + PAYLOAD + 0xFF  (matches the fixed RX)
 * - Timing: 1 kbps; carrier ≈ 7.8125 kHz; 7 carrier cycles/bit + pad to 1000 µs.
 */

#include <Arduino.h>
#include "driver/dac.h"

// -------- Pins & IO --------
#define TX_PIN             25            // DAC1 on ESP32 (GPIO25)
#define UART_BAUD          115200

// -------- Carrier & bit timing --------
#define LUT_SIZE           64
#define SAMPLE_DELAY_US    2             // 2 µs per sample -> carrier ≈ 7.8125 kHz
#define BIT_RATE_HZ        1000          // 1 kbps (bit period = 1000 µs)
#define CYCLES_PER_BIT     7             // 7 * (64 * 2 µs) = 896 µs, then 104 µs pad

// Derived
constexpr uint32_t BIT_PERIOD_US      = 1000000UL / BIT_RATE_HZ;                      // 1000
constexpr uint32_t CARRIER_HZ         = 1000000UL / (LUT_SIZE * SAMPLE_DELAY_US);     // ~7812.5
constexpr uint32_t CARRIER_PERIOD_US  = LUT_SIZE * SAMPLE_DELAY_US;                   // 128
constexpr uint32_t BIT_US_FROM_LUTS   = (uint32_t)CYCLES_PER_BIT * CARRIER_PERIOD_US; // 896

static_assert(LUT_SIZE == 64, "This sketch assumes a 64-entry LUT.");
static_assert(CYCLES_PER_BIT >= 1, "Use at least 1 carrier cycle per bit.");

// -------- Framing (matches receiver) --------
#define SYNC_PATTERN       0xAA
#define SYNC_REPEATS       3
#define SOF_BYTE           0x55
#define EOF_BYTE           0xFF

#define MAX_PAYLOAD        64

// -------- 64-sample 8-bit sine LUT (0–255 centered at 128) --------
const uint8_t sineLUT[LUT_SIZE] = {
  128,140,152,164,175,185,195,203,210,216,220,223,224,223,220,216,
  210,203,195,185,175,164,152,140,128,116,104, 92, 81, 71, 61, 53,
   46, 40, 36, 33, 32, 33, 36, 40, 46, 53, 61, 71, 81, 92,104,116,
  128,140,152,164,175,185,195,203,210,216,220,223,224,223,220,216
};

// =================== Core DBPSK primitives ===================

// Emit exactly one LUT cycle at the given phase offset (0 or LUT_SIZE/2).
inline void emitCarrierCycle(uint8_t phaseOffset)
{
  for (uint8_t i = 0; i < LUT_SIZE; ++i) {
    uint8_t idx = (i + phaseOffset) & (LUT_SIZE - 1);
    dacWrite(TX_PIN, sineLUT[idx]);
    delayMicroseconds(SAMPLE_DELAY_US);
  }
}

// DBPSK phase state: true=positive, false=negative
bool phasePositive = true;

// Send one data bit with DBPSK (bit=1 keep phase, bit=0 flip phase)
void sendBit(uint8_t bitVal)
{
  if (bitVal == 0) phasePositive = !phasePositive; // flip on zero
  const uint8_t phase = phasePositive ? 0 : (LUT_SIZE / 2);

  // Emit near-continuous carrier during the bit window
  for (uint32_t k = 0; k < CYCLES_PER_BIT; ++k) {
    emitCarrierCycle(phase);
  }

  // Pad to land exactly on the bit boundary
  if (BIT_US_FROM_LUTS < BIT_PERIOD_US) {
    delayMicroseconds(BIT_PERIOD_US - BIT_US_FROM_LUTS); // e.g., ~104 µs
  }
}

// Send one byte LSB-first (matches receiver)
void sendByte(uint8_t b)
{
  for (int i = 0; i < 8; ++i) {
    sendBit( (b >> i) & 0x01 );
  }
}

// Send a framed message: [AA x3] + 55 + LEN + PAYLOAD + FF
void sendFrame(const uint8_t* payload, uint8_t len)
{
  // Sync
  for (uint8_t i = 0; i < SYNC_REPEATS; ++i) sendByte(SYNC_PATTERN);

  // Start of frame
  sendByte(SOF_BYTE);

  // Length
  sendByte(len);

  // Payload
  for (uint8_t i = 0; i < len; ++i) sendByte(payload[i]);

  // End of frame
  sendByte(EOF_BYTE);
}

// =================== Setup / Loop ===================

void setup()
{
  Serial.begin(UART_BAUD);
  while (!Serial) { /* wait for Serial on some boards */ }

  // Enable the DAC on GPIO25; idle at mid-scale
  dacWrite(TX_PIN, 128);

  Serial.println();
  Serial.println(F("=== DBPSK Transmitter (ESP32) ==="));
  Serial.printf("TX_PIN=DAC1(GPIO25), UART=%d bps\n", (int)UART_BAUD);
  Serial.printf("Carrier ~ %lu Hz (LUT=%d, %d us/sample)\n",
                (unsigned long)CARRIER_HZ, (int)LUT_SIZE, (int)SAMPLE_DELAY_US);
  Serial.printf("Bit rate = %d bps (bit period = %lu us), cycles/bit = %d (~%lu us)\n",
                (int)BIT_RATE_HZ, (unsigned long)BIT_PERIOD_US, (int)CYCLES_PER_BIT,
                (unsigned long)BIT_US_FROM_LUTS);
  Serial.println(F("Framing: [0xAA x3] + 0x55 + LEN + PAYLOAD + 0xFF"));
  Serial.println(F("Type your message and press Enter to transmit.\n"));
}

void loop()
{
  static char    inbuf[MAX_PAYLOAD + 1];
  static uint8_t payload[MAX_PAYLOAD];

  if (!Serial.available()) return;

  const size_t n = Serial.readBytesUntil('\n', inbuf, MAX_PAYLOAD);
  if (n == 0) return;
  inbuf[n] = '\0';

  // Copy to payload buffer (strip CR)
  uint8_t len = 0;
  for (size_t i = 0; i < n && len < MAX_PAYLOAD; ++i) {
    char c = inbuf[i];
    if (c == '\r') continue;
    payload[len++] = (uint8_t)c;
  }

  Serial.printf("TX %u bytes: \"", (unsigned)len);
  for (uint8_t i = 0; i < len; ++i) Serial.write(payload[i]);
  Serial.println("\"");

  // Optional: reset phase each frame for repeatability (RX is differential so this is not required)
  // phasePositive = true;

  delay(30);               // short settle before frame
  sendFrame(payload, len); // transmit
  delay(120);              // gap after frame
}
