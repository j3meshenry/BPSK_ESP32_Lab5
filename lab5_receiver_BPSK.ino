/*
  CECS 460 – Lab 5: LUT-Based BPSK Receiver
  Student B – Receiver (BPSK RX)
  ESP32 DevKit v1
  ------------------------------------------
  Tasks:
   • Sample incoming GPIO waveform
   • Detect phase flips (zero-cross or correlation)
   • Reconstruct bitstream
   • Forward decoded bytes to PC via UART
*/

#include <Arduino.h>

#define RX_PIN          26        // Connected to TX GPIO 25
#define ADC_CHANNEL     9         // GPIO26 → ADC1_CH9
#define THRESHOLD       2048      // Mid-level threshold for 12-bit ADC
#define BIT_RATE_HZ     1000      // Must match transmitter
#define SAMPLES_PER_BIT 64        // Must match transmitter LUT length
#define SAMPLE_RATE_HZ  (BIT_RATE_HZ * SAMPLES_PER_BIT)
#define UART_BAUD       115200

// Reference LUTs (0° and 180°)
float ref_cos[SAMPLES_PER_BIT];
float ref_negcos[SAMPLES_PER_BIT];

void setupLUT() {
  for (int i = 0; i < SAMPLES_PER_BIT; i++) {
    float angle = 2 * PI * i / SAMPLES_PER_BIT;
    ref_cos[i] = cos(angle);
    ref_negcos[i] = -ref_cos[i];
  }
}

void setup() {
  Serial.begin(UART_BAUD);
  analogReadResolution(12);
  pinMode(RX_PIN, INPUT);
  setupLUT();
  Serial.println("BPSK Receiver Ready...");
}

int sampleADC() {
  return analogRead(RX_PIN);
}

// Correlate sampled waveform with reference LUTs
int detectPhase(float samples[], int len) {
  float corr_pos = 0, corr_neg = 0;
  for (int i = 0; i < len; i++) {
    corr_pos += samples[i] * ref_cos[i];
    corr_neg += samples[i] * ref_negcos[i];
  }
  return (corr_pos > corr_neg) ? 1 : 0;   // 1 = same phase, 0 = flipped
}

void loop() {
  static float buffer[SAMPLES_PER_BIT];
  static uint8_t bitIndex = 0;
  static uint8_t byteBuffer = 0;
  static unsigned long lastBitMicros = micros();

  // --- 1. Sample one bit period ---
  for (int i = 0; i < SAMPLES_PER_BIT; i++) {
    buffer[i] = (float)(sampleADC() - THRESHOLD);
    delayMicroseconds(1e6 / SAMPLE_RATE_HZ);
  }

  // --- 2. Detect phase and recover bit ---
  int bit = detectPhase(buffer, SAMPLES_PER_BIT);

  // --- 3. Shift into byte and send to PC ---
  byteBuffer = (byteBuffer << 1) | (bit & 0x01);
  bitIndex++;
  if (bitIndex == 8) {
    Serial.write(byteBuffer);     // UART → PC
    bitIndex = 0;
    byteBuffer = 0;
  }

  // --- 4. Maintain bit rate ---
  unsigned long bitPeriod = 1e6 / BIT_RATE_HZ;
  unsigned long elapsed = micros() - lastBitMicros;
  if (elapsed < bitPeriod)
    delayMicroseconds(bitPeriod - elapsed);
  lastBitMicros = micros();
}
