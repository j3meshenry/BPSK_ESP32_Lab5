/*
  CECS 460 – Lab 5: LUT-Based BPSK Receiver (SIMPLE OUTPUT)
  Student B – Receiver (BPSK RX)
  ESP32 DevKit v1
  ------------------------------------------
  Simple version with clean output
*/

#include <Arduino.h>

// Pin Configuration
#define RX_PIN          26        // Connected to transmitter GPIO 25

// BPSK Parameters
#define BIT_RATE_HZ     1000      // Must match transmitter
#define SAMPLES_PER_BIT 64        // Must match transmitter LUT length
#define SAMPLE_RATE_HZ  (BIT_RATE_HZ * SAMPLES_PER_BIT)  // 64,000 Hz
#define UART_BAUD       115200

// Reference LUTs (0° and 180° phase)
float ref_cos[SAMPLES_PER_BIT];
float ref_negcos[SAMPLES_PER_BIT];

// Sampling state
volatile int sampleBuffer[SAMPLES_PER_BIT];
volatile int sampleIndex = 0;
volatile bool bufferReady = false;

// Timer
hw_timer_t *samplingTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Setup LUT with cosine reference waveforms
void setupLUT() {
  for (int i = 0; i < SAMPLES_PER_BIT; i++) {
    float angle = 2.0 * PI * i / SAMPLES_PER_BIT;
    ref_cos[i] = cos(angle);
    ref_negcos[i] = -ref_cos[i];
  }
}

// Timer ISR: Sample the input pin at 64 kHz
void IRAM_ATTR onSampleTimer() {
  // Read digital pin state
  int sample = digitalRead(RX_PIN);
  
  // Convert to bipolar: LOW→-1, HIGH→+1 (for correlation)
  sampleBuffer[sampleIndex] = (sample == HIGH) ? 1 : -1;
  sampleIndex++;
  
  // Check if we've captured a complete bit period (64 samples)
  if (sampleIndex >= SAMPLES_PER_BIT) {
    bufferReady = true;
    sampleIndex = 0;
  }
}

// Correlate sampled waveform with reference LUTs
int detectPhase(volatile int samples[], int len) {
  float corr_pos = 0.0;
  float corr_neg = 0.0;
  
  for (int i = 0; i < len; i++) {
    corr_pos += samples[i] * ref_cos[i];
    corr_neg += samples[i] * ref_negcos[i];
  }
  
  // Return detected phase: 1 = 0° (positive correlation), 0 = 180° (negative)
  return (corr_pos > corr_neg) ? 1 : 0;
}

void setup() {
  Serial.begin(UART_BAUD);
  delay(1000);
  
  pinMode(RX_PIN, INPUT);
  
  // Initialize LUT
  setupLUT();
  
  Serial.println("\n=== BPSK Receiver Ready ===");
  Serial.println("Receiving...\n");
  
  // Setup hardware timer for precise 64 kHz sampling
  samplingTimer = timerBegin(SAMPLE_RATE_HZ);
  timerAttachInterrupt(samplingTimer, &onSampleTimer);
  timerStart(samplingTimer);
}

void loop() {
  static uint8_t bitIndex = 0;
  static uint8_t byteBuffer = 0;
  static int lastPhase = -1;
  
  // Wait for a complete bit period to be sampled
  if (bufferReady) {
    portENTER_CRITICAL(&timerMux);
    
    // Copy samples to local array
    int localBuffer[SAMPLES_PER_BIT];
    for (int i = 0; i < SAMPLES_PER_BIT; i++) {
      localBuffer[i] = sampleBuffer[i];
    }
    bufferReady = false;
    
    portEXIT_CRITICAL(&timerMux);
    
    // Detect current phase using correlation
    int currentPhase = detectPhase(localBuffer, SAMPLES_PER_BIT);
    
    // Decode bit using differential encoding
    int bit;
    if (lastPhase == -1) {
      // First bit: initialize reference phase
      bit = 0;  // Assume starting bit is 0
      lastPhase = currentPhase;
    } else {
      // Differential decoding: phase change = 1, no change = 0
      bit = (currentPhase != lastPhase) ? 1 : 0;
      lastPhase = currentPhase;
    }
    
    // Shift bit into byte buffer (MSB first)
    byteBuffer = (byteBuffer << 1) | (bit & 0x01);
    bitIndex++;
    
    // When we have a complete byte, output it
    if (bitIndex == 8) {
      // Simply print the character
      Serial.print((char)byteBuffer);
      
      // Reset for next byte
      bitIndex = 0;
      byteBuffer = 0;
    }
  }
  
  // Small delay to prevent busy waiting
  delayMicroseconds(100);
}
