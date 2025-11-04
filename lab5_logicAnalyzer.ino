/*

File: Lab5_logicAnalyzer.ino 
Functionality: Creates the logic analyzer for our program.
Author: James Henry
Group # 5
Course: CECS 460 - System on Chip Design

 */

#include <soc/timer_group_struct.h>
#include <soc/timer_group_reg.h>

// --- Global Definitions (Configuration) ---

// GPIO Pin Definitions
const int TX_PIN = 18; // Monitor TX from Student A (GPIO 18)
const int RX_PIN = 19; // Monitor RX to Student B (GPIO 19)
const long BAUD_RATE = 115200; // Serial Stream Speed for PC

// --- Sampling Configuration ---
// Target: 1,000,000 Hz (1 MHz) for ~8.68 samples per bit at 115200 baud
// Formula: 80,000,000 Hz / PRESCALER / TIMER_COUNT = 1,000,000 Hz
const long TIMER_PRESCALER = 8;     // 80 MHz / 8 = 10 MHz tick rate
const long TIMER_COUNT = 10;        // 10 MHz / 10 = 1 MHz sampling

// Capture Window: 2000 samples = 2 milliseconds of data
const int SAMPLES_PER_CAPTURE = 2000;

// --- Hardware and State Variables ---
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// --- Trigger and State Tracking ---
volatile int captureCounter = 0; // > 0 means actively sampling
volatile bool dataReady = false; // True when capture complete, ready to send

// --- Circular Buffer Configuration ---
const size_t BUFFER_SIZE = 4096; // Must be power of 2
volatile uint8_t circularBuffer[BUFFER_SIZE];
volatile size_t head = 0; // Write position (Producer - ISR)
volatile size_t tail = 0; // Read position (Consumer - loop)

// --- Buffer Helper Functions ---

// Check if buffer is empty
bool isBufferEmpty() {
    return head == tail;
}

// Add a sample to the circular buffer (called from ISR)
void IRAM_ATTR addSample(uint8_t tx_state, uint8_t rx_state) {
    // Pack both states into one byte: Bit 0 = TX, Bit 1 = RX
    uint8_t packed_data = (tx_state << 0) | (rx_state << 1);
    
    // Write sample to buffer
    circularBuffer[head] = packed_data;
    
    // Advance head with fast modulo (bitwise AND)
    head = (head + 1) & (BUFFER_SIZE - 1);
    
    // Handle buffer overflow: if head catches tail, move tail (lose oldest data)
    if (head == tail) {
        tail = (tail + 1) & (BUFFER_SIZE - 1);
    }
}

// --- Interrupt Service Routine (ISR) for Periodic Sampling ---
void IRAM_ATTR onTimer() {
    // Feed the watchdog timer to prevent panic
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
    
    // Check if we're in active capture mode
    if (captureCounter > 0) {
        // Fast GPIO read using direct register access (much faster than digitalRead)
        uint32_t gpio_state = REG_READ(GPIO_IN_REG);
        
        // Extract individual pin states
        uint8_t tx_state = (gpio_state >> TX_PIN) & 0x01;
        uint8_t rx_state = (gpio_state >> RX_PIN) & 0x01;
        
        // Store the sample
        addSample(tx_state, rx_state);
        
        // Decrement capture counter
        captureCounter--;
        
        // Check if capture window is complete
        if (captureCounter == 0) {
            timerAlarmDisable(timer); // Stop timer until next trigger
            dataReady = true;          // Signal main loop that data is ready
        }
    }
}

// --- External Interrupt for Trigger (Start Bit Detection) ---
void IRAM_ATTR onTrigger() {
    // Only trigger if previous capture is complete
    if (captureCounter == 0) {
        portENTER_CRITICAL_ISR(&timerMux);
        
        // Immediately capture the trigger edge (don't miss the start bit!)
        uint32_t gpio_state = REG_READ(GPIO_IN_REG);
        uint8_t tx_state = (gpio_state >> TX_PIN) & 0x01;
        uint8_t rx_state = (gpio_state >> RX_PIN) & 0x01;
        
        // Reset buffer for fresh capture
        head = 0;
        tail = 0;
        
        // Capture the first sample (the trigger edge itself)
        addSample(tx_state, rx_state);
        
        // Set counter (minus 1 since we already captured one sample)
        captureCounter = SAMPLES_PER_CAPTURE - 1;
        
        // Clear data ready flag
        dataReady = false;
        
        // Start the sampling timer
        timerAlarmEnable(timer);
        
        portEXIT_CRITICAL_ISR(&timerMux);
    }
}

// --- Setup Function ---
void setup() {
    // Initialize serial communication
    Serial.begin(BAUD_RATE);
    delay(2000); // Give PC time to connect (don't block forever)
    
    Serial.println("\n=== ESP32 Logic Analyzer - CECS 460 Lab 5 ===");
    Serial.println("Sampling Rate: 1 MHz");
    Serial.print("Capture Window: ");
    Serial.print(SAMPLES_PER_CAPTURE);
    Serial.println(" samples (2 ms)");
    Serial.print("Monitoring TX: GPIO ");
    Serial.println(TX_PIN);
    Serial.print("Monitoring RX: GPIO ");
    Serial.println(RX_PIN);
    Serial.println("Waiting for trigger (falling edge on TX)...\n");
    
    // Configure GPIO pins as inputs
    pinMode(TX_PIN, INPUT);
    pinMode(RX_PIN, INPUT);
    
    // Setup hardware timer for 1 MHz sampling
    timer = timerBegin(0, TIMER_PRESCALER, true); // Timer 0, prescaler 8, count up
    timerAttachInterrupt(timer, &onTimer, true);  // Attach ISR, edge-triggered
    timerAlarmWrite(timer, TIMER_COUNT, true);    // Set alarm period (auto-reload)
    // Note: Timer NOT enabled yet - will start on trigger
    
    // Setup external interrupt for trigger detection
    // Falling edge = start bit (UART idle HIGH, start bit LOW)
    attachInterrupt(digitalPinToInterrupt(TX_PIN), onTrigger, FALLING);
    
    Serial.println("Setup complete. Ready to capture!");
}

// --- Main Loop (Data Consumer) ---
void loop() {
    // Check if capture is complete and data is ready to send
    if (dataReady && !isBufferEmpty()) {
        
        // Copy buffer indices to local variables (thread-safe snapshot)
        portENTER_CRITICAL(&timerMux);
        size_t local_head = head;
        size_t local_tail = tail;
        portEXIT_CRITICAL(&timerMux);
        
        // Calculate number of samples to send
        size_t samples_to_send;
        if (local_head >= local_tail) {
            samples_to_send = local_head - local_tail;
        } else {
            samples_to_send = (BUFFER_SIZE - local_tail) + local_head;
        }
        
        // Send start marker
        Serial.write('S');
        
        // Stream samples with flow control
        size_t sent_count = 0;
        while (local_tail != local_head) {
            // Wait for serial buffer space (prevent overflow)
            while (Serial.availableForWrite() < 1) {
                delayMicroseconds(10);
            }
            
            // Send sample
            uint8_t sample = circularBuffer[local_tail];
            Serial.write(sample);
            
            // Advance tail
            local_tail = (local_tail + 1) & (BUFFER_SIZE - 1);
            sent_count++;
        }
        
        // Send end marker
        Serial.write('E');
        
        // Update actual tail position (mark data as consumed)
        portENTER_CRITICAL(&timerMux);
        tail = local_tail;
        dataReady = false;
        portEXIT_CRITICAL(&timerMux);
        
        // Status message
        Serial.print("\n[Capture ");
        Serial.print(sent_count);
        Serial.println(" samples] Waiting for next trigger...\n");
    }
    
    // Small delay to prevent busy-waiting
    delay(1);
}
