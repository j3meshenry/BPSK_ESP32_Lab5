#define TXD2 23
#define RXD2 22

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("ESP32 #1 - Transmitter Ready");
  Serial.println("Type a message and press Enter to send:");
}

void loop() {
  // Send only when you type something in the Serial Monitor
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    message.trim();
    if (message.length() > 0) {
      Serial2.println(message);                // send to ESP32 #2
      Serial.println("Message sent: " + message);
    }
  }

  // Only show received echo if it actually comes back
  if (Serial2.available()) {
    String incoming = Serial2.readStringUntil('\n');
    incoming.trim();
    if (incoming.length() > 0) {
      Serial.println("Received echo: " + incoming);
    }
  }
}
