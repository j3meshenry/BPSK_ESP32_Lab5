#define TXD2 23
#define RXD2 22

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("ESP32 #2 - Receiver Ready");
}

void loop() {
  if (Serial2.available()) {
    String incoming = Serial2.readStringUntil('\n');
    incoming.trim(); // remove stray newlines/spaces

    if (incoming.length() > 0) {
      Serial.println("Received: " + incoming);
      Serial2.println("Echo: " + incoming);
    }
  }
}