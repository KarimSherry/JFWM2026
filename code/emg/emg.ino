const int EMG_PIN = A0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  int emgValue = analogRead(EMG_PIN);

  Serial.print("EMG: ");
  Serial.println(emgValue);

  delay(5);  // ~200 samples/second
}
