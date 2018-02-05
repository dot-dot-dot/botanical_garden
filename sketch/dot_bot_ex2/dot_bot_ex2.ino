int inputs[] = {2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11};

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 30; i++) {
    pinMode(inputs[i], INPUT);
  }
}

void loop() {
  String output = "";
  for (int i = 0; i < 30; i++) {
    output += String(i) + ":" + digitalRead(inputs[i]);
    if (i < 29)
      output += "-";
  }
  Serial.println(output);
  delay(100);
}
