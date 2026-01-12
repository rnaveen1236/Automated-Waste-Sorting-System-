#include <Servo.h>

Servo servos[6];
const int servoPins[6] = {2, 3, 4, 5, 6, 7};  // base to gripper
int angles[6];

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(90);  // neutral start
  }
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      int i = 0;
      char *ptr = strtok((char *)input.c_str(), ",");
      while (ptr != NULL && i < 6) {
        angles[i++] = atoi(ptr);
        ptr = strtok(NULL, ",");
      }

      // Move servos
      for (int j = 0; j < 6; j++) {
        servos[j].write(angles[j]);
      }

      Serial.println("OK");
    }
  }
}
