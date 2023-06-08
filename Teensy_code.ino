String GPS = "GPS:";
String COMPASS = "COMPASS:";
String REMOTE = "REMOTE:";

char mode = 'm';

int startTimeGPS = millis();
int startTimeCOMPASS = millis();

#define LED1 2
#define LED2 3
#define btn_pin 4
#define btn_pressed !digitalRead(btn_pin)


void blinkLED(char x) {
  for (char i=0; i<x; i++) {
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    delay(100);
  }
}

int8_t* decode_MtrCmd(String MtrCmdString) {
  int8_t* angles = new int8_t[2];

  int8_t angle1, angle2;
  if (sscanf(MtrCmdString.c_str(), "MTR_CMD:(%hhd;%hhd)", &angle1, &angle2) == 2) {
    angles[0] = angle1;
    angles[1] = angle2;
  } else {
    delete[] angles;
    angles = nullptr;
  }
  return angles;
}

void setup() {
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(btn_pin, INPUT);
  digitalWrite(btn_pin, HIGH);
  Serial.begin(115200);
}
bool btn_hold = false;
void loop() {
  // Output GPS data every 1 second
  if (millis() - startTimeGPS > 1000) {
    Serial.println(GPS + "gps_placeholder");
    startTimeGPS = millis();
  }

  // Output COMPASS every 0.5 seconds
  if (millis() - startTimeCOMPASS > 500) {
    Serial.println(COMPASS + "compass_placeholder");
    startTimeCOMPASS = millis();
  }

  // If btn is pressed, change auto/manual state
  // if in auto mode, turn on led
  if (btn_pressed && !btn_hold) {
    if (mode == 'a'){
      mode = 'm';
      digitalWrite(LED2, LOW);
    } else if (mode == 'm') {
      mode = 'a';
      digitalWrite(LED2, HIGH);
    }
    char str[8];
    sprintf(str, "MODE:%c", mode);
    Serial.println(str);

    btn_hold = true;
  }
  if (!btn_pressed) btn_hold = false;

  // if correct angle string is recieved:
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');

    int8_t* angles = decode_MtrCmd(input);

    // if angles are extracted correctly, blink LED
    if (angles[0] && angles[1]) {
      blinkLED(5);
    }
  }
}