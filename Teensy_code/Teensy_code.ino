char mode = 'm';

int8_t angle1, angle2;
uint8_t pow1, pow2;

int startTimeGPS = millis();
int startTimeCOMPASS = millis();

#define LED1 2
#define LED2 3
#define btn_pin 4
#define btn_pressed !digitalRead(btn_pin)


void blinkLED(char x, char pwm) {
  for (char i=0; i<x; i++) {
    analogWrite(LED1, pwm);
    delay(100);
    analogWrite(LED1, 0);
    delay(100);
  }
}

bool decode_MtrCmd(String MtrCmdString) {

  if (sscanf(MtrCmdString.c_str(), "MTR_CMD:(%hhd;%hhu)(%hhd;%hhu)", &angle1, &pow1, &angle2, &pow2) != 4) {
    // If fails, set to zeros
    pow1 = 0;
    pow2 = 0;
    angle1 = 0;
    angle2 = 0;
    return false;
  }
  return true;
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

    // Set pow1, pow2, angle1, angle2 variables
    if (decode_MtrCmd(input)) {
      // blink LED with pow1 intensity
      blinkLED(5, pow1);
    }

    
  }
}