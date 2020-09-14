#define PIN_LED 7
unsigned int count, toggle;

int toggle_state(int toggle) {
  toggle=(++toggle)%2;
  return toggle;
}

void change(int del) {
  toggle = toggle_state(toggle);
  delay(del);
  digitalWrite(PIN_LED, toggle);
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  Serial.println("Hello World!");
  toggle = 0;
  digitalWrite(PIN_LED, toggle);
  change(1000);
  change(100);
  change(100);
  change(100);
  change(100);
  change(100);
  change(100);
  change(100);
  change(100);
  change(100);
  change(100);
  change(100);
  change(100);
  while (1) {}
}
