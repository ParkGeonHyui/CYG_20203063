#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9         // [1234] LED�� �Ƶ��̳� GPIO 9�� �ɿ� ����
#define PIN_SERVO 10    // [3069] �������͸� �Ƶ��̳� GPIO 10�� �ɿ� ����
#define PIN_IR A0     // [3070] ���ܼ� ������ �Ƶ��̳� �Ƴ��α� A0�ɿ� ����

// Framework setting
#define _DIST_TARGET 255    // [2635] ��ǥ�ϴ� ��ġ
#define _DIST_MIN 100   //[3050] �Ÿ��� �ּڰ��� 100mm
#define _DIST_MAX 400                   //[3068] �Ÿ��� �ִ밪 410mm
#define _ITERM_MAX 120

// Distance sensor
#define _DIST_ALPHA 0.3   //[3054] ���� ��������
// [3131] EMA���Ͱ�
// Servo range
#define _DUTY_MIN 1370    // [3052] ������ �ּҰ����� microseconds�� ǥ��
#define _DUTY_NEU 1470    // [3070] �����÷���Ʈ �߸���ġ�� ����� ���� duty��
#define _DUTY_MAX 1570                //[3062] ������ �ִ밢���� microseconds�� ��

// Servo speed control
#define _SERVO_ANGLE 50   // [3131] �ִ� ���������� ���� ��ǥ ���� ȸ����
#define _SERVO_SPEED 60   //[3064]���� �ӵ��� 30����
#define LENGTH 30
#define k_LENGTH 8

// Event periods
#define _INTERVAL_DIST 30   // [2635] ���ܼ����� ������Ʈ �ֱ�
#define _INTERVAL_SERVO 30  // [2635] ���� ������Ʈ �ֱ�
#define _INTERVAL_SERIAL 100  // [3070] �ø��� �÷��� ���� �ӵ�

// PID parameters
#define _KP 1 // [2635] P �̵� ����(?)
#define _KI 0.016  // [2635] I �̵� ����
#define _KD 50
// [2635] D �̵� ����
#define X 0

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo; // [3070] ���� ��ü ����

// Distance sensor
float dist_target; // [1234] location to send the ball
float dist_raw; // [3070] �Ÿ��� ������ ���� ���� �� �Ÿ��� �����ϱ� ���� ����
float dist_list[LENGTH], sum, dist_ema;
float dist_filtered;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo,
         last_sampling_time_serial;
// [2635] �� �̺�Ʈ�� ������Ʈ�� ���� �����ϴ� �ð�
//[3054] ���������� ������ �Ÿ�, ���������� ������ ���� ����
bool event_dist, event_servo, event_serial; // [2635] �̺�Ʈ�� �̹� �������� ������Ʈ ����

// Servo speed control
int duty_chg_per_interval; // [3055] �����ӵ� ��� ���� ���� ����
int duty_target, duty_curr; // [3131] ��ǥduty, ����duty

// PID variables
float error_curr, error_prev = 0, control, pterm, dterm, iterm = 0; // [3070] PID ��� ���� ���� ����, ��������, ��Ʈ��(?), p ��, d ��, i �� ���� ����


void setup() {
  // initialize GPIO pins for LED and attach servo
  pinMode(PIN_LED, OUTPUT); //[3062] �� LED Ȱ��ȭ
  myservo.attach(PIN_SERVO); // [3070] ���� ������ ���� ���� �ʱ�ȭ

  // initialize global variables
  duty_target, duty_curr = _DUTY_NEU; // [3055] duty_target, duty_curr �ʱ�ȭ
  last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial = 0;
  // [3055] ���ø� Ÿ�� ���� �ʱ�ȭ
  dist_raw, dist_ema = _DIST_MIN; // [3055] dist ���� �ʱ�ȭ
  pterm = iterm = dterm = 0; // [2635] pid ������� �켱 p �� ����ϵ���

  // move servo to neutral position
  //myservo.writeMicroseconds(_DUTY_NEU);

  // initialize serial port
  Serial.begin(57600);

  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) / (float)(_SERVO_ANGLE) * (_SERVO_SPEED / 1000.0) * _INTERVAL_SERVO; // [3131] ������ ���� ���ǵ忡 ���� duty change per interval ���� ��ȯ

}


void loop() {
  // [3055] indentation ����(space 4ĭ)
  /////////////////////
  // Event generator //
  /////////////////////
  unsigned long time_curr = millis(); // [3070] �̺�Ʈ ������Ʈ �ֱ� ����� ���� ���� �ð�
  // [3070] �̺�Ʈ �ֱⰡ ���ƿö����� ����ð��� ���ϸ� ��ٸ����� ��.
  if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  ////////////////////
  // Event handlers //
  ////////////////////

  if (event_dist) {
    event_dist = false; // [2635] ������Ʈ ���
    dist_filtered = ir_distance_filtered(); // [2635] ���� �� �޾ƿͼ� ������ ����

    error_curr = (_DIST_TARGET - dist_filtered); // [2635] ���� ���
    pterm = _KP * (error_curr / (_DIST_MAX - _DIST_MIN) * (_DUTY_MAX - _DUTY_MIN)); // [2635] p���� ����
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    //control = (_KP * pterm); // [2635] control ��, i�� d�� ���� 0
    control = pterm + dterm + iterm;
    duty_target = _DUTY_NEU + control;
    error_prev = error_curr;
    // duty_target = f(duty_neutral, control)
    //duty_target = control/(_DIST_MAX-_DIST_MIN)*(_DUTY_MAX-_DUTY_MIN)+_DUTY_NEU;

    //if (iterm > _ITERM_MAX) iterm = _ITERM_MAX;
    //else if (iterm < -_ITERM_MAX) iterm = -_ITERM_MAX;
    if (abs(iterm) > _ITERM_MAX) iterm = 0;
    if (duty_target < _DUTY_MIN) // [2635] ��ذ� �Ѿ�� ��� �ذ����� ����
    {
      duty_target = _DUTY_MIN;
    }
    if (duty_target > _DUTY_MAX)
    {
      duty_target = _DUTY_MAX;
    }
  }

  if (event_servo) {
    event_servo = false; // [2635] ������Ʈ ���
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if (duty_target > duty_curr) { // [3070]
      duty_curr += duty_chg_per_interval;
      if (duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval * 2;
      if (duty_curr < duty_target) duty_curr = duty_target;
    }

    // update servo position
    myservo.writeMicroseconds((int)duty_curr); // [3070]

    event_servo = false; // [3055] ��� �۾��� ������ �̺�Ʈ�� �ٽ� false��
  }

  if (event_serial) {
    event_serial = false; // [3070] �̺�Ʈ �ֱⰡ �Դٸ� �ٽ� false�� ����� �̺�Ʈ�� ����
    Serial.print("IR:");
    Serial.print(dist_filtered);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
    
    event_serial = false; // [3055] event_serial false�� ����
  }
}

float f(float raw, float a, float b) {
  return 300 / (b - a) * (raw - a) + 100;
}

float ir_distance(void) { // return value unit: mm
  float val; // [3055] ���� val ����
  float volt = float(analogRead(PIN_IR)); // [3055] volt������ ���ܼ� ���� ������ �Է�
  val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0; // [3055] volt �� ����
  return f(val, 66, 300); // [3055] val ����
}

float ir_distance_filtered() { // ���¿� ���� ir_filter �ҽ��� ����߽��ϴ�.
  float sum = 0;
  float alpha = _DIST_ALPHA;
  int iter = 0;
  int a, b;
  while (iter < LENGTH)
  {
    dist_list[iter] = ir_distance();
    sum += dist_list[iter];
    iter++;
  }

  for (int i = 0; i < LENGTH - 1; i++) {
    for (int j = i + 1; j < LENGTH; j++) {
      if (dist_list[i] > dist_list[j]) {
        float tmp = dist_list[i];
        dist_list[i] = dist_list[j];
        dist_list[j] = tmp;
      }
    }
  }

  for (int i = 0; i < k_LENGTH; i++) {
    sum -= dist_list[i];
  }
  for (int i = 1; i <= k_LENGTH; i++) {
    sum -= dist_list[LENGTH - i];
  }

  float dist_cali = sum / (LENGTH - 2 * k_LENGTH);

  return alpha * dist_cali + (1 - alpha) * dist_filtered;
}