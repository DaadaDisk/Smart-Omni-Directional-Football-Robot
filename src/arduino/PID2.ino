#define ENA_A 4
#define IN1_A 23
#define IN2_A 22


#define ENB_B 7
#define IN3_B 8
#define IN4_B 9

#define ENC_C 10
#define IN5_C 11
#define IN6_C 12

#define ENCODER_A_PIN_A 18
#define ENCODER_A_PIN_B 19

#define ENCODER_B_PIN_A 20
#define ENCODER_B_PIN_B 21

#define ENCODER_C_PIN_A 2
#define ENCODER_C_PIN_B 3

#define SAMPLE_TIME 50
#define RAMP_STEP_RPM 10

volatile long pulses_M1 = 0;
volatile long pulses_M2 = 0;
volatile long pulses_M3 = 0;

unsigned long last_time = 0;

const float PPR_M1 = 823.5;
const float PPR_M2 = 819.4;
const float PPR_M3 = 822.0;

float desired_rpm_M1 = 0;
float desired_rpm_M2 = 0;
float desired_rpm_M3 = 0;

float target_rpm_M1 = 0;
float target_rpm_M2 = 0;
float target_rpm_M3 = 0;

float current_rpm_M1 = 0;
float current_rpm_M2 = 0;
float current_rpm_M3 = 0;

float Kp = 0.9;
float Ki = 1.2;
float Kd = 0.05;

float error_M1 = 0;
float integral_M1 = 0;
float last_error_M1 = 0;

float error_M2 = 0;
float integral_M2 = 0;
float last_error_M2 = 0;

float error_M3 = 0;
float integral_M3 = 0;
float last_error_M3 = 0;

float pwm_out_M1 = 0;
float pwm_out_M2 = 0;
float pwm_out_M3 = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  pinMode(ENA_A, OUTPUT);
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);

  pinMode(ENB_B, OUTPUT);
  pinMode(IN3_B, OUTPUT);
  pinMode(IN4_B, OUTPUT);

  pinMode(ENC_C, OUTPUT);
  pinMode(IN5_C, OUTPUT);
  pinMode(IN6_C, OUTPUT);

  pinMode(ENCODER_A_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_A_PIN_B, INPUT_PULLUP);

  pinMode(ENCODER_B_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN_B, INPUT_PULLUP);

  pinMode(ENCODER_C_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_C_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN_A), countM1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN_A), countM2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C_PIN_A), countM3, RISING);

  last_time = millis();
}

void loop() {

  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    parseIncomingData(data);
  }

  unsigned long current_time = millis();
  unsigned long time_change = current_time - last_time;

  if (time_change >= SAMPLE_TIME) {

    noInterrupts();

    long snap_M1 = pulses_M1;
    long snap_M2 = pulses_M2;
    long snap_M3 = pulses_M3;

    pulses_M1 = 0;
    pulses_M2 = 0;
    pulses_M3 = 0;

    interrupts();

    current_rpm_M1 = ((float)snap_M1 / PPR_M1) * (60000.0 / time_change);
    current_rpm_M2 = ((float)snap_M2 / PPR_M2) * (60000.0 / time_change);
    current_rpm_M3 = ((float)snap_M3 / PPR_M3) * (60000.0 / time_change);

    target_rpm_M1 = updateRamp(target_rpm_M1, desired_rpm_M1);
    target_rpm_M2 = updateRamp(target_rpm_M2, desired_rpm_M2);
    target_rpm_M3 = updateRamp(target_rpm_M3, desired_rpm_M3);

    pwm_out_M1 = computePID(
      target_rpm_M1,
      current_rpm_M1,
      error_M1,
      integral_M1,
      last_error_M1
    );

    pwm_out_M2 = computePID(
      target_rpm_M2,
      current_rpm_M2,
      error_M2,
      integral_M2,
      last_error_M2
    );

    pwm_out_M3 = computePID(
      target_rpm_M3,
      current_rpm_M3,
      error_M3,
      integral_M3,
      last_error_M3
    );

    driveMotor(
      ENA_A,
      IN1_A,
      IN2_A,
      pwm_out_M1,
      false
    );

    driveMotor(
      ENB_B,
      IN3_B,
      IN4_B,
      pwm_out_M2,
      true
    );

    driveMotor(
      ENC_C,
      IN5_C,
      IN6_C,
      pwm_out_M3,
      false
    );

//    Serial.print("Cur:");
//    Serial.print(current_rpm_M1, 1);
//    Serial.print(",");
//
//    Serial.print(current_rpm_M2, 1);
//    Serial.print(",");
//    Serial.println(current_rpm_M3, 1);

    //Serial.print(digitalRead(IN1_A));
    //Serial.print(",");
    //Serial.println(digitalRead(IN2_A));
    last_time = current_time;
  }
}

float updateRamp(float current, float target) {

  if (current < target) {

    current += RAMP_STEP_RPM;

    if (current > target) {
      current = target;
    }
  }

  else if (current > target) {

    current -= RAMP_STEP_RPM;

    if (current < target) {
      current = target;
    }
  }

  return current;
}

float computePID(
  float target,
  float current,
  float &error,
  float &integral,
  float &last_error
) {

  error = target - current;

  integral += error * (SAMPLE_TIME / 1000.0);

  integral = constrain(integral, -255, 255);

  float derivative =
    (error - last_error) /
    (SAMPLE_TIME / 1000.0);

  float output =
    (Kp * error) +
    (Ki * integral) +
    (Kd * derivative);

  last_error = error;

  return constrain(output, -255, 255);
}

void driveMotor(
  int pwm_pin,
  int in1,
  int in2,
  float pwm_val,
  bool reverse_hardware
) {

  if (reverse_hardware) {
    pwm_val = -pwm_val;
  }

  if (pwm_val > 0) {

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    analogWrite(
      pwm_pin,
      (int)pwm_val
    );
  }

  else if (pwm_val < 0) {

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    analogWrite(
      pwm_pin,
      (int)abs(pwm_val)
    );
  }

  else {

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);

    analogWrite(pwm_pin, 0);
  }
}

void parseIncomingData(String data) {

  int firstComma = data.indexOf(',');

  int secondComma =
    data.indexOf(',', firstComma + 1);

  if (
    firstComma != -1 &&
    secondComma != -1
  ) {

    desired_rpm_M1 =
      data.substring(
        0,
        firstComma
      ).toFloat();

    desired_rpm_M2 =
      data.substring(
        firstComma + 1,
        secondComma
      ).toFloat();

    desired_rpm_M3 =
      data.substring(
        secondComma + 1
      ).toFloat();
  }
}

void countM1() {

  if (
    digitalRead(ENCODER_A_PIN_B) == LOW
  ) {
    pulses_M1--;
  }

  else {
    pulses_M1++;
  }
}

void countM2() {

  if (
    digitalRead(ENCODER_B_PIN_B) == LOW
  ) {
    pulses_M2--;
  }

  else {
    pulses_M2++;
  }
}

void countM3() {

  if (
    digitalRead(ENCODER_C_PIN_B) == LOW
  ) {
    pulses_M3--;
  }

  else {
    pulses_M3++;
  }
}
