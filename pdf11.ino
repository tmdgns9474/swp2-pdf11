#include <Servo.h>

// =========================================================
// [실제 회로 사진(사진 4, 5) 기준 핀 설정]
// =========================================================
#define PIN_TRIG  12    // 초음파 센서 TRIG (12번 핀)
#define PIN_ECHO  13    // 초음파 센서 ECHO (13번 핀)
#define PIN_LED   9     // 유효 범위 점등 LED (9번 핀)
#define PIN_SERVO 10    // 서보모터 제어 핀 (10번 핀)

// 거리 측정 범위 설정 (cm)
#define _DIST_MIN 18.0
#define _DIST_MAX 36.0

// 서보 듀티 펄스 폭 설정 (us 단위)
#define _DUTY_MIN 550   // 0도 (us)
#define _DUTY_MAX 2400  // 180도 (us)
#define _DUTY_NEU ((_DUTY_MIN + _DUTY_MAX) / 2) // 90도 중립

float alpha = 0.25;     // EMA 필터 계수

Servo myservo;

float dist_raw = _DIST_MIN;
float dist_ema = _DIST_MIN;
int last_duty = _DUTY_MIN;

void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LED, OUTPUT);
  
  myservo.attach(PIN_SERVO, _DUTY_MIN, _DUTY_MAX);
  myservo.writeMicroseconds(_DUTY_MIN);
  
  Serial.begin(57600);
  dist_ema = _DIST_MIN;
}

void loop() {
  // 1. 초음파 센서 거리 측정 (raw data)
  dist_raw = measure_distance();

  // 2. 범위 필터 (Range Filter) 및 LED 제어
  if (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX) {
    digitalWrite(PIN_LED, HIGH); // 18~36cm 범위 내: LED ON
  } else {
    digitalWrite(PIN_LED, LOW);  // 범위 밖: LED OFF
    
    // 측정 실패 및 이탈 시 0도로 튀는 현상 방지
    if (dist_raw < _DIST_MIN) dist_raw = _DIST_MIN;
    else if (dist_raw > _DIST_MAX) dist_raw = _DIST_MAX;
  }

  // 3. EMA 필터 적용
  dist_ema = alpha * dist_raw + (1.0 - alpha) * dist_ema;

  // 4. 거리를 서보 듀티(펄스 폭)로 비례 변환
  int target_duty = map_distance_to_duty(dist_ema);

  // 5. 모터 떨림(Jittering) 방지 불감대 로직 (10us 이상 변화 시에만 동작)
  if (abs(target_duty - last_duty) >= 10) {
    myservo.writeMicroseconds(target_duty);
    last_duty = target_duty;
  }

  // 6. 슬라이드 지정 시리얼 출력 양식
  Serial.print("Min:");       Serial.print(_DIST_MIN);
  Serial.print(",dist:");     Serial.print(dist_raw);
  Serial.print(",ema:");      Serial.print(dist_ema);
  Serial.print(",Servo:");    Serial.print(last_duty);
  Serial.print(",Max:");      Serial.print(_DIST_MAX);
  Serial.println("");

  delay(30);
}

// 초음파 거리 측정 함수
float measure_distance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_ECHO, HIGH, 30000); // 30ms 타임아웃
  if (duration == 0) return _DIST_MIN;
  
  return duration * 0.0343 / 2.0;
}

// 거리(18~36cm) -> 서보 듀티(_DUTY_MIN ~ _DUTY_MAX) 변환 함수
int map_distance_to_duty(float dist) {
  if (dist <= _DIST_MIN) return _DUTY_MIN;
  if (dist >= _DIST_MAX) return _DUTY_MAX;
  
  return (int)(_DUTY_MIN + (dist - _DIST_MIN) * (_DUTY_MAX - _DUTY_MIN) / (_DIST_MAX - _DIST_MIN));
}
