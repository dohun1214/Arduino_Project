#include <SoftwareSerial.h>
#include <DHT11.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// I2C 주소 0x27, 16x2 LCD 설정
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define trigPin 13       // 초음파 센서 트리거 핀 정의
#define echoPin 12       // 초음파 센서 에코 핀 정의

int bluetoothTx = 2;     // 블루투스 모듈 TX 핀
int bluetoothRx = 3;     // 블루투스 모듈 RX 핀

int flameSensorPin = 7;  // 화염 감지 센서 핀
int flamePin = 8;        // 화염 감지 시 LED 출력 핀

int speakerPin = 9;      // 부저 핀

int alertLED = 11;       // 초음파 센서 감지 시 경고 LED 핀

// 소프트웨어 시리얼: 블루투스 통신용
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

// DHT11 온습도 센서: 데이터 핀 4번
DHT11 dht11(4);

int LED1 = 5;            // 조도에 따른 LED 출력 핀
int lightPin = A0;       // 조도 센서(아날로그 입력) 핀
int motor_control = 6;   // 서보 모터 제어 핀
int tactPin = 10;        // 택트 스위치(긴급 버튼) 핀

Servo servo;             // 서보 객체 생성

char mode = 'A';         // 현재 모드: 'A' = 자동, 'M' = 수동

int isMotionDetected = 0;     // 모션(초음파) 감지 상태 플래그
int isEmergencyActive = 0;    // 긴급 버튼 활성화 상태 플래그

// 센서 측정값들을 블루투스로 전송하는 함수
void sendSensorData(int temperature, int humidity, int brightness, long cm) {
  bluetooth.print("Temperature: ");
  bluetooth.print(temperature);
  bluetooth.print(" °C\tHumidity: ");
  bluetooth.print(humidity);
  bluetooth.print(" %\tLight: ");
  bluetooth.print(brightness);
  bluetooth.print("\tDistance: ");
  bluetooth.print(cm);
  bluetooth.println(" cm");
}

// 초음파 센서에서 구한 마이크로초 값을 센티미터로 변환하는 함수
long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

// 택트 스위치(긴급 버튼) 눌렀을 때 실행되는 함수
void emergencyButton() {
  isEmergencyActive = 1;                        // 긴급 상태 활성화
  digitalWrite(alertLED, HIGH);                 // 경고 LED 켜기
  delay(200);
  digitalWrite(alertLED, LOW);                  // 경고 LED 끄기
  lcd.clear();                                  // LCD 초기화
  lcd.setCursor(0, 0);
  lcd.print("!!! EMERGENCY !!!");               // LCD에 경고 메시지 출력
  bluetooth.println("Emergency Button Pressed!"); // 블루투스로 긴급 메시지 전송
  tone(speakerPin, 5000, 500);                  // 부저 울리기
}

void setup() {
  Serial.begin(9600);        // 시리얼 모니터 통신 속도 설정
  bluetooth.begin(9600);     // 블루투스 통신 속도 설정

  // 각 핀 모드를 설정
  pinMode(LED1, OUTPUT);          // 조도 LED 출력
  pinMode(trigPin, OUTPUT);       // 초음파 센서 트리거 출력
  pinMode(echoPin, INPUT);        // 초음파 센서 에코 입력
  pinMode(flameSensorPin, INPUT); // 화염 센서 입력
  pinMode(flamePin, OUTPUT);      // 화염 감지 시 LED 출력
  pinMode(alertLED, OUTPUT);      // 알림용 LED 출력
  pinMode(speakerPin, OUTPUT);    // 부저 출력
  pinMode(tactPin, INPUT);        // 택트 스위치 입력
  pinMode(A0, INPUT);             // 조도 센서 입력

  lcd.begin();         // LCD 초기화
  lcd.backlight();     // LCD 백라이트 켜기
}

void loop() {
  // 온도와 습도 값을 저장할 변수 초기화
  int temperature = 0;
  int humidity = 0;
  long duration, cm;  // 초음파 센서용 변수

  // 초음파 센서로 거리 측정 (트리거 신호 발생)
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 에코 핀에서 반환된 시간 측정
  duration = pulseIn(echoPin, HIGH);

  // 측정된 시간(마이크로초)을 센티미터로 변환
  cm = microsecondsToCentimeters(duration);

  // 블루투스로부터 명령어 읽기
  char cmd = bluetooth.read();

  // DHT11 센서로부터 온습도 읽기 시도
  if (dht11.readTemperatureHumidity(temperature, humidity) == 0) {

    // 블루투스로 'A' 또는 'M' 명령어 수신 시 모드 변경
    if (cmd == 'A' || cmd == 'M') {
      if (mode != cmd) {
        mode = cmd;
        if (mode == 'A') {
          bluetooth.println("Auto mode");   // 자동 모드 진입 메시지
        } else if (mode == 'M') {
          bluetooth.println("Manual mode"); // 수동 모드 진입 메시지
        }
      }
    }

    // 조도 센서 값 읽기 및 LED 밝기 제어
    int brightness = analogRead(lightPin) / 4; // 0~1023 값을 0~255로 변환
    if (mode == 'A') {
      // 자동 모드: 주변 밝기가 일정 수준 이하일 때 LED 켜기
      if (brightness <= 200) {
        analogWrite(LED1, 255 - brightness); // 밝기가 어두울수록 LED 밝음
      } else {
        digitalWrite(LED1, LOW); // 충분히 밝으면 LED 끔
      }
    } else if (mode == 'M') {
      // 수동 모드: 블루투스 명령에 따라 LED 제어
      if (cmd == '1') {
        bluetooth.println("LED ON");
        digitalWrite(LED1, HIGH); // LED 켜기
      } else if (cmd == '0') {
        bluetooth.println("LED OFF");
        digitalWrite(LED1, LOW);  // LED 끄기
      }
    }

    // 온도가 일정 기준 이상일 때 서보 모터(팬) 제어
    if (mode == 'A') {
      // 자동 모드: 온도가 35°C 이상이면 팬 작동
      if (temperature >= 35) {
        servo.attach(motor_control); // 서보 핀 연결
        servo.write(0);              // 서보를 0도로 이동 (팬 회전 시작)
        delay(500);
        servo.write(180);            // 서보를 180도로 이동 (팬 회전 지속)
      } else {
        servo.detach(); // 기준 이하일 때 서보 분리 (팬 정지)
      }
    } else if (mode == 'M') {
      // 수동 모드: 블루투스 명령에 따라 팬 제어
      if (cmd == '2') {
        bluetooth.println("Fan ON");
        servo.attach(motor_control);
        servo.write(0);
        delay(500);
        servo.write(180);
      } else if (cmd == '3') {
        bluetooth.println("Fan OFF");
        servo.detach();
      }
    }

    // 블루투스로 'S' 명령어 수신 시 센서 데이터 전송
    if (cmd == 'S') {
      sendSensorData(temperature, humidity, brightness, cm);
    }
  }

  // 화염 감지 센서가 화염을 감지하면 LED 켜고 부저 울림
  if (digitalRead(flameSensorPin) == LOW) {
    digitalWrite(flamePin, HIGH);    // 화염 감지 LED 켜기
    tone(speakerPin, 5000, 500);     // 부저 울리기
  } else {
    digitalWrite(flamePin, LOW);     // 화염 미감지 시 LED 끄기
  }

  // 초음파 센서로 물체가 3cm 이하로 접근 시 모션 감지 처리
  if (cm <= 3) {
    isMotionDetected = 1;             // 모션 감지 상태로 설정
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Motion Detected");     // LCD에 모션 감지 메시지 출력
    tone(speakerPin, 5000, 500);      // 부저 울리기
    bluetooth.print("Motion Detected"); // 블루투스로 모션 감지 메시지 전송
    digitalWrite(alertLED, HIGH);     // 경고 LED 깜빡임
    delay(100);
    digitalWrite(alertLED, LOW);
  } else {
    isMotionDetected = 0;             // 모션 감지 해제
  }

  // 택트 스위치(긴급 버튼) 눌렀을 때 처리
  if (digitalRead(tactPin)) {
    emergencyButton();  // 긴급 처리 함수 호출
  } else {
    isEmergencyActive = 0; // 택트 스위치 미눌림 시 긴급 상태 해제
  }

  // 긴급 상태나 모션 감지 상태가 아닐 때 LCD에 온습도 정보 표시
  if (!isEmergencyActive && !isMotionDetected) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp:");
    lcd.print(temperature);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Humi:");
    lcd.print(humidity);
    lcd.print("%");
    digitalWrite(alertLED, LOW); // 알림 LED 끄기
  }

  // 블루투스 통신이 들어오면 처리 (현재 시리얼 예제로만 포함)
  if (bluetooth.available()) {
    // Serial.println((char)bluetooth.read());
  }

  // 시리얼 모니터 통신이 들어오면 블루투스로 전송 (현재 주석 처리)
  if (Serial.available()) {
    // bluetooth.print((char)Serial.read());
  }
  
}