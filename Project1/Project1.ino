#include <SoftwareSerial.h>
#include <DHT11.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define trigPin 13  //초음파센서
#define echoPin 12  //초음파센서

int bluetoothTx = 2;  //블루투스
int bluetoothRx = 3;  //블루투스

int flameSensorPin = 7;  //화염감지 센서
int flamePin = 8;        //화염감지해서 LED

int speakerPin = 9;  //부저

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);
DHT11 dht11(4);         //온습도센서
int LED1 = 5;           // 온도에 따른 LED
int lightPin = A0;      //조도센서
int motor_control = 6;  //서보모터
Servo servo;


char mode = 'A';

//센서 측정값들 블루투스로 보내주는 함수
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

//초음파센서 함수
long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}



void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  pinMode(LED1, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(flameSensorPin, INPUT);
  pinMode(flamePin, OUTPUT);

  lcd.begin();
  lcd.backlight();
}

void loop() {

  //온도 습도
  int temperature = 0;
  int humidity = 0;
  long duration, cm;  //초음파 변수

  //duration
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);

  //cm
  cm = microsecondsToCentimeters(duration);

  //bluetooth로 읽은 값
  char cmd = bluetooth.read();


  if (dht11.readTemperatureHumidity(temperature, humidity) == 0) {

    if (cmd == 'A' || cmd == 'M') {
      if (mode != cmd) {
        mode = cmd;
        if (mode == 'A') {
          bluetooth.println("Auto mode");
        } else if (mode == 'M') {
          bluetooth.println("Manual mode");
        }
      }
    }

    //주변 밝기에 따라 LED밝기
    int brightness = analogRead(lightPin) / 4;
    if (mode == 'A') {
      if (brightness <= 200) {  //밝기가 200이하면 analog 출력, 200이상이면 LED끔
        analogWrite(LED1, 255 - brightness);
      } else {
        digitalWrite(LED1, LOW);
      }
    } else if (mode == 'M') {
      if (cmd == '1') {
        bluetooth.print("LED ON");
        digitalWrite(LED1, HIGH);
      } else if (cmd == '0') {
        bluetooth.print("LED OFF");
        digitalWrite(LED1, LOW);
      }
    }




    // LED밝기 끝

    //   Serial.print("Temperature: ");
    //   Serial.print(temperature);
    //   Serial.print(" °C\tHumidity: ");
    //   Serial.print(humidity);
    //   Serial.print(" %\t");
    //   Serial.print("Brightness : ");
    //   Serial.print(brightness);
    //   Serial.print("\t");
    //   Serial.print(cm);
    //   Serial.println(" cm");
    if (cmd == 'S') {
      sendSensorData(temperature, humidity, brightness, cm);
    }
  }

  // 일정 온도 넘으면 팬 돌아가게
  if (temperature >= 30) { // if문 숫자 수정
    //팬 돌아가게
    servo.attach(motor_control);
    servo.write(0);
    delay(500);
    servo.write(180);
  } else {
    servo.detach();
  }
  //온도 습도 끝

  //화염 감지해서 LED 켜기
  if (digitalRead(flameSensorPin) == LOW) {
    digitalWrite(flamePin, HIGH);
    //부저
    tone(speakerPin, 5000, 500);
  } else
    digitalWrite(flamePin, LOW);


  // lcd출력
  if (cm <= 0) {  // if문 거리 숫자 수정
    //가까이 오면 lcd출력, 부저 울리기
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Motion Detected");
    tone(speakerPin, 5000, 500);
  } else {
    digitalWrite(flamePin, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp:");
    lcd.print(temperature);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Humi:");
    lcd.print(humidity);
    lcd.print("%");
  }



  // bluetooth.available() 블루투스로 보낸 데이터가 도착했을때
  // bluetooth.print() 블루투스로 데이터를 보냄
  // bluetooth.read() 블루투스로 보낸 데이터
  // Serial.available() 시리얼 모니터에 값을 입력했을때
  // Serial.print() 시리얼 모니터에 출력
  // Serial.read() 시리얼 모니터로 입력한 데이터

  //블루투스
  if (bluetooth.available()) {
    // Serial.println((char)bluetooth.read());
  }

  if (Serial.available()) {
    // bluetooth.print((char)Serial.read());
  }
}