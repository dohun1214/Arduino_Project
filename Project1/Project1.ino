#include <SoftwareSerial.h>
#include <DHT11.h>
#include <Servo.h>

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

  if (dht11.readTemperatureHumidity(temperature, humidity) == 0) {

    //주변 밝기에 따라 LED밝기
    int brightness = analogRead(lightPin) / 4;
    if (brightness <= 200) {  //밝기가 200이하면 analog 출력, 200이상이면 LED끔
      analogWrite(LED1, 255 - brightness);
    } else {
      digitalWrite(LED1, LOW);
    }
    // LED밝기 끝

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C\tHumidity: ");
    Serial.print(humidity);
    Serial.print(" %\t");
    Serial.print("Brightness : ");
    Serial.print(brightness);
    Serial.print("\t");
    Serial.print(cm);
    Serial.println(" cm");

    bluetooth.print("Temperature: ");
    bluetooth.print(temperature);
    bluetooth.print(" °C\tHumidity: ");
    bluetooth.print(humidity);
    bluetooth.print(" %\t");
    bluetooth.print("light : ");
    bluetooth.print(brightness);
    bluetooth.println();
    bluetooth.print(cm);
    bluetooth.println(" cm");
  }

  // 일정 온도 넘으면 팬 돌아가게
  if (temperature >= 26) {
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
  }


  else
    digitalWrite(flamePin, LOW);

  // bluetooth.available() 블루투스로 보낸 데이터가 도착했을때
  // bluetooth.print() 블루투스로 데이터를 보냄
  // bluetooth.read() 블루투스로 보낸 데이터
  // Serial.available() 시리얼 모니터에 값을 입력했을때
  // Serial.print() 시리얼 모니터에 출력
  // Serial.read() 시리얼 모니터로 입력한 데이터

  //블루투스
  if (bluetooth.available()) {
    Serial.println((char)bluetooth.read());
  }

  if (Serial.available()) {
    bluetooth.print((char)Serial.read());
  }
}