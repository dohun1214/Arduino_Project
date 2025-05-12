#include <SoftwareSerial.h>
#include <DHT11.h>
#include <Servo.h>

int bluetoothTx = 2;
int bluetoothRx = 3;

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);
DHT11 dht11(4);
int LED1 = 7;
int LED2 = 9;
int lightPin = A0;
int motor_control = 8;
Servo servo;

void setup() {
  Serial.begin(9600);
  delay(100);
  bluetooth.begin(9600);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
}

void loop() {

  // //온도 습도
  // int temperature = 0;
  // int humidity = 0;

  // if (dht11.readTemperatureHumidity(temperature, humidity) == 0) {
  //   Serial.print("Temperature: ");
  //   Serial.print(temperature);
  //   Serial.print(" °C\tHumidity: ");
  //   Serial.print(humidity);
  //   Serial.print(" %\t");
  // }

  // // 일정 온도 넘으면 팬 돌아가게
  // if (temperature >= 26) {
  //   //팬 돌아가게
  //   servo.attach(motor_control);
  //   servo.write(0);
  //   delay(500);
  //   servo.write(180);
  // } else {
  //   servo.detach();
  // }
  // //온도 습도 끝


  // //주변 밝기에 따라 LED밝기
  // int brightness = analogRead(lightPin) / 4;
  // Serial.print("light : ");
  // Serial.println(brightness);
  // if (brightness <= 200) {  //밝기가 200이하면 analog 출력, 200이상이면 LED끔
  //   analogWrite(LED2, 255 - brightness);
  // } else {
  //   digitalWrite(LED2, LOW);
  // }
  // // LED밝기 끝


  //블루투스
  char cmd;
  if (bluetooth.available()) {
    cmd=(char)bluetooth.read();
    Serial.print("Command=");
    Serial.println(cmd);

    if(cmd=='1'){
      Serial.println("LED ON");
    }
    if(cmd=='2'){
      Serial.println("LED OFF");
    }
  }



  
}
