#include <DHT11.h>
#include <Servo.h>
DHT11 dht11(2);
int LED1 = 7;
int LED2 = 9;
int lightPin = A0;
int motor_control = 8;
Servo servo;

void setup() {
  Serial.begin(9600);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
}

void loop() {

  int temperature = 0;
  int humidity = 0;

  if (dht11.readTemperatureHumidity(temperature, humidity) == 0) {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C\tHumidity: ");
    Serial.print(humidity);
    Serial.print(" %\t");
  }


  if (temperature >= 26) {
    //팬 돌아가게
    servo.attach(motor_control);
    servo.write(0);
    delay(500);
    servo.write(180);
  } else {
    servo.detach();
  }
  int brightness = analogRead(lightPin) / 4;
  Serial.print("light : ");
  Serial.println(brightness);
  if (brightness <= 200) {  //밝기가 200이하면 analog 출력, 200이상이면 LED끔
    analogWrite(LED2, 255 - brightness);
  } else {
    digitalWrite(LED2, LOW);
  }
}
