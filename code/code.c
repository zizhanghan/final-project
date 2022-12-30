#include "Adafruit_LiquidCrystal.h"
#define heartratePin A1
#include "DFRobot_Heartrate.h"

DFRobot_Heartrate heartrate(DIGITAL_MODE); ///< ANALOG_MODE or DIGITAL_MODE
Adafruit_LiquidCrystal lcd(0);
uint16_t heartrateValue=0;
uint8_t count;
byte heart[8] = {
  0b00000,
  0b01010,
  0b11111,
  0b11111,
  0b11111,
  0b01110,
  0b00100,
  0b00000
};

byte block[8] = {
  0b11111,
  0b10101,
  0b00000,
  0b00000,
  0b00000,
  0b10001,
  0b11011,
  0b11111
};
byte normal[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b00000,
  0b00000,
  0b00000
};

byte small[8] = {
  0b00000,
  0b00000,
  0b00100,
  0b01010,
  0b11111,
  0b00000,
  0b00000,
  0b00000
};

byte medium[8] = {
  0b00000,
  0b00100,
  0b01010,
  0b10001,
  0b11111,
  0b00000,
  0b00000,
  0b00000
};

byte large[8] = {
  0b00100,
  0b01010,
  0b10001,
  0b10001,
  0b11111,
  0b00000,
  0b00000,
  0b00000
};
int sensorPin = A0;   // select the input pin for the potentiometer
int sensorValue;
void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.createChar(0, normal);
  lcd.createChar(1, small);
  lcd.createChar(2, medium);
  lcd.createChar(3, large);
  lcd.createChar(4, heart);
  lcd.createChar(5, block);
  lcd.setCursor(0, 0);
  lcd.write(byte(4)); // when calling lcd.write() '0' must be cast as a byte
  lcd.print(" = ");
  lcd.setCursor(0, 1);  
  lcd.print("Bre: ");
  lcd.setCursor(5, 1); 
  lcd.write(byte(0));
  // declare the ledPin as an OUTPUT:
  //pinMode(ledPin, OUTPUT);
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  uint8_t rateValue;
  uint8_t b;
  uint8_t s;
  uint8_t g;
  heartrate.getValue(heartratePin); ///< A1 foot sampled values
  rateValue = heartrate.getRate(); ///< Get heart rate value 

  heartrateValue = heartrate.getValue(heartratePin);  ///< A1 foot sampled values
  count = heartrate.getCnt();
  Serial.print("heartrateValue=");
  Serial.println(heartrateValue);
  if(rateValue)  {
    
    b=(rateValue/100)%10;
    s=(rateValue/10)%10;
    g=rateValue%10;
      if(heartrateValue<900){
        lcd.setCursor(11, 0);
        lcd.write(byte(4));
        delay(300);
      }
      else if(heartrateValue>900){
        lcd.setCursor(11, 0);
        lcd.write(byte(5));
        //delay(300);
      }
      
      if(b==0){
        lcd.setCursor(5, 0);
        lcd.print("0");
      }
      else {
        lcd.setCursor(5, 0);
        lcd.print("1");
      }

      
      if(s==0){
        lcd.setCursor(6, 0);
        lcd.print("0");
      }
      else if(s==1){
        lcd.setCursor(6, 0);
        lcd.print("1");
      }
      else if(s==2){
        lcd.setCursor(6, 0);
        lcd.print("2");
      }
      else if(s==3){
        lcd.setCursor(6, 0);
        lcd.print("3");
      }
      else if(s==4){
        lcd.setCursor(6, 0);
        lcd.print("4");
      }
      else if(s==5){
        lcd.setCursor(6, 0);
        lcd.print("5");
      }
      else if(s==6){
        lcd.setCursor(6, 0);
        lcd.print("6");
      }
      else if(s==7){
        lcd.setCursor(6, 0);
        lcd.print("7");
      }
      else if(s==8){
        lcd.setCursor(6, 0);
        lcd.print("8");
      }
      else {
        lcd.setCursor(6, 0);
        lcd.print("9");
      }


      if(g==0){
        lcd.setCursor(7, 0);
        lcd.print("0");
      }
      else if(g==1){
        lcd.setCursor(7, 0);
        lcd.print("1");
      }
      else if(g==2){
        lcd.setCursor(7, 0);
        lcd.print("2");
      }
      else if(g==3){
        lcd.setCursor(7, 0);
        lcd.print("3");
      }
      else if(g==4){
        lcd.setCursor(7, 0);
        lcd.print("4");
      }
      else if(g==5){
        lcd.setCursor(7, 0);
        lcd.print("5");
      }
      else if(g==6){
        lcd.setCursor(7, 0);
        lcd.print("6");
      }
      else if(g==7){
        lcd.setCursor(7, 0);
        lcd.print("7");
      }
      else if(g==8){
        lcd.setCursor(7, 0);
        lcd.print("8");
      }
      else {
        lcd.setCursor(7, 0);
        lcd.print("9");
      }
      }
  if(sensorValue<=330)
  {
    lcd.setCursor(5, 1); 
    lcd.write(byte(0));
    lcd.setCursor(7, 1);
    lcd.print("    ");
    delay(300);
    }
  else if((sensorValue>330)&&(sensorValue<=360)){
    lcd.setCursor(5, 1); 
    lcd.write(byte(1));
    lcd.setCursor(7, 1);
    lcd.print("    ");
    delay(300);
  }
  else if((sensorValue>360)&&(sensorValue<=400)){
    lcd.setCursor(5, 1); 
    lcd.write(byte(2));
    lcd.setCursor(7, 1);
    lcd.print("    ");
    delay(300);
  }
  else if(sensorValue>400){
    lcd.setCursor(5, 1); 
    lcd.write(byte(3));
    lcd.setCursor(7, 1);
    lcd.print("clam");
    delay(300);
  }
  delay(20);

  //Serial.println(sensorValue);
  //delay(200);
  // turn the ledPin on
  //digitalWrite(ledPin, HIGH);
  // stop the program for <sensorValue> milliseconds:
  //delay(sensorValue);
  // turn the ledPin off:
  //digitalWrite(ledPin, LOW);
  // stop the program for for <sensorValue> milliseconds:
  //delay(sensorValue);
}
