#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <TridentTD_LineNotify.h>

#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

Adafruit_8x16minimatrix matrix = Adafruit_8x16minimatrix();

uint8_t counter = 0;
const byte interruptPin1 = 14;
const byte interruptPin2 = 16;
const int buzzer = 13; //buzzer to arduino pin 13
const int slave_addr = 0x91;  // Slave Address of Modbus Device
const int dirpin = 18;       // Direction Control Pin
uint8_t buf[128];
int8_t indexMax = 0;
int8_t m_error = 0;
unsigned long prev_t;
int counter_start = 0;
int counter_stop = 0;
int time_capture = 0;
hw_timer_t *timer_0 = NULL;
bool stage_unit = false;
bool stage_temp = false;
bool stage_running = false;
bool stage_mode = false;
String response;


#include <iostream>
#include <iomanip> // for std::hex
#include <cstdint>

// #define LM73_ADDR 0x4D6
#define LM73_ADDR 0x4D
#define LINE_TOKEN "xxxxx" 
#define WIFI_NAME "xxxx" 
#define WIFI_PASSWORD "xxxx" 

#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
#define INFLUXDB_ORG "xxxxxxxxxxxxx"
#define INFLUXDB_BUCKET "Meter_Dashboard"

// Time zone info
#define TZ_INFO "UTC+7"

// Declare InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Declare Data point
Point sensor("meter_data");

double temp=0;

//----------------------------------------------"readTemperature()"--------------------------------------------------------------//

float readTemperature() {
  Wire1.beginTransmission(LM73_ADDR);
  Wire1.write(0x00);
  Wire1.endTransmission();
  uint8_t count = Wire1.requestFrom(LM73_ADDR, 2);
  float temp = 0.0;

  if (count == 2) {
    byte buff[2];
    buff[0] = Wire1.read();
    buff[1] = Wire1.read();
    temp += (int)(buff[0]<<1);
    if (buff[1]&0b10000000) temp += 1.0;
    if (buff[1]&0b01000000) temp += 0.5;
    if (buff[1]&0b00100000) temp += 0.25;
    if (buff[0]&0b10000000) temp *= -1.0;
  }

  return temp;
}

//--------------------------------------------------

int hexArrayToDecimal(const uint8_t* dataArray, size_t startIndex, size_t endIndex) {
    // Ensure endIndex does not exceed array bounds
    if (endIndex >= startIndex) {
        int decimalValue = 0;
        for (size_t i = startIndex; i <= endIndex; ++i) {
            decimalValue = (decimalValue << 8) | dataArray[i];
        }
        return decimalValue;
    } else {
        // Handle invalid indices
        std::cerr << "Invalid indices." << std::endl;
        return -1;
    }
}

float read_meter(uint8_t* packet, const int endbit){
  digitalWrite(dirpin, HIGH); //set Pin Direction control to "HIGH" of send packet
  delay(10);
  Serial1.write(packet, 8);//write packet
  //https://www.arduino.cc/reference/en/language/functions/communication/serial/write/
  delay(100);
  digitalWrite(dirpin, LOW); //set Pin Direction control to "LOW" of receive packet
  Data_recv(slave_addr);//receive packet
  float Value = hexArrayToDecimal(buf, 3, endbit);
  Serial.println(Value);
  return Value;
}


//----------------------------------------------"void setup()","void loop()"--------------------------------------------------------------//
void setup() {
  Serial.begin(115200);
  Wire1.begin(4, 5);
  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);
  Serial.printf("WiFi connecting ", WIFI_NAME);

  pinMode(18, OUTPUT);
  pinMode(buzzer, OUTPUT); // Set buzzer - pin 13 as an output
  pinMode(interruptPin1,INPUT_PULLUP);
  pinMode(interruptPin2,INPUT_PULLUP);
  // configure serial to connect with modbus device
  // and set <m_rtu> serial to let <m_rtu> use that serial
  // RXmax pin =23 -> RO pin :23,  TXmax pin =19 -> DI pin :19
  Serial1.begin(1200, SERIAL_8E1, 23, 19);
  matrix.begin(0x70);  // pass in the address
  matrix.clear();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500); 
  }

  Serial.printf("\nWiFi connected\nIP : ");
  Serial.println(WiFi.localIP());
  LINE.setToken(LINE_TOKEN);


  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");
  // Check server connection
  pinMode(17, OUTPUT);
  pinMode(15, OUTPUT);
  if (digitalRead(interruptPin1) == 1){
    stage_mode = true;
  }

  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
  attachInterrupt(digitalPinToInterrupt(interruptPin1),set_unit,FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2),set_temp,FALLING);
  // matrix.setBrightness(15);
  // matrix.setRotation(3);
  matrix.setRotation(1); 
  matrix.setTextSize(1); 
  matrix.setTextWrap(false);
  matrix.setTextColor(LED_ON); 
  // Add tags to the data point
  sensor.addTag("device", "ESP32");
  
}

void set_unit(){
  if (stage_running == false){
    Serial.print("Hello");
    stage_unit = !stage_unit;
    Serial.print(stage_unit);
  }
};

void set_temp(){
  if (stage_running == false){
    Serial.print("Hello");
    stage_temp = !stage_temp;
    Serial.print(stage_temp);
  }
};

void loop() {
  // delay(5000);
  if (stage_mode == true) {
    digitalWrite(15, LOW);  // turn the yellow LED on 
    digitalWrite(17, HIGH);
  }
  else{
    tone(buzzer, 1000); // Send 1KHz sound signal...
    delay(500);        // ...for 1 sec
    noTone(buzzer);     // Stop sound...
  }
  Serial.println("Standing By");
  stage_running = false;
  counter_start = millis();
  response = "";
  
  while(true){
    counter_stop = millis();
    time_capture = counter_stop - counter_start;
    if (time_capture >= 30000){
      break;
    }
    if (stage_unit == 0){
      matrix.clear();
      matrix.setCursor(2,0);
      if (stage_temp == 0){
        matrix.print("CU"); 
      }
      else if (stage_temp == 1){
        matrix.print("FU"); 
      }
    }
    else if (stage_unit == 1){
      matrix.clear();
      matrix.setCursor(2,0);
      if (stage_temp == 0){
        matrix.print("CW"); 
      }
      else if (stage_temp == 1){
        matrix.print("FW"); 
      }
    }
    matrix.writeDisplay();
  }
  if (stage_mode == true) {
    digitalWrite(17, LOW);
    digitalWrite(15, HIGH);  // turn the yellow LED on 
  }
  else{
    tone(buzzer, 1000); // Send 1KHz sound signal...
    delay(200);        // ...for 1 sec
    noTone(buzzer);     // Stop sound...
    delay(50);
    tone(buzzer, 1000); // Send 1KHz sound signal...
    delay(200);        // ...for 1 sec
    noTone(buzzer);     // Stop sound...
  }
  stage_running = true;
  float t = readTemperature();
  Serial.print("\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" *C ");
  // SendWriteRequestToThingSpeak(t);
  
  uint8_t packet1[8] = {  0x91,       0x03,           0x00,0x73,           0x00,0x01,          0x68,0x81};
  float watt = read_meter(packet1, 4);

  uint8_t packet2[8] = {  0x91,       0x03,           0x00,0x6e,           0x00,0x02,          0xb8,0x86};
  float energy = read_meter(packet2, 6);

  if ( stage_unit == 0) {
    matrix.clear();
    if (stage_temp == 0){
      response = String((int) t) + "C";
      if (m_error == 1) {
        LINE.notify("อากาศ"+String(t,3)+"*C"+ "ใช้พลังงานไป " + "0(ไม่มีการเชื่อมต่อ)" + "Wh ");
      }
      else{
        LINE.notify("อากาศ"+String(t,3)+"*C"+ "ใช้พลังงานไป " + energy + "Wh ");
      }
    }
    else if (stage_temp == 1){
      float f = (t * 9/5) + 32 ;
      response = String((int) f) + "F";
      if (m_error == 1) {
        LINE.notify("อากาศ"+String(t,3)+"*C"+ "ใช้พลังงานไป " + "0(ไม่มีการเชื่อมต่อ)" + "Wh ");
      }
      else{
        LINE.notify("อากาศ"+String(f,3)+"*F"+ "ใช้พลังงานไป " + energy + "Wh ");
      }
    }
    for (int8_t x=8; x>=-24; x--) {
      matrix.clear();
      matrix.setCursor(x,0);
      matrix.print(response);
      matrix.writeDisplay();
      delay(100);
    }
    if (m_error == 0) {
      response = String((float) energy/1000) + "KWh";
    }
    else{
      response = "Error";
    } 
    for (int8_t x=8; x>=-48; x--) {
      matrix.clear();
      matrix.setCursor(x,0);
      matrix.print(response);
      matrix.writeDisplay();
      delay(100);
    }
  }
  else if ( stage_unit == 1) {
    matrix.clear();
    if (stage_temp == 0){
      response = String((int) t) + "C";
      if (m_error == 1) {
        LINE.notify("อากาศ"+String(t,3)+"*C"+ "ใช้พลังงานไป " + "0(ไม่มีการเชื่อมต่อ)" + "Wh ");
      }
      else{
        LINE.notify("อากาศ"+String(t,3)+"*C"+ "กำลังไฟ " + watt + "W");
      }
    }
    else if (stage_temp == 1){
      float f = (t * 9/5) + 32 ;
      response = String((int) f) + "F";
      if (m_error == 1) {
        LINE.notify("อากาศ"+String(t,3)+"*C"+ "ใช้พลังงานไป " + "0(ไม่มีการเชื่อมต่อ)" + "Wh ");
      }
      else{
        LINE.notify("อากาศ"+String(f,3)+"*F"+ "กำลังไฟ " + watt + "W");
      }
    }
    for (int8_t x=8; x>=-24; x--) {
      matrix.clear();
      matrix.setCursor(x,0);
      matrix.print(response);
      matrix.writeDisplay();
      delay(100);
    }
    if (m_error == 0) {
      if(stage_unit == 0){
        response = String((float) energy/1000) + "kWh";
      }
      else{
        response = String((float) watt) + "W";
      }
    }
    else{
      response = "Error";
    } 
    for (int8_t x=8; x>=-48; x--) {
      matrix.clear();
      matrix.setCursor(x,0);
      matrix.print(response);
      matrix.writeDisplay();
      delay(100);
    }
  }
  
  if (m_error == 0) {
    // Clear fields for reusing the point. Tags will remain the same as set above.
    sensor.clearFields();

    // Store measured value into point
    // Report RSSI of currently connected network
    sensor.addField("temperature", t);
    sensor.addField("power", watt);
    sensor.addField("energy", energy);
  }
  // Print what are we exactly writing
  Serial.println("Writing ");

  // Check WiFi connection and reconnect if needed
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wifi connection lost");
  }

  // Write point
  if (!client.writePoint(sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  } 
}

void Data_recv(uint8_t SS) {
  unsigned long prev_t = millis();  //resopnse check, time is over 20 sec set error code = 1 (Timeout Error code)
  while (Serial1.available() == 0) {
    if (millis() - prev_t > 20000) {  ///20000
      m_error = 1;                    // TIMEOUT
      return;
    }
  }

  indexMax = 0;
  m_error = 0;
  prev_t = millis();                 // This section for keep data. Time between byte data are not over 500 ms. The data store in array buf[].
  while (Serial1.available() > 0) {  // If time between byte data  over 500 ms not accept.
    buf[indexMax++] = Serial1.read();
    // indexMax = indexMax + 1;
    while (Serial1.available() == 0) {
      if (millis() - prev_t > 500) {
        break;  // no more data
      }
    }
  }
}

