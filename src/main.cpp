#include <Arduino.h>
/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Servo.h>

#include <Ticker.h> 
#include "elapsedMillis.h"
//#include "expo.h"


// von ESP32 ROBOTAUTO
#ifndef LED_BUILTIN
#define LED_BUILTIN 16
#endif

//Mini d1: MAC: 48:3f:da:a4:36:57

// Master ESP Board MAC Address:  94:B9:7E:D9:F0:50


#define batt_PIN A0
uint16_t battspannung =0;
#define ADC_FAKTOR 0.952

#define NUM_SERVOS 4

#define MAX_TICKS 3400
#define MIN_TICKS 1700
uint16_t   servomittearray[NUM_SERVOS] = {}; // Werte fuer Mitte
volatile uint16_t   servoimpulsarray[NUM_SERVOS] = {1500,1500,1500,1500}; // 
volatile uint8_t servopinarray[NUM_SERVOS] = {15,13,12,14 };

uint16_t servowertbereich = 0;

uint16_t maxwinkel = 180;

uint8_t buttonstatus = 0;
uint8_t tonindex = 0;
void playTon(int ton);
#define START_TON 3
#define LICHT_ON 2

#define TON_PIN 5
elapsedMillis tonposition;




uint8_t broadcastAddress[] = {0x48, 0x3F, 0xDA, 0xA4, 0x36, 0x57};
uint8_t masterbroadcastaddress[] = {0x94,0xB9,0x7E, 0xD9,0xF0,0x50};

typedef enum {
    ESP_NOW_SEND_SUCCESS = 0,       /**< Send ESPNOW data successfully */
    ESP_NOW_SEND_FAIL,              /**< Send ESPNOW data fail */
} esp_now_send_status_t;


long unsigned int ledintervall = 1000;
Ticker timer;
elapsedMillis ledmillis;
elapsedMillis tonmillis;
elapsedMicros tonmicros;
int tonfolge[3] = {554, 329, 440};

struct ServoPins
{
  Servo servo;
  int servoPin;
  String servoName;
  int initialPosition;  
};

volatile uint8_t servoindex = 0;
volatile uint16_t servoimpulsdauer = 0;
volatile uint16_t servozeitsumme = 0; // cumulate servoimpulses

#define PAKETINTERVALL 20000
#define SERVO_PAUSE_BIT 7
#define TIMERFAKTOR 5 // Umrechnung Impulszeit
volatile uint16_t paketintervall = PAKETINTERVALL; // 20 ms
volatile int interrupts;
std::vector<ServoPins> servoPins = 
{
  { Servo(), 12 , "Dir", 90}, // Richtung
  { Servo(), 14 , "Pitch", 90}, // Gas
  { Servo(), 13 , "Elbow", 90},
  { Servo(), 15 , "Gripper", 90},
};

struct RecordedStep
{
  int servoIndex;
  int value;
  int delayInStep;
};
std::vector<RecordedStep> recordedSteps;

bool recordSteps = false;
bool playRecordedSteps = false;

unsigned long previousTimeInMilli = millis();

void playTon(uint8_t ton)
{
  // Cis: 554
  // e: 330
  // a: 440
 tone(TON_PIN,tonfolge[ton],800);
 // tone(TON_PIN,440,800);
}


void writeServoValues(int servoIndex, int value)
{
  if (recordSteps)
  {
    RecordedStep recordedStep;       
    if (recordedSteps.size() == 0) // We will first record initial position of all servos. 
    {
      for (uint8_t i = 0; i < servoPins.size(); i++)
      {
        recordedStep.servoIndex = i; 
        recordedStep.value = servoPins[i].servo.read(); 
        recordedStep.delayInStep = 0;
        recordedSteps.push_back(recordedStep);         
      }      
    }
    unsigned long currentTime = millis();
    recordedStep.servoIndex = servoIndex; 
    recordedStep.value = value; 
    recordedStep.delayInStep = currentTime - previousTimeInMilli;
    recordedSteps.push_back(recordedStep);  
    previousTimeInMilli = currentTime;         
  }
  //Serial.printf("pin: [%d] servoindex: [%d] value: [%d]  \n",servoPins[servoIndex].servoPin , servoIndex, value);
  servoPins[servoIndex].servo.write(value); 

}


//Structure example to receive data
//Must match the sender structure
typedef struct canal_struct 
{
  uint16_t lx;
  uint16_t ly;
  uint16_t rx;
  uint16_t ry;

 uint8_t digi;

  uint16_t x;
  uint16_t y;
} canal_struct;

//Create a struct_message called canaldata
canal_struct canaldata;

canal_struct outdata;
// Variable to store if sending data was successful
String success;


#define ESP_NOW_SEND_SUCCESS 0

// Callback when data is sent
/*
// von RobotAuto_T
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}
*/
// von https://randomnerdtutorials.com/esp-now-esp8266-nodemcu-arduino-ide/

/*
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}
*/


// MAC: 44:17:93:14:f7:17

//callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) 
{
  // blink
  memcpy(&canaldata, incomingData, sizeof(canaldata));
 
  //Serial.print("Bytes received: ");
  //Serial.printf("%d %d %d %d %d %d \n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);


  //Serial.printf("lx: %d lx. %d\t", canaldata.lx, canaldata.ly);
  //Serial.printf("rx: %d rx. %d\t", canaldata.rx, canaldata.ry);
  //Serial.printf("x: %d x. %d\n", canaldata.x, canaldata.y);
  //Serial.print(canaldata.lx);
  //Serial.print(" ");
  //Serial.print("ly: ");
  //Serial.print(canaldata.ly);
  //Serial.print(" int ");
  //Serial.print(interrupts);
  //Serial.print(" ");
  //Serial.print("digi: ");
  //Serial.println(canaldata.digi);

 // lx
 
  // 
  uint8_t lx = canaldata.x;
  servoimpulsarray[0] = canaldata.lx ;
  writeServoValues(0,  lx);
  uint8_t ly = canaldata.y;
  servoimpulsarray[1] = canaldata.ly ;
  writeServoValues(1,  ly);
  //Serial.printf("lx: %d ly: %d \n",lx,ly);
  servoimpulsarray[2] = canaldata.rx ;
  
  servoimpulsarray[3] = canaldata.ry ;
  servoindex = 0;
  paketintervall = PAKETINTERVALL;
  servoimpulsdauer = servoimpulsarray[servoindex];
  //Serial.printf("*servoindex: %d pin: %d servoimpulsdauer: %d\n",servoindex, servopinarray[servoindex] ,servoimpulsdauer);

  paketintervall -= servoimpulsdauer; // 
  digitalWrite(servopinarray[servoindex], HIGH);
  timer1_write(servoimpulsdauer * TIMERFAKTOR);


if (canaldata.digi & (1<<START_TON))
{
  if (!(buttonstatus & (1<<START_TON)))
  {
    //Serial.println("digi start");
    //Serial.println(canaldata.digi);
    buttonstatus |= (1<<START_TON);
    tonindex = 0;

  }
  
}
}


void setUpPinModes()
{

  for (uint8_t i = 0; i < servoPins.size(); i++)
  {
    servoPins[i].servo.attach(servoPins[i].servoPin);
    servoPins[i].servo.write(servoPins[i].initialPosition);  
    pinMode(servoPins[servoindex].servoPin,OUTPUT) ;
    digitalWrite(servoPins[servoindex].servoPin,LOW);
  }
 
}

// https://www.esp8266.com/viewtopic.php?f=6&t=19867
// https://www.visualmicro.com/page/Timer-Interrupts-Explained.aspx
void IRAM_ATTR servoISR()
{
   interrupts++;
  digitalWrite(servopinarray[servoindex], LOW); // vorherigen Impuls beenden

  if(servoindex < (NUM_SERVOS -1))
  {
    //  Serial.printf("+servoindex: %d pin: %d servoimpulsdauer: %d\n",servoindex, servopinarray[servoindex] ,servoimpulsdauer);

    //digitalWrite(servoPins[servoindex].servoPin,LOW); // stop current impuls
    servoindex++;
    servoimpulsdauer = servoimpulsarray[servoindex];
    digitalWrite(servopinarray[servoindex], HIGH);
    timer1_write(servoimpulsdauer * TIMERFAKTOR); //mitte 1500, min 1050, max 1950

  }
 
}




void setup() 
{
   //Initialize Serial Monitor
  Serial.begin(115200);
  Serial.println();
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

  pinMode(TON_PIN,OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);

  pinMode(A0,INPUT);
  // https://community.platformio.org/t/esp8266-gibberish-serial-monitor-output/30027

 for (uint8_t i = 0;i<NUM_SERVOS; i++)
 {
    pinMode(servopinarray[i],OUTPUT);
    digitalWrite(servopinarray[i],LOW);
 }
 //timer1_isr_init();
 timer1_attachInterrupt(servoISR);
 
 //timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);

   
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  
  
  WiFi.disconnect();
  
  //Init ESP-NOW
  if (esp_now_init() != 0) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  setUpPinModes();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);

  esp_now_register_recv_cb(OnDataRecv);
  
  
  //esp_now_register_send_cb(OnDataSent);
}

void loop() 
{
  if(tonmicros > 4)
  {
    //digitalWrite(4,!digitalRead(4));
    tonmicros = 0;
  }
 
  if (ledmillis >= ledintervall)
  {
    //digitalWrite(4,!(digitalRead(4))); 
    ledmillis = 0;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    //Serial.print("ESP Board MAC Address:  ");
    //Serial.println(WiFi.macAddress());
 
    battspannung = analogRead(batt_PIN);
    float battspannungfloat = battspannung* ADC_FAKTOR;
    outdata.x = uint16_t(battspannungfloat);
    Serial.printf("battspannung: %2.2f outdata.x %d\n",battspannungfloat, outdata.x);
     //
     outdata.y = canaldata.lx;
    //Serial.printf("masterbroadcastaddress[0]: %d x: %d y: %d\n",masterbroadcastaddress[0],outdata.x,outdata.y);

  /*
    uint8_t result = esp_now_send(masterbroadcastaddress, (uint8_t *) &outdata, sizeof(canal_struct));
    Serial.printf("esp_now_send result: %d\n",result);
   */ 
    
//

    //Serial.println("led");
  }

  if (buttonstatus & (1<<START_TON))
  {
    
    if (tonindex == 0)
    {
      tonmillis = 0;
      Serial.print("************ TON start index: ");
      Serial.println(tonindex);
      playTon(tonindex);
      tonindex++;
    }
    
    
    if (tonindex < 3)
    {
        if (tonmillis > 850)
        {
          Serial.print("************ TON next index: ");
          Serial.println(tonindex);
          playTon(tonindex);
          tonindex++;
          
          tonmillis = 0;
        }
        
    }
    else
    {
      Serial.println("************ TON END");
      buttonstatus &= ~(1<<START_TON);
      tonindex = 0;
      //timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
    }
    
  }

}