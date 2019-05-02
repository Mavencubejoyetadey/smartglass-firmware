
/*
     BLE MODE ,Check wifi Always (if found connectd else wait and deep sleep), Wakeup from deep sleep 1 min alys
   
    
    1. DEEP SLEEP AFTER SUCCESFULL DATA SENDING THROUGH BLE
    2. GPIO 33 PULL HIGH to WAKEUP AND COMMUNICXATE OVER BLE
    3. IF BLE CLIENT DNT CONNECT FOR 10 Sec AUTO CONNECT TO WIFI WHICH ID , PASS SEND BY BLE CLIENT  
    4. EEPROM WIFI & PASS SAVE   (CNETID:NETPASS)
    5. WHEN BLE IS CONNECTED WIFI CLIENT WILL OFF AND VISEVARSA
    6. Read ADC Battery Voltage & Tranismit it Through BLE And Webserver  
    7. BUZZER PLAY for 1 sec when it tigger
    8. UDP Wifi Strength BROADCAST
    9. UDP CONTROL GPIO
    PIN CONFIG: ADC(Battery Voltage)= PIN34 , Buzzer Pin = 5 , Touch Wateup pi =33 /IO2 , Proxy_Pin: 12
    
    KEYWORD : EEPROM , ESP32 , DEEP_SLEEP , TOUCH_WAKEUP , RTC_WAKEUP ,Bluetooth_LOW_ENERGY, ESP32_SERVER , ADC (Battery voltage);
    code update : 18th March 19
    
*/



#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h>
#include <WiFiUdp.h>
WiFiUDP udp;


#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define BUTTON_PIN_BITMASK 0x200000000            // DEEP SLEEP WAKE PIN GPIO 33
#define uS_TO_S_FACTOR 1000000                    /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 40                          //Time ESP32 will WAKEUP from sleep (in seconds)

//touch wakeup
#define Threshold 40                 /* Greater the value, more the sensitivity */
RTC_DATA_ATTR int bootCount = 0;    //touch_pad_t touchPin;

void callback(){
  //Serial.println("Now, system is sleeping! Please! Touch the Touchpad to wake up the System");
}

                                //BUZZER PIN 5
#define ADC 34                                    //ADC Battery Voltage PIN 34
#define Proxy 12                                  //Proxy Pin 12

uint64_t chipid=ESP.getEfuseMac();                 //UNIQE CHIP ID
char ble[200];                                    //BLE chip id varible
int bleid = (chipid % 8000000)/900;
  
unsigned long notConnectedSince = 0;
bool isWiFiCodeRunning = false;

#define BUZZER 26 
const int ledChannel = 0;
const int resolution = 8;

void ring(){
  EEPROM.begin(512);
  String ringtimesave = EEPROM.readString(500);
  EEPROM.end();
  Serial.print("Ringing count: ");  
  Serial.println(ringtimesave.toInt());
 
  for (int i=0 ; i<ringtimesave.toInt(); i++) {  
    
ledcSetup(ledChannel, 3500, resolution);
      ledcWrite(ledChannel, 100);
      delay(100);
      
      // no sound
      ledcSetup(ledChannel, 0, resolution);
      ledcWrite(ledChannel, 0);
      delay(150);
      
       ledcSetup(ledChannel, 2500, resolution);
       ledcWrite(ledChannel, 100);
      delay(100);
      
      //no sound
      ledcSetup(ledChannel, 0, resolution);
      ledcWrite(ledChannel, 0);
      delay(50);
      
      ledcSetup(ledChannel, 3000, resolution);
      ledcWrite(ledChannel, 100);
      delay(100);
      
      //no sound 
      ledcSetup(ledChannel, 0, resolution);
      ledcWrite(ledChannel, 0);
      delay(50);
      
      ledcSetup(ledChannel, 3500, resolution);
      ledcWrite(ledChannel, 100);
      delay(100);
      
      //no sound 
      ledcSetup(ledChannel, 0, resolution);
      ledcWrite(ledChannel, 0);
      delay(200);
      
      ledcSetup(ledChannel, 3500, resolution);
      ledcWrite(ledChannel, 100);
      delay(100);
      
      //no sound 
      ledcSetup(ledChannel, 0, resolution);
      ledcWrite(ledChannel, 0);
      delay(1000);
  }
 }


 void goSleep() {                                                 // DEEP SLEEP FUNCTION
  Serial.println("DS5:.....");
  delay(2000);
  esp_deep_sleep_start();  
}


void setupWiFi(void) {                          //WIFI SETUP 
 
  Serial.begin(115200);
  ledcAttachPin(BUZZER, ledChannel);
  WiFi.setHostname("Harrell Eyewear");
  connectNetDetails();                        //WiFi.begin("linksys", "securecube");

  udp.begin(44444);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("...");  
  }

  unsigned long wifiConnectStart = millis();
  
  while (WiFi.status() != WL_CONNECTED) {
       if (WiFi.status() == WL_CONNECT_FAILED) {
        //Serial.println("Failed to connect to WiFi. Please verify credentials: ");
        delay(4000);
  }
    delay(500);
    Serial.println("...");
    if (millis() - wifiConnectStart > 15000) {
   // goSleep();  // goto deep sleep 
  }
 }

}

bool flag=true;

void loopWiFi(void)  {            

  char incomingPacket[255];
  int packetSize = udp.parsePacket();
  if (packetSize){
    int len = udp.read(incomingPacket, 255);
    
    if(incomingPacket[0] == 'A') {
      Serial.println ("Turning ON BUZZER! {Wi-Fi}");
      ring();
    }  
  }

   if (WiFi.status() == WL_CONNECTED ) {                                                // BroadCast IP Address 
    IPAddress broadcastIp;
    broadcastIp = ~WiFi.subnetMask() | WiFi.gatewayIP();
    udp.beginPacket(broadcastIp,44444);
    sprintf(ble,"Harrell Eyewear %d",bleid); 
    int ADCValue = analogRead(ADC);                                                    // Read ADC Battery Voltage AnalogPin 34
    udp.print(WiFi.RSSI());                                                           //udp.write((uint8_t*)WiFi.localIP().toString().c_str(), WiFi.localIP().toString().length());
    udp.print("|");
    udp.print(ble);
    udp.print("|");
    udp.print(ADCValue);
    Serial.println(WiFi.localIP().toString()+ " | "+ WiFi.RSSI() + " || "+ ble +"||"+ADCValue);
    udp.endPacket();
    delay(1000);
  }

   if (WiFi.RSSI()< -75 && flag==true ){
        ring();
        flag= false;
    }
     
   else if(WiFi.RSSI()> -70 && flag==false){
       flag= true;
     }
 }




//BLE SECTION


BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
uint8_t txValue = 0;


class MyServerCallbacks: public BLEServerCallbacks {                           // CHECK THE BLE CLIENT
    
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      WiFi.mode(WIFI_OFF);
      isWiFiCodeRunning = false;
      notConnectedSince = 0;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


class MyCallbacks: public BLECharacteristicCallbacks {                         //BLE RECCEIVED FROM APPLICATION
    void onWrite (BLECharacteristic * pCharacteristic) {
      std :: string rxValue = pCharacteristic-> getValue ();
      Serial.println (rxValue [0]);

      if (rxValue.length ()> 0) {
        parseCmd(rxValue);                                                     //function for wifi id & password

        for (int i = 0; i <rxValue.length (); i ++) {
           Serial.print (rxValue [i]);
        }
        Serial.println ();
        Serial.println ("*********");
      }

      if (rxValue.find ("A") != -1) {                                       // Process the character received from the application. If A turns on the LED. B turns off the LED
         ring();
      }
    }
};





void parseCmd(std::string rxValue)  {                                    // function for ble app sent wifi id & password
  
  if(rxValue[0] == 'B') {                                                // Ring time Duration command B5 (5sec)
    Serial.print("Set Buzzer= ");
    char ringtime[10] ={};
    int r = 0;
    for (int i = 1; i < rxValue.length(); i++) {
      ringtime[r++] = rxValue[i];
    }
    int btime = atoi (ringtime);
    String btChar =String (btime);

    Serial.println(btChar);
   
    EEPROM.begin(512);
    EEPROM.writeString(500, btChar);
    String ringtimesave = EEPROM.readString(500);
    EEPROM.commit();
    EEPROM.end();
    Serial.print("Save Buzzer Count=  ");
    Serial.println(ringtimesave);
  }


  else if(rxValue[0] == 'C') {                                        // WiFi netid & password, format    "CSSID:PASSOUT"
  char ssid[20]={};
  char pass[20]={};
  
  bool isNetID = true;
  int j = 0;
  int k = 0;
  
  for (int i = 1; i < rxValue.length(); i++) {
    if(isNetID) {
      if(rxValue[i] != ':') {
        ssid[j++] = rxValue[i];
      } else {
          ssid[j++] = NULL;
          isNetID = false;
      }
    }else{
       pass[k++] = rxValue[i];
    }
  }
  Serial.println("Received SSID & PASS From App:");
  
  Serial.println(ssid);
  Serial.println(pass);
  saveNetDetails(ssid,pass);
  WiFi.begin(ssid,pass);
  setupWiFi();
  }
}



//EEPROM DATA SAVE
 
int WIFI_SSID_ADDRS = 0;
int WIFI_PASS_ADDRS = 100;

void saveNetDetails(String ssid, String pass) {
  EEPROM.begin(512);
  EEPROM.writeString(WIFI_SSID_ADDRS, ssid);
  EEPROM.writeString(WIFI_PASS_ADDRS, pass);
  EEPROM.commit();
  EEPROM.end();
}


void connectNetDetails() {    //EEPROM DATA SAVE AND VALUE PASS
  EEPROM.begin(512);
  String ssid = EEPROM.readString(WIFI_SSID_ADDRS);
  String pass = EEPROM.readString(WIFI_PASS_ADDRS);
  EEPROM.end();

  char cssid[22] = {0};
  char cpass[22] = {0};
  
  ssid.toCharArray(cssid, ssid.length()+1);
  pass.toCharArray(cpass, pass.length()+1);
  
  Serial.println("Stored ssid & Pass");
  Serial.println(cssid);
  Serial.println(cpass);

  WiFi.begin(cssid, cpass);
}






void setup() {                                                         //BLE CONFIG
  
  Serial.begin(115200);
  pinMode (Proxy, INPUT);                                             // Proxy PIn Decleartion
  //pinMode (BUZZER, OUTPUT);                                         // BUZZER Pin Decleartion
  ledcAttachPin(BUZZER, ledChannel);
  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);     // Deep Sleep timer wakeup
  //touch wakeup
  touchAttachInterrupt(T3,callback,Threshold);  
  esp_sleep_enable_touchpad_wakeup();                               // Configure Touchpad as wakeup source Setup interrupt on Touch Pad 3 (GPIO15)

  sprintf(ble,"Harrell Eyewear %d",bleid); 
  BLEDevice::init(ble);
 
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX,BLECharacteristic::PROPERTY_NOTIFY);                    
  pCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX,BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println(ble);
}


void loop() {                                                   // BLE DATA SENDING LOOP
 
 if(digitalRead(Proxy) == HIGH){
   Serial.println("Proxy Activated [BLE]");
   goSleep();
  }
    
  if (deviceConnected) {                                       //BLE Connected with phone
    notConnectedSince = 0;   
    int ADCValue = analogRead(ADC);                           // Read ADC Battery Voltage AnalogPin 34
    char buffer[5];
    itoa (ADCValue,buffer,10);
    Serial.printf("BLE Connected & Goto Deep sleep after 10 sec ADC:: %d \n  ", ADCValue);
    pCharacteristic->setValue(buffer);
    pCharacteristic->notify();
   
    delay(100);                           
    
    if(millis() > 20000) {         
       goSleep(); //sleep
    }
  }

 if(millis() > 20000) {
   if(notConnectedSince == 0) {                  //First time checking for bluetooth connection failed
      notConnectedSince = millis();             //Save the current time for future reference
    }
   else if(millis() - notConnectedSince > 1*20000 && isWiFiCodeRunning == false) { 
     setupWiFi();                                 
     isWiFiCodeRunning = true;
   }
 }
    
 if(isWiFiCodeRunning)loopWiFi();
 delay(100);        

}
