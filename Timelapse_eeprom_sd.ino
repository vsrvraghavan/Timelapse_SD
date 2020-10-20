
#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory

// define the number of bytes you want to access
#define EEPROM_SIZE 1


#define uS_TO_S_FACTOR 1000000  //Conversion factor for micro seconds to seconds
#define TAKE_PHOTO_INTERVAL  30        //Time ESP32 will go to sleep (in seconds)
#define TIME_TO_SLEEP   TAKE_PHOTO_INTERVAL/2       //Time ESP32 will go to sleep (in seconds)


RTC_DATA_ATTR int pictureNumber = 0;


int freq = 5000;
int ledCHannel = 1;
int res = 8;
int brightness;

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);
  //Serial.setDebugOutput(true);
  //Serial.println();
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 3;
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  rtc_gpio_hold_dis(GPIO_NUM_4);
  pinMode(GPIO_NUM_4, INPUT);
  digitalWrite(GPIO_NUM_4, HIGH);
  Serial.println("LED HIGH");
  
  ledcSetup(ledCHannel, freq, res);
  ledcAttachPin(4, ledCHannel);
  ledcWrite(ledCHannel, 255);
  //rtc_gpio_hold_dis(GPIO_NUM_4);
      
  print_wakeup_reason();
  //Set timer to 5 seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + "");

  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  setCameraSensorSettings();
  
  //Serial.println("Starting SD Card");
  if(!SD_MMC.begin()){
    Serial.println("SD Card Mount Failed");
    return;
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD Card attached");
    return;
  }
    
  camera_fb_t * fb = NULL;
  
  // Take Picture with Camera
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  
  ledcWrite(ledCHannel, 0);
  rtc_gpio_hold_dis(GPIO_NUM_4);
  
  // initialize EEPROM with predefined size
  //EEPROM.begin(EEPROM_SIZE);
  //pictureNumber = EEPROM.read(0) + 1;
  pictureNumber++;
  // Path where new picture will be saved in SD Card
  String path = "/picture" + String(pictureNumber) +".jpg";

  fs::FS &fs = SD_MMC; 
  Serial.printf("Picture file name: %s\n", path.c_str());
  
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file in writing mode");
  } 
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
    //EEPROM.write(0, pictureNumber);
    //EEPROM.commit();
  }
  file.close();
  esp_camera_fb_return(fb); 
  
  // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  pinMode(GPIO_NUM_4, OUTPUT);
  digitalWrite(GPIO_NUM_4, LOW);  
  rtc_gpio_hold_en(GPIO_NUM_4);
  
  delay(2000);
  Serial.println("Going to sleep now");
  delay(2000);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void loop() {
  
}

//Function that prints the reason by which ESP32 has been awaken from sleep
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

sensor_t* setCameraSensorSettings(){

    sensor_t * s = esp_camera_sensor_get();     
    s->set_brightness(s, 2);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    s->set_ae_level(s, 1);       // -2 to 2
    s->set_aec_value(s, 0);    // 0 to 1200
    s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    s->set_agc_gain(s, 20);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    s->set_bpc(s, 1);            // 0 = disable , 1 = enable
    s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
    s->set_vflip(s, 0);          // 0 = disable , 1 = enable
    s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    
  
}

/*********
  VSRV Raghavan 
  This program takes photo every given seconds, this overcomes limitation of ESP32-CAM to FLASHING LED light when taking photo and storing it in SD card
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
/*
#include "esp_camera.h"
#include "Arduino.h"

#include "SPIFFS.h"

#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory


#define ID_ADDRESS            0x00
#define COUNT_ADDRESS         0x01
#define ID_BYTE               0xAA
#define EEPROM_SIZE           0x0F
// define the number of bytes you want to access
#define EEPROM_SIZE 1

uint16_t nextImageNumber = 0;


// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


#define uS_TO_S_FACTOR 1000000  //Conversion factor for micro seconds to seconds
#define TAKE_PHOTO_INTERVAL  30        //Time ESP32 will go to sleep (in seconds)
#define TIME_TO_SLEEP   TAKE_PHOTO_INTERVAL/2       //Time ESP32 will go to sleep (in seconds)


RTC_DATA_ATTR int pictureNumber = 0;
RTC_DATA_ATTR bool camMode = true;

RTC_DATA_ATTR bool formatted = false;

String tempImagePath = "/spiffs/temp_image.jpg";

camera_config_t setCameraConfig(){

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 3;
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  return config;
}

sensor_t* setCameraSensorSettings(){

    sensor_t * s = esp_camera_sensor_get();     
    s->set_brightness(s, 2);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 4);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    s->set_ae_level(s, 1);       // -2 to 2
    s->set_aec_value(s, 0);    // 0 to 1200
    s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    s->set_agc_gain(s, 20);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    s->set_bpc(s, 1);            // 0 = disable , 1 = enable
    s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
    s->set_vflip(s, 0);          // 0 = disable , 1 = enable
    s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    
  
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
    
  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  //Set timer to 5 seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");

  if(camMode){         
    capturePhoto();
    camMode = false;    
  }else{
    //Serial.println("Starting SD Card");
    if(!SD_MMC.begin()){
      Serial.println("SD Card Mount Failed");
      return;
    }    
    uint8_t cardType = SD_MMC.cardType();
    if(cardType == CARD_NONE){
      Serial.println("No SD Card attached");
      return;
    }    
    savePhotoToSD();
    camMode = true;
  }  
  // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  pinMode(GPIO_NUM_4, OUTPUT);
  Serial.println("Setting LED Low");  
  digitalWrite(GPIO_NUM_4, LOW);
  rtc_gpio_hold_en(GPIO_NUM_4);
  
  Serial.println("Going to sleep now");  
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void loop() {
  
}


void capturePhoto(){
    rtc_gpio_hold_dis(GPIO_NUM_4);
    pinMode(GPIO_NUM_4, OUTPUT);
    Serial.println("HIGH");
    digitalWrite(GPIO_NUM_4, HIGH);
    //rtc_gpio_hold_en(4);


    bool stat = SPIFFS.begin();
    if(!stat){
      Serial.println("SPIFFS Mount Failed");
      return;
    }      
    // Init Camera
    camera_config_t config = setCameraConfig();
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", err);
      return;
    }   
    
    sensor_t * s = setCameraSensorSettings();
             
    //take new image
    camera_fb_t * fb = NULL;
    //obtain camera frame buffer
    fb = esp_camera_fb_get();
    if (!fb) 
    {
      Serial.println("Camera capture failed");
      Serial.println("Exiting now"); 
      while(1);   //wait here as something is not right
    }
  
    //String path = "/spiffs/temp_image.jpg";
    fs::FS &fs = SPIFFS;
      //create new file
    File file = fs.open(tempImagePath.c_str(), FILE_WRITE);
    if(!file)
    {
      Serial.println("Failed to create file");
      Serial.println("Exiting now"); 
      while(1);   //wait here as something is not right    
    } 
    else 
    {
      file.write(fb->buf, fb->len); 
    }
    file.close();
   
    //return camera frame buffer
    esp_camera_fb_return(fb);
    Serial.printf("Temp Image saved: %s\n", tempImagePath.c_str());
}


void savePhotoToSD(){

    pictureNumber++;
    
    if(!SPIFFS.begin()){
      Serial.println("SPIFFS Mount Failed");
      return;
    }  
    if(!SD_MMC.begin()){
      Serial.println("SD Card Mount Failed");
      return;
    }
    
    uint8_t cardType = SD_MMC.cardType();
    if(cardType == CARD_NONE){
      Serial.println("No SD Card attached");
      return;
    }
    
    String sdImagePath = "/picture" + String(pictureNumber) +".jpg";
    fs::FS &sdfs = SD_MMC;
    File destFile = sdfs.open(sdImagePath.c_str(), FILE_WRITE);

    fs::FS &spiffs = SPIFFS;
      //create new file
    File sourceFile = spiffs.open(tempImagePath.c_str());
    if(!sourceFile)
    {
      Serial.println("Failed to read file");
      Serial.println("Exiting now"); 
      while(1);   //wait here as something is not right    
      return;
    } 
    


    
    //File sourceFile = SPIFFS.open(tempImagePath);
    static uint8_t buf[512];
    while( sourceFile.read( buf, 512) ) {
        Serial.println("Writing Data");
        destFile.write( buf, 512 );
    }
    destFile.close();
    sourceFile.close();
    Serial.println(sdImagePath);    
}


//Function that prints the reason by which ESP32 has been awaken from sleep
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

*/
/*

#include "esp_camera.h"
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"

#include "SPIFFS.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32

#define uS_TO_S_FACTOR 1000000  //Conversion factor for micro seconds to seconds
#define TAKE_PHOTO_INTERVAL  50        //Time ESP32 will go to sleep (in seconds)
#define TIME_TO_SLEEP   TAKE_PHOTO_INTERVAL/2       //Time ESP32 will go to sleep (in seconds)

RTC_DATA_ATTR int pictureNumber = 0;
RTC_DATA_ATTR bool camMode = true;
String tempImagePath = "/spiffs/temp_image.jpg";

#include <EEPROM.h> // read and write from flash memory
// define the number of bytes you want to access
#define ID_ADDRESS            0x00
#define COUNT_ADDRESS         0x01
#define ID_BYTE               0xAA
#define EEPROM_SIZE           0x0F

uint16_t nextImageNumber = 0;



// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22





#include "WiFiConnect.h" 
#include <WiFiClient.h>                      
#ifdef ARDUINO_ARCH_ESP8266  
#include <ESP8266HTTPClient.h>
#else  
#include <HTTPClient.h>
#endif
#include "ESP32_FTPClient.h"

#include <PubSubClient.h> // Allows us to connect to, and publish to the MQTT broker

WiFiConnect wc;

// For internet connection
//WiFiClient client;
HTTPClient http;
//ftp.drivehq.com
ESP32_FTPClient ftp ("ftp.drivehq.com", "raghuver", "raga@098");

// MQTT
// Make sure to update this for your own MQTT Broker!
const char* mqtt_server = "192.168.0.11";
const char* mqtt_topic = "Esp-Cam-Status";
const char* mqtt_username = "camera";
const char* mqtt_password = "camera";
// The client id identifies the ESP8266 device. Think of it a bit like a hostname (Or just a name, like Greg).
const char* clientID = "ESP-CAM";


// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
PubSubClient pubsubclient(mqtt_server, 1883, wifiClient); // 1883 is the listener port for the Broker

bool wifiConnected = false;
 
void configModeCallback(WiFiConnect *mWiFiConnect) {
  Serial.println("Entering Access Point");
}


bool startWiFi(boolean showParams = false) {

  bool wifiConnected = false;
  wc.setDebug(true);
  
  // Set our callbacks 
  wc.setAPCallback(configModeCallback);
*/
  //wc.resetSettings(); //helper to remove the stored wifi connection, comment out after first upload and re upload

    /*
       AP_NONE = Continue executing code
       AP_LOOP = Trap in a continuous loop - Device is useless
       AP_RESET = Restart the chip
       AP_WAIT  = Trap in a continuous loop with captive portal until we have a working WiFi connection
    */
/*
    //wc.startConfigurationPortal(AP_WAIT);

    //delay(120000);
    
    if (!wc.autoConnect()) { // try to connect to wifi
      // We could also use button etc. to trigger the portal on demand within main loop 
      wc.startConfigurationPortal(AP_WAIT);//if not connected show the configuration portal
    }else{
      wifiConnected = true;
      Serial.println("connected to Wifi");
      logMessage("ESP-CAM/status", "connected to Wifi");
      logMessage("ESP-CAM/ipaddress", WiFi.localIP().toString());
            
    }
    return wifiConnected;
}


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);
  logMessage("ESP-CAM/log", "....");
  logMessage("ESP-CAM/log", "....");  

  
  


  //wifiConnected = startWiFi(); 
  
  
  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  //Set timer to 5 seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  logMessage("ESP-CAM/log", "Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");

  

  if(camMode){
    camera_config_t config = setCameraConfig();
    //////wifiConnected = startWiFi(); 
    //bool formatted = SPIFFS.format();    
    bool stat = SPIFFS.begin(true);
    if(!stat){
      logMessage("ESP-CAM/log", "SPIFFS Mount Failed");
      return;
    }      
    // Init Camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      logMessage("ESP-CAM/log", "Camera init failed with error :"+ err);
      return;
    }    
    capturePhoto();
    camMode = false;    
  }else{
    //Serial.println("Starting SD Card");
    if(!SD_MMC.begin()){
      logMessage("ESP-CAM/log", "SD Card Mount Failed");
      return;
    }    
    uint8_t cardType = SD_MMC.cardType();
    if(cardType == CARD_NONE){
      Serial.println("No SD Card attached");
      return;
    }    
    savePhotoToSD();
    camMode = true;
  }  
  // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  pinMode(GPIO_NUM_4, OUTPUT);
  logMessage("ESP-CAM/log", "Setting LED Low");  
  digitalWrite(GPIO_NUM_4, LOW);
  rtc_gpio_hold_en(GPIO_NUM_4);
  
  logMessage("ESP-CAM/log", "Going to sleep now");  
  deep_sleep();
  logMessage("ESP-CAM/log", "This will never be printed");
  
}

void loop() {
  
}





void capturePhoto(){
    pictureNumber++; 


    if (!EEPROM.begin(EEPROM_SIZE))
    {
      logMessage("ESP-CAM/log", "Failed to initialise EEPROM"); 
      logMessage("ESP-CAM/log", "Exiting now"); 
      while(1);   //wait here as something is not right
    }

    if(EEPROM.read(ID_ADDRESS) != ID_BYTE)    //there will not be a valid picture number
    {
      logMessage("ESP-CAM/log", "Initializing ID byte & restarting picture count");
      nextImageNumber = 0;
      EEPROM.write(ID_ADDRESS, ID_BYTE);  
      EEPROM.commit(); 
    }
    else                                      //obtain next picture number
    {
      EEPROM.get(COUNT_ADDRESS, nextImageNumber);
      nextImageNumber +=  1;    
      logMessage("ESP-CAM/log", "Next image number: " +nextImageNumber);
    }

    rtc_gpio_hold_dis(GPIO_NUM_4);    
    pinMode(GPIO_NUM_4, OUTPUT);
    Serial.println("HIGH");
    digitalWrite(GPIO_NUM_4, HIGH);
    //rtc_gpio_hold_en(4);
        
    sensor_t * s = setCameraSensorSettings();         
    //take new image
    camera_fb_t * fb = NULL;
    //obtain camera frame buffer
    fb = esp_camera_fb_get();
    if (!fb) 
    {
      logMessage("ESP-CAM/log", "Camera capture failed");
      logMessage("ESP-CAM/log", "Exiting now"); 
      while(1);   //wait here as something is not right
    }else{
      EEPROM.put(COUNT_ADDRESS, nextImageNumber);
      EEPROM.commit();
    }
  
    //String path = "/spiffs/temp_image.jpg";
    fs::FS &fs = SPIFFS;
      //create new file
    File file = fs.open(tempImagePath.c_str(), FILE_WRITE);
    if(!file)
    {
      logMessage("ESP-CAM/log", "Failed to create file");
      logMessage("ESP-CAM/log", "Exiting now"); 
      while(1);   //wait here as something is not right    
    } 
    else 
    {
      file.write(fb->buf, fb->len); 
      logMessage("ESP-CAM/log", "Temp Image saved : " + tempImagePath);
    }
    file.close();
    
    //pinMode(4, OUTPUT);
    //logMessage("ESP-CAM/log", "Setting LED Low");  
    //digitalWrite(4, LOW);
    
    if(wifiConnected){
      uploadPhotoToFTP(fb, "/picture" + String(nextImageNumber) +".jpg");
    }else{
      logMessage("ESP-CAM/log", "Wifi Not Connected : ");  
    }
    
    
   
    //return camera frame buffer
    esp_camera_fb_return(fb);
    //Serial.printf("Temp Image saved: %s\n", tempImagePath.c_str());
    //logMessage("ESP-CAM/log", "Temp Image saved : ");    
}


void savePhotoToSD(){
    
    SPIFFS.begin();
    SD_MMC.begin();
    String sdImagePath = "/picture" + String(nextImageNumber) +".jpg";
    File destFile = SD_MMC.open(sdImagePath, FILE_WRITE);
    File sourceFile = SPIFFS.open(tempImagePath);
    static uint8_t buf[1024];
    while( sourceFile.read( buf, 1024) ) {
        destFile.write( buf, 1024 );
    }
    destFile.close();
    sourceFile.close();
    logMessage("ESP-CAM/log", "Saved to SD Card : "+ sdImagePath);  
     
}

void uploadPhotoToFTP(camera_fb_t * fb, String filename){
    const char *f_name = filename.c_str();
    File sourceFile = SPIFFS.open(tempImagePath);
    logMessage("ESP-CAM/log", "Uploading via FTP");
    ftp.OpenConnection();
    ftp.InitFile("Type I");
    ftp.NewFile( f_name );
    ftp.WriteData(fb->buf, fb->len);
    ftp.CloseFile();
    logMessage("ESP-CAM/log", "Uploaded file to ftp : "+ filename);
  
}

//Function that prints the reason by which ESP32 has been awaken from sleep
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : logMessage("ESP-CAM/log", "Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : logMessage("ESP-CAM/log", "Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : logMessage("ESP-CAM/log", "Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : logMessage("ESP-CAM/log", "Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : logMessage("ESP-CAM/log", "Wakeup caused by ULP program"); break;
    //default : logMessage("ESP-CAM/log", "Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
    default : logMessage("ESP-CAM/log", "Wakeup was not caused by deep sleep: " + wakeup_reason); break;
  }
}


camera_config_t setCameraConfig(){

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 3;
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  return config;
}

sensor_t* setCameraSensorSettings(){

    sensor_t * s = esp_camera_sensor_get();     
    s->set_brightness(s, 2);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 4);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    s->set_ae_level(s, 1);       // -2 to 2
    s->set_aec_value(s, 0);    // 0 to 1200
    s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    s->set_agc_gain(s, 20);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    s->set_bpc(s, 1);            // 0 = disable , 1 = enable
    s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
    s->set_vflip(s, 0);          // 0 = disable , 1 = enable
    s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    
  
}

void deep_sleep()
{
  logMessage("ESP-CAM/log", "Going to sleep after: " + String( millis() ) + "ms");
  Serial.flush();
  esp_deep_sleep_start();
}


void logMessage(String topic, String msg){
  const char* mqtt_topic = topic.c_str();
  const char* mqtt_msg = msg.c_str();
  Serial.println(mqtt_topic);
  Serial.println(mqtt_msg);
  
  Serial.print("wifiConnected : ");
  Serial.println(wifiConnected);
  if(wifiConnected){
    pubsubclient.connect(clientID, mqtt_username, mqtt_password);
    pubsubclient.publish(mqtt_topic, mqtt_msg, true);
  }
 
}

*/
