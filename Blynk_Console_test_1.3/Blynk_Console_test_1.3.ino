/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  Attention! Please check out TinyGSM guide:
    https://tiny.cc/tinygsm-readme

  Change GPRS apm, user, pass, and Blynk auth token to run :)
  Feel free to apply it to any other example. It's simple!

 *************************************************************/

/* Fill-in your Template ID (only if using Blynk.Cloud) */

#define BLYNK_TEMPLATE_ID "TMPL6_azdJY2Y"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "KEXguZsoVfr_BlLQZbeIkRWhbpVTNQmi"

// Select your modem:
#define TINY_GSM_MODEM_SIM7600

// Default heartbeat interval for GSM is 60
// If you want override this value, uncomment and set this option:
//#define BLYNK_HEARTBEAT 30

#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

BlynkTimer timer;

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = BLYNK_AUTH_TOKEN;

// Your GPRS credentials
// Leave empty, if missing user or pass
char apn[] = "TPG";
char user[] = "";
char pass[] = "";

#define SerialAT Serial1
#define UART_BAUD 115200
#define PIN_DTR 25
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4
#define LED_PIN 12
#define BAT_ADC 35
#define POWER_PIN 25
#define IND_PIN 36

//Neopixel pinout
#define PIXEL_PIN 2    // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT 3  // Number of NeoPixels

// NeoPixel brightness, 0 (min) to 255 (max)
#define BRIGHTNESS 150  // Set BRIGHTNESS to about 1/5 (max = 255)

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

//RED
#define PURPLE strip.Color(136, 14, 225)  //136, 14, 212

//Human detection sensor
#define human1 15  //1st human sensor pin

//Relays
#define Relay1 13
#define Relay2 19
#define Relay3 18
#define Relay4 5

bool humanP = false;

bool reply = false;

TinyGsm modem(SerialAT);

//ESPNOW portion//

// REPLACE WITH YOUR RECEIVER MAC Address
// uint8_t broadcastAddress[] = { 0xA0, 0x76, 0x4E, 0x1B, 0xF7, 0xE8 };  //lolin c3 pico
// uint8_t broadcastAddress[] = { 0x60, 0x55, 0xF9, 0x23, 0x43, 0x60 };  //Lolin C3 Mini v1.0.0    can work
uint8_t broadcastAddress[] = { 0x60, 0x55, 0xF9, 0x23, 0x20, 0xC8 };  //Lolin C3 Mini v1.0.0    number 2 unit
// uint8_t broadcastAddress[] = { 0x60, 0x55, 0xF9, 0x2B, 0xB5, 0x20 };  //Lolin C3 Mini v1.0.0    number 3 unit
// uint8_t broadcastAddress[] = { 0xA0, 0x76, 0x4E, 0x1D, 0x76, 0x84 };      //Lolin C3 Mini

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  bool occupied;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  // Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


BLYNK_WRITE(V0) {
  int pinValue = param.asInt();
  if (pinValue == 1) {

    digitalWrite(LED_PIN, LOW);
    Blynk.logEvent("LED STATE", "OFF");  //Sending Events
  } else {
    digitalWrite(LED_PIN, HIGH);
    Blynk.logEvent("LED STATE", "ON");  //Sending Events
  }
}

//Syncing the output state with the app at startup
BLYNK_CONNECTED() {
  Blynk.virtualWrite(V4, digitalRead(human1));  // will cause BLYNK_WRITE(V4) to be executed
  Blynk.syncVirtual(V0);                        // will cause BLYNK_WRITE(V0) to be executed
}

// This function sends Arduino's up time every second to Virtual Pin (5).
// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.
void sendHumanSensor() {
  int humanSense1 = digitalRead(human1);

  Serial.print("Human Detection Sensor:");
  Serial.println(humanSense1);

  // You can send any value at any time.
  // Please don't send more that 10 values per second.

  Blynk.virtualWrite(V4, humanSense1);
}

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i = 0; i < n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

void InitESPNow() {
  //If the initialization was successful
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  //If there was an error
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for (int i = 0; i < strip.numPixels(); i++) {  // For each pixel in strip...
    strip.setPixelColor(i, color);               //  Set pixel's color (in RAM)
    strip.show();                                //  Update strip to match
    delay(wait);                                 //  Pause for a moment
  }
}

/*
// State machine; 1 = All off and vacant; 2 = Occupied; 3 = Disinfection.

0 = All off and vacant; 
1 = UV light off but normal light on and Occupied; 
2 = UV light on; normal lights off and Occupied; 

*/

// ///////////// Finite State Machine Definitions /////////////
// enum { Vacant,
//        Occupied,
//        Disinfection };
// unsigned char stateMachine;

//number count for human check;
int m = 0;

// //State machine
int stateMachine = 0;

//Timer parameters;
bool timerActive;
// float timerLength = 1;  //10 mins
unsigned long startTime;
// float disinfectionPeriod = 1.5;  //check for 1.5 minutes and remain indicator light to maintain as RED
float disinfectionPeriod = 0.4;  //check for 1.5 minutes and remain indicator light to maintain as RED
unsigned long duration = disinfectionPeriod * 60 * 1000;
unsigned long timeStampCheck = 0;


void setup() {
  Serial.begin(115200);  // Set console baud rate
  SerialAT.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(100);

  // Onboard LED light, it can be used freely
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  //Setup of Neopixel for logo
  strip.begin();  // Initialize NeoPixel strip object (REQUIRED)
  strip.show();   // Initialize all pixels to 'off'
  strip.setBrightness(BRIGHTNESS);
  colorWipe(PURPLE, 10);  //

  // POWER_PIN : This pin controls the power supply of the Modem
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);

  // PWR_PIN ： This Pin is the PWR-KEY of the Modem
  // The time of active low level impulse of PWRKEY pin to power on module , type 500 ms
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(500);
  digitalWrite(PWR_PIN, LOW);

  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  pinMode(Relay3, OUTPUT);
  pinMode(Relay4, OUTPUT);
  pinMode(human1, INPUT);

  LightON();
  delay(1000);
  LightOFF();

  myData.occupied = false;

  // Set device as a Wi-Fi Station
  // WiFi.mode(WIFI_STA);

  // Set the device as a Station and Soft Access Point simultaneously
  WiFi.mode(WIFI_AP_STA);
  // WiFi.disconnect();

  delay(100);

  // Set device as a Wi-Fi Station
  // Serial.println("Connecting to Wi-Fi");
  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Serial.println("Connecting to Wi-Fi.");

  // while (WiFi.status() != WL_CONNECTED) {
  //   Serial.print(".");
  //   delay(300);
  // }

  // Serial.println();
  // Serial.print("Connected with IP: ");
  // Serial.println(WiFi.localIP());
  // Serial.println();


  DBG("Wait...");
  delay(3000);


  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("Initializing modem...");
  if (!modem.restart()) {
    Serial.println("Failed to restart modem, attempting to continue without restarting");
  }

  String name = modem.getModemName();
  delay(500);
  Serial.println("Modem Name: " + name);

  Blink(1);

  Blynk.begin(auth, modem, apn, user, pass);
  // Setup a function to be called every second
  timer.setInterval(2000L, sendHumanSensor);

  delay(1000);

  //Blink twice to indicate ESPNOW is initallized.
  Blink(2);


  Serial.println("Initializing ESP-NOW...");
  // Init ESP-NOW
  InitESPNow();

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  Blink(3);

  Serial.println("Done setup...");
}

void loop() {
  int humanSense1 = digitalRead(human1);
  Serial.print(humanSense1);
  Serial.print("; ");
  Serial.print(m);
  Serial.print("; ");
  Serial.print(stateMachine);
  Serial.print("; ");
  Serial.println(humanP);

  Blynk.run();
  timer.run();

  // Set values to send
  strcpy(myData.a, "THIS IS A G!MM");
  myData.b = random(1, 200);
  myData.c = random(0.0, 100.0);

  //State Machine (Rough way)//
  ///Vacant State/////

  // checkHuman(humanSense1);
  if (stateMachine == 0) {  // Vacant
    myData.occupied = false;
    Serial.print("; " + myData.occupied);
    Serial.print("; " + digitalRead(human1));
    Serial.println(" Human Undetected");

    //All lights are off
    LightOFF();

    checkHuman(humanSense1);
    if (humanP == true) {
      stateMachine = 1;  // Occupied
    }
    delay(20);

  } else if (stateMachine == 1) {  // Occupied
    myData.occupied = true;
    Serial.print("; " + myData.occupied);
    Serial.println("; " + humanSense1);
    Serial.println(" Human Detected");
    Serial.println("; " + m);
    delay(10);

    //LED light is ON
    LightON();

    checkHuman(humanSense1);
    if (humanP != false) {
      stateMachine = 1;  // Occupied
    } else if (humanP == false) {
      stateMachine = 2;  // Disinfection
    }
    delay(20);

  } else if (stateMachine == 2) {  // Disinfection

    Serial.println("State is Disinfection and UV ON...");
    disinfection();
    UVON();

    myData.occupied = true;
    Serial.println("; " + myData.occupied);
    Serial.println("; " + humanSense1);
    Serial.println(" Disinfecting...; UV On...");

    // Timer now active
    timerActive = true;

    // Record the start time in milliseconds
    startTime = millis();

    // Calculate duration to activate timer for in milliseconds
    // Calculation will be using unsigned long aritmatic
    // as first argument is an unsigned long

    // The following code will only be reached when the timer is active
    // Check every second if the selected time has elapsed or human detection is triggered

    while (timerActive) {
      Serial.print("Timelapse: ");
      Serial.println((millis() - startTime));
      // Duration reached so change state and off all lights
      if ((millis() - startTime) >= duration) {
        startTime = millis();
        stateMachine = 0;  //Vacant
        timerActive = false;
        LightOFF();
        myData.occupied = false;
        Serial.println("Done disinfecting, now Vacant...");
        break;
      }
      
      humanSense1 = digitalRead(human1);

///having issue here; don't know why when hand interject it will go to state 0 instead of state 1;
      //Check human detection
      if (humanSense1 == 1) {
        timerActive = false;
        stateMachine = 1;  //Occupied
        Serial.println("Human detected... Now is occupied..");
        break;
        stateMachine = 1;  //Occupied
      } else {
        m = 0;
      }
      delay(10);
    }
    //// having this issue here
  }
    // Send message via ESP-NOW
    esp_err_t
      result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    if (result == ESP_OK) {
      Serial.println(";   Sent with success");
      delay(100);  // wait 100ms
    } else {
      Serial.println(";   Error sending the data");
      delay(100);
    }
    delay(10);
  
}

/////////////Human detection code////////////////////

// bool checkHuman(bool a, int b) {  // take a look at this again, recode this portion
bool checkHuman(int a) {
  // bool humanP = false;
  if (a == 1) {
    m += 1;
    if (m >= 10 && (a == 1)) {

      humanP = true;

    } else {

      humanP = false;

    }
  } else {

    m = 0;
    humanP = false;

  }
  return humanP;
}

void LightON() {
  digitalWrite(Relay1, HIGH);
  delay(100);
}

void LightOFF() {
  digitalWrite(Relay1, LOW);
  delay(100);
}

void UVON() {
  digitalWrite(Relay1, HIGH);
  // delay(100);
}

void disinfection() {
  digitalWrite(Relay1, LOW);  //Shut off UV light to off
  delay(1000);                //pause 3s to full reset as ZERO ; 2000ms

  //disinfestion pattern sequence
  digitalWrite(Relay1, HIGH);  //trigger UV light sequence
  delay(250);
  digitalWrite(Relay1, LOW);
  delay(250);
  digitalWrite(Relay1, HIGH);
  delay(250);
}

void Blink(int a) {
  for (int i = 0; i < a; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(400);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }
}

/*
//////////////Finite state machine function////////////////

void vsState() {
  int humanSense1 = digitalRead(human1);

  switch (stateMachine) {
    case Vacant:
      Serial.println("State is Vacant and all OFF...");
      do {
        myData.occupied = false;
        Serial.print("; " + myData.occupied);
        Serial.print("; " + humanSense1);
        Serial.println(";   Human Undetected");
        checkHuman(humanSense1);
        LightOFF();
      } while (humanP == true);
      stateMachine = Occupied;
      break;
      // myData.occupied = false;
      // Serial.print("; " + myData.occupied);
      // Serial.print(";   Human Undetected");
      // LightOFF();

    case Occupied:
      Serial.println("State is Occupied and all ON...");
      do {
        myData.occupied = true;
        Serial.print("; " + myData.occupied);
        Serial.println("; " + humanSense1);
        Serial.println(";   Human Detected");
        Serial.println("; " + m);
        delay(100);
        LightON();

        if (humanSense1 != true) {
          stateMachine = Disinfection;
          break;
        }
        // checkHuman(humanSense1, m);
        // } while (humanP == false);
      } while (humanSense1 == false);
      stateMachine = Disinfection;
      startTime = millis();
      break;

    case Disinfection:
      Serial.println("State is Disinfection and UV ON...");
      disinfection();
      // startTime = millis();
      //need to add timer here

      do {
        myData.occupied = true;
        Serial.println("; " + myData.occupied);
        Serial.println("; " + humanSense1);
        Serial.println(";   Disinfecting...; UV On...");

        // Duration reached so change state and off all lights
        if ((millis() - startTime) >= duration) {
          startTime = millis();
          stateMachine = Vacant;
          LightOFF();
          break;
        }

        checkHuman(humanSense1);  //this is causing some issue; there is no while loop here therefore it will just change out;

        // UVON()  //might need to change this to leave light ON, UV ON.
      } while (humanP == true);  //or use humanSense to immediately change
      stateMachine = Occupied;
      break;

    default:
      stateMachine = Vacant;
      break;
  }
}
*/
