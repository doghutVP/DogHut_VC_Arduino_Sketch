// VIRTUAL CAMERA BY DOGHUT
// INPUT CONTROL FOR ARDUINO NANO 33 IoT
// Last changed 2020/06/07
// more info at www.doghut.de

#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>  
#include <Smoothed.h>

// wifi login data
char ssid[] = "wifiname";        // your network SSID (name)
char pass[] = "password";    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the Wifi radio's status
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // you can find this written on the board of some Arduino Ethernets or shields

WiFiUDP Udp;                           // A UDP instance to let us send and receive packets over UDP
const IPAddress destIp(192,168,1,7);   // remote IP of the target device
const unsigned int destPort = 8000;    // remote port of the target device where the NodeMCU sends OSC to
const unsigned int localPort = 9000;   // local port to listen for UDP packets at the NodeMCU (another device must send OSC messages to this port)

// define values
const int dialA_pin = 0; // analog pin connected to dialA
const int dialB_pin = 1; // analog pin connected to dialB
const int X_pin = 2; // analog pin connected to X output
const int Y_pin = 3; // analog pin connected to Y output
const int SW_pin = 13; // digital pin connected to switch output

const int smoothScale = 10; // how much the analog value should be smoothed / size of array to average
const int value_threshold = 10; // difference to ignore when changing value on dials & joystick
Smoothed <float> dialA_smoothArray; 
Smoothed <float> dialB_smoothArray; 
Smoothed <float> joystickX_smoothArray; 
Smoothed <float> joystickY_smoothArray; 

// initialize variables
int value_dialA = 0;
float value_dialA_remapped = 0.0;
int value_dialB = 0;
float value_dialB_remapped = 0.0;
bool value_joystickButton = 0;
int value_joystickX = 0;
int init_diff_joystickX = 0;
float value_joystickX_remapped = 0;
int value_joystickY = 0;
int init_diff_joystickY = 0;
float value_joystickY_remapped = 0;

void setup() {
  Serial.begin(115200);
  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);
  setIdleValues();


  dialA_smoothArray.begin(SMOOTHED_AVERAGE, smoothScale);
  dialB_smoothArray.begin(SMOOTHED_AVERAGE, smoothScale);
  joystickX_smoothArray.begin(SMOOTHED_AVERAGE, smoothScale);
  joystickY_smoothArray.begin(SMOOTHED_AVERAGE, smoothScale);



  // Specify a static IP address
  // If you erase this line, you will get a dynamic IP address
  WiFi.config(IPAddress(192,168,1,255),IPAddress(192,168,0,1), IPAddress(255,255,255,0)); 
  
  // Connect to WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(localPort);
}


// WIFI DEBUG MESSAGES
void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}


// ANALOG AND DIGITAL READS

// account for the variances in the idle state of the joysticks
void setIdleValues(){ 
  init_diff_joystickX = 512 - analogRead(X_pin);
  init_diff_joystickY = 512 - analogRead(Y_pin);
}


float sendOSCMessage(OSCMessage message, float value){
  OSCMessage msgOut(message);
  msgOut.add(value);
  Udp.beginPacket(destIp, destPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
}

void loop() { 

  //read from analog pins
  float current_dialA_value = analogRead(dialA_pin);
  float current_dialB_value = analogRead(dialB_pin);
  float current_joystickX_value = analogRead(X_pin) + init_diff_joystickX; // remove initial value offset
  float current_joystickY_value = analogRead(Y_pin)+ init_diff_joystickY; // remove initial value offset

  // add current value to array
  dialA_smoothArray.add(current_dialA_value);
  dialB_smoothArray.add(current_dialB_value);
  joystickX_smoothArray.add(current_joystickX_value);
  joystickY_smoothArray.add(current_joystickY_value);

  // get smoothed/averaged value from array
  float dialA_smoothed = dialA_smoothArray.get();
  float dialB_smoothed = dialB_smoothArray.get();
  float joystickX_smoothed = joystickX_smoothArray.get();
  float joystickY_smoothed = joystickY_smoothArray.get();

  //DIAL A // comment out this section if not connected
  if (dialA_smoothed > constrain(value_dialA + value_threshold,0,1024) || dialA_smoothed < constrain(value_dialA - value_threshold,0,1024)) { // check if value is different
    
     value_dialA = dialA_smoothed; 

    if (dialA_smoothed >= 1024-(value_threshold*2)){ // set to one if value is close to 1024
       sendOSCMessage("/arduino/dialA", 1);
    }

    else if (dialB_smoothed <= value_threshold*2){ // set to zero if value is close to 0
       sendOSCMessage("/arduino/dialA", 0);
    }

    else {
      sendOSCMessage("/arduino/dialA", float(map(dialA_smoothed, 0, 1024, 0, 1000))/1000); 
    }
  }

  //DIAL B // comment out this section if not connected
  if (dialB_smoothed > constrain(value_dialB + value_threshold,0,1024) || dialB_smoothed < constrain(value_dialB - value_threshold,0,1024)) { // check if value is different
    
     value_dialB = dialB_smoothed; 

    if (dialB_smoothed >= 1024-(value_threshold*2)){ // set to one if value is close to 1024
       sendOSCMessage("/arduino/dialB", 1);
    }

    else if (dialB_smoothed <= value_threshold*2){ // set to zero if value is close to 0
       sendOSCMessage("/arduino/dialB", 0);
    }

    else {
      sendOSCMessage("/arduino/dialB", float(map(dialB_smoothed, 0, 1024, 0, 1000))/1000); 
    }
  }

  //Joystick Button // comment out this section if not connected
  if (value_joystickButton != digitalRead(SW_pin)){ // check if value is different, send osc message if it is
    value_joystickButton = digitalRead(SW_pin);
    Serial.println(value_joystickButton);
    sendOSCMessage("/arduino/joystickButton", value_joystickButton);
    }

  //Joystick  X-Axis // comment out this section if not connected
  if (joystickX_smoothed > constrain(512 + value_threshold*2,0,1024) || joystickX_smoothed < constrain(512 - value_threshold*2,0,1024)) { // check if value is different
 
    sendOSCMessage("/arduino/joystickX", float(map(joystickX_smoothed, 0, 1024, -1000, 1000))/1000); 
  }
    
  //Joystick  Y-Axis // comment out this section if not connected
  if (joystickY_smoothed > constrain(512 + value_threshold*2,0,1024) || joystickY_smoothed < constrain(512 - value_threshold*2,0,1024)) { // check if value is different
 
    sendOSCMessage("/arduino/joystickY", float(map(joystickY_smoothed, 0, 1024, -1000, 1000))/1000); 
  }
    
  delay(10);
  

}
