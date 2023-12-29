#include <WiFi.h>
#include <ArduinoJson.h>
// WiFi credentials
const char* ssid = "moto_g54";                    //Enter your wifi hotspot ssid
const char* password =  "prit4102";               //Enter your wifi hotspot password
const uint16_t port = 8002;
const char * host = "192.168.208.23"; 

// External peripherals 
int buzzerPin = 15;
int redLed = 2;
StaticJsonDocument<200> jsonDocument;
DeserializationError error;
int P=0,I=0,D=0;

char incomingPacket[80];
WiFiClient client;

String msg = "0";

void setup(){
   
  Serial.begin(115200);                          //Serial to print data on Serial Monitor

  // Output Pins
  pinMode(buzzerPin, OUTPUT);                      
  pinMode(redLed, OUTPUT);
  // Initially off
  digitalWrite(buzzerPin, HIGH);                // Negative logic Buzzer       
  digitalWrite(redLed, LOW);

  //Connecting to wifi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("...");
  }
 
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
}


void loop() {

  if (!client.connect(host, port)) {
    Serial.println("Connection to host failed");
    digitalWrite(buzzerPin, HIGH);           
    digitalWrite(redLed, LOW); 
    delay(200);
    return;
  }

  while(1){
      msg = client.readStringUntil('\n');         //Read the message through the socket until new line char(\n)
      Serial.println(msg);
      error = deserializeJson(jsonDocument,msg);
      if(error)
      {
        Serial.print("Failed to parse JSON: ");
        Serial.println(error.c_str());
      }
      else 
      {
        P = jsonDocument["P"];
        Serial.println(P);
        I = jsonDocument["I"];
        Serial.println(I);
        D = jsonDocument["D"];
        Serial.println(D);
      }
     
    }
}
