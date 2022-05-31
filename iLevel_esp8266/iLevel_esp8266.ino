#include <FirebaseESP8266.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

#define LED D6
#define rxPin D7
#define txPin D8
SoftwareSerial NodeSerial(rxPin, txPin); // RX | TX

#define WIFI_SSID "Kanteradar_2G"
#define WIFI_PASSWORD "Kanteradar"
#define FIREBASE_HOST "test-95a70-default-rtdb.asia-southeast1.firebasedatabase.app" // Mhuu
#define FIREBASE_KEY "Om9n9rETrjf6qPqm2OusT1vDALNXT1kLi03XG4Ih"
//#define FIREBASE_HOST "realtime-esp8266-cd0c7-default-rtdb.asia-southeast1.firebasedatabase.app" // Arm
//#define FIREBASE_KEY "rZiICWm7NXurizVfkcMo1MxFBgn7yXloR0to2PXo"

FirebaseData firebaseData;
bool state = false;

void setup() {
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(LED, OUTPUT);
  
  Serial.begin(115200);
  NodeSerial.begin(9600);
  Serial.println();
  Serial.println("NodeMCU/ESP8266 Run");
  connectWifi();
  Firebase.begin(FIREBASE_HOST, FIREBASE_KEY);
}

bool is_full = false;
float water_level = 0.0;
float temperature = 0.0;

void loop() {
//    testState();
  setDataValue();
  updateFirebaseData();
  delay(1500); // delay might increase
}

void updateFirebaseData() {
    Firebase.setFloat(firebaseData, "/main/temperature", temperature);
    Firebase.setFloat(firebaseData, "/main/water_level", water_level);
    Firebase.setFloat(firebaseData, "/main/is_full", is_full);
}

void testState() {
    if(Firebase.setBool(firebaseData, "/testing_state/state", state)) {
        Serial.println("Added"); 
        is_full = !is_full;
    } else {
        Serial.println("Error : " + firebaseData.errorReason());
    }
    if(is_full) digitalWrite(LED, HIGH); // TODO change it to is_full
  else digitalWrite(LED, LOW);
}

void setDataValue() {
  String readyToSend = "OK";
  NodeSerial.println(readyToSend); // len is 4 -> O K / n
  delay(100); 
  
  String recieveText = "Recieve: "; // len is 10. Reciving -> F:x|S:xxx.xx|T:xx.xx
  while (NodeSerial.available())
  {
    recieveText += (char)NodeSerial.read();
  }
  
  is_full = recieveText.substring(11, 12) == "1";
  water_level = recieveText.substring(15, 21).toFloat();
  temperature = recieveText.substring(24, 29).toFloat();

  if(is_full) digitalWrite(LED, HIGH); // TODO change it to is_full
  else digitalWrite(LED, LOW);
  
  Serial.println("IsFull: " + String(is_full) + ", WaterLevel: " + String(water_level) + ", Temperature: " + String(temperature));
}

void connectWifi() {
  Serial.begin(115200);
  Serial.println(WiFi.localIP());
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
}
