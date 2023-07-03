#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFiMulti.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <timer.h>


#define WIFI_SSID "HUAWEI-106V40"
#define WIFI_PASSWORD "NewPwds*/@12"
#define WEBSERVER_HOST "192.168.3.10"//IP address of backend server
#define WEBSERVER_PORT 8080 // Port of backend service
#define STOMP_SERVER_ENDPOINT "/iot-pid-ball-and-beam/"//the endpoint to subscribe to stomp server

#define JSON_DOCUMENT_SIZE 2040
#define DEVICE_NAME "alancho"
#define REACT_SERVER_NAME "react-ui"

#define EACH_REQUEST_CHANNEL "/iot-websocket/"
#define EACH_SUBSCRIPTION_PREFIX "/connection/"

#define DEVICE_INFO_CHANNEL "device-system-info/" /* Resgisteres to this channel to send messages */
#define DEVICE_CHANGE_PARAMETERS "changeParameters/" /* Subscribed to this channel to read messages*/

WebSocketsClient webSocketsClient;
uniuno:: Timer timer;
Servo myServo;
int servoPin = 19;
int sensorPin = 35;
float sensorAnalogReading = 0;
float sensorVoltage = 0;
float voltageReference=3.3;
float pinResolution = 4096;
float sum= 0.0;
float distance=0;

//Control Variables
int position = 0;
int copyOfLastPosition = 0;
float setPoint = 11.5;
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError, cumError, rateError;
double outPut;
double Kp = 2.0;//2.0
double Ki = 0.00055; //0.00055
double Kd = 1200;//1200

void connectToWebSocket();
void handleWebSocketEvent(WStype_t type, uint8_t *payload, size_t length); // To recieve the paramets of any message in this callback
void subscribeToChannel(String channelName, String deviceListening);
void processJsonDataInMessageRecieved(String messageRecieved);
String extractObjectFromMessage(String messageRecieved);
void sendControlSystemInformation();
void sendMessage(String channelName, String payload, String deviceListening);
double PID(float input);

void setup() {
  Serial.begin(921600);
  pinMode(LED_BUILTIN, OUTPUT);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED){
    Serial.println("Trying to connect to wifi again...");
    delay(100);
  }
  Serial.println("Connected to wifi");
  myServo.attach(servoPin);
  pinMode(sensorPin,INPUT);
  connectToWebSocket();
  timer.set_interval(sendControlSystemInformation, 100);
  timer.attach_to_loop();
}

void loop() {
  // Permitted working range from 8-19
  sum = 0.0;
  
  for (int i=0; i<100; i++){
    sum+=float(analogRead(sensorPin));  
  }

  sensorAnalogReading = sum/100;
  // Convert anlogic read value to voltage
  sensorVoltage = (sensorAnalogReading * voltageReference) / pinResolution;
  distance =  (-0.7610 * (sensorVoltage * sensorVoltage * sensorVoltage)) + (6.8341 * (sensorVoltage * sensorVoltage)) - ( 22.7881 * sensorVoltage) + 35.9252; // The constanst were found out using least square method to calibrate the sensor
  //Serial.println(distance);
  
  copyOfLastPosition = position;
  position = int(PID(distance) ); // The 25 value is because is exactly the middle between 0-50 that is the limit of position for the servo
  
  // To keep the servo positioning between 0-50 degrees
  if (position >= 50){
    position = 50;
  }else if (position <= 0){
    position = 0;
  }
  myServo.write(50 - position);
  Serial.println(50- position);
  delay(100);

  webSocketsClient.loop();
  timer.tick();
}

void connectToWebSocket(){
  //Building the format of transport request URL http://host:port/myApp/myEndpoint/{server-id}/{session-id}/{transport}
  String urlFormat = STOMP_SERVER_ENDPOINT;
  urlFormat += random(0,999); //server id choosen by the client
  urlFormat += "/";
  urlFormat += random(0,999999); //session id this must be a unique value for all the clients
  urlFormat += "/websocket"; // To learn more about the format of URL request visit: "https://sockjs.github.io/sockjs-protocol/sockjs-protocol-0.3.3.html"

  webSocketsClient.begin(WEBSERVER_HOST, WEBSERVER_PORT, urlFormat);
  webSocketsClient.setExtraHeaders();
  webSocketsClient.onEvent(handleWebSocketEvent);
}

void handleWebSocketEvent(WStype_t type, uint8_t *payload, size_t length){
  switch (type){
  case WStype_DISCONNECTED:
    Serial.println("Desconnected from websocket...");
    break;
  case WStype_CONNECTED: // 2nd if the crendtial are correct then we response with a payload
    {
      Serial.printf("[Open session message *from server ]: %s \n", payload);  
    }
    break;
  case WStype_TEXT:
    {
      String text = (char*) payload;
      if (payload[0] == 'h'){// In case of heartbeat
        Serial.println("Heartbeat!");
      }else if(payload[0] == 'o'){// 1st we send the connection message
        String connectMessage = "[\"CONNECT\\naccept-version:1.1,1.0\\nheart-beat:1000,1000\\n\\n\\u0000\"]";
        webSocketsClient.sendTXT(connectMessage);
        delay(100);
      }else if (text.startsWith("a[\"CONNECTED")){ // Inmediately when we stablish the connection with server, we have to susbcribe to some channels to start to revieve messages
        String deviceId = String(DEVICE_NAME);
        subscribeToChannel(String(DEVICE_CHANGE_PARAMETERS), deviceId); // To check if the dive is connected to wifi and its current status working/not working
        delay(500);

      }else if (text.startsWith("a[\"MESSAGE")){ // This block will be executed whenever we recieve a message from stomp server
        processJsonDataInMessageRecieved(text);
      }
    }
    break;  

  }
}

void subscribeToChannel(String channelName, String deviceListening){
  String subscribeMessage = "[\"SUBSCRIBE\\nid:sub-0\\ndestination:" + String(EACH_SUBSCRIPTION_PREFIX) + channelName + deviceListening + "\\n\\n\\u0000\"]";
  Serial.println("Subscripcion: " + subscribeMessage);
  webSocketsClient.sendTXT(subscribeMessage);
}

void processJsonDataInMessageRecieved(String messageRecieved){
  String jsonObject = extractObjectFromMessage(messageRecieved);
  jsonObject.replace("\\","");
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, jsonObject);

  JsonObject contentRecieved = doc.as<JsonObject>();

  // We will only attend the requests that como from UI
  if (strcmp(contentRecieved["from"], "react-ui-alancho") == 0 && strcmp(contentRecieved["to"], "alancho") == 0){
    if(strcmp(contentRecieved["action"], "changeParameters") == 0){
      double newKp = contentRecieved["newKp"];
      double newKi = contentRecieved["newKi"];
      double newKd = contentRecieved["newKd"];
      float newSetPoint = contentRecieved["newSetPoint"];
      /* Serial.println(newKp);
      Serial.println(newKi,6);
      Serial.println(newKd);
      Serial.println(setPoint); */
      Kp = newKp;
      Ki = newKi;
      Kd = newKd;
      setPoint = newSetPoint;
    }
  }else{
    Serial.println("Message ignores");
  }
}

String extractObjectFromMessage(String messageRecieved){
  char startingChar = '{';
  char finishingChar = '}';

  String tmpData = "";
  bool _flag = false;
  for (int i = 0; i < messageRecieved.length(); i++) {
    char tmpChar = messageRecieved[i];
    if (tmpChar == startingChar) {
      tmpData += startingChar;
      _flag = true;
    }
    else if (tmpChar == finishingChar) {
      tmpData += finishingChar;
      break;
    }
    else if (_flag == true) {
      tmpData += tmpChar;
    }
  }

  return tmpData;
}


double PID(float input){ 
  // Se guarda el tempo actual 
  currentTime = millis();
  // Se calcula el tiempo transcurrido
  elapsedTime = currentTime - previousTime;
  // Se obtiene el error de posición
  error = setPoint - input;
  // Se calcula la integral del error
  cumError += error * elapsedTime;
  // Se calcula la derivada del error
  rateError = (error - lastError) / elapsedTime;
  // Se calcula la salida del controlador
  outPut = Kp * error + Ki * cumError + Kd * rateError;

  // El error actual se convierte en el error pasado
  lastError = error;
  // El tiempo actual se convierte en el tiempo pasado
  previousTime = currentTime;

  // Se regresa la salida del controlador
  return outPut;
}

void sendControlSystemInformation(){

    String deviceId = String(DEVICE_NAME);
    String action = "systemInfo";
    String to = "react-ui-alancho";
    String deviceInfoPayload = "{\\\"from\\\":\\\"" +
                                    deviceId + "\\\",\\\"to\\\":\\\"" +
                                    to + "\\\",\\\"action\\\":\\\"" +
                                    action + "\\\",\\\"kp\\\":\\\"" +
                                    Kp + "\\\",\\\"ki\\\":\\\"" +
                                    String(Ki,6) + "\\\",\\\"kd\\\":\\\"" +
                                    Kd + "\\\",\\\"currentDistance\\\":\\\"" +
                                    distance + "\\\",\\\"setPoint\\\":\\\"" +
                                    setPoint + "\\\",\\\"servoAngleFrom\\\":\\\"" +
                                    copyOfLastPosition + "\\\",\\\"servoAngleTo\\\":\\\"" +
                                    position + "\\\"}";
              
    sendMessage(String(DEVICE_INFO_CHANNEL), deviceInfoPayload, String(DEVICE_NAME));                
}

void sendMessage(String channelName, String payload, String deviceListening){
  String message = "[\"SEND\\ndestination:" + String(EACH_REQUEST_CHANNEL) + channelName + deviceListening + "\\n\\n" + payload + "\\n\\n\\u0000\"]";
  webSocketsClient.sendTXT(message);
}