#include <WiFi.h>
#include <esp_now.h>

uint8_t slaveAddress[] = {0x24,0xec,0x4a,0x20,0x7b,0x28};

struct controller_payload{
  bool buttons[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
  short joystick[2] = {0,0};
};
struct controller_payload contr_payload;
char message_back[12];
esp_now_peer_info_t peerInfo;
String success;

const int button1 = 15;
const int button2 = 7;
const int button3 = 6;
const int button4 = 5;
const int button5 = 2;
const int joystick_btn = 14;
const int button_deload_time = 800;
int time_elapsed = 0;
bool movement_mode = 0;
const int joystick_x = 17;
const int joystick_y = 16;
int lmspeed = 0;
int rmspeed = 0;
int forward_speed = 0;
int rotate_speed = 0;

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

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&message_back, incomingData, sizeof(message_back));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.println(message_back);
}

void setup(){
  Serial.begin(115200);
  pinMode(button1,INPUT_PULLUP);
  pinMode(button2,INPUT_PULLUP);
  pinMode(button3,INPUT_PULLUP);
  pinMode(button4,INPUT_PULLUP);
  pinMode(button5,INPUT_PULLUP);
  pinMode(joystick_btn,INPUT_PULLUP);

  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop(){
  contr_payload.buttons[0] = digitalRead(button1);
  contr_payload.buttons[1] = digitalRead(button2);
  contr_payload.buttons[2] = digitalRead(button3);
  contr_payload.buttons[3] = digitalRead(button4);
  contr_payload.buttons[4] = digitalRead(button5);
  if(digitalRead(joystick_btn) == HIGH && millis() - time_elapsed > button_deload_time){
    time_elapsed = millis();
    contr_payload.buttons[5] = !contr_payload.buttons[5];
  }
  contr_payload.joystick[0] = analogRead(joystick_x);
  contr_payload.joystick[1] = analogRead(joystick_y);
  Serial.print("Joystick Y:");
  Serial.println(contr_payload.joystick[0]);
  Serial.print("Joystick X:");
  Serial.println(contr_payload.joystick[1]);
  if(abs(contr_payload.joystick[0]-2200) <= 100){
      contr_payload.joystick[0] = 0;
    }
  else{
      if(contr_payload.joystick[0] > 2300){
        contr_payload.joystick[0] = 180;
      }
      else{
        contr_payload.joystick[0] = -180;
      }
  }

  if(abs(contr_payload.joystick[1]-2200) <= 100){
      contr_payload.joystick[1] = 0;
    }
  else{
      if(contr_payload.joystick[1] > 2300){
        contr_payload.joystick[1] = 30;
      }
      else{
        contr_payload.joystick[1] = -30;
      }
  }

  // Serial.print("Button 1:");
  // Serial.println(digitalRead(button1));
  // Serial.print("Button 2:");
  // Serial.println(digitalRead(button2));
  // Serial.print("Button 3:");
  // Serial.println(digitalRead(button3));
  // Serial.print("Button 4:");
  // Serial.println(digitalRead(button4));
  // Serial.print("Button 5:");
  // Serial.println(digitalRead(button5));
  // Serial.print("Joystick Button:");
  // Serial.println(digitalRead(joystick_btn));
  Serial.print("MOVEMENT MODE:");
  Serial.println(contr_payload.buttons[5]);
  // Serial.print("Joystick Y mapped:");
  // Serial.println(contr_payload.joystick[0]);
  // Serial.print("Joystick X mapped:");
  // Serial.println(contr_payload.joystick[1]);
  
  
  esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)&contr_payload, sizeof(contr_payload));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(100);
}
