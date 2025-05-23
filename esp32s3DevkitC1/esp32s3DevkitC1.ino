#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <bitmaps.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

uint8_t slaveAddress[] = {0x24,0xec,0x4a,0x20,0x7b,0x28};
bool last_message_status = false;

const int batteryVoltageTable[][3] = {{1680,100,ST77XX_GREEN},{1660,95,ST77XX_GREEN},{1644,90,ST77XX_GREEN},{1632,85,ST77XX_GREEN},{1608,80,ST77XX_GREEN},{1592,75,ST77XX_GREEN},
{1580,70,ST77XX_GREEN},{1564,65,ST77XX_GREEN},{1548,60,ST77XX_GREEN},{1540,55,ST77XX_GREEN},{1520,50,ST77XX_GREEN},
{1508,45,ST77XX_ORANGE},{1496,40,ST77XX_ORANGE},{1484,35,ST77XX_ORANGE},{1472,30,ST77XX_ORANGE},{1460,25,ST77XX_RED},
{1444,20,ST77XX_RED},{1432,15,ST77XX_RED},{1420,10,ST77XX_RED},{1400,5,ST77XX_RED},{1360,0,ST77XX_RED}};
int battery_index = 0;

struct controller_payload{
  bool buttons[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
  short joystick[2] = {0,0};
};
struct receiver_payload{
  byte errorFlag;
  byte batteryVoltageHighByte;
  byte batteryVoltageLowByte;
  bool impactDetected;
  float carYaw;
};

struct receiver_payload rec_payload;
struct controller_payload contr_payload;
esp_now_peer_info_t peerInfo;
String success;

const int button1 = 15;
const int button2 = 7;
const int button3 = 6;
const int button4 = 5;
const int button5 = 2;
const int joystick_x = 17;
const int joystick_y = 16;
int joystick_x_value = 0;
int joystick_y_value = 0;
const int joystick_btn = 14;
const int button_deload_time = 800;
unsigned long time_elapsed = 0;
bool movement_mode = 0;
int current_yaw_index = 0;
int desired_angle = 0;

//TFT VARIABLES
const int TFT_CS = 18;
const int TFT_DC = 8;
const int TFT_RST = 9;
Adafruit_ST7735 display = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
//TFT VARIABLES

//mpu variables
MPU6050 mpu;
float starting_yaw = 0;
float current_yaw = 0;
float prev_yaw = 0;
unsigned long start_timer = 0;
unsigned long stabilization_time = 5000;
uint8_t buffer[64]; 
Quaternion quaternion;
VectorFloat gravity;
float ypr[3];
//mpu variables


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status==0){
    if(last_message_status != status){
      display.fillRect(72,16,24,8,ST77XX_BLACK);
      display.setCursor(72,16);
      display.print("Stable");

      last_message_status = status;
    }
  }
  else{
    if(last_message_status != status){
      display.fillRect(72,16,36,8,ST77XX_BLACK);
      display.setCursor(72,16);
      display.print("Lost");

      last_message_status = status;
    }
  }
}


//The function that is automatically called when a packet is received, here we use the robots yaw to calculate the correct direction of the arrow
//displayed in the tft screen
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&rec_payload, incomingData, len);
  //we add 45 degrees because the original value of looking forward is 0 degrees and the forward vector is +45 and -45 degrees from the starting yaw
  int new_car_yaw_index = rec_payload.carYaw - current_yaw + 45;
  //if the yaw is greater than 360 we have done a full circle, this happens because the right arrow is between yaw 315 and 45 (or -45 and 45 starting from 0)
  new_car_yaw_index += (new_car_yaw_index < 0) * 360;
  //we divide by 90 to get an index which corresponds to the direction the robot is facing
  //0 = right
  //1 = forward
  //2 = left
  //3 = down
  new_car_yaw_index = new_car_yaw_index/90;
  if(current_yaw_index != (new_car_yaw_index % 4)){
    //clear the arrow rectangle bu making it all black, saving the new index as the current one and using the bitmap array to paint the new direction arrow
    display.fillRect(80,60,62,62,ST77XX_BLACK);
    display.drawBitmap(80,60,arrow_bitmaps[new_car_yaw_index % 4],BITMAP_WIDTH,BITMAP_HEIGHT,ST77XX_WHITE);
    current_yaw_index = new_car_yaw_index;
  }
  int battery_voltage = (rec_payload.batteryVoltageHighByte<<8) | rec_payload.batteryVoltageLowByte;
  battery_voltage = 1480;
  // Serial.print("Battery voltage threshold:");
  // Serial.println(batteryVoltageTable[battery_index][0]);
  if(batteryVoltageTable[battery_index][0] > battery_voltage){
    //Serial.print("Battery index:");
    do{
      battery_index++;  
    }
    while(batteryVoltageTable[battery_index][0] > battery_voltage);
    //Serial.println(battery_index);
    // Serial.print("Battery voltage threshold:");
    // Serial.println(batteryVoltageTable[battery_index][0]);
    display.fillRect(60,0,100,8,ST77XX_BLACK);
    display.fillRect(60,0,batteryVoltageTable[battery_index][1],8,batteryVoltageTable[battery_index][2]);
  }
}

void setup(){
  Serial.begin(115200);
  Wire.begin(19,20);

  //setting up the tft display
  display.initR(INITR_BLACKTAB);
  //rotating by 90 degrees making the screen black and setting the curson on the top left corner of the screen
  display.setRotation(1);
  display.fillScreen(ST77XX_BLACK);
  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);

  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  if(!mpu.testConnection()){
    display.print("MPU FAILED, RESTART");
    while(1);
  }   
  Serial.println("MPU6050 connection successful");
  //setting up pinmode for buttons and joystick input
  analogReadResolution(10);
  pinMode(button1,INPUT_PULLUP);
  pinMode(button2,INPUT_PULLUP);
  pinMode(button3,INPUT_PULLUP);
  pinMode(button4,INPUT_PULLUP);
  pinMode(button5,INPUT_PULLUP);
  pinMode(joystick_btn,INPUT_PULLUP);
  

  //setting up the esp now protocol
  display.print("Initializing ESP NOW");
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    display.fillScreen(ST77XX_BLACK);
    display.setCursor(0, 0);
    display.print("Error");
    display.setCursor(0,16);
    display.print("Initializing");
    while(1);
  }
  //registering the callback function when sending data
  esp_now_register_send_cb(OnDataSent);
  
  //register peer 
  display.fillScreen(ST77XX_BLACK);
  display.setCursor(0, 0);
  display.print("Pairing...");
  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    display.fillScreen(ST77XX_BLACK);
    display.setCursor(0, 0);
    display.print("Failed to add peer");
    while(1);
    
  }

  int temp_readings = 0;
  while(temp_readings < 200){
    if (mpu.getFIFOCount() >= 42) {
      if (mpu.dmpGetCurrentFIFOPacket(buffer)) {
        temp_readings++;
        delay(10);
      }
    }
  }
  delay(2000);
  //setting up info to display like battery of the robot direction arrow and connection status
  display.fillScreen(ST77XX_BLACK);
  display.setCursor(0, 0);
  display.print("Robot Bat:");
  display.print("Waiting Data");
  display.setCursor(0,16);
  //display connection status
  display.print("Conn Status:");
  display.setCursor(72,16);
  display.print("Stable");
  //draw arrow bitmap for direction
  display.drawBitmap(80,60,arrow_bitmaps[0],BITMAP_WIDTH,BITMAP_HEIGHT,ST77XX_WHITE);
  
  //registering a callback function for when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  stabilization_time += millis();
}
 
void loop(){
  contr_payload.buttons[0] = digitalRead(button1);
  contr_payload.buttons[1] = digitalRead(button2);
  contr_payload.buttons[2] = digitalRead(button3);
  contr_payload.buttons[3] = digitalRead(button4);
  contr_payload.buttons[4] = digitalRead(button5);
  
  if(digitalRead(joystick_btn) == HIGH && millis() - time_elapsed > button_deload_time){
    time_elapsed = millis();
    movement_mode = !movement_mode;
    //ADD DISPLAY MESSAGE TO KNOW WHAT MODE WE USE
  }
  // joystick_x_value = 1023-analogRead(joystick_x) - 512;
  // joystick_y_value = analogRead(joystick_y) - 512;
  // Serial.print("Joystick Y raw:");
  // Serial.println(joystick_y_value);
  // Serial.print("Joystick X raw:");
  // Serial.println(joystick_x_value);
  // if(abs(joystick_x_value) <= 70){
  //     joystick_x_value = 0;
  //   }

  // if(abs(joystick_y_value) <= 70){
  //     joystick_y_value = 0;
  //   }
  // Serial.print("Joystick Y:");
  // Serial.println(joystick_y_value);
  // Serial.print("Joystick X:");
  // Serial.println(joystick_x_value);
  Serial.print("Car Yaw:");
  Serial.println(rec_payload.carYaw);
  mpu.dmpGetCurrentFIFOPacket(buffer);
  mpu.dmpGetQuaternion(&quaternion, buffer);
  mpu.dmpGetGravity(&gravity, &quaternion);
  mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);
  // joystick_x_value = joystick_x_value * cos(ypr[0] - starting_yaw) - joystick_y_value * sin(ypr[0] - starting_yaw);
  // joystick_y_value = joystick_x_value * sin(ypr[0] - starting_yaw) + joystick_y_value * cos(ypr[0] - starting_yaw); 
  // Serial.print("Joystick Y transformed:");
  // Serial.println(joystick_y_value);
  // Serial.print("Joystick X transformed:");
  // Serial.println(joystick_x_value);
  //Here we calculate the yaw of the controller, first in radians. We invert the value so that clockwise rotation decreases the value (by default the mpu6050 module does the opposite CW movement increases the value)
  //then we use *180/M_PI to turn the radians into degrees from -180 to 180 then we transform it to 0-360 degrees since the joystick and the rc car MPU6050 also use the same logic 
  Serial.print("Controller Yaw rad:");
  current_yaw = -ypr[0];
  Serial.println(current_yaw);
  current_yaw = current_yaw * (180 / M_PI);
  current_yaw += (current_yaw<0)*360;
  //the mpu6050 module uses a DMP (digital motion processor) which takes the data from the gyroscope and acceletometer and applies some calculations instead of them needed to be added by us, like kalman filters etc
  //till the DMP warms up and has stabilized we use the current minus the previous yaw value to calculate the starting yaw
  if(millis() - start_timer < stabilization_time){
      starting_yaw += 1 * (current_yaw - prev_yaw);
      prev_yaw = current_yaw;
    }
  //we subtract the starting yaw from the current, to find the differential yaw Dyaw = yawFinal-yawStarting
  current_yaw = current_yaw - starting_yaw;
  current_yaw += ((current_yaw < 0) * 360);
  Serial.print("Controller Yaw 0-360:");
  Serial.println(current_yaw);

  ///only used for testing delete after installing joystick///
  if(Serial.available() > 0){
    desired_angle = Serial.parseInt();
    Serial.println("Read from serial");
    Serial.read();
  }
  Serial.print("Desired Angle 0 360:");
  Serial.println(desired_angle);

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
  // Serial.print("MOVEMENT MODE:");
  // Serial.println(contr_payload.buttons[5]);
  // Serial.print("Joystick Y mapped:");
  // Serial.println(contr_payload.joystick[0]);
  // Serial.print("Joystick X mapped:");
  // Serial.println(contr_payload.joystick[1]);
  
  // if(joystick_x_value !=0 || joystick_y_value != 0){
  if(desired_angle != 0){
    if(movement_mode){
      WorldPosMovement(joystick_x_value,joystick_y_value);
    }
    else{
      TankControlMovement(joystick_x_value,joystick_y_value);
    }
  }
  // }
  // else{
  //   contr_payload.joystick[0] = joystick_x_value;
  //   contr_payload.joystick[1] = joystick_y_value;
  // }
  esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)&contr_payload, sizeof(contr_payload));
  delay(800);
}

void TankControlMovement(int joystickX,int joystickY){
  int vector_length = (joystickX * joystickX) + (joystickY * joystickY);
  float movement_vector_length = sqrt(vector_length);
  float movement_vector_magnitude = constrain(movement_vector_length/512,0.4,1);
  int desired_angle = int(atan2((double)joystickY,(double)joystickX)* 180/M_PI); 
  if(abs(desired_angle) > 135){
    //left side wheel speed
    contr_payload.joystick[0] = -160;
    //right side wheel speed
    contr_payload.joystick[1] = 160;
      
    //debug
    Serial.print("Left motor Speed:");
    Serial.println(contr_payload.joystick[0]);
    Serial.print("Right motor Speed:");
    Serial.println(contr_payload.joystick[1]);
  }
  else if(abs(desired_angle) < 45){
    //left side wheel speed
    contr_payload.joystick[0] = -160;
    //right side wheel speed
    contr_payload.joystick[1] = 160;
      
    //debug
    Serial.print("Left motor Speed:");
    Serial.println(contr_payload.joystick[0]);
    Serial.print("Right motor Speed:");
    Serial.println(contr_payload.joystick[1]);
  }
  else{
    //left side wheel speed
    contr_payload.joystick[0] = movement_vector_magnitude * 255;
    //right side wheel speed
    contr_payload.joystick[1] = movement_vector_magnitude * 255;
  }
}

void WorldPosMovement(int joystickX,int joystickY){
  // int desired_angle = int(atan2((double)joystickY,(double)joystickX)* 180/M_PI); 
  
  
  int vector_length = (joystickX * joystickX) + (joystickY * joystickY);
  float movement_vector_length = sqrt(vector_length);
  // Serial.print("SQRT:");
  // Serial.println(movement_vector_length);
  float movement_vector_magnitude = constrain(movement_vector_length/512,0,1);
  // Serial.print("MAGNITUDE");
  // Serial.println(movement_vector_magnitude);
  Serial.print("ANGLE:");
  Serial.println((desired_angle + int(current_yaw))%360);
  int angle = int(rec_payload.carYaw - (desired_angle + current_yaw));
  if(angle > 180){angle -= 360;}
  if(angle < -180){angle += 360;}
  Serial.print("ANGLE DIF:");
  Serial.println(angle);
  if(abs(angle) > 15){
    //0 indicates counter clock wise rotation and 1 indicates clockwise direction
    //this expression takes the difference between the two angles(we add 360 to wrap around if needed and mod with 360),
    //if the difference is less that 180 then go counter clock wise
    bool rotate_direction = (angle < 0);
    Serial.print("Turn direction:");
    Serial.println(rotate_direction); 
    float temp_rotation_speed = abs(angle)/180.0;
    Serial.print("TEMP Rotation Speed:");
    Serial.println(temp_rotation_speed);
    float rotation_speed = constrain(temp_rotation_speed,0.2,1);
    Serial.print("Rotation Speed:");
    Serial.println(rotation_speed);
    if(rotate_direction){
      Serial.println("COUNTER CLOCKWISE");
      //left side wheel speed
      contr_payload.joystick[0] = movement_vector_magnitude * 255 - movement_vector_magnitude * 255 * rotation_speed;
      //we map to 80 because low values do not rotate the wheels at all (perhaps due to low voltage?)
      contr_payload.joystick[0] = map(contr_payload.joystick[0],0,255,80,255);
      
      //right side wheel speed
      contr_payload.joystick[1] = movement_vector_magnitude * 255;
      
      //debug
      Serial.print("Left motor Speed:");
      Serial.println(contr_payload.joystick[0]);
      Serial.print("Right motor Speed:");
      Serial.println(contr_payload.joystick[1]);
    }
    else{
      Serial.println("CLOCKWISE");
      //left side wheel speed
      contr_payload.joystick[0] = movement_vector_magnitude * 255;
      //right side wheel speed
      contr_payload.joystick[1] = movement_vector_magnitude * 255 - movement_vector_magnitude * 255 * rotation_speed;
      //we map to 80 because low values do not rotate the wheels at all (perhaps due to low voltage?)
      contr_payload.joystick[1] = map(contr_payload.joystick[1],0,255,80,255);
      
      //debug
      Serial.print("Left motor Speed:");
      Serial.println(contr_payload.joystick[0]);
      Serial.print("Right motor Speed:");
      Serial.println(contr_payload.joystick[1]);
    }
  }
  else{
    contr_payload.joystick[0] = movement_vector_magnitude * 255;
    contr_payload.joystick[1] = movement_vector_magnitude * 255;
    
    //debug
    Serial.print("Left motor Speed:");
    Serial.println(contr_payload.joystick[0]);
    Serial.print("Right motor Speed:");
    Serial.println(contr_payload.joystick[1]);
  }
}
