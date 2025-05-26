#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_NeoPixel.h>
#include "esp_system.h"

#define RGB_PIN     48    // Try this first
#define NUM_PIXELS  1

#define startbyte 0x0F
#define I2Caddress 0x07
#define SERVOMIN 150
#define SERVOMAX 600
#define SERVOSTEP 15

//TREX VARIABLES
int sv[6]={1500,1500,1500,1500,1500,1500};                 // servo positions: if == 0 then the servo is not used
int sd[6]={5,10,-5,-15,20,-20};                      // servo sweep speed/direction
int lmspeed,rmspeed; 
int ldir=5;                                          // how much to change left  motor speed each loop (use for motor testing)
int rdir=5;                                 // left and right motor speed from -255 to +255 (negative value = reverse)
int forwardSpeed,turnSpeed;                          // forward speed = joystickY value ... turn speed = joystickX value
byte lmbrake,rmbrake;                                // left and right motor brake (non zero value = brake)
byte devibrate=50;                                   // time delay after impact to prevent false re-triggering due to chassis vibration
int sensitivity=50;                                  // threshold of acceleration / deceleration required to register as an impact
int lowbat=1320;                                      // adjust to suit your battery: 1320 = 13.20V
byte i2caddr=7;                                      // default I2C address of T'REX is 7. If this is changed, the T'REX will automatically store new address in EEPROM
byte i2cfreq=0; 
//TREX VARIABLES

uint8_t slaveAddress[] = {0x8c,0xbf,0xea,0x86,0xf2,0xb8};
Adafruit_NeoPixel rgb(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

struct controller_payload{
  bool buttons[12];
  short joystick[2];
};

struct acknowledgePayload{
  byte errorFlag;
  byte batteryVoltageHighByte;
  byte batteryVoltageLowByte;
  bool impactDetected;
  float carYaw;
};

struct controller_payload received_payload;
struct acknowledgePayload ack_payload;


esp_now_peer_info_t peerInfo;
bool waiting_response = true;
unsigned long heartbeat_timer = 0;
byte failed_attempts = 0;

//car mpu variables
MPU6050 mpu;
float starting_yaw = 0;
float current_yaw = 0;
float prev_yaw = 0;
unsigned long start_timer = 0;
unsigned long time_elapsed = 0;
unsigned long stabilization_time = 5000;
uint8_t buffer[64]; 
Quaternion quaternion;
VectorFloat gravity;
float ypr[3];
//car mpu variables

//pca9685 + motor variables
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();
int servo_pwm[6] = {SERVOMIN, SERVOMIN, SERVOMIN, SERVOMIN, SERVOMIN, SERVOMIN};
//pca9685 + motor variables

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.print("Response:");
  Serial.println(waiting_response);
  if (status ==0){
  }
  else{
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  heartbeat_timer = millis();
  if(len == sizeof(bool)){
    esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)&waiting_response, sizeof(bool));
    waiting_response = false;
    Serial.print("Pinged, sent response");
    return;
  }
  memcpy(&received_payload, incomingData, sizeof(received_payload));
  int i;
  for(i=0; i<12; i+=2){
    servo_pwm[i/2] = servo_pwm[i/2] + (SERVOSTEP*received_payload.buttons[i]) - (SERVOSTEP*received_payload.buttons[i+1]); 
    servo_pwm[i/2] = constrain(servo_pwm[i/2],SERVOMIN,SERVOMAX);
    pca.setPWM(i/2,0,servo_pwm[i/2]);
  }
  Serial.print("Left motor speed:");
  Serial.println(received_payload.joystick[0]);
  Serial.print("Right Motor Speed:");
  Serial.println(received_payload.joystick[1]);
  MasterSend(startbyte,2,received_payload.joystick[0],lmbrake,received_payload.joystick[1],rmbrake,sv[0],sv[1],sv[2],sv[3],sv[4],sv[5],devibrate,sensitivity,lowbat,i2caddr,i2cfreq);
  delay(50);
  MasterReceive();
}

void setup(){
  Serial.begin(115200);

  //We use the on board rgb of the devkit to display the connection status, its only used for debug so it won't be visible in the final product
  //Blue means the esp32 is in setup mode, if there is no connection in the setup mode the rgb will stay blue
  //Green means that there is a stable connection and the two esp boards communicate 
  //Yellow means that the connection is lost and the esp enters a wait mode till the two devices re establish communication 
  rgb.begin();
  rgb.show(); // Initialize to off
  rgb.setPixelColor(0, rgb.Color(0, 0, 255)); //blue
  rgb.show();
  //used only for debug, prints the reason esp reset usually its reason 1.Manual reset via the button or usb was connected which resets the esp
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("Reset reason: ");
  Serial.println(reason);

  delay(1000);
  //Wire is used to communicate with the MPU6050 and the PCA9685 modules, mpu and pca require wire and not wire1
  Wire.begin(10,9);
  if(mpu.testConnection()) {
    Serial.println("Mpu connection is solid");
    }
  else{
    Serial.println("Mpu connection failed restarting automatically...");
    ESP.restart();
  }
  //initializing mpu6050 module and dmp for filtering mpu measurements
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  // int temp_readings = 0;
  // while(temp_readings < 200){
  //   if (mpu.getFIFOCount() >= 42) {
  //     if (mpu.dmpGetCurrentFIFOPacket(buffer)) {
  //       temp_readings++;
  //       delay(10);
  //     }
  //   }
  // }
  //delay(2000);
  //Wire1 is used solely for communicating with the motor driver board on the 6WD vehicle
  Wire1.begin(1,5,100000);
  WiFi.mode(WIFI_STA);

  // Initializing esp not
  if (esp_now_init() != ESP_OK) {
    Serial.println("Esp now initialization failed, restarting automatically...");
    ESP.restart();
  }

  //We register a callback function that runs when we sent a package
  esp_now_register_send_cb(OnDataSent);
  
  //We register the other esp32 that resides inside the wireless controller using its MAC Address
  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Problem adding the controller as peer, restarting automatically...");
    ESP.restart();
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  //we initialize the pca9685 module
  pca.begin();
  //the standard frequency for pwm motors is 50-60Hz
  pca.setPWMFreq(50);


  while(waiting_response){
    Serial.println("Waiting for connection...");
    delay(100);
  }

  //We don't know how long the esp32 has been waiting for a connection in the code above 
  //so we use millis() to have the correct starting time for our timer variables
  heartbeat_timer = millis();
  stabilization_time = millis();
  start_timer = millis();
}

void loop(){
  if(millis() - heartbeat_timer > 500){
    waiting_response = true;
    rgb.setPixelColor(0, rgb.Color(255, 255, 0)); //red
    rgb.show();
  }
  if(waiting_response){
    Serial.println("Disconnected, waiting ping...");
    MasterSend(startbyte,1,225,1,225,1,sv[0],sv[1],sv[2],sv[3],sv[4],sv[5],devibrate,sensitivity,lowbat,i2caddr,i2cfreq);
    delay(100);
  }
  else{
    rgb.setPixelColor(0, rgb.Color(0, 255, 0)); //green
    rgb.show();
    if(mpu.dmpGetCurrentFIFOPacket(buffer)){
      mpu.dmpGetQuaternion(&quaternion, buffer);
      mpu.dmpGetGravity(&gravity, &quaternion);
      mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);
      current_yaw = -ypr[0] * 180/ M_PI;
      // Serial.print("YAW solo IS:");
      // Serial.println(current_yaw);
      if(millis() - start_timer < stabilization_time){
        starting_yaw += 0.95 * (current_yaw - prev_yaw);
        prev_yaw = current_yaw;
      }
      current_yaw = current_yaw - starting_yaw + 90;
      current_yaw += ((current_yaw < 0) * 360);
      ack_payload.carYaw = current_yaw;
      current_yaw = int(current_yaw) % 360;
    // Serial.print("YAW IS:");
    // Serial.println(current_yaw);
    // Serial.print("WITH STARTING YAW:");
    // Serial.println(starting_yaw);
    // Serial.println();
    }
  esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)&ack_payload, sizeof(ack_payload));
    Serial.println("Sent ack payload");
    delay(100);
  }
}


void MasterReceive()
{//================================================================= Error Checking ==========================================================
  byte d;
  ack_payload.impactDetected = false;
  int i=0;
  Wire1.requestFrom(I2Caddress,24);                                // request 24 bytes from device 007
  
  while(Wire1.available()<24)                                      // wait for entire data packet to be received
  {
    if(i==0) Serial.print("Waiting for slave to send data.");     // Only print message once (i==0)
    if(i>0) Serial.print(".");                                    // print a dot for every loop where buffer<24 bytes
    i++;                                                          // increment i so that message only prints once.
    if(i>79)
    {
      Serial.println("");
      i=1;
    }
  }
  d=Wire1.read();                                                  // read start byte from buffer
  if(d!=startbyte)                                                // if start byte not equal to 0x0F                                                    
  {
    Serial.print(d,DEC);
    while(Wire1.available()>0)                                     // empty buffer of bad data
    {
      d=Wire1.read();
    }
    Serial.println("  Wrong Start Byte");                         // error message
    return;                                                       // quit
  }
  
  //================================================================ Read Data ==============================================================
  Serial.print("Slave Error Message:");                           // slave error report
  ack_payload.errorFlag = Wire1.read();
  Serial.println(ack_payload.errorFlag,BIN);
  
  ack_payload.batteryVoltageHighByte = Wire1.read(); 
  ack_payload.batteryVoltageLowByte = Wire1.read();                                  // T'REX battery voltage   
  i = (ack_payload.batteryVoltageHighByte << 8) | ack_payload.batteryVoltageLowByte;
  Serial.print("Battery Voltage:\t");
  Serial.print(i);
  Serial.println("V");
  
  i=Wire1.read()*256+Wire1.read();
  Serial.print("Left  Motor Current:\t");
  Serial.print(i);Serial.println("mA");                           // T'REX left  motor current in mA
  
  i=Wire1.read()*256+Wire1.read();
  //Serial.print("Left  Motor Encoder:\t");
  //Serial.println(i);                                              // T'REX left  motor encoder count
  
  i=Wire1.read()*256+Wire1.read();
  Serial.print("Right Motor Current:\t");
  Serial.print(i);Serial.println("mA");                           // T'REX right motor current in mA
  
  i=Wire1.read()*256+Wire1.read();
  //Serial.print("Right Motor Encoder:\t");
  //Serial.println(i);                                              // T'REX right motor encoder count
  
  i=Wire1.read()*256+Wire1.read();
  Serial.print("X-axis:\t\t");
  Serial.println(i);                                              // T'REX X-axis
  
  i=Wire1.read()*256+Wire1.read();
  Serial.print("Y-axis:\t\t");
  Serial.println(i);                                              // T'REX Y-axis
  
  i=Wire1.read()*256+Wire1.read();
  Serial.print("Z-axis:\t\t");
  Serial.println(i);                                              // T'REX Z-axis
  
  i=Wire1.read()*256+Wire1.read();
  Serial.print("X-delta:\t\t");
  Serial.println(i);                                              // T'REX X-delta
  if(i != 0){
    ack_payload.impactDetected = true;
  }
  i=Wire1.read()*256+Wire1.read();
  Serial.print("Y-delta:\t\t");
  Serial.println(i);                                              // T'REX Y-delta
  if(i != 0){
    ack_payload.impactDetected = true;
  }
  i=Wire1.read()*256+Wire1.read();
  Serial.print("Z-delta:\t\t");
  Serial.println(i);                                              // T'REX Z-delta
  Serial.print("\r\n\n\n");
  if(i != 0){
    ack_payload.impactDetected = true;
  }
}


void MasterSend(byte sbyte, byte pfreq, int lspeed, byte lbrake, int rspeed, byte rbrake, int sv0, int sv1, int sv2, int sv3, int sv4, int sv5, byte dev,int sens,int lowbat, byte i2caddr,byte i2cfreq){
  Wire1.beginTransmission(I2Caddress);
  Wire1.write(sbyte);
  Wire1.write(pfreq);

  Wire1.write(highByte(lspeed));       // MSB left  motor speed
  Wire1.write( lowByte(lspeed));       // LSB left  motor speed
  Wire1.write(lbrake);                 // left  motor brake
  
  Wire1.write(highByte(rspeed));       // MSB right motor speed
  Wire1.write( lowByte(rspeed));       // LSB right motor speed
  Wire1.write(rbrake);                 // right motor brake
  
  Wire1.write(highByte(sv0));          // MSB servo 0
  Wire1.write( lowByte(sv0));          // LSB servo 0
  
  Wire1.write(highByte(sv1));          // MSB servo 1
  Wire1.write( lowByte(sv1));          // LSB servo 1
  
  Wire1.write(highByte(sv2));          // MSB servo 2
  Wire1.write( lowByte(sv2));          // LSB servo 2
  
  Wire1.write(highByte(sv3));          // MSB servo 3
  Wire1.write( lowByte(sv3));          // LSB servo 3
  
  Wire1.write(highByte(sv4));          // MSB servo 4
  Wire1.write( lowByte(sv4));          // LSB servo 4
  
  Wire1.write(highByte(sv5));          // MSB servo 5
  Wire1.write( lowByte(sv5));          // LSB servo 5
  
  Wire1.write(dev);                    // devibrate
  Wire1.write(highByte(sens));         // MSB impact sensitivity
  Wire1.write( lowByte(sens));         // LSB impact sensitivity
  
  Wire1.write(highByte(lowbat));       // MSB low battery voltage  550 to 30000 = 5.5V to 30V
  Wire1.write( lowByte(lowbat));       // LSB low battery voltage
  
  Wire1.write(i2caddr);                // I2C slave address for T'REX controller
  Wire1.write(i2cfreq);                // I2C clock frequency:   0=100kHz   1=400kHz
  Wire1.endTransmission();             // stop transmitting
  
  Serial.println("Master Command Data Packet Sent");

}
