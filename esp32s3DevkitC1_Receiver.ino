#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

#define startbyte 0x0F
#define I2Caddress 0x07


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
int lowbat=550;                                      // adjust to suit your battery: 550 = 5.50V
byte i2caddr=7;                                      // default I2C address of T'REX is 7. If this is changed, the T'REX will automatically store new address in EEPROM
byte i2cfreq=0; 

uint8_t slaveAddress[] = {0x8c,0xbf,0xea,0x86,0xf2,0xb8};

int forward_speed, rotate_speed;

struct controller_payload{
  bool buttons[13];
  short joystick[2];
};

struct acknowledgePayload{
  byte startingByte;
  byte errorFlag;
  byte batteryVoltageHighByte;
  byte batteryVoltageLowByte;

  byte LMotorCurrentHighByte;
  byte LMotorCurrentLowByte;
  byte LEncoderCountHighByte;
  byte LEncoderCountLowByte;
  
  byte RMotorCurrentHighByte;
  byte RMotorCurrentLowByte;
  byte REncoderCountHighByte;
  byte REncoderCountLowByte;

  byte AccelerometerXHighByte;
  byte AccelerometerXLowByte;
  byte AccelerometerYHighByte;
  byte AccelerometerYLowByte;
  byte AccelerometerZHighByte;
  byte AccelerometerZLowByte;
  
  byte ImpactXHighByte;
  byte ImpactXLowByte;
  byte ImpactYHighByte;
  byte ImpactYLowhByte;
  byte ImpactZHighByte;
  byte ImpactZLowhByte;
};

struct controller_payload received_payload;
struct acknowledgePayload ackPayload;

char message[12] = "Hello back";
esp_now_peer_info_t peerInfo;
String success;

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
  memcpy(&received_payload, incomingData, sizeof(received_payload));
  Serial.print("Bytes received: ");
  Serial.println(len);
  int i;
  for(i=0; i<13; i++){
    Serial.print("Button ");
    Serial.print(i);
    Serial.print(":");
    Serial.println(received_payload.buttons[i]);
  }
  if(received_payload.buttons[5] == HIGH){
    if(received_payload.joystick[1] > 0 ){
      lmspeed = 180;
      rmspeed = -180;
    }
    else if(received_payload.joystick[1] < 0){
      lmspeed = -180;
      rmspeed = 180;
    }
    else{
      lmspeed = 0;
      rmspeed = 0;
    }
  }
  else{
    forward_speed = received_payload.joystick[0];
    rotate_speed = received_payload.joystick[1];
    if(received_payload.joystick[0] > 0){
      if(received_payload.joystick[1] > 0){
        lmspeed = forward_speed + rotate_speed;
        rmspeed = forward_speed - rotate_speed;
      }
      else{
        lmspeed = forward_speed + rotate_speed;
        rmspeed = forward_speed - rotate_speed;
      }
    }
    else{
      if(received_payload.joystick[1] > 0){
        lmspeed = forward_speed - rotate_speed;
        rmspeed = forward_speed + rotate_speed;
      }
      else{
        lmspeed = forward_speed - rotate_speed;
        rmspeed = forward_speed + rotate_speed;
      }
    }
  }
  Serial.print("Joystick Y:");
  Serial.println(received_payload.joystick[0]);
  Serial.print("Joystick X:");
  Serial.println(received_payload.joystick[1]);
  Serial.print("LEFT MOTOR SPEED:");
  Serial.println(lmspeed);
  Serial.print("RIGHT MOTOR SPEED:");
  Serial.println(rmspeed);
  MasterSend(startbyte,1,lmspeed,lmbrake,rmspeed,rmbrake,sv[0],sv[1],sv[2],sv[3],sv[4],sv[5],devibrate,sensitivity,lowbat,i2caddr,i2cfreq);
  delay(50);
  MasterReceive();
}

void setup(){
  digitalWrite(48,LOW);
  Serial.begin(115200);

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
  if(!Wire.begin()){
    Serial.println("Problem with initialization of wire");
  }
}
 
void loop(){
  //esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)&message, strlen(message));
  // if (result == ESP_OK) {
  //   Serial.println("Sent with success");
  // }
  // else {
  //   Serial.println("Error sending the data");
  // }
  // MasterSend(startbyte,2,lmspeed,lmbrake,rmspeed,rmbrake,sv[0],sv[1],sv[2],sv[3],sv[4],sv[5],devibrate,sensitivity,lowbat,i2caddr,i2cfreq);
  // delay(50);
  // MasterReceive();                                   // receive data packet from T'REX controller
  // delay(50);
  // lmspeed+=ldir;
  // if(lmspeed>240 or lmspeed<-240) ldir=-ldir;        // increase / decrease left motor speed and direction (negative values = reverse direction)
  
  // rmspeed+=rdir;
  // if(rmspeed>240 or rmspeed<-240) rdir=-rdir; 
  // for(byte i=0;i<6;i++)                              // sweep servos
  // {
  //   if(sv[i]!=0)                                     // a value of 0 indicates no servo attached
  //   {
  //     sv[i]+=sd[i];                                  // update servo position to create sweeping motion
  //     if(sv[i]>2000 || sv[i]<1000) sd[i]=-sd[i];     // reverse direction of servo if limit reached
  //   }
  // }
}

void MasterReceive()
{//================================================================= Error Checking ==========================================================
  byte d;
  int i=0;
  Wire.requestFrom(I2Caddress,24);                                // request 24 bytes from device 007
  
  while(Wire.available()<24)                                      // wait for entire data packet to be received
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
  d=Wire.read();                                                  // read start byte from buffer
  if(d!=startbyte)                                                // if start byte not equal to 0x0F                                                    
  {
    Serial.print(d,DEC);
    while(Wire.available()>0)                                     // empty buffer of bad data
    {
      d=Wire.read();
    }
    Serial.println("  Wrong Start Byte");                         // error message
    return;                                                       // quit
  }
  
  //================================================================ Read Data ==============================================================
  Serial.print("Slave Error Message:");                           // slave error report
  Serial.println(Wire.read(),BIN);
  
  i=Wire.read()*256+Wire.read();                                  // T'REX battery voltage
  Serial.print("Battery Voltage:\t");
  Serial.print(int(i/10));Serial.println(".");                      
  Serial.print(i-(int(i/10)*10));Serial.println("V");
  
  i=Wire.read()*256+Wire.read();
  Serial.print("Left  Motor Current:\t");
  Serial.print(i);Serial.println("mA");                           // T'REX left  motor current in mA
  
  i=Wire.read()*256+Wire.read();
  Serial.print("Left  Motor Encoder:\t");
  Serial.println(i);                                              // T'REX left  motor encoder count
  
  i=Wire.read()*256+Wire.read();
  Serial.print("Right Motor Current:\t");
  Serial.print(i);Serial.println("mA");                           // T'REX right motor current in mA
  
  i=Wire.read()*256+Wire.read();
  Serial.print("Right Motor Encoder:\t");
  Serial.println(i);                                              // T'REX right motor encoder count
  
  i=Wire.read()*256+Wire.read();
  Serial.print("X-axis:\t\t");
  Serial.println(i);                                              // T'REX X-axis
  
  i=Wire.read()*256+Wire.read();
  Serial.print("Y-axis:\t\t");
  Serial.println(i);                                              // T'REX Y-axis
  
  i=Wire.read()*256+Wire.read();
  Serial.print("Z-axis:\t\t");
  Serial.println(i);                                              // T'REX Z-axis
  
  i=Wire.read()*256+Wire.read();
  Serial.print("X-delta:\t\t");
  Serial.println(i);                                              // T'REX X-delta
  
  i=Wire.read()*256+Wire.read();
  Serial.print("Y-delta:\t\t");
  Serial.println(i);                                              // T'REX Y-delta
  
  i=Wire.read()*256+Wire.read();
  Serial.print("Z-delta:\t\t");
  Serial.println(i);                                              // T'REX Z-delta
  Serial.print("\r\n\n\n");
  
}


void MasterSend(byte sbyte, byte pfreq, int lspeed, byte lbrake, int rspeed, byte rbrake, int sv0, int sv1, int sv2, int sv3, int sv4, int sv5, byte dev,int sens,int lowbat, byte i2caddr,byte i2cfreq){
  Wire.beginTransmission(I2Caddress);
  Wire.write(sbyte);
  Wire.write(pfreq);

  Wire.write(highByte(lspeed));       // MSB left  motor speed
  Wire.write( lowByte(lspeed));       // LSB left  motor speed
  Wire.write(lbrake);                 // left  motor brake
  
  Wire.write(highByte(rspeed));       // MSB right motor speed
  Wire.write( lowByte(rspeed));       // LSB right motor speed
  Wire.write(rbrake);                 // right motor brake
  
  Wire.write(highByte(sv0));          // MSB servo 0
  Wire.write( lowByte(sv0));          // LSB servo 0
  
  Wire.write(highByte(sv1));          // MSB servo 1
  Wire.write( lowByte(sv1));          // LSB servo 1
  
  Wire.write(highByte(sv2));          // MSB servo 2
  Wire.write( lowByte(sv2));          // LSB servo 2
  
  Wire.write(highByte(sv3));          // MSB servo 3
  Wire.write( lowByte(sv3));          // LSB servo 3
  
  Wire.write(highByte(sv4));          // MSB servo 4
  Wire.write( lowByte(sv4));          // LSB servo 4
  
  Wire.write(highByte(sv5));          // MSB servo 5
  Wire.write( lowByte(sv5));          // LSB servo 5
  
  Wire.write(dev);                    // devibrate
  Wire.write(highByte(sens));         // MSB impact sensitivity
  Wire.write( lowByte(sens));         // LSB impact sensitivity
  
  Wire.write(highByte(lowbat));       // MSB low battery voltage  550 to 30000 = 5.5V to 30V
  Wire.write( lowByte(lowbat));       // LSB low battery voltage
  
  Wire.write(i2caddr);                // I2C slave address for T'REX controller
  Wire.write(i2cfreq);                // I2C clock frequency:   0=100kHz   1=400kHz
  Wire.endTransmission();             // stop transmitting
  
  Serial.println("Master Command Data Packet Sent");

}
