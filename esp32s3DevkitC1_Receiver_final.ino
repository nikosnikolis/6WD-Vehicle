#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_NeoPixel.h>
#include "esp_system.h"
#include "esp_task_wdt.h"

#define WDT_TIMEOUT 5


#define RGB_PIN     48    
#define NUM_PIXELS  1

#define startbyte 0x0F
#define I2Caddress 0x07
#define SERVOMIN 150
#define SERVOMAX 600
#define SERVOSTEP 10

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
int lowbat=1280;                                      // adjust to suit your battery: 1280 = 12.80V
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
  int leftCurrent;
  int rightCurrent;
};

struct controller_payload received_payload;
struct acknowledgePayload ack_payload;


esp_now_peer_info_t peerInfo;
bool waiting_response = true;
unsigned long heartbeat_timer = 0;

//car mpu variables
MPU6050 mpu;
float starting_yaw = 0;
float current_yaw = 0;
float prev_yaw = 0;
unsigned long start_timer = 0;
unsigned long time_elapsed = 0;
unsigned long stabilization_time = 2000;
uint8_t buffer[64]; 
Quaternion quaternion;
VectorFloat gravity;
float ypr[3];
//car mpu variables

//Συνάρτηση μετατροπής γωνίας σε PWM
//Με βάση την ελάχιστη και μέγιστη τιμή PWM
double angleToPwm(double angle,double min, double max){
  Serial.print("Turning Angle:");
  Serial.println(angle);
  return map(angle,0,180,min,max);
}
//Μεταβλητές PCA9685
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();
int servo_pwm[][3] = {{SERVOMIN,SERVOMAX,angleToPwm(90,SERVOMIN,SERVOMAX)},
{120,SERVOMAX,angleToPwm(100,120,SERVOMAX)},
{SERVOMIN,490,angleToPwm(90,SERVOMIN,490)},
{SERVOMIN,SERVOMAX,SERVOMIN},
{SERVOMIN,SERVOMAX,SERVOMIN},
{270,370,270}};



Quaternion Multiply_Quaternion(Quaternion q1, Quaternion q2){
  return Quaternion(q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    );
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
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
  int input1 = received_payload.buttons[1] - received_payload.buttons[0];
  int input2 = received_payload.buttons[3] - received_payload.buttons[2];
  int input3 = received_payload.buttons[5] - received_payload.buttons[4];
  int input4 = received_payload.buttons[7] - received_payload.buttons[6];
  int input5 = received_payload.buttons[9] - received_payload.buttons[8];
  int input6 = received_payload.buttons[11] - received_payload.buttons[10];
  Serial.print("BUTTON 11: ");
  Serial.println(received_payload.buttons[11]);
  Serial.print("BUTTON 10");
  Serial.println(received_payload.buttons[10]);
  Serial.print("Input: ");
  Serial.println(input1);
  Serial.print("Input: ");
  Serial.println(input2);
  Serial.print("Input: ");
  Serial.println(input3);
  Serial.print("Input: ");
  Serial.println(input4);
  Serial.print("Input: ");
  Serial.println(input5);
  Serial.print("Input: ");
  Serial.println(input6);
  if(input1 != 0){
    servo_pwm[0][2] = constrain(servo_pwm[0][2] + input1 * SERVOSTEP, servo_pwm[0][0], servo_pwm[0][1]);
    pca.setPWM(0,0,servo_pwm[0][2]);
  }
  if(input2 != 0){
    servo_pwm[1][2] = constrain(servo_pwm[1][2] + input2 * SERVOSTEP, servo_pwm[1][0], servo_pwm[1][1]);
    pca.setPWM(1,0,servo_pwm[1][2]);
  }
  if(input3 != 0){
    servo_pwm[2][2] = constrain(servo_pwm[2][2] + input3 * SERVOSTEP, servo_pwm[2][0], servo_pwm[2][1]);
    pca.setPWM(2,0,servo_pwm[2][2]);
  }
  if(input4 != 0){
    servo_pwm[3][2] = constrain(servo_pwm[3][2] + input4 * SERVOSTEP, servo_pwm[3][0], servo_pwm[3][1]);
    pca.setPWM(3,0,servo_pwm[3][2]);
  }
  if(input5 != 0){
    servo_pwm[4][2] = constrain(servo_pwm[4][2] + input5 * SERVOSTEP, servo_pwm[4][0], servo_pwm[4][1]);
    pca.setPWM(4,0,servo_pwm[4][2]);
  }
  if(input6 != 0){
    servo_pwm[5][2] = constrain(servo_pwm[5][2] + input6 * SERVOSTEP, servo_pwm[5][0], servo_pwm[5][1]);
    pca.setPWM(5,0,servo_pwm[5][2]);
  }

}

void setup(){
  Serial.begin(115200);
  //Κάνουμε χρήση του ενσωματωμένου LED του ESP32 S3 DevKit M1
  //Το χρώμα μπλε σημαίνει ότι το ESP32 S3 είναι στην φάση του setup
  //Το πράσινο χρώμα σημαίνει ότι η επικοινωνία μεταξύ των 2 ESP32 S3 είναι σταθερή 
  //Το κίτρινο χρώμα σημαίνει ότι η επικοινωνία χάθηκε και περιμένουμε επανασύνδεση
  rgb.begin();
  //Την πρώτη φορά που εκτελείται η εντολή show ενεργοποιείται το led χωρίς να φωτίζει κάποιο χρώμα 
  rgb.show(); 
  rgb.setPixelColor(0, rgb.Color(0, 0, 255)); //μπλε χρώμα
  rgb.show();
  delay(1000);
  //Στο κανάλι επικοινωνίας συνδέουμε το MPU6050 και το PCA9685
  Wire.begin(10,9);
  if(mpu.testConnection()) {
    Serial.println("Mpu connection is solid");
    }
  else{
    Serial.println("Mpu connection failed restarting automatically...");
    ESP.restart();
  }
  //Αρχικοποίηση του DMP
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  //Στο κανάλι επικοινωνίας Wire1 συνδέουμε μόνο το τροχοφόρο
  Wire1.begin(1,5,100000);


  //Αρχικοποίηση του ESP NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Esp now initialization failed, restarting automatically...");
    ESP.restart();
  }
  //Εγγραφή της ασύγχρονης συνάρτησης αποστολής πακέτων
  esp_now_register_send_cb(OnDataSent);
  //Προσθήκη του ESP32 S3 του χειριστηρίου ως peer
  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Problem adding the controller as peer, restarting automatically...");
    ESP.restart();
  }
  //Εγγραφή της ασύγχρονης συνάρτησης για την λήψη πακέτων
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  //Αρχικοποίηση του PCA9685
  pca.begin();
  pca.setPWMFreq(50);
  pca.setPWM(0,0,servo_pwm[0][2]);
  pca.setPWM(1,0,servo_pwm[1][2]);
  pca.setPWM(2,0,servo_pwm[2][2]);
  pca.setPWM(3,0,servo_pwm[3][2]);
  pca.setPWM(4,0,servo_pwm[4][2]);
  pca.setPWM(5,0,servo_pwm[5][2]);
  
  yield();
   
  while(waiting_response){
    Serial.println("Waiting for connection...");
    delay(100);
  }
  //Αρχικοποίηση όλων των μετρητών
  heartbeat_timer = millis();
  stabilization_time = millis();
  start_timer = millis();
  esp_task_wdt_config_t wdt_config = {.timeout_ms = WDT_TIMEOUT * 1000, .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, .trigger_panic = true};
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
}

void loop(){
  if(millis() - heartbeat_timer > 500){
    rgb.setPixelColor(0, rgb.Color(255, 255, 0)); //κίτρινο
    rgb.show();
    Serial.println("Disconnected, waiting ping...");
    //Ενεργοποιούμε τα φρένα του οχήματος ώστε να μείνει στάσιμο μέχρι την αποκατάσταση της σύνδεσης
    MasterSend(startbyte,1,225,1,225,1,sv[0],sv[1],sv[2],sv[3],sv[4],sv[5],devibrate,sensitivity,lowbat,i2caddr,i2cfreq);
    esp_task_wdt_reset();
    delay(100);
  }
  else{
    rgb.setPixelColor(0, rgb.Color(0, 255, 0)); //πράσινο
    rgb.show();
    if(mpu.dmpGetCurrentFIFOPacket(buffer)){
      mpu.dmpGetQuaternion(&quaternion, buffer);
      quaternion = Multiply_Quaternion(quaternion, Quaternion(cos(PI/4), sin(PI/4), 0, 0));
      mpu.dmpGetGravity(&gravity, &quaternion);
      mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);
      current_yaw = -ypr[0] * 180/ M_PI;
      if(millis() - start_timer < stabilization_time){
        starting_yaw += 0.95 * (current_yaw - prev_yaw);
        prev_yaw = current_yaw;
      }
      current_yaw = current_yaw - starting_yaw + 90;
      current_yaw += ((current_yaw < 0) * 360);
      current_yaw = int(current_yaw) % 360;
      Serial.print("Car Yaw:");
      Serial.println(current_yaw);
      ack_payload.carYaw = current_yaw;
    }
    MasterSend(startbyte,2,received_payload.joystick[0],lmbrake,received_payload.joystick[1],rmbrake,sv[0],sv[1],sv[2],sv[3],sv[4],sv[5],devibrate,sensitivity,lowbat,i2caddr,i2cfreq);
    delay(50);
    MasterReceive();
    delay(50);
    esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)&ack_payload, sizeof(ack_payload));
    esp_task_wdt_reset();
    Serial.println("Sent ack payload");
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
  ack_payload.leftCurrent = i;
  Serial.print("Left  Motor Current:\t");
  Serial.print(i);Serial.println("mA");                           // T'REX left  motor current in mA
  
  i=Wire1.read()*256+Wire1.read();
  Serial.print("Left  Motor Encoder:\t");
  Serial.println(i);                                              // T'REX left  motor encoder count
  
  i=Wire1.read()*256+Wire1.read();
  ack_payload.rightCurrent = i;
  Serial.print("Right Motor Current:\t");
  Serial.print(i);Serial.println("mA");                           // T'REX right motor current in mA
  
  i=Wire1.read()*256+Wire1.read();
  Serial.print("Right Motor Encoder:\t");
  Serial.println(i);                                              // T'REX right motor encoder count
  
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
