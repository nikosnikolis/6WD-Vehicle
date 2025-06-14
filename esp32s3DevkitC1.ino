#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <bitmaps.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "esp_task_wdt.h"

#define WDT_TIMEOUT 5

uint8_t slaveAddress[] = {0x24,0xec,0x4a,0x20,0x7b,0x28};
bool waiting_response = true;
unsigned long heartbeat_timer = 0;
byte last_status_connected = 2; //0 is connected // 1 is menu // 2 is disconnected

const int batteryVoltageTable[][3] = {{1680,100,ST77XX_GREEN},{1660,95,ST77XX_GREEN},{1644,90,ST77XX_GREEN},
{1632,85,ST77XX_GREEN},{1608,80,ST77XX_GREEN},{1592,75,ST77XX_GREEN},
{1580,70,ST77XX_GREEN},{1564,65,ST77XX_GREEN},{1548,60,ST77XX_GREEN},{1540,55,ST77XX_GREEN},
{1520,50,ST77XX_GREEN},{1508,45,ST77XX_ORANGE},{1496,40,ST77XX_ORANGE},{1484,35,ST77XX_ORANGE},
{1472,30,ST77XX_ORANGE},{1460,25,ST77XX_RED},{1444,20,ST77XX_RED},{1432,15,ST77XX_RED},
{1400,10,ST77XX_RED},{1370,5,ST77XX_RED},{1320,0,ST77XX_RED}};
int battery_index = 0;

struct controller_payload{
  bool buttons[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
  short wheel_speed[2] = {0,0};
};
struct controller_payload contr_payload;

struct receiver_payload{
  byte errorFlag;
  byte batteryVoltageHighByte;
  byte batteryVoltageLowByte;
  bool impactDetected;
  float carYaw;
  int leftCurrent;
  int rightCurrent;
};
struct receiver_payload rec_payload;

esp_now_peer_info_t peerInfo;
String success;

const int button1 = 14;
const int button2 = 17;
const int button3 = 13;
const int button4 = 10;
const int button5 = 6;

const int button6 = 7;
const int button7 = 5;
const int button8 = 15;
const int button9 = 35;
const int button10 = 36;
const int button11 = 48;
const int button12 = 47;

const int joystick_x = 2;
const int joystick_y = 1;
int joystick_x_value = 0;
int joystick_y_value = 0;
const int joystick_btn = 38;
const int button_deload_time = 800;
unsigned long time_elapsed = 0;
int current_yaw_index = 0;
int desired_angle = 0;
int speed_modes[] = {140,170,200,250};
int increment_per_step[] = {20,17,12,8};

//TFT VARIABLES
const int TFT_CS = 18;
const int TFT_DC = 8;
const int TFT_RST = 9;
Adafruit_ST7735 display = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
bool menuMode = false;
byte option_index = 1;
byte speed_mode_index = 0;
bool movement_mode = 0;
String menuText[] = {"Movement Mode:","Tank Mode","World Position","Speed Setting:","Slow","Normal","Fast","Mad Max","Return"};
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
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


//Η εντολή OnDataRecv εκτελείται όταν το ESP32 S3 λάβει ένα πακέτο μέσω του πρωτόκολλου ESP-NOW
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  //Ενημέρωση του heartbeat timer
  heartbeat_timer = millis();
  //Αναγνώριση του πακέτου σύνδεσης του ESP32 S3 του τροχοφόρου οχήματος
  if(len == sizeof(bool)){
    //Ενημέρωση της κατάστασης της επικοινωνίας
    waiting_response = false;
    Serial.println("Received Heartbeat");
    return;
  }
  //Αντιγραφή όλων των δεδομένων του πακέτου του οχήματος
  memcpy(&rec_payload, incomingData, len);
  if(rec_payload.errorFlag != 0){
    display.fillScreen(ST77XX_BLACK);
    display.setCursor(0, 0);
    display.print("Error Code:");
    display.print(rec_payload.errorFlag);
    while(1);
  }
  //Αποθηκεύουμε την διαφορά περιστροφής του οχήματος και του χειριστηρίου 
  int new_car_yaw_index = rec_payload.carYaw - current_yaw + 45;
  //Μετατρέπουμε την γωνία σε ένα εύρος από 0 έως 360 μοίρες
  new_car_yaw_index += (new_car_yaw_index < 0) * 360;
  //Διαιρούμε με 90 ώστε να υπολογίσουμε τον δείκτη κατεύθυνσης 
  //Ο δείκτης αντιστοιχεί σε μία από τις 4 κατευθύνσεις  
  //0 = δεξιά
  //1 = μπροστά
  //2 = αριστερά
  //3 = πίσω
  //4 = δεξιά
  new_car_yaw_index = new_car_yaw_index/90;
  //Αν η νέα κατεύθυνση είναι διαφορετική από την προηγούμενη, ζωγραφίζουμε το βέλος της νέας κατεύθυνσης
  if(current_yaw_index != (new_car_yaw_index % 4)){
    //Χρωματίζουμε μαύρο όλο το πλαίσιο που τοποθετούμε το βέλος
    display.fillRect(80,60,62,62,ST77XX_BLACK);
    //Ζωγραφίζουμε το νέο βέλος με βάση τον δείκτη κατεύθυνσης
    display.drawBitmap(80,60,arrow_bitmaps[new_car_yaw_index % 4],BITMAP_WIDTH,BITMAP_HEIGHT,ST77XX_WHITE);
    current_yaw_index = new_car_yaw_index;
  }

  //Αποθήκευση της τάσης της μπαταρίας
  int battery_voltage = (rec_payload.batteryVoltageHighByte<<8) | rec_payload.batteryVoltageLowByte;
  //Καθώς κατά την χρήση του οχήματος
  if(abs(contr_payload.wheel_speed[0]) > 60 || abs(contr_payload.wheel_speed[1]) > 60){
    battery_voltage += 40;
  }
  if(batteryVoltageTable[battery_index][0] > battery_voltage){
    do{
      battery_index++;  
    }
    while(batteryVoltageTable[battery_index][0] > battery_voltage);
    display.fillRect(60,0,100,8,ST77XX_BLACK);
    display.fillRect(60,1,batteryVoltageTable[battery_index][1],6,batteryVoltageTable[battery_index][2]);
  }
  
  //Εκτύπωση πληροφοριών που λήφθηκαν από το όχημα
  Serial.print("Battery voltage:");
  Serial.println(battery_voltage);
  Serial.print("Left motor current mA:");
  Serial.println(rec_payload.leftCurrent);
  Serial.print("Right motor current mA:");
  Serial.println(rec_payload.rightCurrent);
  Serial.println();
}

void printDisplay(){
  display.fillScreen(ST77XX_BLACK);
  display.setCursor(0, 0);
  display.print("Robot Bat:");
  battery_index = 0;
  display.setCursor(0,8);
  //display connection status
  display.print("Movement Mode:");
  display.setCursor(84,8);
  if(movement_mode){
    display.print("World Pos");
  }
  else{
    display.print("Tank Mode");
  }
  display.setCursor(0,16);
  display.print("Speed Mode:");
  display.print(menuText[4 + speed_mode_index]);
  display.fillRect(80,60,62,62,ST77XX_BLACK);
  display.drawBitmap(80,60,arrow_bitmaps[current_yaw_index % 4],BITMAP_WIDTH,BITMAP_HEIGHT,ST77XX_WHITE);
}

void printMenu(){
  option_index = 1;
  display.fillScreen(ST77XX_BLACK);
  display.setCursor(56, 0);
  display.setTextSize(2);
  display.print("MENU");
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print(menuText[0]);
  display.setCursor(10, 24);
  display.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
  display.print(menuText[1]);
  display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display.setCursor(10, 32);
  display.print(menuText[2]);
  display.setCursor(0, 40);
  display.print(menuText[3]);
  display.setCursor(10, 48);
  display.print(menuText[4]);
  display.setCursor(10, 56);
  display.print(menuText[5]);
  display.setCursor(10, 64);
  display.print(menuText[6]);
  display.setCursor(10, 72);
  display.print(menuText[7]);
  display.setCursor(0, 80);
  display.print(menuText[8]);
  if(!movement_mode){
    display.setCursor(0,24);
  }
  else{
    display.setCursor(0,32);
  }
  display.print("o");
  display.setCursor(0,48 + speed_mode_index * 8);
  display.print("o");
}

void setup(){
  Serial.begin(115200);
  //Wire is used for connecting to the mpu6050 module
  Wire.begin(19,20);

  //setting up the tft display
  display.initR(INITR_BLACKTAB);

  //rotating by 270 degrees making the screen black and setting the curson on the top left corner of the screen
  display.setRotation(3);
  display.fillScreen(ST77XX_BLACK);
  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);

  //initializing mpu6050 module and dmp for filtering mpu measurements
  if(mpu.testConnection()) {
    Serial.println("Mpu connection is solid");
    }
  else{
    Serial.println("Mpu connection failed restarting automatically...");
    delay(1500);
    ESP.restart();
  }
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);

  //setting up pinmode for buttons and joystick input
  analogReadResolution(10);
  pinMode(button1,INPUT_PULLUP);
  pinMode(button2,INPUT_PULLUP);
  pinMode(button3,INPUT_PULLUP);
  pinMode(button4,INPUT_PULLUP);
  pinMode(button5,INPUT_PULLUP);
  pinMode(button6,INPUT_PULLUP);
  pinMode(button7,INPUT_PULLUP);
  pinMode(button8,INPUT_PULLUP);
  pinMode(button9,INPUT_PULLUP);
  pinMode(button10,INPUT_PULLUP);
  pinMode(button11,INPUT_PULLUP);
  pinMode(button12,INPUT_PULLUP);
  pinMode(joystick_btn,INPUT_PULLUP);
  

  //setting up the esp now protocol
  display.print("Initializing ESP NOW");
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    display.fillScreen(ST77XX_BLACK);
    display.setCursor(0, 0);
    display.print("Error Initializing");
    display.setCursor(0,16);
    display.print("Restarting The Device");
    delay(1500);
    ESP.restart();
  }
  //registering the callback function when sending data
  esp_now_register_send_cb(OnDataSent);
  
  //register peer 
  display.setCursor(0, 8);
  display.print("Adding Peer Info...");
  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    display.fillScreen(ST77XX_BLACK);
    display.setCursor(0, 0);
    display.print("Failed to add peer");
    display.setCursor(0, 8);
    display.print("Restarting The Device");
    delay(1500);
    ESP.restart();
  }
  //registering a callback function for when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  display.setCursor(0, 16);
  display.print("Looking for Connection...");
  while(waiting_response){
    esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)&waiting_response, sizeof(waiting_response));
    delay(100);
  }
  // printDisplay();
  // display.setCursor(0, 0);
  stabilization_time = millis();
  start_timer = millis();
  heartbeat_timer = millis();
  esp_task_wdt_config_t wdt_config = {.timeout_ms = WDT_TIMEOUT * 1000, .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, .trigger_panic = true};
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
}
 
void loop(){
  if(menuMode){
    if(last_status_connected == 0){
      printMenu();
      last_status_connected = 1;
      contr_payload.wheel_speed[0] = 0;
      contr_payload.wheel_speed[1] = 0;
      memset(contr_payload.buttons,0,sizeof(contr_payload.buttons)); 
    }
    joystick_y_value = analogRead(joystick_y) - 512;
    if(abs(joystick_y_value) <= 150){
        joystick_y_value = 0;
    }
    Serial.print("Y Val:");
    Serial.println(joystick_y_value);
    byte temp_value = option_index;
    if(joystick_y_value < 0){
      option_index++;
      if(option_index == 3){
        option_index++;
      }
      else if(option_index == 9){
        option_index = 1;
      }
      Serial.print("Temp value:");
      Serial.println(temp_value);
      Serial.print("Option index:");
      Serial.println(option_index);
      display.fillRect(10 - 10*int(temp_value/8),16 + 8 * temp_value,84,8,ST77XX_BLACK);
      display.setCursor(10 - 10*int(temp_value/8),16 + 8 * temp_value);
      display.print(menuText[temp_value]);
      display.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
      display.setCursor(10 - 10*int(option_index/8),16 + 8 * option_index);
      display.print(menuText[option_index]);
      display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    }
    else if(joystick_y_value > 0){
      option_index--;
      if(option_index == 3){
        option_index--;
      }
      else if(option_index == 0){
        option_index = 8;
      }
      display.fillRect(10 - 10*int(temp_value/8),16 + 8 * temp_value,84,8,ST77XX_BLACK);
      display.setCursor(10 - 10*int(temp_value/8),16 + 8 * temp_value);
      display.print(menuText[temp_value]);
      display.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
      display.setCursor(10 - 10*int(option_index/8),16 + 8 * option_index);
      display.print(menuText[option_index]);
      display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    }
    
    if(digitalRead(joystick_btn) == HIGH && millis() - time_elapsed > button_deload_time){
      time_elapsed = millis();
      if((option_index == 1 || option_index == 2) && movement_mode != (option_index - 1)){
        display.fillRect(0,24 + 8*movement_mode,6,8,ST77XX_BLACK);
        movement_mode = !movement_mode;
        display.setCursor(0,24 + 8*movement_mode);
        display.print("o");
      }
      else if((option_index == 4 || option_index == 5 || option_index == 6 || option_index == 7) && speed_mode_index != (option_index - 4)){
        display.fillRect(0,48 + (speed_mode_index * 8),6,8,ST77XX_BLACK);
        speed_mode_index = option_index - 4;
        display.setCursor(0,48 + (speed_mode_index * 8));
        display.print("o");
      }
      else if(option_index == 8){
        menuMode = false;
        last_status_connected = 1;
      }
    }
    Serial.print("Speed Mode:");
    Serial.println(menuText[4+speed_mode_index]);
    Serial.print("Movement Mode:");
    Serial.println(movement_mode);
    esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)&contr_payload, sizeof(contr_payload));
    esp_task_wdt_reset();
    delay(100);
    return;
  }

  if(millis() - heartbeat_timer > 500){
    Serial.println("No heartbeat");
    if(last_status_connected != 2){
      last_status_connected = 2;
      display.fillScreen(ST77XX_BLACK);
      display.setCursor(0, 0);
      display.print("Disconnected...");
      display.setCursor(0, 8);
      display.print("Waiting for connection...");
      Serial.println("DISCONNECTED");
    }
    esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)&waiting_response, sizeof(waiting_response));
    esp_task_wdt_reset();
    delay(100);
  }
  else{
    if(last_status_connected != 0){
      printDisplay();
      last_status_connected = 0;
    }
    contr_payload.buttons[0] = digitalRead(button1);
    contr_payload.buttons[1] = digitalRead(button2);
    contr_payload.buttons[2] = digitalRead(button3);
    contr_payload.buttons[3] = digitalRead(button4);
    contr_payload.buttons[4] = digitalRead(button5);
    contr_payload.buttons[5] = digitalRead(button6);
    contr_payload.buttons[6] = digitalRead(button7);
    contr_payload.buttons[7] = digitalRead(button8);
    contr_payload.buttons[8] = digitalRead(button9);
    contr_payload.buttons[9] = digitalRead(button10);
    contr_payload.buttons[10] = digitalRead(button11);
    contr_payload.buttons[11] = digitalRead(button12);
    
    if(digitalRead(joystick_btn) == HIGH && millis() - time_elapsed > button_deload_time){
      time_elapsed = millis();
      menuMode = !menuMode;
      Serial.println("Activated Menu mode");
    }
    joystick_x_value = 1023-analogRead(joystick_x) - 512;
    joystick_y_value = analogRead(joystick_y) - 512;
    if(abs(joystick_x_value) <= 70){
        joystick_x_value = 0;
      }

    if(abs(joystick_y_value) <= 70){
        joystick_y_value = 0;
      }
    // Serial.print("X joystick:");
    // Serial.println(joystick_x_value);
    // Serial.print("Y joystick:");
    // Serial.println(joystick_y_value);
    if(mpu.dmpGetCurrentFIFOPacket(buffer)){
      mpu.dmpGetQuaternion(&quaternion, buffer);
      mpu.dmpGetGravity(&gravity, &quaternion);
      mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);
    }
    //Here we calculate the yaw of the controller, first in radians. We invert the value so that clockwise rotation decreases the value (by default the mpu6050 module does the opposite CW movement increases the value)
    //then we use *180/M_PI to turn the radians into degrees from -180 to 180 then we transform it to 0-360 degrees since the joystick and the rc car MPU6050 also use the same logic 
    current_yaw = -ypr[0];
    // Serial.print("Current yaw rad:");
    // Serial.print(current_yaw);
    current_yaw = current_yaw * (180 / M_PI);
    // Serial.print("Current yaw -180 180:");
    // Serial.print(current_yaw);
    current_yaw += (current_yaw<0)*360;
    // Serial.print("Current yaw 0 360:");
    // Serial.print(current_yaw);
    //the mpu6050 module uses a DMP (digital motion processor) which takes the data from the gyroscope and acceletometer and applies some calculations instead of them needed to be added by us, like kalman filters etc
    //till the DMP warms up and has stabilized we use the current minus the previous yaw value to calculate the starting yaw
    if(millis() - start_timer < stabilization_time){
        starting_yaw += 1 * (current_yaw - prev_yaw);
        prev_yaw = current_yaw;
      }
    //we subtract the starting yaw from the current, to find the differential yaw Dyaw = yawFinal-yawStarting
    current_yaw = current_yaw - starting_yaw;
    current_yaw += ((current_yaw < 0) * 360);
    // Serial.print("Controller Yaw 0-360:");
    // Serial.println(current_yaw);

    // Serial.print("Desired Angle 0 360:");
    // Serial.println(desired_angle);
    
    if(joystick_x_value !=0 || joystick_y_value != 0){
      if(movement_mode){
        DirectionalMovement(joystick_x_value,joystick_y_value);
      }
      else{
        TankControlMovement(joystick_x_value,joystick_y_value);
      }
    }
    else{
      contr_payload.wheel_speed[0] = joystick_x_value;
      contr_payload.wheel_speed[1] = joystick_y_value;
    }
    esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)&contr_payload, sizeof(contr_payload));
    esp_task_wdt_reset();
    delay(100);
    }
}

void TankControlMovement(int joystickX,int joystickY){
  //Υπολογισμός μέτρου του διανύσματος του σημείου Χ,Υ από το 0,0
  int vector_length = (joystickX * joystickX) + (joystickY * joystickY);
  float movement_vector_length = sqrt(vector_length);
  //Αν η ταχύτητα είναι πολύ χαμηλή το όχημα δε κινείται οπότε ορίζουμε ένα ελάχιστο threshold
  float movement_vector_magnitude = constrain(movement_vector_length/512,0.4,1);
  float threshold = 0.5;
  //Υπολογισμός γωνίας με τον άξονα Χ
  int desired_angle = int(atan2((double)joystickY,(double)joystickX) * 180/M_PI); 
  if(abs(desired_angle) > 135 && movement_vector_magnitude > threshold){
    //Η μέγιστη ταχύτητα που θέλουμε να φτάσουν οι τροχοί
    int target_speed = speed_modes[speed_mode_index] * movement_vector_magnitude;
    //Η ταχύτητα των αριστερών τροχών
    contr_payload.wheel_speed[0] -= increment_per_step[speed_mode_index];
    if(contr_payload.wheel_speed[0] < -target_speed){
      contr_payload.wheel_speed[0] = -target_speed;
    }
    //Η ταχύτητα των δεξιών τροχών
    contr_payload.wheel_speed[1] += increment_per_step[speed_mode_index];
    if(contr_payload.wheel_speed[1] > target_speed){
      contr_payload.wheel_speed[1] = target_speed;
    }
    //Εκτύπωση αριθμών
    Serial.print("Target Speed:");
    Serial.println(target_speed);
    Serial.print("Left motor Speed:");
    Serial.println(contr_payload.wheel_speed[0]);
    Serial.print("Right motor Speed:");
    Serial.println(contr_payload.wheel_speed[1]);
  }
  else if(abs(desired_angle) < 45 && movement_vector_magnitude > threshold){
    //Η μέγιστη ταχύτητα που θέλουμε να φτάσουν οι τροχοί
    int target_speed = speed_modes[speed_mode_index] * movement_vector_magnitude;
    //Η ταχύτητα των αριστερών τροχών
    contr_payload.wheel_speed[0] += increment_per_step[speed_mode_index];
    if(contr_payload.wheel_speed[0] > target_speed){
      contr_payload.wheel_speed[0] = target_speed;
    }
    //Η ταχύτητα των δεξιών τροχών
    contr_payload.wheel_speed[1] -= increment_per_step[speed_mode_index];
    if(contr_payload.wheel_speed[1] < -target_speed){
      contr_payload.wheel_speed[1] = -target_speed;
    }
    //Εκτύπωση αριθμών
    Serial.print("Target Speed:");
    Serial.println(target_speed);
    Serial.print("Left motor Speed:");
    Serial.println(contr_payload.wheel_speed[0]);
    Serial.print("Right motor Speed:");
    Serial.println(contr_payload.wheel_speed[1]);
  }
  else{
    int target_speed = speed_modes[speed_mode_index]* movement_vector_magnitude;
    if(desired_angle > 0){
      //Η ταχύτητα των αριστερών τροχών
      contr_payload.wheel_speed[0] += increment_per_step[speed_mode_index];
      //Η ταχύτητα των δεξιών τροχών
      contr_payload.wheel_speed[1] += increment_per_step[speed_mode_index];
      if(contr_payload.wheel_speed[0] > target_speed){
        contr_payload.wheel_speed[0] = target_speed;
        contr_payload.wheel_speed[1] = target_speed;
      }
    }
    else{
      //Η ταχύτητα των αριστερών τροχών
      contr_payload.wheel_speed[0] -= increment_per_step[speed_mode_index];
      //Η ταχύτητα των δεξιών τροχών
      contr_payload.wheel_speed[1] -= increment_per_step[speed_mode_index];
      if(contr_payload.wheel_speed[0] < -target_speed){
        contr_payload.wheel_speed[0] = -target_speed;
        contr_payload.wheel_speed[1] = -target_speed;
      }
    }
    //Εκτύπωση αριθμών
    Serial.print("Target Speed:");
    Serial.println(target_speed);
    Serial.print("Left motor Speed:");
    Serial.println(contr_payload.wheel_speed[0]);
    Serial.print("Right motor Speed:");
    Serial.println(contr_payload.wheel_speed[1]);
  }
}

void DirectionalMovement(int joystickX,int joystickY){
  //Υπολογισμός γωνίας με τον άξονα Χ
  int desired_angle = int(atan2((double)joystickY,(double)joystickX)* 180/M_PI); 
  //Μετατροπή γωνίας σε εύρος από 0 έως 360 μοίρες
  desired_angle += (desired_angle < 0) * 360;
  //Υπολογισμός μέτρου του διανύσματος του σημείου Χ,Υ από το 0,0
  int vector_length = (joystickX * joystickX) + (joystickY * joystickY);
  float movement_vector_length = sqrt(vector_length);
  float movement_vector_magnitude = constrain(movement_vector_length/512,0,1);
  //Υπολογισμός διαφοράς γωνίας μεταξύ οχήματος και γωνίας περιστροφής που ορίζει ο χρήστης
  int angle = int(rec_payload.carYaw - (desired_angle + current_yaw));
  angle = angle%360;
  //Μετατροπή γωνίας σε εύρος -180 έως 180 μοίρες
  if(angle > 180){angle -= 360;}
  if(angle < -180){angle += 360;}
  int target_speed = speed_modes[speed_mode_index] * movement_vector_magnitude;
  if(movement_vector_magnitude > 0.5){
    if(abs(angle) > 10){
      //Το 0 αντιστοιχεί σε φορά περιστροφής αντίστροφη του ρολογιού ενώ το 1 ίδια με αυτού
      bool rotate_direction = (angle < 0); 
      float temp_rotation_speed = abs(angle)/180.0;
      float rotation_speed = constrain(temp_rotation_speed,0.6,1);
      if(rotate_direction){
        //Η ταχύτητα των αριστερών τροχών
        contr_payload.wheel_speed[0] -= increment_per_step[speed_mode_index];
        if(contr_payload.wheel_speed[0] < -target_speed){
          contr_payload.wheel_speed[0] = -target_speed;
        }
        //Η ταχύτητα των δεξιών τροχών
        contr_payload.wheel_speed[1] += increment_per_step[speed_mode_index];
        if(contr_payload.wheel_speed[1] > target_speed){
          contr_payload.wheel_speed[1] = target_speed;
        }
        //Εκτύπωση αριθμών
        Serial.print("Left motor Speed:");
        Serial.println(contr_payload.wheel_speed[0]);
        Serial.print("Right motor Speed:");
        Serial.println(contr_payload.wheel_speed[1]);
      }
      else{
        //Η ταχύτητα των αριστερών τροχών
        contr_payload.wheel_speed[0] += increment_per_step[speed_mode_index];
        if(contr_payload.wheel_speed[0] > target_speed){
          contr_payload.wheel_speed[0] = target_speed;
        }
        //Η ταχύτητα των δεξιών τροχών
        contr_payload.wheel_speed[1] -= increment_per_step[speed_mode_index];
        if(contr_payload.wheel_speed[1] < -target_speed){
          contr_payload.wheel_speed[1] = -target_speed;
        }
        //Εκτύπωση αριθμών
        Serial.print("Left motor Speed:");
        Serial.println(contr_payload.wheel_speed[0]);
        Serial.print("Right motor Speed:");
        Serial.println(contr_payload.wheel_speed[1]);
      }
    }
    else{
        //Η ταχύτητα των αριστερών τροχών
        contr_payload.wheel_speed[0] += increment_per_step[speed_mode_index];
        //Η ταχύτητα των δεξιών τροχών
        contr_payload.wheel_speed[1] += increment_per_step[speed_mode_index];
        if(contr_payload.wheel_speed[0] > target_speed){
          contr_payload.wheel_speed[0] = target_speed;
          contr_payload.wheel_speed[1] = target_speed;
        }
      
      //Εκτύπωση αριθμών
      Serial.print("Left motor Speed:");
      Serial.println(contr_payload.wheel_speed[0]);
      Serial.print("Right motor Speed:");
      Serial.println(contr_payload.wheel_speed[1]);
    }
  }
}
