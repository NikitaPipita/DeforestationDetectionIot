#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>

const char *ssid = "USER-PC_Network";
const char *pass = "ak1u-uu1d-csnc";
WiFiServer server(80);

const int serial_ID = 22;
const String serial_password = "qwerty";
const String authentication_endpoint = "https://deforestation-proj.herokuapp.com/iot/sessions";
const String refresh_token_endpoint = "https://deforestation-proj.herokuapp.com/iot/refresh";
const String send_signal_endpoint = "https://deforestation-proj.herokuapp.com/iot/connected/state";
String access_token;
String refresh_token;

const int MPU_addr = 0x68;  // I2C-address MPU-6050

//const int32_t calibration_period = 86400000; // Day
const int32_t calibration_period = 60000; // Minute
const int32_t time_to_calibrate = 3000;
const int16_t calibration_delay = 1000;

const int16_t min_XY = -16384;
const int16_t max_XY = 16383;
const int32_t check_delay = 3000;
const int16_t normal_deviation_percentage = 50;
const int8_t recheck_times = 5;

int16_t init_AcX, init_AcY;
int16_t cur_AcX, cur_AcY;

int32_t milliseconds_since_calibration = 0;

bool globalIotStateOk = true;

void setup() {
  Serial.begin(115200);
    
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  Serial.println("Wrote to IMU");
  Serial.println("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address is : ");
  Serial.println(WiFi.localIP());
  server.begin();
  Serial.println("Server started");

  authentication();
  calibrate();
}

void loop(){
  if (milliseconds_since_calibration > calibration_period) {
    calibrate();
  }
  mpu_read_AcX_AcY(cur_AcX, cur_AcY);
  bool isStateOk = check_state();
  if (!isStateOk && globalIotStateOk) {
    delay(check_delay);
    milliseconds_since_calibration += check_delay;
    isStateOk = rechecking_excess_of_deviation();
    if (!isStateOk) {
      send_signal();
      globalIotStateOk = false;
    }
  }
  delay(check_delay);
  milliseconds_since_calibration += check_delay;
}

bool rechecking_excess_of_deviation() {
  int8_t counter = 0;
  bool iotRecheckStatus = false;
  
  while(counter < recheck_times) {
    mpu_read_AcX_AcY(cur_AcX, cur_AcY);
    iotRecheckStatus = check_state();
    if (iotRecheckStatus) {
      break;
    }
    counter++;
    delay(check_delay);
    milliseconds_since_calibration += check_delay;
  }

  return iotRecheckStatus;
}

bool check_state() {
  int16_t initX = int16_t_percentage_one_number_from_another(init_AcX, max_XY);
  int16_t initY = int16_t_percentage_one_number_from_another(init_AcY, max_XY);
  int16_t curX = int16_t_percentage_one_number_from_another(cur_AcX, max_XY);
  int16_t curY = int16_t_percentage_one_number_from_another(cur_AcY, max_XY);

  bool isStateOk = true;
  int16_t difference_X = initX - curX;
  if (difference_X <= -normal_deviation_percentage || normal_deviation_percentage <= difference_X){
    Serial.print("\nDeviation in X by: ");
    Serial.print(difference_X);
    Serial.print("%\n");
    isStateOk = false;
  }

  int16_t difference_Y = initY - curY;
  if (difference_Y <= -normal_deviation_percentage || normal_deviation_percentage <= difference_Y){
    Serial.print("\nDeviation in Y by: ");
    Serial.print(difference_Y);
    Serial.print("%\n");
    isStateOk = false;
  }

  return isStateOk;
}

int16_t int16_t_percentage_one_number_from_another(int16_t first, int16_t second) {
  return (first * 100) / second;
}

//Calibrates initial AcX and AcY values
void calibrate() {
  Serial.print("\nStarted calibration...");
  int32_t milliseconds_since_calibration_start = 0;
  int32_t calibr_AcX = 0;
  int32_t calibr_AcY = 0;
  int16_t counter = 0;
  while(milliseconds_since_calibration_start < time_to_calibrate) {
    mpu_read_AcX_AcY(cur_AcX, cur_AcY);
    if (cur_AcX < min_XY) {
      cur_AcX = min_XY;
    } else if (cur_AcX > max_XY) {
      cur_AcX = max_XY;
    }

    if (cur_AcY < min_XY) {
      cur_AcY = min_XY;
    } else if (cur_AcY > max_XY) {
      cur_AcY = max_XY;
    }

    calibr_AcX += cur_AcX;
    calibr_AcY += cur_AcY;
    counter++;
    delay(calibration_delay);
    milliseconds_since_calibration_start += calibration_delay;
  }
  init_AcX = (calibr_AcX / counter);
  init_AcY = (calibr_AcY / counter);
  milliseconds_since_calibration = 0;
  globalIotStateOk = true;

  Serial.print("\nInitial Accelerometer Values: \n");
  Serial.print("AcX: ");
  Serial.print(init_AcX);
  Serial.print("\nAcY: ");
  Serial.print(init_AcY);
  Serial.print("\n");
}

void mpu_read_AcX_AcY(int16_t &AcX, int16_t &AcY){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // started from register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,4,true);  // request 4 registers
  AcX = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
}

void send_signal(){
  int signalResponseCode = send_signal_to_server();
  if (signalResponseCode != 200) {
    refresh_tokens();
    signalResponseCode = send_signal_to_server();
    if (signalResponseCode != 200) {
      authentication();
      send_signal_to_server();
    }
  }
}

int send_signal_to_server(){
  if(WiFi.status()== WL_CONNECTED){
    HTTPClient http;

    Serial.println("\nSending IoT active signal to th server...");
    http.begin(send_signal_endpoint);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", String(String("Bearer ") + access_token));
    int httpResponseCode = http.PUT(String(String("{\"iot_id\":") + String(serial_ID) + String(",\"iot_state\":\"active\"}")));

    Serial.println("HTTP Response code: ");
    Serial.println(httpResponseCode);
      
    http.end();
    return httpResponseCode;
  }
  else {
    Serial.println("WiFi Disconnected");
  }
  return 1;
}

int refresh_tokens() {
  if(WiFi.status()== WL_CONNECTED){
    HTTPClient http;

    Serial.println("\nToken refresh...");
    http.begin(refresh_token_endpoint);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(String(String("{\"token\":\"") + refresh_token + String("\"}")));

    if(httpResponseCode == 201) {
      JSONVar jsonData = JSON.parse(http.getString());
      if (JSON.typeof(jsonData) == "undefined") {
        Serial.println("Parsing input failed!");
        return 1;
      }
      JSONVar access_value = jsonData["access_token"];
      JSONVar refresh_value = jsonData["refresh_token"];
      access_token = access_value;
      refresh_token = refresh_value;
    }

    Serial.println("HTTP Response code: ");
    Serial.println(httpResponseCode);
      
    http.end();
    return httpResponseCode;
  }
  else {
    Serial.println("WiFi Disconnected");
  }
  return 1;
}

int authentication(){
  if(WiFi.status()== WL_CONNECTED){
    HTTPClient http;

    Serial.println("\nIoT authentication...");
    http.begin(authentication_endpoint);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(String(String("{\"serial_id\":") + String(serial_ID) + String(",\"serial_password\":\"") + String(serial_password) + String("\"}")));

    if(httpResponseCode == 200) {
      JSONVar jsonData = JSON.parse(http.getString());
      if (JSON.typeof(jsonData) == "undefined") {
        Serial.println("Parsing input failed!");
        return 1;
      }
      JSONVar access_value = jsonData["access_token"];
      JSONVar refresh_value = jsonData["refresh_token"];
      access_token = access_value;
      refresh_token = refresh_value;
    }

    Serial.println("HTTP Response code: ");
    Serial.println(httpResponseCode);
      
    http.end();
    return httpResponseCode;
  }
  else {
    Serial.println("WiFi Disconnected");
  }
  return 1;
}
