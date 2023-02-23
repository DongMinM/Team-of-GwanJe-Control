#include <Timer.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Servo.h>
#define Rx  0
#define Tx  1
#define SLAVE_ADDRESS 0x04 // Set the I2C address of the slave board

const int In1 = 5;
const int In2 = 4;
const int servoPin = 9;
const int servoMin = 0;
const int servoMax = 180;
const int servoDelay = 30;
const int speed1 = 6;
const int goal_height = 10;

int header_pass = 0;
int index = 0;
uint8_t data;
int32_t raw_imu[65];
int sensorReady = 0;
float ax, ay, az, a_total, wx, wy, wz, roll, pitch, yaw, height, pre_height, seconds, minutes, hours, tenMillis, currentMillis;
int32_t raw_longitude, raw_latitude;
float lon, lat, speeds;
byte tenMillis_b[sizeof(tenMillis)], seconds_b[sizeof(seconds)], minutes_b[sizeof(minutes)], hours_b[sizeof(hours)],ax_b[sizeof(ax)], ay_b[sizeof(ay)], az_b[sizeof(az)], wx_b[sizeof(wx)], wy_b[sizeof(wy)], wz_b[sizeof(wz)], roll_b[sizeof(roll)], pitch_b[sizeof(pitch)], yaw_b[sizeof(yaw)], height_b[sizeof(height)], lon_b[sizeof(lon)], lat_b[sizeof(lat)], speeds_b[sizeof(speeds)];
int angle = 120;
char header = 'A';

SoftwareSerial imuSerial(Rx,Tx);
Timer t;
Servo myservo;

void communication(){
  memcpy(ax_b, &ax, sizeof(ax)); memcpy(ay_b, &ay, sizeof(ay)); memcpy(az_b, &az, sizeof(az)); memcpy(wx_b, &wx, sizeof(wx)); memcpy(wy_b, &wy, sizeof(wy)); memcpy(wz_b, &wz, sizeof(wz)); memcpy(roll_b, &roll, sizeof(roll)); memcpy(pitch_b, &pitch, sizeof(pitch)); memcpy(yaw_b, &yaw, sizeof(yaw)); memcpy(height_b, &height, sizeof(height)); memcpy(lon_b, &lon, sizeof(lon)); memcpy(lat_b, &lat, sizeof(lat)); memcpy(speeds_b, &speeds, sizeof(speeds));
  memcpy(hours_b, &hours, sizeof(hours)); memcpy(minutes_b, &minutes, sizeof(minutes)); memcpy(seconds_b, &seconds, sizeof(seconds)); memcpy(tenMillis_b, &tenMillis, sizeof(tenMillis));
  Wire.beginTransmission(SLAVE_ADDRESS); // Begin I2C transmission to slave board
  Wire.write(header);
  Wire.write(ax_b, sizeof(ax)); Wire.write(ay_b, sizeof(ay)); Wire.write(az_b, sizeof(az));
  Wire.write(wx_b, sizeof(wx)); Wire.write(wy_b, sizeof(wy)); Wire.write(wz_b, sizeof(wz));
  Wire.write(roll_b, sizeof(roll));
  Wire.endTransmission();
  Wire.beginTransmission(SLAVE_ADDRESS);
   Wire.write(pitch_b, sizeof(pitch));Wire.write(yaw_b, sizeof(yaw));
  Wire.write(height_b, sizeof(height)); Wire.write(speeds_b, sizeof(speeds));
  Wire.write(lon_b, sizeof(lon)); Wire.write(lat_b, sizeof(lat));
  Wire.write(hours_b, sizeof(hours)); Wire.write(minutes_b, sizeof(minutes));
  Wire.endTransmission(); // End I2C transmission
  Wire.beginTransmission(SLAVE_ADDRESS);
   Wire.write(seconds_b, sizeof(seconds));Wire.write(tenMillis_b, sizeof(tenMillis));

  Wire.endTransmission();
}

void rw_controller(){
  
}

void parachute_trigger(){
  if (a_total < 1.5 and (height < pre_height or height > goal_height)){
     //Move servo from minimum angle to maximum angle
     myservo.write(angle);
    }
}

void setup() {  
  pinMode(servoPin, OUTPUT);
  pinMode(speed1, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  myservo.attach(9);

  Serial.begin(115200);
  Wire.begin(); // Initialize I2C bus
  imuSerial.begin(57600);
  
  t.every(60, communication);
  t.every(20, rw_controller);
  t.every(100, parachute_trigger);
  }



void loop() {
 t.update();
 if(sensorReady==1){
    get_alldata();
    sensorReady=0;
   }
}



void get_alldata(){
    ax = (raw_imu[3] << 8 | raw_imu[2])/32768.0*16;
    ay = (raw_imu[5] << 8 | raw_imu[4])/32768.0*16;
    az = (raw_imu[7] << 8 | raw_imu[6])/32768.0*16;
    if(ax>16) ax=ax-32.0;
    if(ay>16) ay=ay-32.0;
    if(az>16) az=az-32.0;

    a_total = sqrt(ax*ax+ay*ay+az*az);
    
    wx = (raw_imu[14] << 8 | raw_imu[13])/32768.0*2000;
    wy = (raw_imu[16] << 8 | raw_imu[15])/32768.0*2000;
    wz = (raw_imu[18] << 8 | raw_imu[17])/32768.0*2000;
    if(wx>2000) wx=wx-4000.0;
    if(wy>2000) wy=wy-4000.0;
    if(wz>2000) wz=wz-4000.0;

    roll  = (raw_imu[25] << 8 | raw_imu[24])/32768.0*180;
    pitch = (raw_imu[27] << 8 | raw_imu[26])/32768.0*180;
    yaw   = (raw_imu[29] << 8 | raw_imu[28])/32768.0*180;
    if(roll>180) roll=roll-360;
    if(pitch>180) pitch=pitch-360;
    if(yaw>180) yaw=yaw-360;
    
    pre_height = height;
    height   = (raw_imu[42] << 24| raw_imu[41] << 16| raw_imu[40] << 8 | raw_imu[39])/100;

    raw_longitude = (raw_imu[49] << 24 | raw_imu[48] << 16 | raw_imu[47] << 8 | raw_imu[46]);
    raw_latitude  = (raw_imu[53] << 24 | raw_imu[52] << 16 | raw_imu[51] << 8 | raw_imu[50]);
    speeds = (raw_imu[64] << 24 | raw_imu[63] << 16 | raw_imu[62] << 8 | raw_imu[61])/1000*3.6;
    
    lon = (raw_longitude/10000000) + double(raw_longitude%10000000)/6000000.0;
    lat  = (raw_latitude/10000000) + double(raw_latitude %10000000)/6000000.0;
    
}



void serialEvent(){
  if(imuSerial.available() > 0 and sensorReady == 0){
    data = imuSerial.read();

    
    if (data == 85 and header_pass == 0 and index == 0){
      raw_imu[index] = data;
      index = 1;

    }
    else if(data == 81 and header_pass == 0 and index ==1){
      raw_imu[index]=data;
      index = 2;
      header_pass = 1;
      
      currentMillis = millis();
      
      hours = int(currentMillis / 3600000.0);
      minutes = int((currentMillis-hours*3600000.0)/60000.0);
      seconds = int((currentMillis-hours*3600000.0-minutes*60000)/ 1000.0);
      tenMillis = int((currentMillis-hours*3600000.0-minutes*60000-seconds*1000.0)/10.0);

    }
    else if(header_pass == 1 and index < 65){
      raw_imu[index] = data;
      index = index+1;

    }
    else if(header_pass == 1 and index >=65){
      header_pass = 0;
      index = 0;
      sensorReady = 1;
    }
  }
  else{
      header_pass = 0;
      index = 0;
  }
}
