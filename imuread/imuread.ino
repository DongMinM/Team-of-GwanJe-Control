#include <SoftwareSerial.h>
#define Rx  0
#define Tx  1

int header_pass = 0;
int index = 0;
uint8_t data;
uint32_t raw_imu[65];
int fin = 0;
float ax,ay,az,wx,wy,wz,roll,pitch,yaw,height;
uint32_t raw_longitude, raw_latitude;
float lon, lat, speeds;

SoftwareSerial imuSerial(Rx,Tx);

void setup() {
  Serial.begin(9600);
  imuSerial.begin(115200);
  

}

void loop() {
  
  if(fin==1){
    get_alldata();
    Serial.println(roll);
    fin=0;
  }
}

void get_alldata(){
    ax = (raw_imu[3] << 8 | raw_imu[2])/32768.0*16;
    ay = (raw_imu[5] << 8 | raw_imu[4])/32768.0*16;
    az = (raw_imu[7] << 8 | raw_imu[6])/32768.0*16;

    wx = (raw_imu[14] << 8 | raw_imu[13])/32768.0*2000;
    wy = (raw_imu[16] << 8 | raw_imu[15])/32768.0*2000;
    wz = (raw_imu[18] << 8 | raw_imu[17])/32768.0*2000;

    roll  = (raw_imu[25] << 8 | raw_imu[24])/32768.0*180;
    pitch = (raw_imu[27] << 8 | raw_imu[26])/32768.0*180;
    yaw   = (raw_imu[29] << 8 | raw_imu[28])/32768.0*180;

    height   = (raw_imu[42] << 24| raw_imu[41] << 16| raw_imu[40] << 8 | raw_imu[39])/100;

    raw_longitude = (raw_imu[49] << 24 | raw_imu[48] << 16 | raw_imu[47] << 8 | raw_imu[46]);
    raw_latitude  = (raw_imu[53] << 24 | raw_imu[52] << 16 | raw_imu[51] << 8 | raw_imu[50]);

    speeds = (raw_imu[64] << 24 | raw_imu[63] << 16 | raw_imu[62] << 8 | raw_imu[61])/1000*3.6;
    
    lon = (raw_longitude/10000000) + double(raw_longitude%10000000)/6000000.0;
    lat  = (raw_latitude/10000000) + double(raw_latitude %10000000)/6000000.0;
}

void serialEvent(){
      if(imuSerial.available() > 0 and fin == 0){
      data = imuSerial.read();

    
    if (data == 85 and header_pass == 0 and index == 0){
      raw_imu[index] = data;
      index = 1;

    }
    else if(data == 81 and header_pass == 0 and index ==1){
      raw_imu[index]=data;
      index = 2;
      header_pass = 1;
 
    }
    else if(header_pass == 1 and index < 65){
      raw_imu[index] = data;
      index = index+1;

    }
    else if(header_pass == 1 and index >=65){
      header_pass = 0;
      index = 0;
      fin = 1;
    }
  }
  else{
      header_pass = 0;
      index = 0;
      fin = 0;
  }
}
