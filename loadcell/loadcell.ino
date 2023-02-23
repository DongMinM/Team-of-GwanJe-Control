#include "HX711.h"
HX711 scale;
float calibration_factor = -44100; //로드셀 종류나 상황에 따라 적당한 값으로 시작 + or - 어떤 값이 될지 모름


void setup() {
  Serial.begin(115200);
  scale.begin(2, 3); //dt 2 sck3
//  Serial.println("HX711 /calibration sketch");
//  Serial.println("Remove all weight from scale");
//  Serial.println("After readings begin, place known weight on scale");
//  Serial.println("Press + or a to increase calibration factor");
//  Serial.println("Press - or z to decrease calibration factor");
  scale.set_scale();
  scale.tare(); //Reset the scale to 0
  long zero_factor = scale.read_average(); //Get a baseline reading
//  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
//  Serial.println(zero_factor);
}



void loop() {
  scale.set_scale(calibration_factor); //Adjust to this calibration factor
//  Serial.print("Reading: ");
  Serial.print(scale.get_units(), 3); // 뒤에 있는 숫자는 몇 자리까지 보여줄지입니다.
//  Serial.print(" kg");
//  Serial.print(" calibration_factor: ");
//  Serial.print(calibration_factor);
  Serial.println();

//  if (Serial.available())
//
//  {
//
//    char temp = Serial.read(); //시리얼 모니터상에서 캘리를 조정할 수 있다 대신 10씩 움직여지기 때문에 캘리가 크게 변해야 하는 경우는 코드에서 직접 숫자를 변경하고 재실행하는 게 훨씬 빠릅니다.
//
//    if (temp == '+' || temp == 'a')
//
//      calibration_factor += 100;
//
//    else if (temp == '-' || temp == 'z')
//
//      calibration_factor -= 100;
//
//  }

}
