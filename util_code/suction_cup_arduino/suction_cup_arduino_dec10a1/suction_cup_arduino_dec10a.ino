#include <SoftwareSerial.h>
unsigned char suction_on[8] = {0x01, 0x06, 0x00, 0x02, 0x00, 0x01, 0xE9, 0xCA};  //suction
unsigned char suction_off[8] = {0x01, 0x06, 0x00, 0x02, 0x00, 0x02, 0xA9, 0xCB};  //release

String data = ""; // 接收到的16进制字符串
SoftwareSerial suctionSerial(19, 18);  // RX, TX

//float getFeedback(String temperature);  //

void setup()
{
  suctionSerial.begin(115200);
  Serial.begin(115200);
}

void loop()
{
  delay(500);  // 
  for (int i = 0 ; i < 8; i++) {  // 
    suctionSerial.write(suction_on[i]);   // 
  }
  delay(100);  // 
  data = "";
  Serial.println('1');
  while (suctionSerial.available()) {//
    Serial.println('2');
    unsigned char in = (unsigned char)suctionSerial.read();  // 
    Serial.print(in, HEX);
    Serial.print(',');
    data += in;
    data += ',';
  }

//  if (data.length() > 0) { //先输出一下接收到的数据
//    Serial.println();
//    Serial.println(data);
//    Serial.print(getTemp(data));
//    Serial.println("Temp");
//  }
}
//
//float getFeedback(String temp) {
//  int commaPosition = -1;
//  String info[9];  // 
//  for (int i = 0; i < 9; i++) {
//    commaPosition = temp.indexOf(',');
//    if (commaPosition != -1)
//    {
//      info[i] = temp.substring(0, commaPosition);
//      temp = temp.substring(commaPosition + 1, temp.length());
//    }
//    else {
//      if (temp.length() > 0) {  //
//        info[i] = temp.substring(0, commaPosition);
//      }
//    }
//  }
//  return (info[3].toInt() * 256 + info[4].toInt()) / 10.0;
//}
