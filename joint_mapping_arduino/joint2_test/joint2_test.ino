#include <Servo.h>

Servo joint2;  // 创建舵机对象

void setup() {
  Serial.begin(9600);  // 初始化串口
  joint2.attach(10);    // 舵机连接到数字引脚5
  joint2.write(0);    // 初始化舵机到中间位置
}

void loop() {
  for(int i=0;i<=120;i+=2)
  {
    joint2.write(i);
    delay(20);
  }
  delay(1000);
  for(int i=120;i>=0;i-=2)
  {
    joint2.write(i);
    delay(20);
  }
  delay(5000);
  // if (Serial.available() > 0) {
  //   // 读取串口数据
  //   String data = Serial.readStringUntil('\n');
  //   data.trim();  // 去除多余的换行符或空格

  //   // 将接收到的数据转换为浮点数
  //   float angle = data.toFloat();
  //   if (angle >= 0 && angle <= 180) {  // 验证角度范围
  //     joint2.write(angle);  // 控制舵机旋转
  //     Serial.print("舵机移动到角度: ");
  //     Serial.println(angle);  // 回传当前舵机角度
  //     delay(200);
  //     joint2.write(-angle);
  //   } else {
  //     Serial.println("无效的角度数据");  // 如果角度不合法，打印错误信息
  //   }
  // }
}
