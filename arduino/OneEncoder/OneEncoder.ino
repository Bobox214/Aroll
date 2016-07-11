#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

std_msgs::Int32 msgEncoder1;
ros::Publisher pubEncoder1("Encoder1",&msgEncoder1);

const int EncoderPin1A = 2;
const int EncoderPin1B = 3;

int countEncoder1 = 0;
int loopCount = 0;

bool curPin1A;
bool curPin1B;
bool lastPin1A = 0;
bool lastPin1B = 0;

void setup() {
  pinMode(EncoderPin1A, INPUT);
  pinMode(EncoderPin1B, INPUT);
  nh.initNode();
  nh.advertise(pubEncoder1);
}

void loop() {
  curPin1A = digitalRead(EncoderPin1A);
  curPin1B = digitalRead(EncoderPin1B);

  if (curPin1A!=lastPin1A) {
    if (curPin1A == HIGH) {
      if (curPin1B == HIGH)
        countEncoder1 += 1;
      else
        countEncoder1 -= 1;
    }
  }
  lastPin1A = curPin1A;

  loopCount += 1;
  if (loopCount==100) {
      msgEncoder1.data = countEncoder1;
      pubEncoder1.publish(&msgEncoder1);
      loopCount = 0;
  }
  nh.spinOnce();
  delay(1);
}









