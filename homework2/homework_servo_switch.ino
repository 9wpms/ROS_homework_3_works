#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

ros::NodeHandle nh;

std_msgs::Int8 data1;
std_msgs::Int8 data2;
std_msgs::Int16 val1;
std_msgs::Int16 val2;

Servo servo_01, servo_02;

void callback_servo01(const std_msgs::Int16 &msg) {
  val1 = msg;
  servo_01.write(val1.data);
}

void callback_servo02(const std_msgs::Int16 &msg) {
  val2 = msg;
  servo_02.write(val2.data);
}

ros::Publisher switch01_data("switch_info1", &data1);
ros::Publisher switch02_data("switch_info2", &data2);

ros::Subscriber<std_msgs::Int16> servo01_angle("Topic_servo_01", &callback_servo01);
ros::Subscriber<std_msgs::Int16> servo02_angle("Topic_servo_02", &callback_servo02);

const int sw1_red = 12;
const int sw2_green = 13;

void setup() {
  nh.initNode();
  
  nh.advertise(switch01_data);
  nh.advertise(switch02_data);
  nh.subscribe(servo01_angle);
  nh.subscribe(servo02_angle);
  
  pinMode(sw1_red, INPUT);
  pinMode(sw2_green, INPUT);
  digitalWrite(sw1_red, HIGH);
  digitalWrite(sw2_green, HIGH);

  servo_01.attach(10);  // Pin 10
  servo_02.attach(11);  // Pin 11
}

void loop() {
  int sw1_red_value = digitalRead(sw1_red);
  int sw2_green_value = digitalRead(sw2_green);

  data1.data = sw1_red_value;
  data2.data = sw2_green_value;

  switch01_data.publish(&data1);
  switch02_data.publish(&data2);

  nh.spinOnce();
  delay(100);
}