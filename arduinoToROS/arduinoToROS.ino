#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>
#include <std_msgs/Int16.h>

// Setup the PWM Servo Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 125  // Minimum pulse length count
#define SERVOMAX 575  // Maximum pulse length count

ros::NodeHandle nh;

// Function prototypes
void setServo0(const std_msgs::Int16 &msg);
void setServo1(const std_msgs::Int16 &msg);
void setServo2(const std_msgs::Int16 &msg);
void setServo3(const std_msgs::Int16 &msg);
void setServo5(const std_msgs::Int16 &msg);
int angleToPulse(int ang);

// ROS Subscribers for each servo
ros::Subscriber<std_msgs::Int16> sub_servo0("servo0_angle", setServo0);
ros::Subscriber<std_msgs::Int16> sub_servo1("servo1_angle", setServo1);
ros::Subscriber<std_msgs::Int16> sub_servo2("servo2_angle", setServo2);
ros::Subscriber<std_msgs::Int16> sub_servo3("servo3_angle", setServo3);
ros::Subscriber<std_msgs::Int16> sub_servo5("servo5_angle", setServo5);

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);  // Set PWM frequency to 60 Hz

  nh.initNode();

  // Advertise the subscribers
  nh.subscribe(sub_servo0);
  nh.subscribe(sub_servo1);
  nh.subscribe(sub_servo2);
  nh.subscribe(sub_servo3);
  nh.subscribe(sub_servo5);
}

void loop() {
  nh.spinOnce();
  delay(10);
}

// Callback functions for each servo
void setServo0(const std_msgs::Int16 &msg) {
  pwm.setPWM(0, 0, angleToPulse(msg.data));
}

void setServo1(const std_msgs::Int16 &msg) {
  pwm.setPWM(1, 0, angleToPulse(msg.data));
}

void setServo2(const std_msgs::Int16 &msg) {
  pwm.setPWM(2, 0, angleToPulse(msg.data));
}

void setServo3(const std_msgs::Int16 &msg) {
  pwm.setPWM(3, 0, angleToPulse(msg.data));
}

void setServo5(const std_msgs::Int16 &msg) {
  pwm.setPWM(5, 0, angleToPulse(msg.data));
}

// Convert angle to pulse width
int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  Serial.print("Angle: ");
  Serial.print(ang);
  Serial.print(" pulse: ");
  Serial.println(pulse);
  return pulse;
}
