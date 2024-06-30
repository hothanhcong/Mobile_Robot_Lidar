#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <math.h>
#include "Motor.h"


unsigned long startMillis = 0;  // Thời gian bắt đầu đếm
bool countTime = false;
unsigned long prevMillispub = 0;

ros::NodeHandle  nh;

int updatenh = 0;

double radius = 0.0325;


#define LOOPTIME 20

Motor left(11,10,18,19);   //int plus, int minus, int en_a, int en_b
Motor right(9,8,20,21);



volatile long encoder0Pos = 0;    // encoder 1-left
volatile long encoder1Pos = 0;    // encoder 2-right

double left_kp = 9.54, left_ki = 0 ,  left_kd = 0.0;             // modify for optimal performance
double right_kp = 8.9 , right_ki = 0 , right_kd = 0.0;

float demandx= 0.0;
float demandz= 0.0;

double demand_speed_left;
double demand_speed_right;


double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  


unsigned long currentMillis;
unsigned long prevMillis;

float encoder0Diff;
float encoder1Diff;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;

int pos_act_left;
int pos_act_right;

void cmd_vel_cb( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );

std_msgs::Int16 left_wheel_msg;
ros::Publisher left_wheel_pub("wheel_left_ticks", &left_wheel_msg);
std_msgs::Int16 right_wheel_msg; 
ros::Publisher right_wheel_pub("wheel_right_ticks", &right_wheel_msg);


double speed_act_left = 0;
double speed_act_right = 0;

void publishPos(double time);
void change_encoder_left();
void change_encoder_right();
//void print_out();

void setup() {
  

  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(left_wheel_pub);                  //prepare to publish ticks left in ROS topic
  nh.advertise(right_wheel_pub);                  //prepare to publish tick right in ROS topic

  
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-100, 100);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(-100, 100);
  
  attachInterrupt(digitalPinToInterrupt(left.en_b), change_encoder_left, RISING);   
  attachInterrupt(digitalPinToInterrupt(right.en_b), change_encoder_right, RISING);
}

void loop() {
//     nh.spinOnce();
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME){

    prevMillis = currentMillis;
    
//    if(Serial.available() > 0){
//      char c = Serial.read();
//      if(c == 'a'){
//        demandx = 0.5;
//        demandz = 0.5;
//        }
//      else if (c == 's'){
//
//          demandx = 0.2;
//          demandz = 0;
//                  countTime = true;           // Bắt đầu đếm thời gian
//        startMillis = millis();
//          }
//      else if (c == 'd'){
//      
//        demandx = 0.1;
//        demandz = 0;
//        countTime = true;           // Bắt đầu đếm thời gian
//        startMillis = millis();
//      }
//      else if (c == 'w'){
//      
//        demandx = -0.1;
//        demandz = 0;
//                countTime = true;           // Bắt đầu đếm thời gian
//        startMillis = millis();
//      }
//      else if (c == 'f'){
//      
//        demandx = 0;
//        demandz = 0;
//      }
//      }
//
//      
//
//      if (countTime && (currentMillis - startMillis >= 10000)) {
//      demandx = 0;   // Đặt vận tốc về 0
//      demandz = 0;
//      countTime = false;  // Dừng đếm thời gian
//    }

    demand_speed_left = (demandx - (demandz * radius));
    demand_speed_right = (demandx + (demandz * radius));
    
//    demand_speed_left = (demandx - (demandz * (longitude / 2)));
//    demand_speed_right = -(demandx + (demandz * (longitude / 2)));
    


    encoder0Diff = encoder0Pos - encoder0Prev; // Get difference between ticks to compute speed
    encoder1Diff = encoder1Pos - encoder1Prev;
    
    pos_act_left = encoder0Pos;
    pos_act_right = encoder1Pos;


//          Serial.print("pos_act_left: ");
//      Serial.println(encoder0Pos);    
//      Serial.print("pos_act_right: ");
//      Serial.println(encoder1Pos);
    
    speed_act_left = encoder0Diff/496.0*(1000.0/LOOPTIME)*3.14159*0.065;                    
    speed_act_right = encoder1Diff/496.0*(1000.0/LOOPTIME)*3.14159*0.065; // m/s
    

  
    encoder0Prev = encoder0Pos; // Saving values
    encoder1Prev = encoder1Pos;
  
    left_setpoint = demand_speed_left*40;  //Setting required speed as a mul/frac of 1 m/s
    right_setpoint = demand_speed_right*40;
  
    left_input = speed_act_left;  //Input to PID controller is the current difference
    right_input = speed_act_right;

    
    leftPID.Compute();
    rightPID.Compute();
    left.rotate(left_output);
    right.rotate(right_output);
    
     publishPos(LOOPTIME);
     nh.spinOnce();


//    if(updatenh > 10){
//      nh.spinOnce();
//      updatenh = 0;
//    }
//    else{
//      updatenh++;
//    }
    
  }

//  if (currentMillis - prevMillispub >= LOOPTIME*100){
//    print_out();
//    prevMillispub = currentMillis;
//  }
  
 

    
}

//void print_out() {
//    
//      Serial.print(F("linear velocity: "));
//      Serial.println(demand_speed_left);
//      Serial.print("pos_act_left: ");
//      Serial.println(speed_act_left);    
//      Serial.print("pos_act_right: ");
//      Serial.println(speed_act_right);
//    
//    
//
//  }

void publishPos(double time) {
  left_wheel_msg.data = pos_act_left;
  left_wheel_pub.publish(&left_wheel_msg);

  right_wheel_msg.data = pos_act_right;
  right_wheel_pub.publish(&right_wheel_msg);

}


// ************** encoders interrupts **************

// ************** encoder 1 *********************


void change_encoder_left() {
  // look for a low-to-high on channel A
  if (digitalRead(18) == LOW) {
    encoder0Pos = encoder0Pos - 1;  // CW
  } else {
    encoder0Pos = encoder0Pos + 1;  // CCW
  }
}

// ************** encoder 2 *********************
void change_encoder_right() {
  // look for a low-to-high on channel A
  if (digitalRead(20) == LOW) {
    encoder1Pos = encoder1Pos - 1;  // CW
  } else {
    encoder1Pos = encoder1Pos + 1;  // CCW
  }
}
 
