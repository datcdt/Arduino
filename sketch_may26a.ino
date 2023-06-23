#include <Wire.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/time.h>
#include <ArduinoHardware.h>


//initializing all the variables
#define LOOPTIME          100                 //Thời gian lấy mẫu
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter
double SamplingTime = 0.1; //giaya

const int PIN_L_IN1 = 25;
const int PIN_L_IN2 = 23;
const int PIN_L_PWM = 4;
const int PIN_L_ENCOD_A_MOTOR = 2;               //A channel for encoder of left motor
const int PIN_L_ENCOD_B_MOTOR = 16;               //B channel for encoder of left motor

const int PIN_R_IN1 = 31;
const int PIN_R_IN2 = 29;
const int PIN_R_PWM = 5;
const int PIN_R_ENCOD_A_MOTOR = 3;              //A channel for encoder of right motor
const int PIN_R_ENCOD_B_MOTOR = 17;              //B channel for encoder of right motor
const int Standby = 27;
unsigned long long lastMilli = 0;
const double radius = 0.0287;               // Bán kính bánh xe truyền động, m
const double wheelbase = 0.206;               //Khoảng cách giữa 2 bánh xe, m

double speed_req = 0;                         //Vận tốc dài mong muốn từ ROS gửi xuống, m/s
double angular_speed_req = 0;                 //Vận tốc góc mong muốn từ ROS gửi xuống, rad/s

double kpL = 150, kiL = 250, kdL = 0.1;
double speed_req_left = 0;                    //Vận tốc bánh trái mong muốn, m/s
double speed_act_left = 0;                    //Vận tốc bánh trái thực tế, m/s
double left_error = 0;
double left_lastError = 0;
double left_llastError = 0;
double PWM_leftMotor_last = 0; 
double PWM_leftMotor = 0;                         //Xung PWM cấp cho  động cơ trái
double aL = 0,bL = 0,gL = 0;
volatile double pos_left = 0;       //biến đếm xung encoder động cơ trái

double kpR = 200, kiR = 275, kdR = 0.1;
double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double right_error = 0;
double right_lastError = 0;
double right_llastError = 0;
double PWM_rightMotor_last = 0; 
double PWM_rightMotor = 0;                       //Xung PWM cấp cho  động cơ phải
double aR = 0,bR = 0,gR = 0;
volatile double pos_right = 0;      //biến đếm xung encoder động cơ phải

const double max_speed = 0.25;                 //Vận tốc giới hạn, m/s

int encoder_left = 0;
int encoder_right = 0;

ros::NodeHandle nh;
//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message
  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  speed_req_left  = speed_req - angular_speed_req * (wheelbase / 2); //Calculate the required speed for the left motor to comply with commanded linear and angular speeds 
  speed_req_right = speed_req + angular_speed_req * (wheelbase / 2); //Calculate the required speed for the right motor to comply with commanded linear and angular speeds  
} 
geometry_msgs::Pose2D pwm_output_msg; 
geometry_msgs::Pose2D encoder_msg; 
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", &handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type
ros::Publisher pwm_output_pub("pwm_output", &pwm_output_msg);
ros::Publisher encoder_pub("encoder", &encoder_msg);

void setup() {
  nh.initNode();//init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
  nh.advertise(encoder_pub);
  nh.advertise(pwm_output_pub);

  //setting motor speeds to zero
  pinMode(Standby, OUTPUT);
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_L_PWM, OUTPUT);

  pinMode(PIN_R_IN1, OUTPUT);
  pinMode(PIN_R_IN2, OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT);
  // Define the rotary encoder for left motor
  pinMode(PIN_L_ENCOD_A_MOTOR, INPUT_PULLUP);
  pinMode(PIN_L_ENCOD_B_MOTOR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_L_ENCOD_A_MOTOR), encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_R_ENCOD_A_MOTOR, INPUT_PULLUP);
  pinMode(PIN_R_ENCOD_B_MOTOR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_R_ENCOD_A_MOTOR), encoderRightMotor, RISING);
  digitalWrite(Standby,HIGH);
  
}
void loop() {
  nh.spinOnce();
  if ((millis() - lastMilli) >= 100){
    lastMilli = millis();
    if (abs(pos_left) < 5) {
      speed_act_left = 0;
    }
    else {
      speed_act_left = ((pos_left/1441)*2 * PI) *(1000/LOOPTIME)*radius;//rpm !!!! 11*131 = 1441 encoder total pulse coun
    }
      
    if (abs(pos_right) < 5) {
      speed_act_right = 0;                                                 //Avoid taking in account small disturbances
    }
    else {
      speed_act_right = ((pos_right/1441)*2*PI)*(1000/LOOPTIME)*radius;
    }
    
    encoder_left = encoder_left + pos_left;
    encoder_right = encoder_right + pos_right;
    
    if(encoder_left>32000) encoder_left = 0; 
    if(encoder_right>32000) encoder_right = 0; 
    pos_right = 0;
    pos_left = 0;
    // PID cho động cơ trái
    
    left_error = speed_req_left - speed_act_left;
    aL = 2*SamplingTime*kpL + kiL*SamplingTime*SamplingTime + 2*kdL;
    bL = kiL*SamplingTime*SamplingTime - 4*kdL - 2*SamplingTime*kpL;
    gL = 2*kdL;
    PWM_leftMotor = (aL*left_error + bL*left_lastError + gL*left_llastError + 2*SamplingTime*PWM_leftMotor_last)/(2*SamplingTime);
    PWM_leftMotor = constrain(PWM_leftMotor,-255,255);
    PWM_leftMotor_last = PWM_leftMotor;
    left_llastError = left_lastError;
    left_lastError = left_error;
    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      analogWrite(PIN_L_PWM, 0);
    }
    else if (speed_req_left == 0) {                       //Stopping
      analogWrite(PIN_L_PWM, 0);
    }
    else if (PWM_leftMotor > 0) {                         //Going forward
       
      analogWrite(PIN_L_PWM, PWM_leftMotor);
      digitalWrite(PIN_L_IN1, HIGH);
      digitalWrite(PIN_L_IN2, LOW);
    }
    else {                                               //Going backward
      analogWrite(PIN_L_PWM, abs(PWM_leftMotor));
      digitalWrite(PIN_L_IN1, LOW);
      digitalWrite(PIN_L_IN2, HIGH);
    }
    //PID cho động cơ phải
    
    
    right_error = speed_req_right - speed_act_right;
    aR = 2*SamplingTime*kpR + kiR*SamplingTime*SamplingTime + 2*kdR;
    bR = kiR*SamplingTime*SamplingTime - 4*kdR - 2*SamplingTime*kpR;
    gR = 2*kdR;
    PWM_rightMotor = (aR*right_error + bR*right_lastError + gR*right_llastError + 2*SamplingTime*PWM_rightMotor_last)/(2*SamplingTime);
    PWM_rightMotor = constrain(PWM_rightMotor,-255,255);
    PWM_rightMotor_last = PWM_rightMotor;
    right_llastError = right_lastError;
    right_lastError = right_error;
    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      analogWrite(PIN_R_PWM, 0);
    }
    else if (speed_req_right == 0) {                      //Stopping
      analogWrite(PIN_R_PWM, 0);
    }
    else if (PWM_rightMotor > 0) {                        //Going forward
      analogWrite(PIN_R_PWM, PWM_rightMotor);
      digitalWrite(PIN_R_IN1, HIGH);
      digitalWrite(PIN_R_IN2, LOW);
    }else {                                                //Going backward
      analogWrite(PIN_R_PWM, abs(PWM_rightMotor));
      digitalWrite(PIN_R_IN1, LOW);
      digitalWrite(PIN_R_IN2, HIGH);
    }
    noCommLoops++;
    if (noCommLoops == 65535) {
      noCommLoops = noCommLoopMax;
    }
    publishSpeed();
  }
}

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed() {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = LOOPTIME / 1000;   //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);

  encoder_msg.x= encoder_left;
  encoder_msg.y= encoder_right;
  encoder_pub.publish( &encoder_msg);

  pwm_output_msg.x = PWM_leftMotor;
  pwm_output_msg.y = PWM_rightMotor;
  pwm_output_pub.publish(&pwm_output_msg);
 
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}
//Đếm xung encoder động cơ trái
void encoderLeftMotor() {
  if (digitalRead(PIN_L_ENCOD_A_MOTOR) == digitalRead(PIN_L_ENCOD_B_MOTOR)) pos_left++;
  else pos_left--;
}
//Đếm xung encoder động cơ phải
void encoderRightMotor() {
  if (digitalRead(PIN_R_ENCOD_A_MOTOR) == digitalRead(PIN_R_ENCOD_B_MOTOR)) pos_right--;
  else pos_right++;
}
