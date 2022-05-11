#define USE_USBCON

//All The ROS includes
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <MatrixMath.h>
#include <Herkulex.h>
#include <ThreadController.h>
#include <Thread.h>
#include <ArduinoHardware.h>

//Define
#define DRIVE_A 50
#define DRIVE_B 51
#define DRIVE_C 52
#define DRIVE_D 53
#define STEER_A 0
#define STEER_B 3
#define STEER_C 1
#define STEER_D 2

#define GOAL_TIME_SPEED 10
#define GOAL_TIME_ANGLE 1000

//Variables
const double length1 = 0.6;
const double width = 0.375;
double distZtoWheel[4];
double wheelVel[4];
double beta[4];
double steer[4];
double alpha[4];
double wheelZeroVec[4][2];
float cmdangA , cmdangB;
float jac2[8][1];
float wheelrad;
float epsilon[3];

//Crea los Threads
Thread MotorControl = Thread();

ros::NodeHandle nh;

void velCallback( const geometry_msgs::Twist& vel) {

  
  epsilon[0] = vel.linear.x;
  epsilon[1] = 0;
  epsilon[2] = vel.angular.z;

  float radioP = (epsilon[0] / epsilon[2]);

  if (epsilon[0] == 0 && epsilon[2] > 0) {
    radioP = 0.5;
  }

  if (epsilon[0] == 0 && epsilon[2] < 0) {
    radioP = -0.5;
  }

  float steering = tan(length1 / (radioP));

  if (epsilon[2] == 0) {
    steering = 0;
    radioP = 0;
  }

  float angA, angB;
  if (steering > 0) {   //
    angA = (atan2(length1, (radioP + (width / 2))));
    angB = (atan2(length1, (radioP - (width / 2))));
    steer[0] = angA;
    steer[1] = angB;
    steer[2] = 0;
    steer[3] = 0;
  } else if (steering < 0) {
    angA = (atan2(length1, (abs(radioP) - (width / 2))));
    angB = (atan2(length1, (abs(radioP) + (width / 2))));
    steer[0] = -angA;
    steer[1] = -angB;
    steer[2] = 0;
    steer[3] = 0;
  } else {
    steer[0] = 0;
    steer[1] = 0;
    steer[2] = 0;
    steer[3] = 0;
  }

// beta
  for (int i = 0; i < 4; i++) {
      if((steering < 0) || i > 1 || (steering == 0)) {
        beta[i] = 1.5708 + steer[i] - alpha[i];
      }

      else if((steering > 0) && i < 2 ) {
        beta[i] = 1.5708 - steer[i] + alpha[i]; 
      }
  }


  if (steering > 0) {
    beta[0] = 3.1415 - beta[0]; 
    beta[1] = 3.1415 - beta[1]; 
  }

 
  float  jac1[8][3] = {
    {sin(alpha[0] + beta[0]), cos(alpha[0] + beta[0]) * -1, -1 * distZtoWheel[0]* cos(beta[0])},
    {sin(alpha[1] + beta[1]), cos(alpha[1] + beta[1]) * -1, -1 * distZtoWheel[1] * cos(beta[1])},
    {sin(alpha[2] + beta[2]), cos(alpha[2] + beta[2]) * -1, -1 * distZtoWheel[2] * cos(beta[2])},
    {sin(alpha[3] + beta[3]), cos(alpha[3] + beta[3]) * -1, -1 * distZtoWheel[3] * cos(beta[3])},
    {cos(alpha[0] + beta[0]), sin(alpha[0] + beta[0]), distZtoWheel[0] * sin(beta[0])},
    {cos(alpha[1] + beta[1]), sin(alpha[1] + beta[1]), distZtoWheel[1] * sin(beta[1])},
    {cos(alpha[2] + beta[2]), sin(alpha[2] + beta[2]), distZtoWheel[2] * sin(beta[2])},
    {cos(alpha[3] + beta[3]), sin(alpha[3] + beta[3]), distZtoWheel[3] * sin(beta[3])}
  };

 
  matrix.Multiply((float*)jac1, (float*)epsilon, 8, 3, 1, (float*)jac2);



  cmdangA = ((steer[0]) * (180 / 3.1416));
  cmdangB = ((steer[1]) * (180 / 3.1416));


  steer[0] = (57.8499984741 * -1) - cmdangA; 
  steer[1] = 57.8499984741 - cmdangB;
  steer[2] = 57.8499984741;
  steer[3] = 57.8499984741 * -1;



 
  wheelVel[0] = ((-1 * jac2[0][0] / wheelrad) * 180 / (3.1416)); 
  wheelVel[1] = ((jac2[1][0] / wheelrad) * 180 / (3.1416)); 
  wheelVel[2] = ((jac2[2][0] / wheelrad) * 180 / (3.1416)); 
  wheelVel[3] = ((-1 * jac2[3][0] / wheelrad) * 180 / (3.1416));
  
  
  wheelVel[0] = wheelVel[0] / 0.62;
  wheelVel[1] = wheelVel[1] / 0.62;
  wheelVel[2] = wheelVel[2] / 0.62;
  wheelVel[3] = wheelVel[3] / 0.62;

  if (epsilon[0] == 0) { 
    wheelVel[0] = 0;
    wheelVel[1] = 0;
    wheelVel[2] = 0;
    wheelVel[3] = 0;
  }

}


void sendMotorCommands() {

  Herkulex.moveOneAngle(STEER_A, steer[0], GOAL_TIME_ANGLE, LED_BLUE);
  Herkulex.moveOneAngle(STEER_B, steer[1], GOAL_TIME_ANGLE, LED_BLUE);
  
  Herkulex.moveSpeedOne(DRIVE_A, wheelVel[0], GOAL_TIME_SPEED, LED_GREEN);
  Herkulex.moveSpeedOne(DRIVE_B, wheelVel[1], GOAL_TIME_SPEED, LED_GREEN);
  Herkulex.moveSpeedOne(DRIVE_C, wheelVel[2], GOAL_TIME_SPEED, LED_GREEN);
  Herkulex.moveSpeedOne(DRIVE_D, wheelVel[3], GOAL_TIME_SPEED, LED_GREEN);
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);

void setup() {
  Serial.begin(115200);
  Herkulex.beginSerial3(115200); 
  delay(5000);
  Serial.print("Serial 1 y 3 Setted");

  
  steer[2] = 57.8499984741;
  steer[3] = 57.8499984741 * -1;
  steer[0] = 57.8499984741 * -1;
  steer[1] = 57.8499984741;



  wheelZeroVec[0][0] = length1;
  wheelZeroVec[0][1] = -width / 2;
  wheelZeroVec[1][0] = length1;
  wheelZeroVec[1][1] = width / 2;
  wheelZeroVec[2][0] = 0;
  wheelZeroVec[2][1] = width / 2;
  wheelZeroVec[3][0] = 0;
  wheelZeroVec[3][1] = -width / 2;

  
  distZtoWheel[0] = sqrt((pow(wheelZeroVec[0][0], 2)) + (pow(wheelZeroVec[0][1], 2)));
  distZtoWheel[1] = sqrt((pow(wheelZeroVec[1][0], 2)) + (pow(wheelZeroVec[1][1], 2)));
  distZtoWheel[2] = sqrt((pow(wheelZeroVec[2][0], 2)) + (pow(wheelZeroVec[2][1], 2)));
  distZtoWheel[3] = sqrt((pow(wheelZeroVec[3][0], 2)) + (pow(wheelZeroVec[3][1], 2)));

  wheelrad = 0.076; 
  for (int i = 0; i < 4; i++) {

    wheelVel[i] = 0; 

   
    alpha[i] = atan2(wheelZeroVec[i][1], wheelZeroVec[i][0]);


  }
  nh.getHardware()->setBaud(115200); 
  nh.initNode();
  nh.subscribe(sub); 


  Herkulex.reboot(STEER_A);
  Herkulex.reboot(STEER_B);
  Herkulex.reboot(DRIVE_A);
  Herkulex.reboot(DRIVE_B);
  Herkulex.reboot(DRIVE_C);
  Herkulex.reboot(DRIVE_D);
  delay(500);
  Herkulex.initialize();

  MotorControl.onRun(sendMotorCommands);
  MotorControl.setInterval(200);

}

void loop() {
  if ( MotorControl.shouldRun()) {
    MotorControl.run();
  }

  nh.spinOnce();


}
