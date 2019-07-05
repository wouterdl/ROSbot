#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include <std_msgs/Int64MultiArray.h>
#include<iostream>

ros::Publisher speed_pub;
geometry_msgs::Twist set_vel;

float rangesout = 0;
float diff = 0;
float diff0 = 0;
float diffout1 = 0;
float diffout2 = 0;
float LPointsf[8];
float dt = 0.1; // delay/1000 moet seconde zijn

int LPoints[8];
float Pointsavgb;
float pixdif;
float Pointsavgt;
float Pointsavgbo=0;
float Pointsavgto=0;
int middle = 400;
float rotspeed1;
float rotspeed2;
float rotspeed3;
float rotspeed;


float speed;
const float speedset = 0.4;
const float delaydis = 0.1; //m
const float delay = round((delaydis/speedset)*5); // 5 is Hz
const int delayn = static_cast<int>(delay);
float rot[delayn];

float rotate;
float rotateset=0;
int teller = 0;
float Vx, Vy, Inc, spd, spd2;// spd;
float VelX, VelY, speedgood;
float xPoseOud = 0, yPoseOud = 0;

float rfla = 0;
float rfra = 0;
float rfl1 = 0;
float rfr1 = 0;
float rfl2 = 0;
float rfr2 = 0;
float rfl = 0;
float rfr = 0;

float pose0;
float pose1;

void distfl_callback(const sensor_msgs::Range &range){
   rfl = range.range;
}

void distfr_callback(const sensor_msgs::Range &range){
   rfr = range.range;
}

void pose_callback(const geometry_msgs::PoseStamped &pose) {
pose0 = pose.pose.position.x;
pose1 = pose.pose.position.y;
}

void hough_callback(const std_msgs::Int64MultiArray::ConstPtr& coords){
  //Putting message in array
LPoints[0] = coords->data[0];
LPoints[2] = coords->data[2];
LPoints[4] = coords->data[4];
LPoints[6] = coords->data[6];
}

void ranges(){
  if (rfl>1){
    rfl=1;
  }
  if (rfr>1){
    rfr=1;
  }
  rfla=(rfl+rfl1+rfl2)/3;
  rfra=(rfr+rfr1+rfr2)/3;
}

double _Kp = 0.0032;
double _Ki = 0.000022;
double _Kd = 0.00075;
double _pre_error, _integral;

double calculateline( double setpoint, double pv ){
    double error = setpoint - pv;
    double Pout = _Kp * error;
    _integral += error * dt;
    double Iout = _Ki * _integral;
    double derivative = (error - _pre_error) / dt;
    double Dout = _Kd * derivative;
    double output = Pout + Iout + Dout;
    _pre_error = error;
    //std::cout <<  "     error " << error << "     p " << Pout << "     I " << Iout << "     D " << Dout <<  "     Out " << output << std::endl;
    return output;
  }

void rotation(){
std::copy(LPoints, LPoints+8, LPointsf);
Pointsavgb= (LPointsf[0]+LPointsf[4])/2;
Pointsavgt= (LPointsf[2]+LPointsf[6])/2;

if (abs(Pointsavgb)>1000){
  Pointsavgb=Pointsavgbo;
}

if (abs(Pointsavgt)>1000){
  Pointsavgt=Pointsavgto;
}

if (abs(Pointsavgb)<1000){
  Pointsavgbo=Pointsavgb;
}

if (abs(Pointsavgt)<1000){
  Pointsavgto=Pointsavgt;
}

pixdif=Pointsavgt-Pointsavgb;
  rotspeed=calculateline(Pointsavgt, 325); //300 right side 350 left side
  rot[0] = -rotspeed;
rotateset=rot[0];
}

void speedinit(){
    spd=speedset; //Uit als imu data beschikbaar is
    diff0= (rangesout - rfla)/dt;
    diff=(diff0+diffout1+diffout2)/3;
  /*  if (((rfla < 0.6) && (rfla > 0.35) && (speedset > 0)) || ((rfra < 0.6) && (rfra > 0.35 ) && (speedset > 0))){
        speed = spd - (0.11*diff);

    }

    else if (((rfla < 0.36) || (rfra < 0.36))){
            speed = 0;
            rotate = 0;
    }*/
    //else {
        speed = speedset;
        rotate = rotateset;
  //  }
}

double _Kpc = 0;
double _Kic = 0;
double _Kdc = 0;
double _pre_errorc, _integralc;

double calculatecruise( double setpointc, double pvc ){
    double errorc = setpointc - pvc;
    double Poutc = _Kpc * errorc;
    _integralc += errorc * dt;
    double Ioutc = _Kic * _integralc;
    double derivativec = (errorc - _pre_errorc) / dt;
    double Doutc = _Kdc * derivativec;
    double outputc = Poutc + Ioutc + Doutc;
    _pre_errorc = errorc;
    //std::cout <<  "     error " << error << "     p " << Pout << "     I " << Iout << "     D " << Dout <<  "     Out " << output << std::endl;
    return outputc;
  }

void opslag(){
    rfl2=rfl1;
    rfl1=rfl;
    rfr2=rfr1;
    rfr1=rfr;
    diffout2=diffout1;
    diffout1=diff0;
    rangesout=rfla;
    xPoseOud=pose0;
    yPoseOud=pose1;
    rotspeed3=rotspeed2;
    rotspeed2=rotspeed1;
}

void limit(){
            speedgood=calculatecruise(speed, spd)+speed;
            if (speedgood>speedset){
                speedgood=speedset;
            }
            if ((speedset>0) && (speedgood<0)){
                speedgood=0;
            }
            set_vel.linear.x = speedgood;
            set_vel.angular.z = rotate;
            //set_vel.angular.x = Pointsavgt;

    }
int main(int argc, char **argv)
{
    ros::init(argc, argv,"action_controller_B");
    ros::NodeHandle n("~");
    ros::Subscriber distfl_sub = n.subscribe("/range/fl_B", 1, distfl_callback);
    ros::Subscriber distfr_sub = n.subscribe("/range/fr_B", 1, distfr_callback);
    ros::Subscriber pose_sub = n.subscribe("/pose_B", 2, pose_callback);
    ros::Subscriber hough_sub = n.subscribe("/Hough_B", 1, hough_callback);


    ros::Rate loop_rate(10);
    clean_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_B", 1);
    set_vel.linear.x = 0;
    set_vel.linear.y = 0;
    set_vel.linear.z = 0;
    set_vel.angular.x = 0;
    set_vel.angular.y = 0;
    set_vel.angular.z = 0;

    while(ros::ok()){
        ranges();
        rotation();
        speedinit();
        opslag();
        limit();
        clean_pub.publish(set_vel);
        ros::spinOnce();
				loop_rate.sleep();

    }

		return 0;
}
