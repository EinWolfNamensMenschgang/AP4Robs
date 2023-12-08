#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include<geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

// krho>0, kalpha>krho, kbeta<0 to make the system exponentially stable (Siegwart)
// values less than 1 so the robot doesn't overshoot

double x, y, theta;
double results[3];

void chatter_callback(const nav_msgs::Odometry odoms){
    double quatx, quaty, quatz, quatw;
    x = odoms.pose.pose.position.x;
    y = odoms.pose.pose.position.y;
    quatx = odoms.pose.pose.orientation.x;
    quaty = odoms.pose.pose.orientation.y;
    quatz = odoms.pose.pose.orientation.z;
    quatw = odoms.pose.pose.orientation.w;
    
    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    theta = yaw;
    if(theta>M_PI)
        theta = theta -(2*M_PI);
    if(theta<-M_PI)
        theta = theta + (2*M_PI);
}

void linearcontrol(double k_alpha,double k_beta, double k_rho, double goal_x, double goal_y, double goal_theta){ 

    goal_theta = goal_theta*M_PI/180.0;
    double delta_x, delta_y, delta_theta;
    delta_x = goal_x-x;
    delta_y = goal_y-y;
    delta_theta = goal_theta - theta;

    double rho, alpha, beta;
    rho = sqrt((delta_x*delta_x)+(delta_y*delta_y));
    alpha = -theta +atan2(delta_y, delta_x);
    if(alpha>M_PI)
        alpha = alpha-(2*M_PI);
    if(alpha<-M_PI)
        alpha = alpha + (2*M_PI);
    beta = -goal_theta - alpha;
    if(beta>M_PI)
        beta = beta - (2*M_PI);
    if(beta<-M_PI)
        beta = beta + (2*M_PI);
    double v, omega;
    v = k_rho*rho;
    omega = k_alpha*alpha + k_beta*beta;

    results[0] = v;
    results[1] = omega;
    results[2] = rho;

}


int main(int argc, char **argv){

ros::init(argc, argv, "linearcontrol2");
ros::NodeHandle nh;

ros::Publisher mypub;
mypub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
geometry_msgs::Twist twist;

ros::Subscriber mysub;
mysub=nh.subscribe("odom", 1, chatter_callback);

double v, omega;


double Pos1[3];
double Pos2[3];
double Pos3[3];
double Pos4[3];


nh.getParam("/linearcontrol2/Pos1_x", Pos1[0]);
nh.getParam("/linearcontrol2/Pos1_y", Pos1[1]);
nh.getParam("/linearcontrol2/Pos1_theta", Pos1[2]);
nh.getParam("/linearcontrol2/Pos2_x", Pos2[0]);
nh.getParam("/linearcontrol2/Pos2_y", Pos2[1]);
nh.getParam("/linearcontrol2/Pos2_theta", Pos2[2]);
nh.getParam("/linearcontrol2/Pos3_x", Pos3[0]);
nh.getParam("/linearcontrol2/Pos3_y", Pos3[1]);
nh.getParam("/linearcontrol2/Pos3_theta", Pos3[2]);
nh.getParam("/linearcontrol2/Pos4_x", Pos4[0]);
nh.getParam("/linearcontrol2/Pos4_y", Pos4[1]);
nh.getParam("/linearcontrol2/Pos4_theta", Pos4[2]);




double k_alpha = 0.6;
double k_rho = 0.2;
double k_beta = -0.2;
int i = 1;
double goal_x, goal_y, goal_theta;
bool all_pos_reached = false;

ros::Duration(5).sleep(); // wait for startup

while(ros::ok){
    
    switch (i)
    {
    case 1:
        goal_x = Pos1[0];
        goal_y = Pos1[1];
        goal_theta = Pos1[2];
        break;
    case 2:
        goal_x = Pos2[0];
        goal_y = Pos2[1];
        goal_theta = Pos2[2];
        break;
    case 3:
        goal_x = Pos3[0];
        goal_y = Pos3[1];
        goal_theta = Pos3[2];
        break;
    case 4:
        goal_x = Pos4[0];
        goal_y = Pos4[1];
        goal_theta = Pos4[2];
        break;
    default:
        all_pos_reached = true;
        break;
    }
    //ROS_INFO_STREAM(""<<i);
    linearcontrol(k_alpha, k_beta, k_rho, goal_x, goal_y, goal_theta);
    v = results[0];
    omega = results[1];


twist.linear.x=v;
twist.linear.y=0;
twist.linear.z=0;
twist.angular.x=0;
twist.angular.y=0;
twist.angular.z=omega;
mypub.publish(twist);

if(results[2]<=0.1){ // Position reached
    i++;
}

ros::spinOnce();
if(all_pos_reached)
    break;
}

return 0;
}
