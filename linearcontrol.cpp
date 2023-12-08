#include "ros/ros.h"
#include "std_msgs/String.h"
#include <dynamic_reconfigure/server.h>
#include <bolz_hausaufgabe2/linearcontrolConfig.h>
#include "nav_msgs/Odometry.h"
#include<geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

double k_rho, k_alpha, k_beta, goal_x, goal_y, goal_theta, x, y, theta;
bool start = false;
double quatx,quaty,quatz,quatw;
double results[2];

void callback(bolz_hausaufgabe2::linearcontrolConfig &config, uint32_t level) { //Hyperparameter and goalpose 

    k_rho=config.k_rho;
    k_alpha=config.k_alpha;
    k_beta=config.k_beta;
    goal_x=config.goal_x;
    goal_y=config.goal_y;
    goal_theta=config.goal_theta;
    start=config.start;
}
void chatter_callback(const nav_msgs::Odometry odoms){

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
    //ROS_INFO_STREAM(""<<theta);
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
    //ROS_INFO_STREAM("Alpha: "<<alpha);
    beta = delta_theta - alpha;
    if(beta>M_PI)
        beta = beta - (2*M_PI);
    if(beta<-M_PI)
        beta = beta + (2*M_PI);
    //ROS_INFO_STREAM("beta: "<<beta);
    double v, omega;
    v = k_rho*rho;
    omega = k_alpha*alpha + k_beta*beta;

    results[0] = v;
    results[1] = omega;
}


int main(int argc, char **argv){

ros::init(argc, argv, "linearcontrol");
ros::NodeHandle nh;

dynamic_reconfigure::Server<bolz_hausaufgabe2::linearcontrolConfig> server;

dynamic_reconfigure::Server<bolz_hausaufgabe2::linearcontrolConfig>::CallbackType f;
f = boost::bind(&callback, _1, _2);
server.setCallback(f);


ros::Publisher mypub;
mypub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
geometry_msgs::Twist twist;

ros::Subscriber mysub;
mysub=nh.subscribe("odom", 1, chatter_callback);

double v, omega;


while(ros::ok){
//ros::spinOnce();

if(start==false){
v = 0;
omega = 0;
}
else{
    linearcontrol(k_alpha, k_beta, k_rho, goal_x, goal_y, goal_theta);
    v = results[0];
    omega = results[1];
    //v = 1;
    //omega = 1;
}

//vx = v * cos(theta);
//vy = v * sin(theta);
twist.linear.x=v;
twist.linear.y=0;
twist.linear.z=0;
twist.angular.x=0;
twist.angular.y=0;
twist.angular.z=omega;
mypub.publish(twist);
//ros::Duration(0.5).sleep();
ros::spinOnce();
}
return 0;
}
