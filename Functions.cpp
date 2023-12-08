//Functions for AP4Robos
#define _USE_MATH_DEFINES
#define GENERAL_VELOCITY 1.0
#define CRITICAL_DISTANCE 0.1
#define K_ALPHA 0.1
#include <cmath>
#include "messages.h"

namespace Functions{
double getYawFromQuats (Messages::Quaternion quats){ //assumes normalized Quaternions
    double siny_cosp = 2* (quats.w*quats.z+quats.x*quats.y);
    double cosy_cosp = 1 - 2 * (quats.y * quats.y + quats.z * quats.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    //int yaw_rounded = std::round(yaw);
    double theta = yaw;
    if(theta>M_PI)
        theta = theta -(2*M_PI);
    if(theta<-M_PI)
        theta = theta + (2*M_PI);
    return theta;
};

double rad2deg(double angleInRad){
double angleInDegree = angleInRad *180/M_PI;
return angleInDegree;
};

bool wallReached (Messages::Sensor_msg& msg){ //assumes laserscan data is clockwise, yaw counterclockwise, robot starts facing the obstacle
    double yawAngle = getYawFromQuats(msg.odom.orientation);
    int yawAngleRounded = std::round(rad2deg(yawAngle));
    double laserRangeInFront = msg.laser.ranges[360-yawAngleRounded];
    if (laserRangeInFront <= CRITICAL_DISTANCE){
        return true;
    } else {
        return false;
    }
};

bool turnedToAngle(Messages::Odometry_msg& msg, double angleToTurnTo){
    double yaw = getYawFromQuats(msg.orientation);
    if (yaw - angleToTurnTo <=0.1){
        return true;
    } else{
        return false;
    }
};

Messages::Twist_msg_and_distance linearController(double k_alpha,double k_beta, double k_rho, double goal_x, double goal_y, double goal_theta, double currentX, double currentY, double currentYaw){ 

    //goal_theta = goal_theta*M_PI/180.0;
    double delta_x, delta_y, delta_theta;
    delta_x = goal_x-currentX;
    delta_y = goal_y-currentY;
    delta_theta = goal_theta - currentYaw;

    double rho, alpha, beta;
    rho = sqrt((delta_x*delta_x)+(delta_y*delta_y));
    alpha = -currentYaw +atan2(delta_y, delta_x);
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

    Messages::Twist_msg_and_distance msg;
    msg.twist.x_vel = v;
    msg.twist.angular_vel = omega;
    msg.distance = rho;
    return msg;
}
}
/*Twist_msg linearControllerStraight(Odometry_msg& msg){
    Twist_msg output;
    output.linear.x = GENERAL_VELOCITY;
    output.linear.y = 0.0;
    output.linear.z = 0.0;
    double yaw = getYawFromQuats(msg.orientation);
    double angular_vel = -yaw * K_ALPHA;
    output.angular.x = 0.0;
    output.angular.y = 0.0;
    output.angular.z = angular_vel;
    return output;
};


Twist_msg linearControllerCylinder(Odometry_msg& msg){
    Twist_msg output;
    output.linear.x = GENERAL_VELOCITY;
    output.linear.y = 0.0;
    output.linear.z = 0.0;
    double yaw = getYawFromQuats(msg.orientation);
    double angular_vel = -yaw * K_ALPHA;
    output.angular.x = 0.0;
    output.angular.y = 0.0;
    output.angular.z = angular_vel;
    return output;
};*/

