//Functions for AP4Robos
#define _USE_MATH_DEFINES
#define GENERAL_VELOCITY 1.0
#define CRITICAL_DISTANCE 0.1
#define K_ALPHA 0.1
#include <cmath>
#include "messages.h"
#include "Functions.h"


double Functions::quaternionToYaw (Messages::Quaternion quats){ //assumes normalized Quaternions
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


Messages::Quaternion Functions::yawToQuaternion(double yaw) // roll (x), pitch (y), yaw (z), angles are in radians
{
    // Abbreviations for the various angular functions

    double cr = cos(0 * 0.5);
    double sr = sin(0 * 0.5);
    double cp = cos(0 * 0.5);
    double sp = sin(0 * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    Messages::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}


double Functions::rad2deg(double angleInRad){
    double angleInDegree = angleInRad *180/M_PI;
    return angleInDegree;
};

double Functions::deg2rad(double angleInDegree){
    double angleInRad = angleInDegree * M_PI / 180;
    return angleInRad;
}

/*bool Functions::wallReached (Messages::Sensor_msg& msg){ //assumes laserscan data is clockwise, yaw counterclockwise, robot starts facing the obstacle
    double yawAngle = getYawFromQuats(msg.odom.orientation);
    int yawAngleRounded = std::round(rad2deg(yawAngle));
    double laserRangeInFront = msg.laser.ranges[360-yawAngleRounded];
    if (laserRangeInFront <= CRITICAL_DISTANCE){
        return true;
    } else {
        return false;
    }
};

bool Functions::turnedToAngle(Messages::Odometry_msg& msg, double angleToTurnTo){
    double yaw = getYawFromQuats(msg.orientation);
    if (yaw - angleToTurnTo <=0.1){
        return true;
    } else{
        return false;
    }
};*/

Messages::Twist_msg_and_distance Functions::linearController(double k_alpha,double k_beta, double k_rho, double goal_x, double goal_y, double goal_theta, double currentX, double currentY, double currentYaw){ 

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
    beta =  -currentYaw + goal_theta - alpha;
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
    msg.delta_yaw = abs(delta_theta);
    return msg;
}

Messages::Odometry_msg Functions::createWaypoint(double x, double y, double yawInDeg){
    Messages::Odometry_msg temp;
    temp.position.x = x;
    temp.position.y = y;
    temp.orientation = Functions::yawToQuaternion(Functions::deg2rad(yawInDeg));
    return temp;
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

