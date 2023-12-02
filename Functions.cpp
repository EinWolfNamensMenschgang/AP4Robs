//Functions for AP4Robos
#define _USE_MATH_DEFINES
#define GENERAL_VELOCITY 1.0
#define CRITICAL_DISTANCE 0.1
#define K_ALPHA 0.1
#include <cmath>


struct Position{
   double x;
   double y;
   double z;
};

struct Quaternion{
   double x;
   double y;
   double z;
   double w;
};

struct Odometry_msg {
   Position position;
   Quaternion orientation;
   double ranges[360];
};

struct Twist_msg {
    Position linear;
    Position angular;
};

double getYawFromQuats (Quaternion quats){ //assumes normalized Quaternions
    double siny_cosp = 2* (quats.w*quats.z+quats.x*quats.y);
    double cosy_cosp = 1 - 2 * (quats.y * quats.y + quats.z * quats.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    //int yaw_rounded = std::round(yaw);
    return yaw;
};

bool wallReached (Odometry_msg& msg){ //assumes laserscan data is clockwise, yaw counterclockwise, robot starts facing the obstacle
    double yawAngle = getYawFromQuats(msg.orientation);
    int yawAngleRounded = std::round(yawAngle);
    double laserRangeInFront = msg.ranges[360-yawAngleRounded];
    if (laserRangeInFront <= CRITICAL_DISTANCE){
        return true;
    } else {
        return false;
    }
};

bool turnedToAngle(Odometry_msg& msg, double angleToTurnTo){
    double yaw = getYawFromQuats(msg.orientation);
    if (yaw - angleToTurnTo <=0.1){
        return true;
    } else{
        return false;
    }
};

void publish(Twist_msg msg, int ipAdress){
    /*TCP IP shenanigans*/
};

Twist_msg linearControllerStraight(Odometry_msg& msg){
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
};

