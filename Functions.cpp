//Functions for AP4Robos
#define _USE_MATH_DEFINES
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

double getYawFromQuats (Quaternion quats){ //assumes normalized Quaternions
    double siny_cosp = 2* (quats.w*quats.z+quats.x*quats.y);
    double cosy_cosp = 1 - 2 * (quats.y * quats.y + quats.z * quats.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    //int yaw_rounded = std::round(yaw);
    return yaw;
};

bool wallReached (Odometry_msg& msg, double criticalDistance){ //assumes laserscan data is clockwise, yaw counterclockwise, robot starts facing the obstacle
    double yawAngle = getYawFromQuats(msg.orientation);
    int yawAngleRounded = std::round(yawAngle);
    double laserRangeInFront = msg.ranges[360-yawAngleRounded];
    if (laserRangeInFront <= criticalDistance){
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

