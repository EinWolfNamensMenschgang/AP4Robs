#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include "messages.h"

namespace Functions{
Messages::Quaternion yawToQuaternion(double yaw);

double quaternionToYaw (Messages::Quaternion quats);

double deg2rad(double angleInDegree);

double rad2deg(double angleInRad);

bool wallReached (Messages::Sensor_msg& msg);//assumes laserscan data is clockwise, yaw counterclockwise, robot starts facing the obstacle

bool turnedToAngle(Messages::Odometry_msg& msg, double angleToTurnTo);

Messages::Twist_msg_and_distance linearController(double k_alpha,double k_beta, double k_rho, double goal_x, double goal_y, double goal_theta, double currentX, double currentY, double currentYaw);

Messages::Odometry_msg createWaypoint(double x, double y, double yaw);

}
#endif