#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include "messages.h"

namespace Functions{

double getYawFromQuats (Messages::Quaternion quats);

double rad2deg(double angleInRad);

bool wallReached (Messages::Sensor_msg& msg);//assumes laserscan data is clockwise, yaw counterclockwise, robot starts facing the obstacle

bool turnedToAngle(Messages::Odometry_msg& msg, double angleToTurnTo);

Messages::Twist_msg_and_distance linearController(double k_alpha,double k_beta, double k_rho, double goal_x, double goal_y, double goal_theta, double currentX, double currentY, double currentYaw);
}
#endif