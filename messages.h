#ifndef MESSAGES_H
#define MESSAGES_H
namespace Messages{
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
};

struct Laserscan_msg {
   double ranges[360];
};

struct Sensor_msg {
   Odometry_msg odom;
   Laserscan_msg laser;
};

struct Twist_msg {
    double x_vel;
    double angular_vel;
};

struct Twist_msg_and_distance{
    Twist_msg twist;
    double distance;
};
}
#endif