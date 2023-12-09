#ifndef PARSING_H
#define PARSING_H
#include "messages.h"

namespace Parsing{
    bool parse_msg(std::string lidarMsg, std::string* parsedMsg);
    bool isolate_LIDAR_ranges(std::string* parsedMsg, std::array<double, 360>* doubleArray);
    bool isolate_odometry_data(std::string* parsedMsg, Messages::Odometry_msg* odomMsg, Messages::Twist_msg* twistMsg);
}

#endif

