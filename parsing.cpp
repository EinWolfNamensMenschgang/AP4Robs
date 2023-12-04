#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <array>

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

// forward declarations
std::string read_msg(const std::string& file_path);
bool parse_msg(std::string lidarMsg, std::string* parsedMsg);
std::string isolate_string_inbetween_two_signals(std::string* parsedMsg, std::string startSignal, std::string endSignal);
bool isolate_LIDAR_ranges(const std::string* parsedMsg, std::array<double, 360>* doubleArray);
bool isolate_odometry_data(std::string* parsedMsg, Odometry_msg* odomMsg, Twist_msg* twistMsg);


std::string read_msg(const std::string& file_path) {
    // Open the file
    std::ifstream file(file_path);

    // Check if the file is opened successfully
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return "File did not open!";
    }

    // Read the file contents into a string
    std::string fileContent((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    // Close the file
    file.close();

    return fileContent;
}

bool parse_msg(std::string msg, std::string* parsedMsg){
    // Define start signal
    std::string startSignal = "---START---";
    std::string endSignal = "___END___";

    // Find the positions of start signal
    size_t startPosition = msg.find(startSignal);

    // Check if the start signal is found
    if (startPosition == std::string::npos) {
        return false;
    }

    // Find the position of the second occurrence of start signal
    size_t endPosition = msg.find(startSignal, startPosition + startSignal.length());

    // Check if the second start signal is found
    if (endPosition == std::string::npos) {
        return false;
    }

    // Extract the text between the two start signals
    *parsedMsg = msg.substr(startPosition + startSignal.length(), endPosition - (startPosition + startSignal.length()));

    // Check if the extracted text contains an end signal
    size_t endSignalPosition = parsedMsg->find(endSignal);

    // If the parsed msg contains an end signal, remove it
    if (endSignalPosition != std::string::npos) {
        parsedMsg->erase(endSignalPosition, endSignal.length());
    }

    // If the parsed msg does not contain a whole msg return false, else true
    return endSignalPosition != std::string::npos;
}

std::string isolate_string_inbetween_two_signals(std::string* parsedMsg, std::string startSignal, std::string endSignal){
    // Find the positions of start and end signals
    size_t startPosition = parsedMsg->find(startSignal);
    size_t endPosition = parsedMsg->find(endSignal,startPosition);

    // Check if both signals are found
    if (startPosition == std::string::npos || endPosition == std::string::npos) {
        std::cout << "Start or end signal not found!\n";
        return "";
    }

    // Extract the text between the signals
    return parsedMsg->substr(startPosition + startSignal.length(), endPosition - (startPosition + startSignal.length()));
}

bool isolate_LIDAR_ranges(std::string* parsedMsg, std::array<double, 360>* doubleArray) {
    // Isolate string between two signals
    std::string strRanges = isolate_string_inbetween_two_signals(parsedMsg, "\"ranges\": [", "]");
    
    // Create a stringstream from the CSV string
    std::istringstream iss(strRanges);

    // Temporary variable to hold each value while parsing
    double tempValue;

    // Read values from the stringstream and store them in the array
    for (size_t i = 0; i < doubleArray->size(); ++i) {
        if (!(iss >> tempValue)) {
            std::cout << "Error parsing value at position " << i << "\n";
            return false;
        }

        // Check if the character after the value is a comma
        if (iss.peek() == ',') {
            iss.ignore(); // Ignore the comma
        }

        // Store the value in the array
        (*doubleArray)[i] = tempValue;
    }

    return true;
}

bool isolate_odometry_data(std::string* parsedMsg, Odometry_msg* odomMsg, Twist_msg* twistMsg){
    // Isolate the odometry's positions x, y, z, cast it to double, and store it in odomMsg
    std::string odomPosePosition = isolate_string_inbetween_two_signals(parsedMsg, "\"position\": {","}");
    odomMsg->position.x = std::stod(isolate_string_inbetween_two_signals(&odomPosePosition, "\"x\":", ","));
    odomMsg->position.y = std::stod(isolate_string_inbetween_two_signals(&odomPosePosition, "\"y\":", ","));
    odomMsg->position.z = std::stod(isolate_string_inbetween_two_signals(&odomPosePosition, "\"z\":", "\0"));

    // Isolate the odometry's orientation x, y, z, w, cast it to double, and store it in odomMsg
    std::string odomPoseOrientation = isolate_string_inbetween_two_signals(parsedMsg, "\"orientation\": {","}");
    odomMsg->orientation.x = std::stod(isolate_string_inbetween_two_signals(&odomPoseOrientation, "\"x\": ", ","));
    odomMsg->orientation.y = std::stod(isolate_string_inbetween_two_signals(&odomPoseOrientation, "\"y\": ", ","));
    odomMsg->orientation.z = std::stod(isolate_string_inbetween_two_signals(&odomPoseOrientation, "\"z\": ", ","));
    odomMsg->orientation.w = std::stod(isolate_string_inbetween_two_signals(&odomPoseOrientation, "\"w\": ", "\0"));

    // Isolate the linear twist x, y, z, cast it to double, and store it in twistMsg
    std::string odomTwist = isolate_string_inbetween_two_signals(parsedMsg, "\"linear\": {","}");
    twistMsg->linear.x = std::stod(isolate_string_inbetween_two_signals(&odomTwist, "\"x\": ", ","));
    twistMsg->linear.y = std::stod(isolate_string_inbetween_two_signals(&odomTwist, "\"y\": ", ","));
    twistMsg->linear.z = std::stod(isolate_string_inbetween_two_signals(&odomTwist, "\"z\": ", "\0"));

    // Isolate the angular twist x, y, z, cast it to double, and store it in twistMsg
    std::string odomAngularTwist = isolate_string_inbetween_two_signals(parsedMsg, "\"angular\": {","}");
    twistMsg->angular.x = std::stod(isolate_string_inbetween_two_signals(&odomAngularTwist, "\"x\": ", ","));
    twistMsg->angular.y = std::stod(isolate_string_inbetween_two_signals(&odomAngularTwist, "\"y\": ", ","));
    twistMsg->angular.z = std::stod(isolate_string_inbetween_two_signals(&odomAngularTwist, "\"z\": ", "\0"));

    // Return true if all the data is casted successfully
    return true;
}

int main() {
    // Additonal return values to give them to functions as references (pointers)
    std::string* parsedOdomMsg = new std::string;
    Odometry_msg* odomMsg = new Odometry_msg;
    Twist_msg* twistMsg = new Twist_msg;

    // bools for checking
    bool wholeMsg = false;
    bool storingSuccess = false;

    // Read data from file and parse odometry msg
    std::string strOdomMsg = read_msg("/home/re23m007/advancedProgrammingForRobotics/Messages/Odom_data.txt");
    wholeMsg = parse_msg(strOdomMsg, parsedOdomMsg);

    // Store the pose and twist data in the given structs
    storingSuccess = isolate_odometry_data(parsedOdomMsg, odomMsg, twistMsg);

    // Check stored data
    std::cout << *parsedOdomMsg << std::endl;
    std::cout << "Pose Positions (x,y,z) = (" << odomMsg->position.x << "," << odomMsg->position.y << "," << odomMsg->position.z << ")" << std::endl;
    std::cout << "Pose Orientation (x,y,z,w) = (" << odomMsg->orientation.x << "," << odomMsg->orientation.y << "," << odomMsg->orientation.z << odomMsg->orientation.w <<")" <<std::endl;
    std::cout << "Linear Twist (x,y,z) = (" << twistMsg->linear.x << "," << twistMsg->linear.y << "," << twistMsg->linear.z << ")" <<std::endl;
    std::cout << "Angular Twist (x,y,z) = (" << twistMsg->angular.x << "," << twistMsg->angular.y << "," << twistMsg->angular.z << ")" <<std::endl;

    // Free data to avoid memory leaks
    delete parsedOdomMsg;
    delete odomMsg;
    delete twistMsg;

    /*// Additonal return values to give them to functions as references (pointers)
    std::string* parsedMsg = new std::string;
    std::array<double,360> doubleArray;

    // bools for checking
    bool wholeMsg = false;
    bool fullLidarRange = false;

    // Read data from file, parse Lidar msg and isolate the ranges
    std::string lidarMsg = read_LIDAR_msg("/home/re23m007/advancedProgrammingForRobotics/Messages/laserscan.txt");
    wholeMsg = parse_msg(lidarMsg, parsedMsg);
    fullLidarRange = isolate_LIDAR_ranges(parsedMsg, &doubleArray);

    // Print some results
    std::cout << "Whole Msg. isolated: " << wholeMsg << std::endl;
    std::cout << "Whole Msg. isolated: " << fullLidarRange << std::endl;
    for(double item : doubleArray){
        std::cout << item << std::endl;
    }
    
    //Free memory for avoiding memory leaks!!
    delete parsedMsg;*/

    return 0;
}
