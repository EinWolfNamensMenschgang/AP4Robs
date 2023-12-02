#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <array>

std::string read_LIDAR_msg(const std::string& file_path) {
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

bool parse_LIDAR_msg(std::string lidarMsg, std::string* parsedMsg){
    // Define start signal
    std::string startSignal = "---START---";
    std::string endSignal = "___END___";

    // Find the positions of start signal
    size_t startPosition = lidarMsg.find(startSignal);

    // Check if the start signal is found
    if (startPosition == std::string::npos) {
        return false;
    }

    // Find the position of the second occurrence of start signal
    size_t endPosition = lidarMsg.find(startSignal, startPosition + startSignal.length());

    // Check if the second start signal is found
    if (endPosition == std::string::npos) {
        return false;
    }

    // Extract the text between the two start signals
    *parsedMsg = lidarMsg.substr(startPosition + startSignal.length(), endPosition - (startPosition + startSignal.length()));

    // Check if the extracted text contains an end signal
    size_t endSignalPosition = parsedMsg->find(endSignal);

    // If the parsed msg contains an end signal, remove it
    if (endSignalPosition != std::string::npos) {
        parsedMsg->erase(endSignalPosition, endSignal.length());
    }

    // If the parsed msg does not contain a whole msg return false, else true
    return endSignalPosition != std::string::npos;
}

bool isolate_LIDAR_ranges(const std::string* parsedMsg, std::array<double, 360>* doubleArray) {
    // Define start signal
    std::string startSignal = "\"ranges\": [";
    std::string endSignal = "]";

    // Find the positions of start and end signals
    size_t startPosition = parsedMsg->find(startSignal);
    size_t endPosition = parsedMsg->find(endSignal);

    // Check if both signals are found
    if (startPosition == std::string::npos || endPosition == std::string::npos) {
        std::cout << "Start or end signal not found!\n";
        return false;
    }

    // Extract the text between the signals
    std::string strRanges = parsedMsg->substr(startPosition + startSignal.length(), endPosition - (startPosition + startSignal.length()));

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

    // Display the values
    std::cout << "Values from CSV string:\n";
    for (double value : *doubleArray) {
        std::cout << value << " ";
    }

    return true;
}




int main() {
    // Additonal return values to give them to functions as references (pointers)
    std::string* parsedMsg = new std::string;
    std::array<double,360> doubleArray;

    // bools for checking
    bool wholeMsg = false;
    bool fullLidarRange = false;

    // Read data from file, parse Lidar msg and isolate the ranges
    std::string lidarMsg = read_LIDAR_msg("/home/re23m007/advancedProgrammingForRobotics/Messages/laserscan.txt");
    wholeMsg = parse_LIDAR_msg(lidarMsg, parsedMsg);
    fullLidarRange = isolate_LIDAR_ranges(parsedMsg, &doubleArray);

    // Print some results
    std::cout << "Whole Msg. isolated: " << wholeMsg << std::endl;
    std::cout << "Whole Msg. isolated: " << fullLidarRange << std::endl;
    for(double item : doubleArray){
        std::cout << item << std::endl;
    }
    
    //Free memory for avoiding memory leaks!!
    delete parsedMsg;

    return 0;
}
