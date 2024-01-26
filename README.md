# Advanced Programming for Robots
This project implements a TCP/IP connection to read a Turtlebot 3's sensor data and control it along waypoints using a linear controller.

## Flowchart:
![alt text](https://github.com/EinWolfNamensMenschgang/AP4Robs/blob/main/Flowchart.png?raw=true)
## How to run the programm:
git clone the files and run `make` in the folder and then `./programm`. The IP-Adress of the Turtlebot and the ports of the topics are currently hardcoded at the top of the main.cpp file. Change them accordingly. 
## The files:
The Subscriber.cpp and Publisher.cpp files create a TCP/IP connection to the Trutlebot and send or receive data, respectively. The received data is a string. To extract the data the functions in Parsing.cpp are used. Functions.cpp includes math functions and the linear controller.
