# AP4Robs
http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html

# Links to Flow Charts
https://lucid.app/lucidchart/c135d951-767e-48e0-8052-be025c8cb730/edit?viewport_loc=-339%2C59%2C2269%2C1341%2C0_0&invitationId=inv_e5befede-f15c-441f-abaf-46d66ddbcc32
https://lucid.app/lucidchart/83606a56-d042-4ece-9cc7-f783b8765efa/edit?viewport_loc=-608%2C510%2C2941%2C1738%2C0_0&invitationId=inv_fd26caf8-a546-486e-9556-62d4aeaa9100

# shm-sem-prod-cons

A process gets forked and uses shared memories and semaphores to send a struct consisting of position, orientation and ranges to the child process

# Functions.cpp

Helper functions to be used further along the project 

# Subscriber.cpp
Function that takes Server Port (9997 or 9998) and expected number of Chars per Message (/scan ~9000, /odom ?) as arguments and returns the received message as char std::vector IP ADRESS IS CURRENTLY HARDCODED TO TURTLEBOT 4!

# Publisher.cpp
Function that takes a string of a twist message as argument and publishes it on port 9999. IP ADRESS IS CURRENTLY HARDCODED TO TURTLEBOT 4!
