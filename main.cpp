/* File: shm-prod-cons.cpp
 *
 * This program sets up producer/consumer problem using shared memory.
 * Semaphores are used to synchonize the processes
 *
 * Contains both the producer and the consumer code
 * The producer (parent) remains the foreground process, so to stop
 * this program, send it SIGINT (Ctrl-C) and it will send SIGINT to
 * kill the consumer.  Both will detach the shared memory and the
 * producer will remove it and semaphores from the system.
 */

#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <stdlib.h>  
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <iostream>
#include <thread>
#include <pthread.h>
#include "Subscriber.h"
#include "Publisher.h"
#include "Functions.h"
#include "messages.h"
#include "parsing.h"

#define SCAN_RCVBUFSIZE 9000   /* Size of receive buffer */
#define ODOM_RCVBUFSIZE 1000   /* Size of receive buffer */
#define ODOM_PORT 9998 //9997 for /scan, 9998 for /odom
#define SCAN_PORT 9997



// Prototypes
void producerHandler (int sig);  // Signal handler for the producer
void consumerHandler (int sig);  // Signal handler for the consumer

void signalSem (int semid);      // Signal operation on semaphore
void waitSem (int semid);        // Wait operation on semaphore

// Global definitions and variables
const int BUFFER_SIZE = 2;   // Number of buffer slots
struct SharedMemory          // Format of the shared memory
{
   int numItems;             // Count of items in buffer
   int in, out;              // Indexes to back and front
   Messages::Sensor_msg buffer[BUFFER_SIZE];  // Items are Odometry_msgs
};

int shmid;                       // Shared memory id
SharedMemory *shmptr;            // Pointer to shared memory
pid_t child_pid;                 // Result of fork()
int mutex, empty, full, init;    // Semaphore ids
   
int main()
{
  union semun
  {
    int val;
    struct semid_ds *buf;
    ushort *array;
  } arg;

   // create shared memory
   shmid = shmget(IPC_PRIVATE, sizeof(struct SharedMemory), 0666|IPC_CREAT);   
   if (shmid == -1) {
      std::cerr << "shmget failed\n";
      exit(EXIT_FAILURE);
   }

   // set up semaphores
   // mutex - protect numItems
   if ((mutex = semget (IPC_PRIVATE, 1, 0666)) != -1)
   {
      arg.val = 1;
      if (semctl (mutex, 0, SETVAL, arg) == -1)
      {
         perror("mutex -- initialization ");
         // delete sem, detach and delete shm,
         // really should test error codes, too
         semctl (mutex, -1, IPC_RMID);
         shmdt(shmptr);
         shmctl(shmid, IPC_RMID, 0);
         exit(EXIT_FAILURE);
      }  // end semctl failed
   }
   else
   {
      perror ("mutex -- creation ");
      // detach and delete shm, really should test error codes, too
      shmdt(shmptr);
      shmctl(shmid, IPC_RMID, 0);
      exit(4);
   }  // end semget failed

   // full - producer signals item added; consumer waits
   if ((full = semget (IPC_PRIVATE, 1, 0666)) != -1)
   {
      arg.val = 0;
      if (semctl (full, 0, SETVAL, arg) == -1)
      {
         perror("full -- initialization ");
         // delete sem, detach and delete shm,
         // really should test error codes, too
         semctl (full, -1, IPC_RMID);
         semctl (mutex, -1, IPC_RMID);
         shmdt(shmptr);
         shmctl(shmid, IPC_RMID, 0);
         exit(EXIT_FAILURE);
      }  // end semctl failed
   }
   else
   {
      perror ("full -- creation ");
      // detach and delete shm, really should test error codes, too
      shmdt(shmptr);
      shmctl(shmid, IPC_RMID, 0);
      semctl (mutex, -1, IPC_RMID);
      exit(4);
   }  // end semget failed

   // empty - consumer signals item removed; producer waits
   if ((empty = semget (IPC_PRIVATE, 1, 0666)) != -1)
   {
      arg.val = BUFFER_SIZE;
      if (semctl (empty, 0, SETVAL, arg) == -1)
      {
         perror("empty -- initialization ");
         // delete sem, detach and delete shm,
         // really should test error codes, too
         semctl (empty, -1, IPC_RMID);
         semctl (full, -1, IPC_RMID);
         semctl (mutex, -1, IPC_RMID);
         shmdt(shmptr);
         shmctl(shmid, IPC_RMID, 0);
         exit(EXIT_FAILURE);
      }  // end semctl failed
   }
   else
   {
      perror ("empty -- creation ");
      // detach and delete shm, really should test error codes, too
      semctl (full, -1, IPC_RMID);
      semctl (mutex, -1, IPC_RMID);
      shmdt(shmptr);
      shmctl(shmid, IPC_RMID, 0);
      exit(4);
   }  // end semget failed

   // init - producer signals shm intitialization is complete
   if ((init = semget (IPC_PRIVATE, 1, 0666)) != -1)
   {
      arg.val = 0;
      if (semctl (init, 0, SETVAL, arg) == -1)
      {
         perror("init -- initialization ");
         // delete sem, detach and delete shm,
         // really should test error codes, too
         semctl (init, -1, IPC_RMID);
         semctl (empty, -1, IPC_RMID);
         semctl (full, -1, IPC_RMID);
         semctl (mutex, -1, IPC_RMID);
         shmdt(shmptr);
         shmctl(shmid, IPC_RMID, 0);
         exit(EXIT_FAILURE);
      }  // end semctl failed
   }
   else
   {
      perror ("init -- creation ");
      // detach and delete shm, really should test error codes, too
      semctl (empty, -1, IPC_RMID);
      semctl (full, -1, IPC_RMID);
      semctl (mutex, -1, IPC_RMID);
      shmdt(shmptr);
      shmctl(shmid, IPC_RMID, 0);
      exit(4);
   }  // end semget failed

//------------------TODO: Insert Waypoint Array Creation ---------------------------
int numberOfWaypoints = 10;
int waypointIndex = 0;
Messages::Odometry_msg waypoints[numberOfWaypoints];

//----------------------------------------------------------------------------------

   /* fork a child process */
   child_pid = fork();
   if (child_pid < 0) { /* error occurred */
      std::cerr << "Fork Failed\n";
      exit(-1);
   }

   if (child_pid == 0) // child process - the consumer TWIST_MSG PUBLISHER
   { 
      signal (SIGINT, consumerHandler);  // catch SIGINT
      std::cout << "I am the consumer\n";
//           << "Consumer will sleep for a bit to make sure that producer "
//           << "initializes shared memory" << std::endl;
//      sleep(1);

      waitSem(init);
      // attach shared memory at shmptr
      std::cout << "Consumer: attaching shared memory\n";
      shmptr = (SharedMemory*)shmat(shmid, NULL, 0);
      if (shmptr == (SharedMemory *)-1) {
         std::cerr << "shmat failed\n";
         exit(EXIT_FAILURE);
      }
      std::cout << "Consumer: memory attached\n";

      // consuming loop
      while (true)
      {
         waitSem(full);
         waitSem(mutex);
         shmptr->numItems--;
         int out = shmptr->out;
         Messages::Sensor_msg nextConsumed = shmptr->buffer[out];
         shmptr->out = (shmptr->out + 1) % BUFFER_SIZE;
         signalSem(mutex); 
         signalSem(empty);
         std::cout << "Consumer read: x: " << nextConsumed.odom.position.x << " y: " << nextConsumed.odom.position.y << " ranges: "<< nextConsumed.laser.ranges[0]
              << " at index " << out << std::endl;
         sleep(0.1);

//------------TODO: Insert Calculate Position From Laser Readings---------------
      double laserX;
      double laserY;
      double laserYAW;
//------------------------------------------------------------------------------
      double trustOdom = 1.0; //For Testing
      double trustLaser = 0.0; //trustOdom + trustLaser = 1;
      double assumedX = trustOdom*nextConsumed.odom.position.x + trustLaser*laserX;
      double assumedY = trustOdom*nextConsumed.odom.position.y + trustLaser*laserY;
      double odomYAW = Functions::getYawFromQuats(nextConsumed.odom.orientation);
      double assumedYAW = trustOdom*odomYAW + trustLaser*laserYAW; //Or just use odomYAW if calculating laserYAW is too difficult

      Messages::Twist_msg_and_distance lc_msg;
      Messages::Odometry_msg goal = waypoints[waypointIndex];
      double goalYaw = Functions::getYawFromQuats(goal.orientation);
      double k_alpha = 0.6;
      double k_rho = 0.2;
      double k_beta = -0.2;

      lc_msg = Functions::linearController(k_alpha, k_beta, k_rho, goal.position.x, goal.position.y, goalYaw, assumedX, assumedY, assumedYAW);    

Messages::Twist_msg cmd_vel = lc_msg.twist;
std::string twist_string = "---START---{\"linear\": " + std::to_string(cmd_vel.x_vel)+ ", \"angular\": " + std::to_string(cmd_vel.angular_vel) + "}___END___";


if(lc_msg.distance < 0.05){
   waypointIndex++;
}else if(waypointIndex == numberOfWaypoints){ 
Publisher::publish("---START---{\"linear\": 0.0, \"angular\": 0.0}___END___");

}else{
Publisher::publish(twist_string);
}

      }  // end consuming loop
   }  // end consumer code
   else  // parent process - the producer
   { 
      signal (SIGINT, producerHandler);  // catch SIGINT
      // attach shared memory
      shmptr = (SharedMemory *)shmat(shmid, (void *)0,0);
      if (shmptr == (SharedMemory *)-1) {
         std::cerr << "shmat failed\n";
         exit(EXIT_FAILURE);
      }
      // initialize shared memory
      shmptr->in = 0;
      shmptr->out = 0;
      shmptr->numItems = 0;
      signalSem(init);
      
      // producing loop
      while (true)
      {
        std::vector<char> odom_msg;
        std::vector<char> laser_msg;
        bool odomSuccess = false;
        bool laserSuccess = false;
        bool castSuccess = false;
        Messages::Sensor_msg msg;
        Messages::Odometry_msg* odomPointer = &msg.odom;
        std::array<double, 360>* laserPointer = &msg.laser.ranges;
        while(!castSuccess){
        std::thread OdomSub(Subscriber::subscribe, ODOM_PORT, ODOM_RCVBUFSIZE, std::ref(odom_msg));
        std::thread LaserSub(Subscriber::subscribe, SCAN_PORT, SCAN_RCVBUFSIZE, std::ref(laser_msg));
        OdomSub.join(); //wait for OdomSub to finish
        LaserSub.join(); //wait for LaserSub to finish
        std::string odomStringData(odom_msg.begin(), odom_msg.end());
        std::cout << "odomStringData: " << odomStringData << std::endl;
        std::string laserStringData(laser_msg.begin(), laser_msg.end());
        std::cout << "laserStringData: " << laserStringData << std::endl;
         std::string* odomString = new std::string;
         std::string* laserString = new std::string;
         Parsing::parse_msg(odomStringData, odomString);
         Parsing::parse_msg(laserStringData, laserString);
         odomSuccess = Parsing::isolate_odometry_data(odomString, odomPointer);
         laserSuccess = Parsing::isolate_LIDAR_ranges(laserString, laserPointer);
         castSuccess = odomSuccess && laserSuccess;
         //std::cout << "Laser: " << laserSuccess << std::endl;
         //std::cout << "Odom : " << odomSuccess << std::endl;
         delete odomString;
         delete laserString;
        }
         
         waitSem(empty);
         waitSem(mutex);
         int in = shmptr->in;
         shmptr->buffer[in] = msg;
         shmptr->in = (shmptr->in + 1) % BUFFER_SIZE;
         shmptr->numItems++;
         signalSem(mutex);
         signalSem(full);
         std::cout << "Producer wrote: x:" << msg.odom.position.x << " y: "<< msg.odom.position.y << " range: " << msg.laser.ranges[0] << std::endl;
         sleep(0.1); 
      }  // end producing loop
   }  // end producer code
}  // end main

// Signal handler for the consumer
void consumerHandler (int sig)
{
   // detach shared memory
   std::cout << "Consumer detaching shared memory\n";
   if (shmdt(shmptr) == -1) {
      std::cerr << "shmdt failed\n";
      exit(EXIT_FAILURE);
   }
   exit(EXIT_SUCCESS);
}  // end consumerHandler

// Signal handler for the producer
void producerHandler (int sig)
{
   // kill child and wait for it
   std::cout << "Producer killing consumer\n";
   kill (child_pid, SIGINT);
   wait (NULL);

   // delete semaphores - should check returns
   std::cout << "Producer removing semaphores\n";
   semctl (init, -1, IPC_RMID);
   semctl (empty, -1, IPC_RMID);
   semctl (full, -1, IPC_RMID);
   semctl (mutex, -1, IPC_RMID);

   // detach shared memory
   std::cout << "Producer detaching and removing shared memory\n";
   if (shmdt(shmptr) == -1) {
      std::cerr << "shmdt failed\n";
      exit(EXIT_FAILURE);
   }
   // remove shared memory
   if (shmctl(shmid, IPC_RMID, 0) == -1) {
      std::cerr << "shmctl(IPC_RMID) failed\n";
      exit(EXIT_FAILURE);
   }
   exit(EXIT_SUCCESS);
}  // end producerHandler

// Signal operation on semaphore
void signalSem (int semid)
{
   // semaphore operation to signal
   struct sembuf release = {0, 1, SEM_UNDO}; 

   if (semop(semid, &release, 1) == -1)
   {
      perror("semid -- release ");
      exit(EXIT_FAILURE);
   }  // end release failure

}  // end signalSem

// Wait operation on semaphore
void waitSem (int semid)
{
   // semaphore operation to wait
   struct sembuf acquire = {0, -1, SEM_UNDO};
   
   if (semop(semid, &acquire, 1) == -1)
   {
      perror("semid -- acquire ");
      exit(EXIT_FAILURE);
   }  // end acquire failure
   
}  // end waitSem
