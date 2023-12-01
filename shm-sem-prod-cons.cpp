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
using namespace std;

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
   Odometry_msg buffer[BUFFER_SIZE];  // Items are Odometry_msgs
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
      cerr << "shmget failed\n";
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

   /* fork a child process */
   child_pid = fork();
   if (child_pid < 0) { /* error occurred */
      cerr << "Fork Failed\n";
      exit(-1);
   }

   if (child_pid == 0) // child process - the consumer
   { 
      signal (SIGINT, consumerHandler);  // catch SIGINT
      cout << "I am the consumer\n";
//           << "Consumer will sleep for a bit to make sure that producer "
//           << "initializes shared memory" << endl;
//      sleep(1);

      waitSem(init);
      // attach shared memory at shmptr
      cout << "Consumer: attaching shared memory\n";
      shmptr = (SharedMemory*)shmat(shmid, NULL, 0);
      if (shmptr == (SharedMemory *)-1) {
         cerr << "shmat failed\n";
         exit(EXIT_FAILURE);
      }
      cout << "Consumer: memory attached\n";

      // consuming loop
      while (true)
      {
         //cout << "Consumer attempting to read\n";
//         while (shmptr->numItems == 0)
//            ;  // do nothing
         //cout << "Consumer - Wait on full\n";
         waitSem(full);
         //cout << "Consumer - Wait on mutex\n";
         waitSem(mutex);
         //cout << "Consumer - Execute critical section\n";
         shmptr->numItems--;
         int out = shmptr->out;
         Odometry_msg nextConsumed = shmptr->buffer[out];
         shmptr->out = (shmptr->out + 1) % BUFFER_SIZE;
         //sleep(rand()%3);  // 0-2 seconds
         //cout << "Consumer - Signal mutex\n";
         signalSem(mutex); 
         //cout << "Consumer - Signal empty\n";
         signalSem(empty);
         cout << "Consumer read: x: " << nextConsumed.position.x << " y: " << nextConsumed.position.y << " ranges: "<< nextConsumed.ranges[0]
              << " at index " << out << endl;
         sleep(0.1);  // 0.1 seconds
      }  // end consuming loop
   }  // end consumer code
   else  // parent process - the producer
   { 
      signal (SIGINT, producerHandler);  // catch SIGINT
      //cout << "I am the producer\n";

      // attach shared memory
      //cout << "Producer: attaching shared memory\n";
      shmptr = (SharedMemory *)shmat(shmid, (void *)0,0);
      if (shmptr == (SharedMemory *)-1) {
         cerr << "shmat failed\n";
         exit(EXIT_FAILURE);
      }
      //cout << "Producer: memory attached\n";

      // initialize shared memory
      //cout << "Producer: initializing shared memory\n";
      shmptr->in = 0;
      shmptr->out = 0;
      shmptr->numItems = 0;
      //cout << "Producer: shared memory initialized\n";
      signalSem(init);
      
      // producing loop
      while (true)
      {
         //cout << "Producer attempting write\n";
         // random integer between 0 and 9999
//         while (shmptr->numItems == BUFFER_SIZE)
//            ;  // do nothing
        Odometry_msg msg;
        msg.position.x = 0.0;
        msg.position.y = 0.0;
        msg.position.z = 0.0;
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;

         //cout << "Producer - Wait on empty\n";
         waitSem(empty);
         //cout << "Producer - Wait on mutex\n";
         waitSem(mutex);
         //cout << "Producer - Execute critical section\n";
         int in = shmptr->in;
         shmptr->buffer[in] = msg;
         shmptr->in = (shmptr->in + 1) % BUFFER_SIZE;
         shmptr->numItems++;
         //sleep(rand()%3);  // 0-2 seconds
         //cout << "Producer - Signal mutex\n";
         signalSem(mutex);
         //cout << "Producer - Signal full\n";
         signalSem(full);
         cout << "Producer wrote: x:" << msg.position.x << " y: "<< msg.position.y << " range: " << msg.ranges[0]
              << " at index " << in << endl;
         sleep(1);  // 0-2 seconds
      }  // end producing loop
   }  // end producer code
}  // end main

// Signal handler for the consumer
void consumerHandler (int sig)
{
   // detach shared memory
   cout << "Consumer detaching shared memory\n";
   if (shmdt(shmptr) == -1) {
      cerr << "shmdt failed\n";
      exit(EXIT_FAILURE);
   }
   exit(EXIT_SUCCESS);
}  // end consumerHandler

// Signal handler for the producer
void producerHandler (int sig)
{
   // kill child and wait for it
   cout << "Producer killing consumer\n";
   kill (child_pid, SIGINT);
   wait (NULL);

   // delete semaphores - should check returns
   cout << "Producer removing semaphores\n";
   semctl (init, -1, IPC_RMID);
   semctl (empty, -1, IPC_RMID);
   semctl (full, -1, IPC_RMID);
   semctl (mutex, -1, IPC_RMID);

   // detach shared memory
   cout << "Producer detaching and removing shared memory\n";
   if (shmdt(shmptr) == -1) {
      cerr << "shmdt failed\n";
      exit(EXIT_FAILURE);
   }
   // remove shared memory
   if (shmctl(shmid, IPC_RMID, 0) == -1) {
      cerr << "shmctl(IPC_RMID) failed\n";
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
