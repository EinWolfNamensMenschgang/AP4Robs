#include <iostream>
#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_addr() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */

#define RCVBUFSIZE 32   /* Size of receive buffer */


int main(int argc, char *argv[])
{
    int sock;                        /* Socket descriptor */
    struct sockaddr_in echoServAddr; /* Echo server address */
    unsigned short ServPort;     /* Echo server port */
    char *servIP;                    /* Server IP address (dotted quad) */
    char *cmdString;                /* String to send to echo server */
    char echoBuffer[RCVBUFSIZE];     /* Buffer for echo string */
    unsigned int cmdStringLen;      /* Length of string to echo */
    char* linear;
    char* angular;
    int bytesRcvd, totalBytesRcvd;   /* Bytes read in single recv() 
                                        and total bytes read */

    if ((argc < 3) || (argc > 3))    /* Test for correct number of arguments */
    {
       fprintf(stderr, "Usage: %s <Server IP> <Linear Velocity> <Angular Velocity>\n",
               argv[0]);
       exit(1);
    }

    servIP = argv[1];             /* server IP address (dotted quad) */                                        
    linear = argv[2];             /* linear velocity */
    angular = argv[3];            /* angular velocity */
    ServPort = 9999;              /* Use given port */      


    /* Create a reliable, stream socket using TCP */
    if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        std::cout << "socket() failed" << std::endl;

    /* Construct the server address structure */
    memset(&echoServAddr, 0, sizeof(echoServAddr));     /* Zero out structure */
    echoServAddr.sin_family      = AF_INET;             /* Internet address family */
    echoServAddr.sin_addr.s_addr = inet_addr(servIP);   /* Server IP address */
    echoServAddr.sin_port        = htons(ServPort); /* Server port */

    /* Establish the connection to the echo server */
    if (connect(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0)
        std::cout << "connect() failed" << std::endl;

    /* hier wird der input von linear und angular auf einen String geschrieben in dem Format für den Turtlebot 
    und auf ein char* umgewandelt zum senden über TCP/IP */
    cmdString = sprintf(echoBuffer, "\"---START---{\"linear\":%d,\"angular\":%d}___END___\"", linear, angular);
    cmdStringLen = strlen(cmdString);          /* Determine input length */

    /* Send the string to the server */
    if (send(sock, cmdString, cmdStringLen, 0) != cmdStringLen)
        std::cout << "send() sent a different number of bytes than expected" << std::endl;


    printf("\n");    /* Print a final linefeed */

    close(sock);
    exit(0);        //exit auf loop umwandeln, sodass commander neu gestartet wird und anfangs auf input wartet
}
