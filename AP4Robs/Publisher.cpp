#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_addr() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */
#include <stdio.h>  /* for perror() */
#include <stdlib.h> /* for exit() */
#include <vector>
#include <iostream>
#include <string>
#include "Publisher.h"
#include "Subscriber.h"

#define RCVBUFSIZE 9000   /* Size of receive buffer */
#define SERVERPORT 9999 //9999 for /cmd_vel

/*void Subscriber::DieWithError(const char *errorMessage)
{
    perror(errorMessage);
    exit(1);
}*/

//std::string testString = "---START---{\"linear\": 0.0, \"angular\": 0.0}___END___";

void Publisher::publish(const char* serverIP,std::string twist_msg){ 
int sock;                        /* Socket descriptor */
    struct sockaddr_in echoServAddr; /* Echo server address */
    unsigned short echoServPort;     /* Echo server port */
    const char *servIP;                    /* Server IP address (dotted quad) */
    const char *echoString;                /* String to send to echo server */
    //char echoBuffer[messageSize];     /* Buffer for echo string */
    unsigned int echoStringLen;      /* Length of string to echo */
    int bytesRcvd, totalBytesRcvd;   /* Bytes read in single recv() 
                                        and total bytes read */

    servIP = serverIP;             /* First arg: server IP address (dotted quad) */
    echoString = twist_msg.c_str();         /* Second arg: string to echo */

    echoServPort = SERVERPORT; /* Use given port, if any */
  

    /* Create a reliable, stream socket using TCP */
    if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        Subscriber::DieWithError("socket() failed");

    /* Construct the server address structure */
    memset(&echoServAddr, 0, sizeof(echoServAddr));     /* Zero out structure */
    echoServAddr.sin_family      = AF_INET;             /* Internet address family */
    echoServAddr.sin_addr.s_addr = inet_addr(servIP);   /* Server IP address */
    echoServAddr.sin_port        = htons(echoServPort); /* Server port */

    /* Establish the connection to the echo server */
    if (connect(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0)
        Subscriber::DieWithError("connect() failed");

    echoStringLen = strlen(echoString);          /* Determine input length */

    /* Send the string to the server */
    if (send(sock, echoString, echoStringLen, 0) != echoStringLen)
        Subscriber::DieWithError("send() sent a different number of bytes than expected");
    close(sock);
}


/*int main(){
    publish(testString);
    return 0;
}*/