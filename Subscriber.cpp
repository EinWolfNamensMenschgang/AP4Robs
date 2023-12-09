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
#include "Subscriber.h"
#include "messages.h"
#define SCAN_RCVBUFSIZE 9000   /* Size of receive buffer */
#define ODOM_PORT 9998 //9997 for /scan, 9998 for /odom
#define SCAN_PORT 9997

void Subscriber::DieWithError(const char *errorMessage)
{
    perror(errorMessage);
    exit(1);
}


void Subscriber::subscribe(int port, int messageSize, std::vector<char>& returnString){ //SERVERPORT, message size is expected number of chars in one Message
int sock;                        /* Socket descriptor */
    struct sockaddr_in echoServAddr; /* Echo server address */
    unsigned short echoServPort;     /* Echo server port */
    const char *servIP;                    /* Server IP address (dotted quad) */
    const char *echoString;                /* String to send to echo server */
    char echoBuffer[messageSize];     /* Buffer for echo string */
    unsigned int echoStringLen;      /* Length of string to echo */
    int bytesRcvd, totalBytesRcvd;   /* Bytes read in single recv() 
                                        and total bytes read */

    servIP = "192.168.100.54";             /* First arg: server IP address (dotted quad) */
    //echoString = "";         /* Second arg: string to echo */

    echoServPort = port; /* Use given port, if any */
  

    /* Create a reliable, stream socket using TCP */
    if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        DieWithError("socket() failed");

    /* Construct the server address structure */
    memset(&echoServAddr, 0, sizeof(echoServAddr));     /* Zero out structure */
    echoServAddr.sin_family      = AF_INET;             /* Internet address family */
    echoServAddr.sin_addr.s_addr = inet_addr(servIP);   /* Server IP address */
    echoServAddr.sin_port        = htons(echoServPort); /* Server port */

    /* Establish the connection to the echo server */
    if (connect(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0)
        DieWithError("connect() failed");

    //echoStringLen = strlen(echoString);          /* Determine input length */

    /* Send the string to the server */
    //if (send(sock, echoString, echoStringLen, 0) != echoStringLen)
        //DieWithError("send() sent a different number of bytes than expected");

    /* Receive the same string back from the server */
    totalBytesRcvd = 0;
    std::vector<char> finalMessage;
    printf("Received: ");                /* Setup to print the echoed string */
    while (totalBytesRcvd < messageSize)
     {
        /* Receive up to the buffer size (minus 1 to leave space for
           a null terminator) bytes from the sender */
        if ((bytesRcvd = recv(sock, echoBuffer, messageSize - 1, 0)) <= 0)
            DieWithError("recv() failed or connection closed prematurely");
        
        totalBytesRcvd += bytesRcvd; /* Keep tally of total bytes */
        for(int i = 0; i<bytesRcvd; i++){
            finalMessage.push_back(echoBuffer[i]);
        }
        echoBuffer[bytesRcvd] = '\0'; 
         /* Terminate the string! */
        //printf("%s", echoBuffer);      /* Print the echo buffer */
     } 
    finalMessage.push_back('\0');
    for (char i: finalMessage)
        std::cout << i;
    printf("\n");    /* Print a final linefeed */

    close(sock);
    returnString = finalMessage;
    return;
}

/*int main(){
    std::vector<char> final_msg = subscribe(SERVERPORT, RCVBUFSIZE);
    return 0;
}*/