#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H
#include <vector>
#include <string>
namespace Subscriber{
void DieWithError(const char *errorMessage);
void subscribe(const char* serverIP, int port, int messageSize, std::vector<char> &returnString);
}
#endif 