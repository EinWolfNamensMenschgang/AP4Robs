#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <vector>
namespace Subscriber{
void DieWithError(const char *errorMessage);
void subscribe(int port, int messageSize, std::vector<char> &returnString);
}
#endif 