#pragma once
#include <vector>
namespace Subscriber{
void DieWithError(const char *errorMessage){};
void subscribe(int port, int messageSize, std::vector<char> &returnString){};
}