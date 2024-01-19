#ifndef PUBLISHER_H
#define PUBLISHER_H_
#include <string>

namespace Publisher{
void publish(const char* serverIP, std::string twist_msg);
}

#endif // End of the file...