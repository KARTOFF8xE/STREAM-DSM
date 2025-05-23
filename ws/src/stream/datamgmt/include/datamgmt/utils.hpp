#pragma once 

#include <string>

/**
 * @brief Returns the name of the Host.
 * 
 * @return Name of the Host.
 */
std::string getHostname();

// TODO
std::string getFullName(std::string name, std::string nameSpace);

// TODO
void truncateAtSubstring(char* str, const char* substr);