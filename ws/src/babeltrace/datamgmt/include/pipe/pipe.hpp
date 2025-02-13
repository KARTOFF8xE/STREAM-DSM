#pragma once

#include "datamgmt/common.hpp"

/**
 * @brief Transforms the payload into a message and writes it to the pipe.
 * 
 * @param __fd File descriptor for the write end of the pipe.
 * @param payload The payload that shall be send to the pipe.
 * @param __n Size of the message.
 * 
 * @return The number written. Or -1.
 */
template<typename T>
ssize_t writeT(int __fd, const T payload, size_t __n = MSGSIZE);

/**
 * @brief Reads a message from the pipe and transforms it into the espected payload.
 * 
 * @param __fd File descriptor for the read end of the pipe.
 * @param payload The payload that shall be read from the pipe.
 * @param __n Size of the message.
 * 
 * @return The number read. -1 for errors or 0 for EOF.
 */
template<typename T>
ssize_t readT(int __fd, const T &payload, size_t __n = MSGSIZE);