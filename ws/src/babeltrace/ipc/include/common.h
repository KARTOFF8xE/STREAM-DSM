#pragma once

#include <iostream>

#define KEY_LOCATION "/tmp/example.ipc-key"

#define printError \
  std::cerr \
    << "Error " << strerrorname_np(errno) << " (" __FILE__ ":" << __LINE__ << "): " \
    << std::strerror(errno) << '\n'

typedef struct
{
  __syscall_slong_t senderId;
  char name[32];
} Request;

typedef struct
{
  __syscall_slong_t recieverId;
  char msg[64];
} Response;