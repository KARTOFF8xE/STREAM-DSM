#pragma once

#define STRINGSIZE 16

struct Data {
    char msg[STRINGSIZE];
    bool subscribe;
};
#define MSGSIZE sizeof(Data)