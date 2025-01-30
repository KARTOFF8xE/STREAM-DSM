#pragma once

#include "interface.h"

class ParticipantFactory {
    public:
        static IParticipant *getParticipant(const char *event_name);
};