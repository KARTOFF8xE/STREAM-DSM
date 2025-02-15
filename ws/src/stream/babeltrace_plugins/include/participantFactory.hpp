#pragma once

#include "interface.hpp"

class ParticipantFactory {
    public:
        /**
         * @brief Selects the correct type of participant.
         *
         * @return A Participant.
         */
        static IParticipant *getParticipant(const char *event_name);
};