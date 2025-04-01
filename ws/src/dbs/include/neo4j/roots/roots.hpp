#pragma once

#include <string>
#include <fmt/core.h>


namespace createRoot {

    // TODO
    std::string getPayloadCreateNameSpaceAndLinkPassiveHelpers(std::string fullName);
    std::string getPayloadCreateProcessAndLinkPassiveHelpers(std::string pids);
    std::string getPayloadCreateProcessAndUpdatePassiveHelpers(std::string fullName, std::string pids);

}