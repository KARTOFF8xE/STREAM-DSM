#include <bits/stdc++.h>
#include <iostream>
#include <map>

#include <ipc/ipc-client.hpp>
#include <neo4j/roots/roots.hpp>
#include <curl/myCurl.hpp>

#include "datamgmt/relationmgmt/relationmgmt.hpp"
#include "pipe/pipe.hpp"


namespace relationMgmt {

void relationMgmt(std::string name) {
    std::string payload = createRoot::getPayloadCreateNameSpaceAndLinkPassiveHelpers(name);
    curl::push(payload);
}

}