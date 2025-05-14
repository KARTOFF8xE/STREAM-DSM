#include <iostream>
#include <string>
#include <curl/curl.h>
#include <fmt/core.h>

#include <babeltrace2/babeltrace.h>

#include "interface.hpp"
#include "participantFactory.hpp"
#include "participants/node.hpp"
#include "participants/publisher.hpp"
#include "participants/subscriber.hpp"
#include "participants/service.hpp"
#include "participants/client.hpp"
#include "participants/dummy.hpp"

IParticipant *ParticipantFactory::getParticipant(const char *event_name) {
    if (strcmp(
        "ros2:rcl_node_init",
        event_name) == 0) {
            return new Node();
    }
    if (strcmp(
        "ros2:rcl_publisher_init",
        event_name) == 0) {
            return new Publisher();
    }
    if (strcmp(
        "ros2:rcl_subscription_init",
        event_name) == 0) {
            return new Subscriber();
    }
    if (strcmp(
        "ros2:rcl_service_init",
        event_name) == 0) {
            return new Service();
    }
    if (strcmp(
        "ros2:rcl_client_init",
        event_name) == 0) {
            return new Client();
    }
    // TODO: add client traffic here
    
    return new Dummy();
}