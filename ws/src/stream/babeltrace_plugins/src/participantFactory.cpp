#include <iostream>
#include <string>
#include <curl/curl.h>
#include <fmt/core.h>

#include <babeltrace2/babeltrace.h>

#include "interface.h"
#include "participantFactory.h"
#include "participants/node.h"
#include "participants/publisher.h"
#include "participants/subscriber.h"
#include "participants/service.h"
#include "participants/client.h"
#include "participants/dummy.h"

IParticipant *ParticipantFactory::getParticipant(const char *event_name) {
    if (strcmp(
        "ros2:rcl_node_init",
        event_name) == 0) {
            Node *node = new Node();
            return node;
    }
    if (strcmp(
        "ros2:rcl_publisher_init",
        event_name) == 0) {
            Publisher *publisher = new Publisher();
            return publisher;
    }
    if (strcmp(
        "ros2:rcl_subscription_init",
        event_name) == 0) {
            Subscriber *subscriber = new Subscriber();
            return subscriber;
    }
    if (strcmp(
        "ros2:rcl_service_init",
        event_name) == 0) {
            Service *service = new Service();
            return service;
    }
    if (strcmp(
        "ros2:rcl_client_init",
        event_name) == 0) {
            Client *client = new Client();
            return client;
    }
    // TODO: add client traffic here
    
    return new Dummy();
}