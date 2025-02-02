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
            FooNode *fooNode = new FooNode();
            return fooNode;
    }
    if (strcmp(
        "ros2:rcl_publisher_init",
        event_name) == 0) {
            FooPublisher *fooPublisher = new FooPublisher();
            return fooPublisher;
    }
    if (strcmp(
        "ros2:rcl_subscription_init",
        event_name) == 0) {
            FooSubscriber *fooSubscriber = new FooSubscriber();
            return fooSubscriber;
    }
    if (strcmp(
        "ros2:rcl_service_init",
        event_name) == 0) {
            FooService *fooService = new FooService();
            return fooService;
    }
    if (strcmp(
        "ros2:rcl_client_init",
        event_name) == 0) {
            FooClient *fooClient = new FooClient();
            return fooClient;
    }
    // TODO: add client traffic here
    
    return new FooDummy();
}