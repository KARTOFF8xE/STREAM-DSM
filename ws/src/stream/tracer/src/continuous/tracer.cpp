#include <lttng/lttng.h>
#include <iostream>
#include <cstring>
#include <csignal>
#include <thread>
#include <chrono>
#include <vector>
#include <filesystem>

#include "threadPool.hpp"

#define PATH "/tmp/continuous_traces"

volatile sig_atomic_t stopFlag = 0;

void handle_sigint(int) {
    stopFlag = 1;
}


int main() {
    signal(SIGINT, handle_sigint);
    lttng_destroy_session("continuousSession");

    int ret = lttng_create_session("continuousSession", PATH);
    if (ret < 0) {
        std::cerr << "Failed to create LTTng session: " << ret << std::endl;
        return 1;
    }
    std::cout << "LTTng session created successfully." << std::endl;

    lttng_domain lttngDomain {
        .type       = lttng_domain_type::LTTNG_DOMAIN_UST
    };
    lttng_handle *lttngHandle = lttng_create_handle("continuousSession", &lttngDomain);

    lttng_channel *lttngChannel = lttng_channel_create(&lttngDomain);
    strncpy(lttngChannel->name, "continuousChannel", LTTNG_SYMBOL_NAME_LEN);
    ret = lttng_enable_channel(lttngHandle, lttngChannel);
    if (ret < 0) {
        std::cerr << "Failed to enable LTTng channel: " << ret << std::endl;
        return 1;
    }

    lttng_event *lttngEvent = lttng_event_create();
        strncpy(lttngEvent->name, "ros2:rcl_publish", LTTNG_SYMBOL_NAME_LEN);
        lttng_enable_event(lttngHandle, lttngEvent, lttngChannel->name);


    lttng_rotation_schedule *rotationSchedule = lttng_rotation_schedule_periodic_create();
    lttng_rotation_schedule_periodic_set_period(rotationSchedule, 1000000);

    lttng_session_add_rotation_schedule("continuousSession", rotationSchedule);

    lttng_start_tracing("continuousSession");

    lttng_condition *condition  = lttng_condition_session_rotation_completed_create();
    ret = lttng_condition_session_rotation_set_session_name(condition, "continuousSession");
    if (ret != LTTNG_CONDITION_STATUS_OK) {
        std::cerr << "lttng_condition_session_rotation_set_session_name failed with code: " << ret << std::endl;
    }
    lttng_action    *action     = lttng_action_notify_create();
    lttng_trigger   *trigger    = lttng_trigger_create(condition, action);
    ret = lttng_register_trigger_with_automatic_name(trigger);
    if (ret != LTTNG_OK) {
        std::cerr << "lttng_register_trigger failed with code: " << ret << std::endl;
    }

    lttng_notification_channel *notificationChannel = lttng_notification_channel_create(lttng_session_daemon_notification_endpoint);
    ret = lttng_notification_channel_subscribe(notificationChannel, condition);
    if (ret != LTTNG_NOTIFICATION_CHANNEL_STATUS_OK) {
        std::cerr << "lttng_notification_channel_subscribe failed with code: " << ret << std::endl;
    }

    ThreadPool pool(5);

    while (!stopFlag) {
        struct lttng_notification *notification;
        enum lttng_notification_channel_status status;

        status = lttng_notification_channel_get_next_notification(notificationChannel, &notification);
        if (status == LTTNG_NOTIFICATION_CHANNEL_STATUS_OK) {
            const lttng_evaluation *evaluation = lttng_notification_get_evaluation(notification);
            const lttng_trace_archive_location *location;
            ret = lttng_evaluation_session_rotation_completed_get_location(evaluation, &location);
            if (ret != LTTNG_EVALUATION_STATUS_OK) {
                std::cerr << "lttng_evaluation_session_rotation_completed_get_location failed with code: " << ret << std::endl;
            }

            const char *path;
            ret = lttng_trace_archive_location_local_get_absolute_path(location, &path);
            if (ret != LTTNG_TRACE_ARCHIVE_LOCATION_STATUS_OK) {
                std::cerr << "lttng_trace_archive_location_local_get_absolute_path failed with code: " << ret << std::endl;
            }

            pool.enqueue(path);

            lttng_notification_destroy(notification);
        } else if (status == LTTNG_NOTIFICATION_CHANNEL_STATUS_CLOSED) {
            break;
        }
    }

    std::cout << "finalizing...";
    lttng_unregister_trigger(trigger);
    lttng_trigger_destroy(trigger);
    ret = lttng_notification_channel_unsubscribe(notificationChannel, condition);
    if (ret != LTTNG_NOTIFICATION_CHANNEL_STATUS_OK) {
        std::cerr << "lttng_notification_channel_unsubscribe failed with code: " << ret << std::endl;
    }
    lttng_notification_channel_destroy(notificationChannel);
    lttng_action_destroy(action);
    lttng_condition_destroy(condition);
    lttng_event_destroy(lttngEvent);
    ret = lttng_stop_tracing("continuousSession");
    if (ret != 0) {
        std::cerr << "lttng_stop_tracing failed with code: " << ret << std::endl;
    }
    ret = lttng_destroy_session("continuousSession");
    if (ret != 0) {
        std::cerr << "lttng_destroy_session failed with code: " << ret << std::endl;
    }

    std::filesystem::remove_all(PATH); 

    std::cout << "done" << std::endl;

    return 0;
}