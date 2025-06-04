#include <lttng/lttng.h>
#include <iostream>
#include <cstring>
#include <string.h>
#include <csignal>
#include <thread>
#include <chrono>
#include <vector>
#include <filesystem>
#include <sys/mman.h>

#include "ipc/sharedMem.hpp"
#include "threadPool.hpp"

#define PATH "/tmp/structural_traces"


volatile sig_atomic_t stopFlag = 0;

void handle_sigint(int) {
    shm_unlink("/babeltonato");
    lttng_destroy_session("structuralSession");
    exit(EXIT_SUCCESS);
    stopFlag = 1;
}

std::vector<lttng_event*> enable_events(lttng_handle *handle, const char *channel_name) {
    const char *event_names[] = {
        "ros2:rcl_node_init",
        "ros2:rcl_publisher_init",
        "ros2:rcl_subscription_init",
        "ros2:rcl_service_init",
        "ros2:rcl_client_init",
        "ros2:rcl_timer_init",
        "ros2:rclcpp_timer_link_node",
        "ros2:rcl_lifecycle_state_machine_init",
        "ros2:rcl_lifecycle_transition",
        // "ros2:rclcpp_subscription_callback_added",
        // "ros2:rclcpp_subscription_init",
        // "ros2:rclcpp_service_callback_added",
        // "ros2:rclcpp_timer_callback_added",
        // "ros2:callback_start",
        // "ros2:callback_end",
    };

    std::vector<lttng_event*> enabled_events;

    for (const char *event_name : event_names) {
        lttng_event *event = lttng_event_create();
        if (!event) {
            std::cerr << "Failed to create LTTng event for " << event_name << std::endl;
            break;
        }

        std::strncpy(event->name, event_name, LTTNG_SYMBOL_NAME_LEN);
        int ret = lttng_enable_event(handle, event, channel_name);
        if (ret != 0) {
            std::cerr << "Failed to enable event '" << event_name << "': " << lttng_strerror(ret) << std::endl;
            lttng_event_destroy(event);
            break;
        }

        enabled_events.push_back(event);
    }

    return enabled_events;
}

int main() {
    signal(SIGINT, handle_sigint);

    lttng_destroy_session("structuralSession");

    sharedMem::SHMChannel<sharedMem::TraceMessage> channel("/babeltonato");

    int ret = lttng_create_session("structuralSession", PATH);
    if (ret < 0) {
        std::cerr << "Failed to create LTTng session: " << ret << std::endl;
        return 1;
    }
    std::cout << "LTTng session created successfully." << std::endl;

    lttng_domain lttngDomain {
        .type       = lttng_domain_type::LTTNG_DOMAIN_UST
    };
    lttng_handle *lttngHandle = lttng_create_handle("structuralSession", &lttngDomain);

    lttng_channel *lttngChannel = lttng_channel_create(&lttngDomain);
    strncpy(lttngChannel->name, "structuralChannel", LTTNG_SYMBOL_NAME_LEN);
    ret = lttng_enable_channel(lttngHandle, lttngChannel);
    if (ret < 0) {
        std::cerr << "Failed to enable LTTng channel: " << ret << std::endl;
        return 1;
    }

    std::vector<lttng_event *> events = enable_events(lttngHandle, lttngChannel->name);
    lttng_event_context ctx;
    ctx.ctx = lttng_event_context_type::LTTNG_EVENT_CONTEXT_PROCNAME;
    lttng_add_context(lttngHandle, &ctx, "ros2:rcl_node_init", "structuralChannel");
    lttng_event_context ctx2;
    ctx2.ctx = lttng_event_context_type::LTTNG_EVENT_CONTEXT_VPID;
    lttng_add_context(lttngHandle, &ctx2, "ros2:rcl_node_init", "structuralChannel");

    lttng_rotation_schedule *rotationSchedule = lttng_rotation_schedule_periodic_create();
    lttng_rotation_schedule_periodic_set_period(rotationSchedule, 1000000);

    lttng_session_add_rotation_schedule("structuralSession", rotationSchedule);

    ret = lttng_start_tracing("structuralSession");
    if (ret != 0) {
        std::cerr << "Failed to start tracing: " << lttng_strerror(ret) << std::endl;
        return 1;
    }

    lttng_condition *condition  = lttng_condition_session_rotation_completed_create();
    ret = lttng_condition_session_rotation_set_session_name(condition, "structuralSession");
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

    std::string lastPath = "";
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

            if (lastPath != path && strstr(path, PATH)) {
                pool.enqueue(path);
                lastPath = path;
            }

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
    ret = lttng_stop_tracing("structuralSession");
    if (ret != 0) {
        std::cerr << "lttng_stop_tracing failed with code: " << ret << std::endl;
    }
    ret = lttng_destroy_session("structuralSession");
    if (ret != 0) {
        std::cerr << "lttng_destroy_session failed with code: " << ret << std::endl;
    }

    std::filesystem::remove_all(PATH); 

    std::cout << "done" << std::endl;

    return 0;
}