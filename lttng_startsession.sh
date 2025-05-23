#!/bin/zsh

lttng-relayd -d
lttng create tracingSession --live 1000000 -U net://localhost
lttng enable-event --userspace 'ros2:rcl_node_init'
lttng enable-event --userspace 'ros2:rcl_publisher_init'
lttng enable-event --userspace 'ros2:rcl_subscription_init'
lttng enable-event --userspace 'ros2:rcl_service_init'
lttng enable-event --userspace 'ros2:rcl_client_init'
lttng enable-event --userspace 'ros2:rcl_timer_init'
lttng enable-event --userspace 'ros2:rclcpp_timer_link_node'
lttng enable-event --userspace 'ros2:rcl_lifecycle_state_machine_init'
lttng enable-event --userspace 'ros2:rcl_lifecycle_transition'
# lttng enable-event --userspace 'ros2:rmw_client_init' // TODO: for client-server communication traffic (gid)
# lttng enable-event --userspace 'ros2:rmw_send_request' // TODO: for client-server communication traffic count requests
# lttng enable-event --userspace 'ros2:rmw_send_response' // TODO: for client-server communication traffic count responses
lttng add-context --userspace --type=procname
lttng add-context --userspace --type=vpid 
lttng start
