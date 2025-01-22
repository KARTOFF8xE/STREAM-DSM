#!/bin/zsh

lttng-relayd -d
lttng create tracingSession --live 1000000 -U net://localhost
lttng enable-event --userspace 'ros2:rcl_node_init'
lttng enable-event --userspace 'ros2:rcl_publisher_init'
# lttng enable-event --userspace 'ros2:rcl_publish'
lttng add-context --userspace --type=procname
lttng add-context --userspace --type=vpid 
lttng start