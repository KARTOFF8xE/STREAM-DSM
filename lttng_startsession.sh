#!/bin/zsh

lttng-relayd -d
lttng create tracingSession --live 1000000 -U net://localhost
lttng enable-event --userspace 'ros2:rcl_publish'
lttng start