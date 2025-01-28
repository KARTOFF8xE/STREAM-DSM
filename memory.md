### run ros2_tracing CLI:
``` bash
    $ ros2 trace -p ~/.ros2_tracing -s ros2_traced -u ros2:rcl_node_init -l
        UST tracing enabled (1 events)
            ros2:rcl_node_init
        kernel tracing disabled
        context (3 names)
            procname
            vpid
            vtid
        writing tracing session to: /home/Georg/.ros/tracing/session-20250115075204
        press enter to start...
        # {ENTER to start} -> User Input
        press enter to stop...
        # {ENTER to stop} -> User Input
        stopping & destroying tracing session
    $ babeltrace ~/.ros2_tracing
        [07:52:17.476102526] (+?.?????????) 2279fadf17a2 ros2:rcl_node_init: { cpu_id = 1 }, { vpid = 20227, vtid = 20227, procname = "talker" }, { node_handle = 0x63CDC2EFB6E0, rmw_handle = 0x63CDC30308D0, node_name = "minimal_publisher", namespace = "/" }
```

### run lttng CLI:
#### classic:

```bash
    $ lttng create lttng_tracing --output ~/.lttng
        Session lttng_tracing created.
        Traces will be output to /home/Georg/.lttng
    $ lttng enable-event --userspace 'ros2:rcl_node_init'
        ust event ros2:rcl_node_init created in channel channel0
    $ lttng add-context --userspace --type=vpid 
        ust context vpid added to all channels
    $ lttng add-context --userspace --type=vtid
        ust context vtid added to all channels
    $ lttng add-context --userspace --type=procname
        ust context procname added to all channels
    $ lttng start                                  
        Tracing started for session tracing_session2
    $ lttng stop 
        Waiting for data availability
        Tracing stopped for session tracing_session2
    $ babeltrace .lttng2/             
        [07:52:17.476095213] (+?.?????????) 2279fadf17a2 ros2:rcl_node_init: { cpu_id = 1 }, { vpid = 20227, vtid = 20227, procname = "talker" }, { node_handle = 0x63CDC2EFB6E0, rmw_handle = 0x63CDC30308D0, node_name = "minimal_publisher", namespace = "/" }
```

#### live:

```bash
    $ lttng-relayd -d
    $ lttng create --live 1000000 -U net://localhost
    Live session {foo} created.
    Traces will be output to tcp4://127.0.0.1:5342/ [data: 5343]
    Live timer interval set to 1000000 us
    $ lttng enable-event --userspace 'ros2:rcl_node_init'
        ust event ros2:rcl_node_init created in channel channel0
    $ lttng add-context --userspace --type=vpid 
        ust context vpid added to all channels
    $ lttng add-context --userspace --type=vtid
        ust context vtid added to all channels
    $ lttng add-context --userspace --type=procname
        ust context procname added to all channels
    $ lttng start                                  
        Tracing started for session tracing_session2
    $ lttng stop 
        Waiting for data availability
        Tracing stopped for session tracing_session2
    $ babeltrace .lttng2/             
        [07:52:17.476095213] (+?.?????????) 2279fadf17a2 ros2:rcl_node_init: { cpu_id = 1 }, { vpid = 20227, vtid = 20227, procname = "talker" }, { node_handle = 0x63CDC2EFB6E0, rmw_handle = 0x63CDC30308D0, node_name = "minimal_publisher", namespace = "/" }
```

#### start neo4j

```bash
    $ docker run \                     
        --restart always \                    
        --publish=7474:7474 --publish=7687:7687 \
        neo4j:5.26.1
```

#### small example
* start neo4j
    ```bash
        $ docker run \                     
            --restart always \                    
            --publish=7474:7474 --publish=7687:7687 \
            --env NEO4J_AUTH=neo4j/123456789 \
            neo4j:5.26.1
    ```
* run devcontainer
* `./lttng_startsession.zsh`
* `ros2 run tracer babel`
* do whatever needs to be tracked