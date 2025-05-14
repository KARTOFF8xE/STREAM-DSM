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

#### start InfluxDB
    ```bash
	docker run -d --name influxdb -p 8086:8086 -v influxdb_data:/var/lib/influxdb2 influxdb:latest
    ```

#### small example
    ```bash
	TIMESTAMP=$(date +%s)
	curl -X POST "http://localhost:8086/api/v2/write?org=TUBAF&bucket=STREAM&precision=s" \
     	    -H "Authorization: Token WVvSEEbHPqeMFpcgWqThaEcU6u6SWJ-L26ct4oRuEJmKdMOk-ZG8XlKA5xcitJXENa2r2YNLNwxjE6-KKkx8xw==" \
     	    --data-raw "temperature,sensor=room1 value=23.5 $TIMESTAMP"

    ```

#### possibility for detecting client/server-communication:
* detect server Node:
    ```log
        ros2:rcl_node_init:     { procname = "server", vpid = 19157 }, { node_handle = 0x5F5A11C26E70, rmw_handle = 0x5F5A11DA7A80, node_name = "add_two_ints_server", namespace = "/" }
    ```
* give server Node property "server" (match by PID):
    ```log
        ros2:rcl_service_init:  { procname = "server", vpid = 19157 }, { service_handle = 0x5F5A11F170E0, node_handle = 0x5F5A11C26E70, rmw_service_handle = 0x5F5A11F31AD0, service_name = "/add_two_ints" }
    ```
* detect client Node:
    ```log
        ros2:rcl_node_init:     { procname = "client", vpid = 19181 }, { node_handle = 0x597901C00EC0, rmw_handle = 0x597901D8B100, node_name = "add_two_ints_client", namespace = "/" }
    ```
* give client Node property "client" and store gid (match by PID):
    ```log
        ros2:rmw_client_init:   { procname = "client", vpid = 19181 }, { rmw_client_handle = 0x597901F18980, gid = [ [0] = 1, [1] = 15, [2] = 112, [3] = 183, [4] = 237, [5] = 74, [6] = 142, [7] = 220, [8] = 0, [9] = 0, [10] = 0, [11] = 0, [12] = 0, [13] = 0, [14] = 20, [15] = 4 ] }
    ```
* Link client with service (match by service_name, add property "usage" to link)
    ```log
        ros2:rcl_client_init:   { procname = "client", vpid = 19181 }, { client_handle = 0x597901F01780, node_handle = 0x597901C00EC0, rmw_client_handle = 0x597901F18980, service_name = "/add_two_ints" }
    ```
<!-- ros2:rmw_send_request:  { procname = "client", vpid = 19181 }, { rmw_client_handle = 0x597901F18980, request = 0x597901F21100, sequence_number = 1 } -->
<!-- ros2:rmw_take_request:  { procname = "server", vpid = 19157 }, { rmw_service_handle = 0x5F5A11F31AD0, request = 0x5F5A11F38BA0, client_gid = [ [0] = 1, [1] = 15, [2] = 112, [3] = 183, [4] = 237, [5] = 74, [6] = 142, [7] = 220, [8] = 0, [9] = 0, [10] = 0, [11] = 0, [12] = 0, [13] = 0, [14] = 20, [15] = 4 ], sequence_number = 1, taken = 1 } -->
* increment "usage"-counter of link between server (match by PID) and client (match by gid):
    ```log
        ros2:rmw_send_response: { procname = "server", vpid = 19157 }, { rmw_service_handle = 0x5F5A11F31AD0, response = 0x5F5A11F398D0, client_gid = [ [0] = 1, [1] = 15, [2] = 112, [3] = 183, [4] = 237, [5] = 74, [6] = 142, [7] = 220, [8] = 0, [9] = 0, [10] = 0, [11] = 0, [12] = 0, [13] = 0, [14] = 20, [15] = 4 ], sequence_number = 1, timestamp = 1738137710236125455 }
    ```
```
