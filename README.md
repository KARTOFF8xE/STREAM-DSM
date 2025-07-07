# DiplArbeit_Container

> This is the practical part of my [diploma thesis](https://github.com/KARTOFF8xE/DiplArbeit): `Aggregation and Pre-Processing of Network and Process Data for Monitoring Purposes in ROS2 Applications`.


## Clone 
* `git clone https://github.com/KARTOFF8xE/STREAM-DSM.git --recurse-submodules`


## Build
### Devcontainer (VSC)
* build devcontainer
* go to `ws/src/dbs/src/curl/myCurl.cpp` and change `URLNEO4J` to `http://neo4j:7474`
* go to `ws/src/dbs/src/curl/myCurl.cpp` and change `URLINFLUXDB` to `http://influxdb:8086`
* go to `ws/src/stream/tracer/src/structuralLO/tracer.cpp` and remove `.buf_type   = LTTNG_BUFFER_PER_UID,` from the initilisation of `lttngDomain` (employs LTTng Rotations for tracing structural Data)
* go to `ws/src/stream/tracer/src/continuous/tracer.cpp` and remove `.buf_type   = LTTNG_BUFFER_PER_UID,` from the initilisation of `lttngDomain`
* go to `ws/src/stream/tracer/src/structural/tracer.cpp` and remove `.buf_type   = LTTNG_BUFFER_PER_UID,` from the initilisation of `lttngDomain` (employs LTTng-Live for tracing structural Data)
* `colcon build --symlink-install` within `ws`


### Host System
* `sudo ./scripts/setupScript.sh`
* `colcon build --symlink-install` within `ws`


## Run
* on host: `docker compose -f .devcontainer/docker-compose.yml up -d`
* `scripts/start.tmux`
* I suggest [ros2_performance](https://github.com/Irobot-ros/ros2-performance) for exemplary ROS2 networks (a script for installation lies within `scripts/`)


## Run tests
* build test container: `docker build -f .devcontainer/Dockerfile.testspace -t testspace .`
* run testscript: `./scripts/testScript.sh` (take look into the script to adapt some paramenters)
* when the tests have finished:
    &rarr; `python3 scripts/calc_latencies_csv.py <path/to/testSet>` calculates the median and mean latencies for all test cases of a testset
        &rarr; there are two testsets (_BASE_ and _TRACE_)
    &rarr; `python3 script/calc_latencies_diff.csv <path/to/testSetTrace> <path/to/testSetTrace>` calculates the differences between the two created testsets
* Further scripts are meant for analysing testdata (gathered from grafana), run the commands without arguments to see which files are necessitated.

## Observe
* [grafana](http://localhost:3000/)
* [neo4j](http://localhost:7474/)



## Impressum
* @KARTOFF8xE