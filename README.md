# DiplArbeit_Container

```
This is my practical part, for my [diploma thesis](https://github.com/KARTOFF8xE/DiplArbeit): `Aggregation and Pre-Processing of Network and Process Data for Monitoring Purposes in ROS2 Applications`.
```

## TODOs
- [ ] bring [ipc](https://github.com/Nummer42O/STREAM-IPC) to http communication
- [ ] add disk and memory utilisation
- [ ] add callback duration
- [ ] add request-response quantities
- [ ] add configuration to integrate expert knowledge
- [ ] add Evaluator
- [ ] clean Code and optimise

## Build
### Devcontainer (VSC)
* build devcontainer
* `colcon build --symlink-install` within `ws`

### Host System
* `sudo ./scripts/setupScript.sh`
* `colcon build --symlink-install` within `ws`


## Run
* on host: `docker compose -f .devcontainer/docker-compose.yml up -d`
* `scripts/start.tmux`


## Run tests
* build test container: `docker build -f .devcontainer/Dockerfile.testspace -t testspace .`
* run testscript: `./scripts/testScript.sh` (take look into the script to adapt some paramenters)
* when the tests have finished:
    &rarr; `python3 scripts/calc_latencies_csv.py <path/to/testSet>` calculates the median and mean latencies for all test cases of a testset
        &rarr; there are two testsets (_BASE_ and _TRACE_)
    &rarr; `python3 script/calc_latencies_diff.csv <path/to/testSetTrace> <path/to/testSetTrace>` calculates the differences between the two created testsets

## Observe
* [grafana](http://localhost:3000/)
* [neo4j](http://localhost:7474/)


## Maintain

## Impressum
* @KARTOFF8xE