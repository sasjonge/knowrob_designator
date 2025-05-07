# knowrob_designator

**Work in Progress**

This package is under active development and **not yet stable**. Interfaces, functionality, and structure may change frequently.

---

## Overview

The `knowrob_designator` package provides a ROS action interface for logging structured semantic data — called **designators** — into the [KnowRob](https://www.github.com/knowrob/knowrob) knowledge base. 

A designator is a nested symbolic description (e.g., of actions, goals, objects) expressed as a JSON structure. This structure is parsed into RDF-style triples and submitted to KnowRob using its `tell` interface.

---

## Package Structure

```text
knowrob_designator/
├── action/
│   └── LogDesignator.action           # Action definition for logging designators
├── CMakeLists.txt                     # Build configuration
├── Dockerfile                         # Container setup (ROS + KnowRob)
├── launch/
│   └── knowrob_designator_service.launch  # Launches KnowRob and the action server
├── package.xml                        # ROS package metadata
├── README.md                          # You are here
├── run_knowrob_designator.sh          # Optional helper script (not required)
├── scripts/
│   ├── designator_parser.py           # Parses JSON designators into triples (no ROS dependency)
│   ├── knowrob_designator_client.py   # Example client that sends a test designator
│   └── knowrob_designator_service.py  # Action server that logs designators to KnowRob
└── srv/                               # (empty, safe to delete or populate if needed)
````

---

## Running the System via Docker

This section describes how to launch the full system in a Dockerized environment.

### Start the KnowRob and Designator Action Server

```bash
docker run -it --entrypoint bash knowrob_designator:latest
# Inside container:
root@<container_id>:/catkin_ws# roslaunch knowrob_designator knowrob_designator_service.launch
```

This will:

* Launch KnowRob with predefined settings
* Start the `knowrob_designator_service.py` action server

---

### In a second terminal: Run the Designator Client

Open a new terminal and run:

```bash
docker exec -it <your_container_name> bash
# Inside container:
root@<container_id>:/catkin_ws# rosrun knowrob_designator knowrob_designator_client.py
```

---

## Example Flow

The client will send a test JSON designator like:

```json
{
  "anAction": {
    "type": "Transporting",
    "objectActedOn": { "anObject": { "type": "Milk" } },
    "target": {
      "theLocation": {
        "goal": { "theObject": { "name": "Table1" } }
      }
    }
  }
}
```

This is parsed into triples and sent to KnowRob, where it becomes part of the symbolic knowledge base.

---

## Dependencies

* ROS (Noetic recommended)
* KnowRob (`knowrob/knowrob` Docker image)
* `knowrob_ros` action interface

