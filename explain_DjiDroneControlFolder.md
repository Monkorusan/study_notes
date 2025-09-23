# DJI Drone Control Package: Architecture and Relationships

This document provides a detailed explanation of the architecture and relationships between the main classes in the DJI drone control package, as illustrated in the block diagram. The design leverages ROS 2's modularity and distributed system capabilities to enable scalable, maintainable, and robust multi-drone control.

## Node

The `Node` class is the fundamental building block in ROS 2. It provides essential methods for communication and scheduling:
- **create_publisher()**: Advertises topics for other nodes to subscribe to.
- **create_subscription()**: Subscribes to topics published by other nodes.
- **create_client()**: Enables service calls to other nodes.
- **create_service()**: Provides services for other nodes to call.
- **create_timer()**: Schedules periodic tasks.

Both `Driver` and `DistributedNode` inherit from `Node`, gaining all these capabilities. This allows them to interact seamlessly within the ROS 2 ecosystem, handling messages, services, and timed events.

## Driver

The `Driver` class extends `Node` and acts as the hardware abstraction layer for a single drone. It encapsulates:
- **State attributes**: `position`, `camera_position`, `gimbal_state`, `battery`, etc., representing the drone's current status.
- **Control methods**: `takeoff()`, `land()`, `emergency()`, `rotate_gimbal()`, `pilot()`, etc., which send commands to the drone.
- **ROS 2 publishers/subscribers**: For topics like battery state, position, gimbal state, and camera position.

This class is responsible for direct communication with the drone, processing sensor data, and executing flight commands. It abstracts away low-level details, providing a clean interface for higher-level control.

## Controller

The `Controller` class inherits from `Driver`, building on its capabilities to provide advanced control logic for a single drone:
- **Parameter management**: Methods for loading configuration parameters from files and retrieving their values.
- **Pose topic management**: Handles topics related to the drone's position and orientation (`positions_topic`).
- **Custom callbacks**: Allows for dynamic response to parameter changes and sensor updates.

This class is focused on single-drone control, configuration, and state management, making it easy to adapt to different drone models and missions.

## DistributedNode

The `DistributedNode` class also inherits from `Node`, but its primary role is distributed coordination:
- **Online node tracking**: Maintains a list of online nodes (`online_nodes`) and a dictionary for absence detection (`_absent_dict`).
- **Online topic subscription**: Listens to a dedicated topic (`online_topic`) to detect which nodes are present in the network.
- **Event handling**: Provides methods to add or remove handlers for online node events, and triggers these events when the network changes.
- **State update logic**: Periodically checks node status and updates the online list, enabling robust multi-drone management.

This class is essential for enabling dynamic, distributed control in multi-robot systems, allowing nodes to join or leave the network seamlessly.

## DistributedController

The `DistributedController` class inherits from both `Controller` and `DistributedNode`, combining their strengths:
- **Single-drone control**: Inherits all state management and command capabilities from `Controller`.
- **Distributed coordination**: Gains online node tracking and event handling from `DistributedNode`.
- **Data synchronization**: Provides methods like `get_positions()` and `sync_array()` to access and synchronize data across multiple drones.
- **Dynamic topic management**: Automatically manages subscriptions and data flow as drones join or leave the network.

This hybrid class enables scalable, flexible control of a fleet of drones, supporting both individual and group behaviors.

## DistributedSubscription

The `DistributedSubscription` class is used by `DistributedController` to manage data from multiple drones:
- **Dynamic subscription management**: Automatically creates and destroys subscriptions to topics published by online nodes.
- **Data aggregation**: Maintains arrays and dictionaries of messages from all active drones.
- **Update logic**: Periodically refreshes subscriptions and synchronizes data, ensuring up-to-date information for decision-making.
- **Filtering and conversion**: Supports custom filters and conversion functions to process incoming data as needed.

This class abstracts the complexity of managing many subscriptions, making it easy to aggregate and process data from a distributed fleet.

---

## Summary

The DJI drone control package is designed for modularity and scalability. By separating hardware abstraction (`Driver`), single-drone control (`Controller`), distributed coordination (`DistributedNode`), and dynamic data management (`DistributedSubscription`), the system supports robust multi-drone operations in ROS 2. This architecture can be adapted for other types of autonomous robots, making it a strong foundation for distributed robotics projects.
