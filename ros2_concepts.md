---
tags: ROS 2
---

# ROS 2 Concepts

[![hackmd-github-sync-badge](https://hackmd.io/acmkN6_8T_uOOUFzX-TRBQ/badge)](https://hackmd.io/acmkN6_8T_uOOUFzX-TRBQ)


## Nodes

Node 可以看成一個一個模組，模組之間互動的機制就是 ROS 主要的功能

![Nodes](https://index.ros.org/doc/ros2/_images/Nodes-TopicandService.gif)

## Topics

Topic 是 Node 之間溝通的管道，Node 可以 Publish 來發送資料，Subscribe 來接收資料

這樣的傳輸方式可以點對點傳輸

![Topics 1-1](https://index.ros.org/doc/ros2/_images/Topic-SinglePublisherandSingleSubscriber.gif)

也可以一對多，甚至可以多對多

![Topics 1-N N-N](https://index.ros.org/doc/ros2/_images/Topic-MultiplePublisherandMultipleSubscriber.gif)

## Services

Service 是另一種 Node 之間溝通的方法，Service 只在明確跟他 request 時才會傳資料給你

### 跟 Topic 的比較

|         Service         |              Topic               |
|:-----------------------:|:--------------------------------:|
| call-and-response model |    publisher-subscriber model    |
|   要呼叫才會提供資料       | Subscribe 後可以接收連續性的資料 |

### Commands
* `ros2 service list`
* `ros2 service type <service_name>`
* `ros2 service list -t`
* `ros2 service find <type_name>`
* `ros2 interface show <type_name>.srv`
* `ros2 service call <service_name> <service_type> <arguments>`

### Related Content

另一個 [Tutorial](https://discourse.ubuntu.com/t/call-services-in-ros-2/15261)，用一個機器手臂來實際應用 Service


## Parameters

主要拿來設定 Node 的組態（configuration），Parameters 可以是 `integers`、`floats`、`booleans`、`strings` 或 `lists`

### Commands
* `ros2 param list`
* `ros2 param get <node_name> <parameter_name>`
* `ros2 param set <node_name> <parameter_name> <value>`
* `ros2 param dump <node_name>`
* `ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>`
> Parameters 是用 `YAML` 格式儲存

