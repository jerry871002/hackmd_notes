---
tags: ROS 2
GA: UA-150355160-2
---

# ROS 2 - Concepts

[![hackmd-github-sync-badge](https://hackmd.io/acmkN6_8T_uOOUFzX-TRBQ/badge)](https://hackmd.io/acmkN6_8T_uOOUFzX-TRBQ)


## Nodes

Node 可以看成一個一個模組，模組之間互動的機制就是 ROS 主要的功能

![Nodes](https://index.ros.org/doc/ros2/_images/Nodes-TopicandService.gif)

### Commands
* `ros2 run <package_name> <executable_name>`
* `ros2 node list`
* `ros2 node info <node_name>`

## Topics

Topic 是 Node 之間溝通的管道，Node 可以 Publish 來發送資料，Subscribe 來接收資料

這樣的傳輸方式可以點對點傳輸

![Topics 1-1](https://index.ros.org/doc/ros2/_images/Topic-SinglePublisherandSingleSubscriber.gif)

也可以一對多，甚至可以多對多

![Topics 1-N N-N](https://index.ros.org/doc/ros2/_images/Topic-MultiplePublisherandMultipleSubscriber.gif)

### Commands
* `rqt_graph`
    * 拿來觀察 Node 跟 Topic 之間的關係
    ![rqt_graph](https://index.ros.org/doc/ros2/_images/rqt_graph.png)
* `ros2 topic list`
* `ros2 topic list -t`
    * 檢查 type
* `ros2 topic echo <topic_name>`
* `ros2 topic info <topic_name>`
* `ros2 interface show <type>.msg`
* `ros2 topic pub <topic_name> <msg_type> '<args>'`
    * `ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"`
    * `ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"`
    * **`--once`** 只 publish 一次 **`--rate`** 可以設定 publish 的頻率
* `ros2 topic hz <topic_name>`


## Services

Service 是另一種 Node 之間溝通的方法，Service 只在明確跟他 request 時才會傳資料給你

![Services](https://index.ros.org/doc/ros2/_images/Service-SingleServiceClient.gif)

可以有多個 Service client 但只能有一個 Service server

![Services](https://index.ros.org/doc/ros2/_images/Service-MultipleServiceClient.gif)

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

主要拿來設定 Node 的組態（configuration），Parameters 可以是 `integers`、`floats`、`booleans`、`strings` 或 `lists` 等

### Commands
* `ros2 param list`
* `ros2 param get <node_name> <parameter_name>`
* `ros2 param set <node_name> <parameter_name> <value>`
* `ros2 param dump <node_name>`
* `ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>`
> Parameters 是用 `YAML` 格式儲存

## Actions

Action 就像 Service，可以執行長時間運行的任務，提供定期的 feedback 並可以取消

Action 包含三個部分
* **goal**：想達成的事
* **result**：執行 action 的結果
* **feedback**：執行過程中的反饋

Action 雖然跟 Service 很像，不過如上述所言，有兩個不同的地方
* Action 是**可以被打斷的**（preemptable）
* Action 可以有**穩定的 feedback**，Service 就只有單一的 response

Action 的機制是 **client-server model**：action client 送一個 goal 給 action server，action server 回傳 feedback stream 和一個 result

Action 通常拿來做 navigation，目標點是 goal，一路上會提供 feedback

![Actions](https://index.ros.org/doc/ros2/_images/Action-SingleActionClient.gif)

### Commands
* `ros2 node info /turtlesim`
    * 可以看有哪些 action
* `ros2 action list`
* `ros2 action list -t`
    * 看 action 的 type
* `ros2 action info <action_name>`
* `ros2 interface show <action_name>.action`
* `ros2 action send_goal <action_name> <action_type> <values>`
    * `<value>` 的格式是 YAML
    * 最後面加 `--feedback` 可以觀察 feedback

### Related content
[這篇](https://design.ros2.org/articles/actions.html)對 action 有更詳細的介紹


