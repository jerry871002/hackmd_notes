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

Service 是另一種 Node 之間溝通的方法

跟 Topic 的比較

|         Service         |              Topic               |
|:-----------------------:|:--------------------------------:|
| call-and-response model |    publisher-subscriber model    |
|   要呼叫才會提供資料       | Subscribe 後可以接收連續性的資料 |

