---
tags: ROS 2
GA: UA-150355160-2
---

# ROS 2 - Tools

## rqt_graph
拿來觀察 Node 跟 Topic 之間的關係，[前一篇](/acmkN6_8T_uOOUFzX-TRBQ)有提到

啟動用 `rqt_graph`

![rqt_graph](https://index.ros.org/doc/ros2/_images/rqt_graph.png)

## rqt_console
拿來看 log messages（雖然 log messages 本來就會在 terminal 上面顯示，但這個工具可以讓我們用更有組織的方式查看、過濾甚至清除 log messages）

啟動用 `ros2 run rqt_console rqt_console`

![rqt_console](https://index.ros.org/doc/ros2/_images/console.png)

ROS 2 裡面的 log messages 分成五個等級
* Fatal
* Error
* Warn
* Info（預設等級，若沒有特別設定只會顯示這個等級以上的）
* Debug

可以像這樣重新設定  
`ros2 run turtlesim turtlesim_node --ros-args --log-level WARN`

## Reference
* [Using rqt_console](https://index.ros.org/doc/ros2/Tutorials/Rqt-Console/Using-Rqt-Console/), ROS Index