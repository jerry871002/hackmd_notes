---
tags: ROS 2
GA: UA-150355160-2
---

# ROS 2 - Launch and Play Back

## `ros2 launch`

當 Node 越來越多時，每次都要開一堆 terminal 來啟動跟設定就會變得超級麻煩，這時候就需要 launch file 來幫忙這件事

雖然說 `ros2 launch` 可以一次啟動**所有的** nodes，但如果你不想一次啟動所有的 nodes 的話，就要寫 launch file 來指定你要啟動的 nodes 跟那些 nodes 的 configuration

launch file 大概長這樣（用 `Python` 寫喔）
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            node_namespace='turtlesim1',
            node_executable='turtlesim_node',
            node_name='sim'
        ),
        Node(
            package='turtlesim',
            node_namespace='turtlesim2',
            node_executable='turtlesim_node',
            node_name='sim'
        ),
        Node(
            package='turtlesim',
            node_executable='mimic',
            node_name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

這個 launch file 會打開兩個 turtlesim 視窗，然後讓第二個視窗的烏龜模仿第一個視窗的烏龜的動作

啟動的方式是輸入 `ros2 launch <launch_file_name>`

啟動完成後可以用 `rqt_graph` 來看看 nodes 之間的關係

## `ros2 bag`

### 安裝

這邊記錄一下安裝的時候遇到的問題，官網給的指令是

```bash
sudo apt-get install ros-<distro>-ros2bag ros-<distro>-rosbag2*
```

不過照這個指令安裝的話會遇到一堆 dependencies 的問題（因為我沒有裝 ROS 1），仔細看一下會發現是 `ros-dashing-rosbag2-bag-v2-plugins` 這個套件在搞鬼

所以把除了他以外的裝一裝就好了（至少在這篇 tutorial 可以 work），可以用 `apt list ros-<distro>-rosbag2*` 看看總共要裝哪些東西

### record

我們只能記錄有人 publish on 的 topic，所以先用 `ros2 topic list` 看看有哪些 topic 可以用

用 `ros2 bag record <topic_name>` 開始記錄，記錄完成後按 `Ctrl + c` 終止

一次想記錄多個 topic 的話，可以像這樣

`ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose`

這邊的 `-o` 是幫紀錄檔取名字，不然預設的格式是 `rosbag2_year_month_day-hour_minute_second`

也可以在指令中加 `-a` 來記錄**所有的** topic，但他不推薦使用（好像有可能會 crash 掉系統）

### info

用 `ros2 bag info <bag_file_name>` 來查看關於這個 bag 的資訊，會顯示像下面這樣的東西

```bash
Bag size:          228.5 KiB
Storage id:        sqlite3
Duration:          48.47s
Start:             Oct 11 2019 06:09:09.12 (1570799349.12)
End                Oct 11 2019 06:09:57.60 (1570799397.60)
Messages:          3013
Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 9 | Serialization Format: cdr
                 Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 3004 | Serialization Format: cdr
```

### play

用 `ros2 bag play <bag_file_name>` 來讓系統重現剛剛記錄的東西

### Related content

* [這個 GitHub Repo](https://github.com/ros2/rosbag2) 有對 `ros2 bag` 更詳細的介紹
* [這篇](https://index.ros.org/doc/ros2/Tutorials/Ros2bag/Overriding-QoS-Policies-For-Recording-And-Playback/#ros2bag-qos-override)則介紹 QoS 跟 `ros2 bag` 之間的關係

## Reference
* [Creating a launch file](https://index.ros.org/doc/ros2/Tutorials/Launch-Files/Creating-Launch-Files/), ROS Index
* [Recording and playing back data](https://index.ros.org/doc/ros2/Tutorials/Ros2bag/Recording-And-Playing-Back-Data/), ROS Index