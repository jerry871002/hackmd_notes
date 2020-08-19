---
tags: ROS 2
GA: UA-150355160-2
---

# ROS 2 -  Publisher and subscriber in Python

這篇可以跟前一篇 [ROS 2 -  Publisher and subscriber in C++](/QiJrNvOaTJWJ2nVd8FDBOg?both) 對照著看，基本上是要做一模一樣的功能，只是現在用 Python 做而已

因為很多步驟前面都解釋過了，這邊會快速帶過喔

## Create package

```bash
ros2 pkg create --build-type ament_python py_pubsub
```

進到 `dev_ws/src/py_pubsub/py_pubsub` 裡面下載程式

```bash
wget https://raw.githubusercontent.com/ros2/examples/master/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```

完整程式如下

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Publisher 程式碼導覽

```python
import rclpy
from rclpy.node import Node
```

* 常用的東西放在 `rclpy` 裡面（跟 C++ 裡面的 `rclcpp/rclcpp.hpp` 一樣意思）
* 等等我們的 class 要繼承 `Node`，把他單獨寫出來比較好用

---

```python
from std_msgs.msg import String
```

這個就是我們要用的 message type 囉，等等我們也要把 `rclpy` 跟 `std_msgs` 加進 `package.xml` 裡面

---

```python
def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
```

設定一個 timer 每 0.5 秒會呼叫 `timer_callback`

---

```python
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```

被呼叫時（每 0.5 秒）會 publish 一個 `String` 到 `topic` 

## Publisher's dependencies

### `package.xml`

加這些

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

### `setup.py`

他有說要先確定 `maintainer`、`maintainer_email`、`description` 跟 `license` 要跟 `package.xml` 一樣，不過我都沒改，所以就沒做這步了

比較重要的是下面這個部分

```python
entry_points={
    'console_scripts': [
        'talker = py_pubsub.publisher_member_function:main',
    ],
},
```

### `setup.cfg`

這個部分已經自動產生了，只是要檢查一下而已

```cfg
[develop]
script-dir=$base/lib/py_pubsub
[install]
install-scripts=$base/lib/py_pubsub
```

加這些是讓我們的 executable 被加到 `lib/` 裡面，因為 `ros2 run` 會去那邊找（跟 C++ 的 `CMakeLists.txt` 的功用很像）

## Subscriber

要來創造下一個 node 了，下載程式碼（到 `dev_ws/src/py_pubsub/py_pubsub`）

```bash
wget https://raw.githubusercontent.com/ros2/examples/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```

程式長這樣，這次不導覽了自己看

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Dependencies

這次只有 `setup.py` 要改

```python
entry_points={
    'console_scripts': [
        'talker = py_pubsub.publisher_member_function:main',
        'listener = py_pubsub.subscriber_member_function:main',
    ],
},
```

## Build and run

以下是已經做過很多次的部分，就不多做解釋囉

* `rosdep install -i --from-path src --rosdistro <distro> -y`
* `colcon build --packages-select py_pubsub`
* `. install/setup.bash`

開兩個 terminal 分別執行
* `ros2 run py_pubsub talker`
* `ros2 run py_pubsub listener`

應該就可以順利看到結果囉

## Related content

* [這個 repo](https://github.com/ros2/examples/tree/master/rclpy/topics) 有用不同的方式寫出一樣的功能
:::info
不過這個範例寫在 class 裡面的方式我最喜歡，裡面介紹的其他方式我覺得 node 多起來之後會很雜亂
:::

## Reference

* [Writing a simple publisher and subscriber (Python)](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber/), ROS Index

