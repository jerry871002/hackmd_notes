---
tags: ROS 2
GA: UA-150355160-2
---

# ROS 2 -  Publisher and subscriber in C++

## Create package

用之前建立的 workspace，到 `src/` 資料夾底下建立 package

```
ros2 pkg create --build-type ament_cmake cpp_pubsub
```

進到 `dev_ws/src/cpp_pubsub/src` 中下載範例程式

```
wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_publisher/member_function.cpp
```

完整程式如下

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
  }
```

## Publisher 程式碼導覽

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
```

這個 `rclcpp/rclcpp.hpp` 檔案包含了 ROS 2 中最常用的部分，基本上你在 ROS 2 寫 C++ 就一定會 include 到這個檔案

:::info
rcl 的意思是 **ROS Client Library**，所以 rclcpp 就是 ROS Client Library for C++，rclpy 就是 ROS Client Library for Python
:::

`std_msgs/msg/string.hpp` 則是定義了一個 built-in message type 讓我們能在 publish 東西時使用

這兩個檔案同時也是這個 package 的 dependencies，稍後會把他們加進 `package.xml` 跟 `CMakeLists.txt` 中


---

```cpp
class MinimalPublisher : public rclcpp::Node
```

ROS 2 的 node 都是從這個 `rclcpp::Node` 類別繼承下來的喔（用 C++ 寫的話）

---

```cpp
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }
```

* `minimal_publisher` 是這個 node 的名字
* `publisher_` 定義了一個會傳送 `std_msgs::msg::String` 類型訊息給名為 `topic` 的 topic（好拗口）
* `timer_` 定義了每 500ms 會執行一次 `MinimalPublisher::timer_callback`

---

```cpp
Private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
```

`RCLCPP_INFO` 是一個 marco，他確保所有 publish 的訊息會印到 console

---

```cpp
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

* `rclcpp::init` 初始化 ROS 2
* `rclcpp::spin` 開始處理來自 node 的 data（timer 跟 callback）

## Publisher's dependencies

接下來要開始處理 dependencies 的問題

### `package.xml`

在 `package.xml` 加入以下內容

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

### `CMakeLists.txt`

在 `CMakeLists.txt` 裡 `find_package(ament_cmake REQUIRED)` 這行後面加入以下內容

```
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```

加這堆有的沒的就是要讓 `ros2 run` 可以找到我們的 executable 並順利執行

## Subscriber 程式碼導覽

一樣先去下載它

```bash
wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_subscriber/member_function.cpp
```

完成程式如下

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

接下來也是稍微來看一下裡面寫了什麼

---

```cpp
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }
```

* 跟上面的 publisher 一樣，也是繼承自 `Node`。這次 node 的名字叫做 `minimal_subscriber`
* `subscription_` 會接收名叫 `topic` 的 topic 傳來的資料，接收時會呼叫 `topic_callback`

---

```cpp
private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
```

蠻單純的，就是用 `RCLCPP_INFO` 把東西印出來而已

## Subscriber's dependencies

### `package.xml`

沒有東西要加，因為跟 publisher 需要的 dependencies 一樣

### `CMakeLists.txt`

在 `CMakeLists.txt` 裡繼續加入以下內容

```
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

## Build and run

先在 workspace 的根目錄確定所有 dependencies 都已經安裝好了

```
rosdep install -i --from-path src --rosdistro <distro> -y
```

:::warning
在做這步的時候發現在 `ros2 pkg create` Python 的 package 時有一個 bug，[這個 GitHub issue](https://github.com/ros2/ros2_documentation/issues/508) 有解釋並給解決方案
:::

接著來 build 我們的 package

```bash
colcon build --packages-select cpp_pubsub
```

Build 完記得 source 我們的環境

```bash
. install/setup.bash
```

開兩個 terminal 分別執行（記得都要 source 環境）
* `ros2 run cpp_pubsub talker`
* `ros2 run cpp_pubsub listener`

## Related content

* [這個 repo](https://github.com/ros2/examples/tree/master/rclcpp/topics/minimal_publisher) 有用不同的方式寫出一樣的功能的介紹（主要是介紹用 `lambda` 的寫法）

## Reference

* [Writing a simple publisher and subscriber (C++)](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/), ROS Index
* [Pyhton Simple Service: Depend ament_python error](https://github.com/ros2/ros2_documentation/issues/508), GitHub

