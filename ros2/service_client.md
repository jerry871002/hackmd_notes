---
tags: ROS 2
GA: UA-150355160-2
---

# ROS 2 - Simple service and client

[![hackmd-github-sync-badge](https://hackmd.io/AaXUv_bxRoKLVUSIuidq2g/badge)](https://hackmd.io/AaXUv_bxRoKLVUSIuidq2g)


如同第一篇 [ROS 2 - Concepts](/acmkN6_8T_uOOUFzX-TRBQ) 裡說的，nodes 之間用 service 溝通時，傳送 request 的是 client node，回覆 response 的則是 service node。而訊息的內容則是由 `.srv` 檔案決定的

因為之前把 C++ 跟 Python 分成兩篇寫覺得有些多餘，這邊就只分成兩個 section 好了

## C++

### Create package

在 `dev_ws/src` 裡建立 package

```bash
ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces
```

* `example_interfaces` 裡定義了我們要用到的 `.srv`
    ```
    int64 a
    int64 b
    ---
    int64 sum
    ```
* 因為用了 `--dependencies`，我們就不用手動去改 `package.xml` 跟 `CMakeLists.txt` 了

### Add server code

在 `dev_ws/src/cpp_srvcli/src` 中加入 `add_two_ints_server.cpp`，內容如下

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
```

### 稍微導覽一下

```cpp
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
         std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
        request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", 
        (long int)response->sum);
}
```

就是加起來囉

---

```cpp
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");
```

建立一個叫做 `add_two_ints_server` 的 node

---

```cpp
rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);
```

把 service 跟前面的 add 函式綁在一起

:::info
導覽有講跟沒講一樣哈哈，不過程式碼真的蠻直觀的，看一下應該都會懂（只是 C++ 的 namespace 比較複雜，每個都落落長）
:::

### Add executable for server

在 `CMakeLists.txt` 加入以下內容

```
add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
rclcpp example_interfaces)

install(TARGETS
  server
  DESTINATION lib/${PROJECT_NAME})
```

### Add client code

在 `dev_ws/src/cpp_srvcli/src` 中加入 `add_two_ints_client.cpp`，內容如下

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
```

不想導覽了，只是覺得他用了一堆模板，寫起來超級複雜而已

### Add executable for client

再更新一下 `CMakeLists.txt`，應該會變這樣
```
cmake_minimum_required(VERSION 3.5)
project(cpp_srvcli)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
  rclcpp example_interfaces)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp example_interfaces)

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

### Build and run

不用解釋的步驟如下

```bash
rosdep install -i --from-path src --rosdistro <distro> -y
colcon build --packages-select cpp_srvcli
. install/setup.bash
```

開始用用看吧
* `ros2 run cpp_srvcli server`
* `ros2 run cpp_srvcli client <a> <b>`

理論上 server 在開始時會顯示
```
[INFO] [rclcpp]: Ready to add two ints.
```
當收到 request 實則會有類似以下的內容
```
[INFO] [rclcpp]: Incoming request
a: 2 b: 3
[INFO] [rclcpp]: sending back response: [5]
```

## Python

方法大同小異，這邊就快速把要用到的指令貼一貼

### Create package

```
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
```

### Server node

`dev_ws/src/py_srvcli/py_srvcli` 裡建立 `service_member_function.py`

```python
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Client node

`dev_ws/src/py_srvcli/py_srvcli` 裡建立 `client_member_function.py`

```python
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (minimal_client.req.a, minimal_client.req.b, response.sum))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Add entry points

在 `setup.py` 裡面設定 executables

```python
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
```

### Build and run 

```
rosdep install -i --from-path src --rosdistro <distro> -y
colcon build --packages-select py_srvcli
. install/setup.bash
```

執行
* `ros2 run py_srvcli service`
* `ros2 run py_srvcli client 2 3`

:::info
看完這兩種不同語言的範例，我只想說**給我 Python，C++ 謝謝再聯絡**  
好啦平心而論，我想日常使用應該是 Python 比較好用，因為寫起來簡單，雖然 C++ 也是可以用 `using` 來讓程式碼簡短好讀一點，但還是沒有 Python 那樣簡潔好懂。  
不過 C++ 的強項在編譯期的型別檢查，不知道開發機器人這種系統時，這樣的功能是不是很重要（能幫忙避免在執行時發生錯誤之類的），如果是這樣的話 C++ 就有他好用的地方了。
:::

## Related content
* [這個 repo](https://github.com/ros2/examples/tree/master/rclcpp/services) 有關於 C++ 範例的更詳細說明
* [這個 repo](https://github.com/ros2/examples/tree/master/rclpy/services) 有關於 Python 範例的更詳細說明

## Reference
* [Writing a simple service and client (C++)](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Service-And-Client/), ROS Index
* [Writing a simple service and client (Python)](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Py-Service-And-Client/), ROS Index