---
tags: ROS 2
GA: UA-150355160-2
---

# ROS 2 - Tutorials 

[![hackmd-github-sync-badge](https://hackmd.io/DwaZUt-2Qk6NG-cRO8QigQ/badge)](https://hackmd.io/DwaZUt-2Qk6NG-cRO8QigQ)


這些 tutorials 來自[這個 Youtube 播放清單](https://www.youtube.com/watch?v=5DlrkkH3LS4&list=PLK0b4e05LnzYNBzqXNm9vFD9YXWp6honJ)，比較偏向教你一些基本的操作，對觀念的部分沒有著墨太多（跟官網的比起來）

## How to launch a ROS 2 node

1. 每次開始前都要 `source /opt/ros/crystal/setup.bash` 來設定環境
    > ROS 2 有推出新的版本了，source 前要先確定自己用的是哪一個
3. 指令的格式是 `ros2 run <package_name> <executable_name>`
4. `<package_name>` 的部分先用 `ros2 pkg list` 檢查有哪些 packages 已經安裝了
5. `<executable_name>` 的部分用 `ros2 pkg executables` 檢查有哪些可用
   輸出的格式是 `<package_name> <executable_name>`
   一個 package 可以有多個 executable 
4. 開兩個 terminal
   一個執行 `ros2 run examples_rclcpp_minimal_publisher publisher_not_composable`
   另一個執行 `ros2 run examples_rclcpp_minimal_subscriber subscriber_not_composable`
   觀察看看結果

## How to Publish & Subscribe to a ROS 2 Topic

1. `ros2 topic list` 可以用來檢查現在有哪些 topic
2. 如果要 publish 一個 topic 的話，指令的格式是 `ros2 topic pub <topic_name> <msg> <structure>`
3. 前面的 `<msg>` 部分，用 `ros2 msg list` 看哪些可以用（the **type** of the message）
   我們先選 `std_msgs/String`，可以用 `ros2 msg show std_msgs/String` 看說明
4. `ros2 topic pub /chatter std_msgs/String "data: Hello ROS Developers"`
5. 先用 `ros2 topic list` 檢查有沒有 publish 成功
   接著用 `ros2 topic echo /chatter` subscibe 看看
   
## How to create a ROS 2 Workspace

1. ROS1 用 `catkin_ws`，ROS 2 則是用 `ros2_ws`
2. 用 `mkdir -p my_ros2_ws/src` 建立 workspace
3. 進到 `my_ros2_ws/` 然後執行 `colcon build`
4. 執行 `source install/local_setup.bash && source install/setup.bash` 來 setup 這個 workspace

他提供的步驟

| Step | Description |
| :--: | ----------- |
| 1    | **Install ROS 2** <br> I will use ROSDS for this example, which allows me to create a ROSject which already has ROS 2 Crystal installed on it . Heck, no time to go through the installation now. Unless you have > 24 hours per day, you can try ROSDS too 🙂 |
| 2    | **Create workspace directory** <br>`~$ mkdir -p ros2_ws/src && cd ros2_ws` |
| 3    | **Build workspace with `colcon`** <br> `~/ros2_ws$ colcon build` |
| 4    | **Source workspace setup files** <br> `~/ros2_ws$ source install/local_setup.bash && source install/setup.bash` |

## How to create a ROS 2 Package for C++

1. 在你剛剛建立的 workspace 裡面建立 package（要在 src 資料夾底下） 
   語法：`ros2 pkg create <package_name> --build-type <build_type> --dependencies <dependencies_separated_by_single_space>`
2. 執行 `ros2 pkg create ros2_cpp_pkg --build-type ament_cmake --dependencies rclcpp`
3. 進到剛建立的 `ros2_cpp_pkg/src` 並建立 `ros2_cpp_code.cpp` 來寫點 C++
    ```cpp
    #include "rclcpp/rclcpp.hpp"

    int main(int argc, char *argv[]) {
      rclcpp::init(argc, argv);
      auto node = rclcpp::Node::make_shared("ObiWan");

      RCLCPP_INFO(node->get_logger(),
                  "Help me Obi-Wan Kenobi, you're my only hope");

      rclcpp::shutdown();
      return 0;
    }
    ```
4. 打開 `CMakeLists.txt` 並加上以下內容
    ```cmake
    add_executable(cpp_code src/ros2_cpp_code.cpp)
    ament_target_dependencies(cpp_code rclcpp)

    install(TARGETS
        cpp_code
        DESTINATION lib/${PROJECT_NAME}
    )
    ```
    注意等等的 executable 會被叫做 `cpp_code`
5. 回到 workspace 的根目錄編譯 `colcon build --symlink-install`
6. 再 source 一次 `source install/setup.bash`
7. 執行 `ros2 run ros2_cpp_pkg cpp_code`

## How to create a ROS 2 Package for Python

1. 感覺步驟跟上個部分大同小異，先進到 workspace 的 src 資料夾
2. `ros2 pkg create ros2_demo_py`
3. 刪除 `CMakeLists.txt`，建立 `setup.py` 和 `setup.cfg` 然後修改 `package.xml`
4. `setup.py` 裡寫這些
    ```python
    from setuptools import setup

    package_name = 'ros2_demo_py'

    setup(
        name=package_name,
        version='0.7.0',
        packages=[package_name],
        install_requires=['setuptools'],
        zip_safe=True,
        author='You',
        author_email='you@youremail.com',
        maintainer='YourFirstname Lastname',
        maintainer_email='your@youremail.com',
        keywords=['ROS'],
        classifiers=[
            'Intended Audience :: Developers',
            'License :: OSI Approved :: Apache Software License',
            'Programming Language :: Python',
            'Topic :: Software Development',
        ],
        description='A simple ROS 2 Python package',
        license='Apache License, Version 2.0',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'demo = ros2_demo_py.demo:main'
            ],
        },
    )
    ```
5. `setup.cfg` 寫
    ```cfg
    [develop]
    script-dir=$base/lib/ros2_demo_py
    [install]
    install-scripts=$base/lib/ros2_demo_py
    ```
6. `package.xml` 改成這些
    ```xml
    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="2">
      <name>ros2_demo_py</name>
      <version>0.7.3</version>
      <description>A simple ROS 2 Python package</description>

      <maintainer email="sloretz@openrobotics.org">Shane Loretz</maintainer>
      <license>Apache License 2.0</license>

      <exec_depend>rclpy</exec_depend>
      <exec_depend>std_msgs</exec_depend>

      <!-- These test dependencies are optional
      Their purpose is to make sure that the code passes the linters -->
      <test_depend>ament_copyright</test_depend>
      <test_depend>ament_flake8</test_depend>
      <test_depend>ament_pep257</test_depend>
      <test_depend>python3-pytest</test_depend>

      <export>
        <build_type>ament_python</build_type>
      </export>
    </package>
    ```
7. 建立一個 `ros2_demo_py/` 然後進去 `mkdir ros2_demo_py && cd ros2_demo_py`
8. 把這個資料夾做成 Python 的 package `touch demo.py && touch __init__.py`
9. `demo.py`
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
10. 回到 workspace 的根目錄編譯 `colcon build --symlink-install`
11. 記得 `source install/setup.bash`
12. `ros2 run ros_demo_py demo` 