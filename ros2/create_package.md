---
tags: ROS 2
GA: UA-150355160-2
---

# ROS 2 -  Creating packages

[![hackmd-github-sync-badge](https://hackmd.io/SKdyvJewQlKa3ik1_swfKg/badge)](https://hackmd.io/SKdyvJewQlKa3ik1_swfKg)


> A package can be considered a container for your ROS 2 code.

有點像把你做的東西打包起來，這樣才能發佈或是跟別人分享你的成果。ROS 2 使用 **ament** 作為他的 build system；使用 **colcon** 作為他的 build tool。創建一個 package 時可以使用 CMake 或 Python

:::info
我還沒搞清楚 ament、colcon、CMake 這些東西實際用起來的差別在哪，可能之後慢慢跟著做會有比較深入的了解

[GitHub 上有 issue](https://github.com/ros2/ros2/issues/576) 這樣解釋：

* `colcon` is a **build tool** that can build packages of multiple build systems (python, cmake, ament_cmake, ament_python, catkin...)
* The **build systems** used for ROS 2 are `ament_cmake` and `ament_python`
:::

## Package 的組成

因為我很懶，所以底下就用複製貼上的。列表是需要的檔案，底下則是做出來的檔案結構會長怎樣

### CMake

* `package.xml` file containing **meta information** about the package
* `CMakeLists.txt` file that describes **how to build** the code within the package

```
my_package/
     CMakeLists.txt
     package.xml
```

### Python

* `package.xml` file containing **meta information** about the package
* `setup.py` containing instructions for **how to install** the package
* `setup.cfg` is **required when a package has executables**, so `ros2 run` can find them
* `/<package_name>` - a directory with the same name as your package, used by ROS 2 tools to find your package, contains `__init__.py`

```
my_package/
      setup.py
      package.xml
      resource/my_package
```

### Packages 在 workspace 中的位置

Best practice 是在 workspace 中創建一個 `src/` 資料夾，把所有的 packages 放在裡面，裡面的 packages 可以有不同的 build type（CMake 或 Python）

```
workspace_folder/
    src/
      package_1/
          CMakeLists.txt
          package.xml

      package_2/
          setup.py
          package.xml
          resource/package_2
      ...
      package_n/
          CMakeLists.txt
          package.xml
```

## Create a package

記得要到 `src/` 裡面下指令喔，這邊他有多加了一個 `--node-name` 參數來指定 executable 的名字

### CMake

`ros2 pkg create --build-type ament_cmake <package_name>`

會回傳以下結果，可以認真看看他 create 了什麼東西

```
going to create a new package
package name: my_package
destination directory: /home/user/dev_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['<name> <email>']
licenses: ['TODO: License declaration']
build type: ament_cmake
dependencies: []
node_name: my_node
creating folder ./my_package
creating ./my_package/package.xml
creating source and include folder
creating folder ./my_package/src
creating folder ./my_package/include/my_package
creating ./my_package/CMakeLists.txt
creating ./my_package/src/my_node.cpp
```

### Python

`ros2 pkg create --build-type ament_python <package_name>`

會回傳以下結果，重點也是他 create 了什麼東西

```
going to create a new package
package name: my_package
destination directory: /home/user/dev_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['<name> <email>']
licenses: ['TODO: License declaration']
build type: ament_python
dependencies: []
node_name: my_node
creating folder ./my_package
creating ./my_package/package.xml
creating source folder
creating folder ./my_package/my_package
creating ./my_package/setup.py
creating ./my_package/setup.cfg
creating folder ./my_package/resource
creating ./my_package/resource/my_package
creating ./my_package/my_package/__init__.py
creating folder ./my_package/test
creating ./my_package/test/test_copyright.py
creating ./my_package/test/test_flake8.py
creating ./my_package/test/test_pep257.py
creating ./my_package/my_package/my_node.py
```

## Build a package

都建立好了之後就要來 build 囉，回到 workspace 的根目錄後用 `colcon build`

預設是會 build 在這個 workspace 裡面全部的 packages，如果 packages 很多的話可能會超慢，這時候就可以用 `--packages-select` 來指定 build 哪些 packages 了

`colcon build --packages-select <package_name_1> <package_name_2>`

## Source the setup file

Build 完之後會產生一個 `install/` 資料夾，裡面有設置環境變數所需的檔案，所以我們來執行

`. install/setup.bash`

:::warning
注意這個 `install/` 資料夾是在 workspace 的根目錄底下喔，不是在 package 裡面，我剛剛搞錯了一直找不到
:::

這樣我們就可以開始使用囉

`ros2 run <package_name> <executable_name>`

## Package 的內容

### CMake

```
my_package1/
    CMakeLists.txt
    include/
        my_package1/
    package.xml
    src/
        my_node1.cpp

```

之後我們要做更多 node 時就要在 `src/` 資料夾裡面做

看完檔案結構我們可以去看看 `package.xml`，裡面有一些可以設定的東西，像是 `<maintainer>`、`<description>`、`<license>` 等資訊。另外也有一些有關 dependencies 的資訊在裡面（不過那個到目前為止都是自動產生的，先不用去動它）

### Python

```
my_package2/
    my_package2/
        __init__.py
        my_node2.py
    package.xml
    resource/
        my_package2
    setup.cfg
    setup.py
    test/
        test_copyright.py
        test_flake8.py
        test_pep257.py
```

之後我們要做更多 node 時就要在 `<package_name>/` 資料夾裡面做

Python 的 package 除了有 `package.xml` 外，還多了一個 `setup.py`，裡面的資訊必須要跟 `package.xml` 一樣，例如

這個 `package.xml`
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_package2</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="jerry@todo.todo">jerry</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

會對應到這個 `setup.py`

```python
from setuptools import setup

package_name = 'my_package2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jerry',
    maintainer_email='jerry@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node2 = my_package2.my_node2:main'
        ],
    },
)
```

## Reference
* [Creating your first ROS 2 package](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/), ROS Index
* [Why there are so many ament things in colcon? Will they remain in the future releases?](https://github.com/ros2/ros2/issues/576), GitHub