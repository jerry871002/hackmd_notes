---
tags: ROS 2
GA: UA-150355160-2
---

# ROS 2 - Creating a workspace

[![hackmd-github-sync-badge](https://hackmd.io/wo2LvYsNSmOE74e-ShrTng/badge)](https://hackmd.io/wo2LvYsNSmOE74e-ShrTng)


> A workspace is a directory containing ROS 2 packages

workspace 有分成兩種

* overlay：輔助的 workspace，可以在裡面添加新的 packages
* underlay：安裝 ROS 2 時的主要 workspace

我們自己寫的東西放在 overlay 裡面，這樣就不會跟 underlay 的東西互相干擾

## 創建目錄

假設我們創建了一個叫做 `dev_ws/` 的 workspace

```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

這邊有說一個 best practice 是把 packages 放在 `src/` 裡面

## Resolve dependencies

假設你的 workspace 裡面已經有了一些東西，在 build 之前要先檢查 dependencies

`rosdep install -i --from-path src --rosdistro <distro> -y`

:::info
我好像又沒有裝 `rosdep` 了，先來裝一下

```bash
sudo apt install python-rosdep2
sudo rosdep init
rosdep update
```
:::

如果檢查成功的話會顯示

`#All required rosdeps installed successfully`

一般來說，dependencies 會寫在一個 `.xml` 檔案中，之後會再詳細介紹怎麼用

## Build

回到 workspace 的根目錄用 `colcon build`

:::info
我也沒裝 `colcon`，來裝一下

```bash
sudo apt install python3-colcon-common-extensions
```
:::

還有一些有用的參數
* `--packages-up-to`：可以只 build 你需要的 package（節省時間）
* `--symlink-install`：好像是讓你改 Python script 的時候不用重新 build
* `--event-handlers console_direct+`：可以把 build 時候的 log 顯示出來

Build 完之後會跑出一些新的資料夾（`ls`）
```
build  install  log  src
```

## Source the overlay

Build 完之後就要來使用啦，跟之前一樣 source 一個環境是做每件事的第一步驟

打開一個新的 terminal
1. 先 source 原本的 ROS 2 環境作為 underlay  
    `source /opt/ros/<distro>/setup.bash`
2. 進到你的 workspace  
    `. install/local_setup.bash`
    
這樣你就可以用 underlay 跟 overlay 的東西啦！如果你的 underlay 跟 overlay 裡面有相同的 package，以 overlay 的為主喔（前提是你要記得 source 它）

## Reference
* [Creating a workspace](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/), ROS Index