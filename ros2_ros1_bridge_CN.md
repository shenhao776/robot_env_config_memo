# ROS 1 与 ROS 2 之间的通信桥接

该功能包提供了一个网络桥接工具，能够实现 ROS 1 和 ROS 2 之间的消息交互。

当前该桥接工具采用 C++ 实现，原因是开发初期 ROS 2 的 Python API 尚未开发完成。因此，其支持范围仅限于桥接工具编译时已存在的消息/服务类型。ROS 2 预编译二进制包中附带的桥接工具，已支持常用的 ROS 接口（消息/服务），例如 [ros2/common_interfaces 代码仓库](https://github.com/ros2/common_interfaces) 中列出的接口功能包以及 `tf2_msgs`。关于 ROS 1 与 ROS 2 接口如何关联的更多细节，请参考 [官方文档](doc/index.rst)。

若需使用桥接工具支持其他接口（包括自定义类型），你需要从源码编译桥接工具（编译步骤见下文），且需提前在独立的 ROS 1 和 ROS 2 工作空间中编译自定义类型，并通过 `source` 命令设置其环境变量。关于具体的配置示例，请参考 [官方文档](doc/index.rst)。


## 效率优化说明
为保证效率，只有当某一话题在桥接工具两侧（ROS 1 和 ROS 2）均存在匹配的“发布者-订阅者”对时，该话题才会被桥接。这会导致一个问题：若当前无其他订阅者，使用 `ros2 topic echo <话题名>` 命令无法正常工作，且会报错 `无法确定所传递话题的类型`（原错误信息：`Could not determine the type for the passed topic`）—— 因为动态桥接工具尚未对该话题建立桥接。

**解决方案**：可显式指定话题类型，命令格式为 `ros2 topic echo <话题名> <话题类型>`。此时 `echo` 命令本身会作为订阅者，触发桥接工具对该话题的桥接。

而在 ROS 1 侧，`rostopic echo` 命令没有显式指定话题类型的选项，因此若当前无其他订阅者，该命令无法与动态桥接工具配合使用。作为替代方案，你可以使用 `--bridge-all-2to1-topics` 选项，将所有 ROS 2 话题桥接到 ROS 1。这样即使没有匹配的 ROS 1 订阅者，`rostopic echo`、`rostopic list`、`rqt` 等工具也能识别到这些话题。

如需查看更多选项，请运行命令：`ros2 run ros1_bridge dynamic_bridge -- --help`。


## 前提条件
要运行桥接工具，你需要满足以下任一条件：
* 获取 [ROS 2 预编译二进制包](https://github.com/ros2/ros2/releases)；
* 从源码编译桥接工具及其他 ROS 2 功能包。

完成上述操作后，你即可运行下文描述的两个示例。

运行所有示例前，需先通过 `source` 命令设置桥接工具安装目录的环境变量（无论桥接工具是预编译还是源码编译）。此外，你还需通过 `source` 命令设置 ROS 1 环境变量，或至少手动设置 `ROS_MASTER_URI` 并启动 `roscore`。


### 编译与使用桥接工具所需的 ROS 1 功能包
* `catkin`（构建工具）
* `roscpp`（C++ 客户端库）
* `roslaunch`（用于启动 `roscore` 可执行文件）
* `rosmsg`（消息相关工具）
* `std_msgs`（标准消息类型）
* Python 功能包 `rospkg`


### 运行示例所需的额外 ROS 1 功能包
* `rosbash`（提供 `rosrun` 可执行命令）
* `roscpp_tutorials`（C++ 示例功能包）
* `rospy_tutorials`（Python 示例功能包）
* `rostopic`（话题相关工具）
* `rqt_image_view`（图像可视化工具）


## 从源码编译桥接工具
在继续操作前，请先按照 [官方说明](https://github.com/ros2/ros2/wiki/Installation) 安装从源码编译 ROS 2 所需的前提依赖。

过去，编译该功能包需要对 ROS 1 打补丁，但在最新版本中已不再需要。若编译过程中遇到问题，请先确认你的 `ros_comm` 和 `rosbag` 版本是否至少为 `1.11.16`。

桥接工具使用 `pkg-config` 查找 ROS 1 功能包，使用 CMake 的 `find_package()` 命令查找 ROS 2 功能包。因此，`CMAKE_PREFIX_PATH` 环境变量中不应包含 ROS 1 的路径，否则可能会覆盖 ROS 2 功能包。

以下是 Linux 和 OSX 系统的编译步骤：


### 步骤 1：编译除桥接工具外的所有 ROS 2 功能包
首先，使用常规的 colcon 命令编译除 `ros1_bridge`（桥接工具功能包）外的所有功能包。建议在此步骤中不要设置 ROS 1 环境变量，以免其他库路径干扰编译。

```bash
colcon build --symlink-install --packages-skip ros1_bridge
```
*说明：`--symlink-install` 表示创建符号链接安装，可避免每次修改源码后重新编译安装。*


### 步骤 2：设置 ROS 1 环境变量
接下来需要通过 `source` 命令设置 ROS 1 环境变量。以 Linux 系统 + ROS Melodic 版本为例：
```bash
source /opt/ros/melodic/setup.bash
# 若为 OSX 系统，命令类似如下（路径需根据实际安装目录调整）：
# . ~/ros_catkin_ws/install_isolated/setup.bash
```


### 步骤 3：添加自定义消息/服务功能包的环境变量（可选）
桥接工具会自动支持当前环境变量路径下、且已建立 ROS 1 与 ROS 2 关联映射的消息/服务功能包。因此，若需桥接自定义消息/服务，需在编译桥接工具前，将包含这些自定义类型的 ROS 1 或 ROS 2 工作空间添加到环境变量中。

实现方式有两种：
1. **显式依赖配置**：在桥接工具的 `package.xml` 文件中，添加对自定义消息/服务功能包的显式依赖。这样 colcon 在编译桥接工具时，会自动将这些功能包添加到环境变量路径中。
2. **手动设置环境**：通过 `source` 命令手动设置自定义工作空间的环境变量。示例如下：
   ```bash
   # 前提：已设置好 ROS 1 安装环境（通过 source 命令）
   # 设置 ROS 2 安装目录的环境变量：
   . <ROS 2 安装目录>/local_setup.bash
   # 若存在 ROS 1 覆盖工作空间，命令类似如下（路径需调整）：
   # . <ROS 1 覆盖工作空间安装目录>/setup.bash
   # 若存在 ROS 2 覆盖工作空间，命令类似如下（路径需调整）：
   # . <ROS 2 覆盖工作空间安装目录>/local_setup.bash
   ```


### 步骤 4：编译 ROS 1 桥接工具
仅编译 `ros1_bridge` 功能包：
```bash
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

*注意：若在内存受限的系统上编译，建议通过设置环境变量限制并行任务数量，例如：`export MAKEFLAGS=-j1`。*


## 示例 1：运行桥接工具及示例发布者（talker）和订阅者（listener）
示例中的发布者（talker）和订阅者（listener）可分别为 ROS 1 或 ROS 2 节点，桥接工具会透明地传递消息。

*注意：运行这些示例时，务必仅设置指定工作空间的环境变量（通过 source 命令）。若环境中同时包含 ROS 1 和 ROS 2 工作空间的路径，大多数工具都会报错。*


### 示例 1a：ROS 1 发布者（talker） + ROS 2 订阅者（listener）

#### 步骤 1：启动 ROS 1 的 roscore
```bash
# Shell A（仅 ROS 1 环境）：
. /opt/ros/melodic/setup.bash
# 若为 OSX 系统，命令类似如下（路径需调整）：
# . ~/ros_catkin_ws/install_isolated/setup.bash
roscore
```


#### 步骤 2：启动动态桥接工具
动态桥接工具会监听当前可用的 ROS 1 和 ROS 2 话题，一旦检测到匹配的话题，就会开始桥接该话题的消息。

```bash
# Shell B（ROS 1 + ROS 2 混合环境）：
# 先设置 ROS 1 环境变量：
. /opt/ros/melodic/setup.bash
# 若为 OSX 系统，命令类似如下（路径需调整）：
# . ~/ros_catkin_ws/install_isolated/setup.bash
# 再设置包含桥接工具的 ROS 2 环境变量：
. <包含桥接工具的 ROS 2 安装目录>/setup.bash
# 示例（若使用 ROS 2 Dashing 版本）：
# . /opt/ros/dashing/setup.bash
# 设置 ROS_MASTER_URI（指向 roscore 地址）：
export ROS_MASTER_URI=http://localhost:11311
# 启动动态桥接工具：
ros2 run ros1_bridge dynamic_bridge
```

启动后，程序会定期在终端中输出当前 ROS 1 和 ROS 2 中可用的话题列表。


#### 步骤 3：启动 ROS 1 发布者（talker）
```bash
# Shell C：
. /opt/ros/melodic/setup.bash
# 若为 OSX 系统，命令类似如下（路径需调整）：
# . ~/ros_catkin_ws/install_isolated/setup.bash
# 启动 ROS 1 的 talker 节点（来自 rospy_tutorials 功能包）：
rosrun rospy_tutorials talker
```

启动后，ROS 1 发布者节点会在终端中持续打印已发布的消息。


#### 步骤 4：启动 ROS 2 订阅者（listener）
```bash
# Shell D：
. <ROS 2 安装目录>/setup.bash
# 启动 ROS 2 的 listener 节点（来自 demo_nodes_cpp 功能包）：
ros2 run demo_nodes_cpp listener
```

启动后，ROS 2 订阅者节点会在终端中持续打印接收到的消息（来自 ROS 1 发布者，经桥接工具传递）。


#### 验证桥接状态
查看 Shell B（桥接工具终端）的输出，会出现类似如下的日志，表明已为 `/chatter` 话题建立桥接：
```
created 1to2 bridge for topic '/chatter' with ROS 1 type 'std_msgs/String' and ROS 2 type 'std_msgs/String'
```
*说明：`1to2` 表示桥接方向为“ROS 1 → ROS 2”。*


#### 停止桥接
按 `Ctrl-C` 停止所有程序。当停止发布者或订阅者时，Shell B 会输出类似如下的日志，表明桥接已拆除：
```
removed 1to2 bridge for topic '/chatter'
```


#### 示例截图
下图展示了所有终端窗口及其预期输出内容：
![ROS 1 发布者与 ROS 2 订阅者](doc/ros1_talker_ros2_listener.png)


### 示例 1b：ROS 2 发布者（talker） + ROS 1 订阅者（listener）
步骤与示例 1a 类似，仅需调整发布者和订阅者的启动命令，具体如下：

#### 步骤 1：启动 ROS 1 的 roscore
```bash
# Shell A：
. /opt/ros/melodic/setup.bash
# 若为 OSX 系统，命令类似如下（路径需调整）：
# . ~/ros_catkin_ws/install_isolated/setup.bash
roscore
```


#### 步骤 2：启动动态桥接工具
```bash
# Shell B：
. /opt/ros/melodic/setup.bash
# 若为 OSX 系统，命令类似如下（路径需调整）：
# . ~/ros_catkin_ws/install_isolated/setup.bash
. <包含桥接工具的 ROS 2 安装目录>/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge
```


#### 步骤 3：启动 ROS 2 发布者（talker）
```bash
# Shell C：
. <ROS 2 安装目录>/setup.bash
# 启动 ROS 2 的 talker 节点（来自 demo_nodes_py 功能包）：
ros2 run demo_nodes_py talker
```


#### 步骤 4：启动 ROS 1 订阅者（listener）
```bash
# Shell D：
. /opt/ros/melodic/setup.bash
# 若为 OSX 系统，命令类似如下（路径需调整）：
# . ~/ros_catkin_ws/install_isolated/setup.bash
# 启动 ROS 1 的 listener 节点（来自 roscpp_tutorials 功能包）：
rosrun roscpp_tutorials listener
```


## 示例 2：运行桥接工具实现图像交互
该示例将演示桥接工具传递更大、更复杂的消息（图像）：  
- 一个 ROS 2 节点发布从相机获取的图像；  
- 在 ROS 1 侧，使用 `rqt_image_view` 工具在图形界面（GUI）中渲染图像；  
- 同时，ROS 1 发布者可发送消息，切换 ROS 2 节点的图像翻转选项。


### 步骤 1：启动 roscore 和桥接工具
```bash
# Shell A：
. /opt/ros/melodic/setup.bash
# 若为 OSX 系统，命令类似如下（路径需调整）：
# . ~/ros_catkin_ws/install_isolated/setup.bash
roscore
```

```bash
# Shell B：
. /opt/ros/melodic/setup.bash
# 若为 OSX 系统，命令类似如下（路径需调整）：
# . ~/ros_catkin_ws/install_isolated/setup.bash
. <包含桥接工具的工作空间安装目录>/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge
```


### 步骤 2：启动 ROS 1 图像可视化工具
```bash
# Shell C：
. /opt/ros/melodic/setup.bash
# 若为 OSX 系统，命令类似如下（路径需调整）：
# . ~/ros_catkin_ws/install_isolated/setup.bash
# 启动 rqt_image_view 并监听 /image 话题：
rqt_image_view /image
```


### 步骤 3：启动 ROS 2 图像发布者
```bash
# Shell D：
. <包含 ROS 2 的工作空间安装目录>/setup.bash
# 启动 ROS 2 图像发布节点（来自 image_tools 功能包，模拟相机输出）：
ros2 run image_tools cam2image
```

此时，你应能在 `rqt_image_view` 的 GUI 中看到实时图像——这些图像来自 ROS 2 的 `cam2image` 节点，通过桥接工具传递到 ROS 1 侧。


### 步骤 4：测试反向桥接（ROS 1 → ROS 2）
为测试桥接工具的反向通信功能，可从 ROS 1 向 ROS 2 节点发布消息，控制图像是否翻转：  
通过向 `flip_image` 话题发布 `std_msgs/Bool` 类型的消息（`true` 或 `false`），ROS 2 的 `cam2image` 节点会根据消息内容决定是否翻转图像后再发布。

有两种方式发布消息：
1. 使用 `rqt` 的 `Message Publisher` 插件：在 GUI 中选择话题 `flip_image`、消息类型 `std_msgs/Bool`，并设置 `data` 字段为 `true` 或 `false`；  
2. 使用 `rostopic pub` 命令（示例如下）：

```bash
# Shell E：
. /opt/ros/melodic/setup.bash
# 若为 OSX 系统，命令类似如下（路径需调整）：
# . ~/ros_catkin_ws/install_isolated/setup.bash
# 方式 1：以 1Hz 频率发布“图像翻转”消息（data: true）
rostopic pub -r 1 /flip_image std_msgs/Bool "{data: true}"
# 方式 2：以 1Hz 频率发布“图像不翻转”消息（data: false）
rostopic pub -r 1 /flip_image std_msgs/Bool "{data: false}"
```


### 示例截图
下图展示了所有终端窗口及其预期输出内容（注：该截图拍摄于支持 ROS Indigo 版本的时期，当前应使用 Melodic 版本）：
![ROS 2 相机与 ROS 1 rqt](doc/ros2_camera_ros1_rqt.png)


## 示例 3：运行桥接工具实现 AddTwoInts 服务通信
该示例将桥接两个服务：
- ROS 1 侧：来自 [ros/roscpp_tutorials](https://github.com/ros/ros_tutorials) 功能包的 `TwoInts` 服务；  
- ROS 2 侧：来自 [ros2/roscpp_examples](https://github.com/ros2/examples) 功能包的 `AddTwoInts` 服务。


### 服务匹配原理
编译 `ros1_bridge` 时，工具会自动查找所有已安装的 ROS 1 和 ROS 2 服务，并通过以下规则匹配：  
1. 功能包名相同；  
2. 服务名相同；  
3. 请求（request）和响应（response）中的字段相同。  

若满足以上所有条件，桥接工具会自动为该服务建立桥接。此外，也可通过创建 YAML 文件手动指定服务配对关系，详情参考 [官方文档](doc/index.rst)。


### 前提准备
为确保示例正常运行，请确认：
1. 已安装 `roscpp_tutorials`（ROS 1 功能包）；  
2. 编译 `ros1_bridge` 时，已正确设置 `roscpp_tutorials` 的环境变量（通过 source 命令）。


### 步骤 1：启动 ROS 主节点（roscore）
```bash
# Shell A：
. <ROS 1 安装目录>/setup.bash
# 启动 roscore 并指定端口 11311：
roscore -p 11311
```


### 步骤 2：启动动态桥接工具
```bash
# Shell B：
. <ROS 1 安装目录>/setup.bash
. <ROS 2 安装目录>/setup.bash
# 设置 ROS_MASTER_URI 指向 roscore：
export ROS_MASTER_URI=http://localhost:11311
# 启动动态桥接工具：
ros2 run ros1_bridge dynamic_bridge
```


### 步骤 3：启动 ROS 1 服务端（TwoInts）
```bash
# Shell C：
. <ROS 1 安装目录>/setup.bash
export ROS_MASTER_URI=http://localhost:11311
# 启动 ROS 1 的 TwoInts 服务端（来自 roscpp_tutorials 功能包）：
rosrun roscpp_tutorials add_two_ints_server
```


### 步骤 4：启动 ROS 2 客户端（AddTwoInts）
```bash
# Shell D：
. <ROS 2 安装目录>/setup.bash
# 启动 ROS 2 的 AddTwoInts 客户端（来自 demo_nodes_cpp 功能包）：
ros2 run demo_nodes_cpp add_two_ints_client
```

启动后，ROS 2 客户端会向 ROS 1 服务端发送请求（经桥接工具传递），并在终端中打印服务响应结果。
