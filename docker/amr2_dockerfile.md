# AMR2 Dockerfile (X86) Usage Guide 🤖

This guide provides instructions on how to set up and run the AMR2 ROS2 environment using Docker on an x86 machine.

-----

## 1\. Install Docker

First, you need to install Docker on your Ubuntu system. Follow the official installation guide.

  * **Installation Link**: [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)

-----

## 2\. Create Configuration Files

In the same directory, create two files: `Dockerfile` and `requirements.txt`.

### `Dockerfile`

```dockerfile
FROM osrf/ros:humble-desktop-full

# set zone to Asia/Tokyo
ENV TZ=Asia/Tokyo
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# apt packages
RUN apt-get update && apt-get install -y \
      python3-pip \
      python3-serial \
      ros-humble-tf-transformations \
      vim \
      iputils-ping \
      net-tools \
      zsh \
      wget \
      libpcap-dev \
      ros-humble-pointcloud-to-laserscan \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      'ros-humble-turtlebot3*' \
      'ros-humble-librealsense2*' \
      'ros-humble-realsense2-*' \
      ros-humble-image-view \
      && rm -rf /var/lib/apt/lists/*

# install Python
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# compile Livox-SDK2 and setup zsh
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
      cd Livox-SDK2 && \
      mkdir build && \
      cd build && \
      cmake .. && \
      make -j$(nproc) && \
      sudo make install && \
      cd ../.. && \
      rm -rf Livox-SDK2 && \
      wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh && \
      sh install.sh && \
      rm install.sh && \
      git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
      git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting

# Comment out the conflicting line in rosidl-argcomplete.zsh
RUN sed -i 's/^autoload -U +X compinit && compinit/# &/' /opt/ros/humble/share/rosidl_cli/environment/rosidl-argcomplete.zsh

# Modify .zshrc plugins line
RUN sed -i 's/^plugins=(git)$/plugins=(git zsh-syntax-highlighting zsh-autosuggestions)/' ~/.zshrc

# workspace
WORKDIR /root/shared_files

# add LD_LIBRARY_PATH to .zshrc
RUN echo '# added' >> ~/.zshrc && echo '# livox lidar sdk' >> ~/.zshrc && \
      echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib' >> ~/.zshrc && \
      echo 'source /opt/ros/humble/setup.zsh' >> ~/.zshrc && \
      echo 'source ~/shared_files/thirdparty_ws/install/setup.zsh' >> ~/.zshrc && \
      echo 'source ~/shared_files/ros2_ws/install/setup.zsh' >> ~/.zshrc && \
      echo '# TURTLEBOT3_MODEL' >> ~/.zshrc && \
      echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.zshrc

# entrance point
CMD ["zsh"]
```

### `requirements.txt`

```txt
paho-mqtt
pyserial-asyncio
```

-----

## 3\. Build the Docker Image

Navigate to the directory containing your files and run the build command. Replace `image_name` with your desired name for the image.

```zsh
docker build -t image_name .
```

-----

## 4\. Run the Container for Testing

You can run the container to test it. There are different commands depending on whether you have a GPU.

#### With GPU Support

This command maps your GPU, display, and shared files into the container.

```zsh
docker run -it -v /tmp/.x11-unix:/tmp/.x11-unix \
 -v ~/shared_files:/root/shared_files \
 -v /dev:/dev --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
 -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE \
 --privileged --name "test"\
 --net=host --rm image_name zsh
```

#### Without GPU Support

This command is for systems without a dedicated NVIDIA GPU.

```zsh
docker run -it -v /tmp/.x11-unix:/tmp/.x11-unix \
 -v ~/shared_files:/root/shared_files \
 -v /dev:/dev \
 -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE \
 --privileged --name "test"\
 --net=host --rm image_name zsh
```

-----

## 5\. How to Run the ROS2 Workspace

First, start your container if it's not already running (replace `ros2_ws_x86` if you named it differently).

```zsh
sudo docker start ros2_ws_x86
```

Then, open three separate terminals to launch the different parts of the robot's software stack.

#### Terminal 1: Robot Bringup

```zsh
sudo docker exec -it ros2_ws_x86 zsh
ros2 launch indoor_robot robot_bringup_launch.py
```

#### Terminal 2: Navigation

```zsh
sudo docker exec -it ros2_ws_x86 zsh
ros2 launch indoor_robot navigation2.launch.py params_file:=/root/shared_files/ros2_ws/src/indoor_robot/config/nav2_params_3d.yaml use_sim_time:=False
```

#### Terminal 3: UDP Receiver

```zsh
sudo docker exec -it ros2_ws_x86 zsh
ros2 run indoor_robot udp_receiver_node
```
