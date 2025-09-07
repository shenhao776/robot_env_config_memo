AMR2 Dockerfile (X86) Usage Guide
This guide provides instructions on how to build and run the Docker image for the AMR2 project on an x86 architecture.

1. Install Docker
First, ensure that Docker is installed on your Ubuntu system. Follow the official installation guide:

Install Docker Engine on Ubuntu

2. Create Dockerfile and requirements.txt
Create the following two files, Dockerfile and requirements.txt, in the same directory.

Dockerfile
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

# install Python packages
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# compile Livox-SDK2 and set up zsh
RUN git clone [https://github.com/Livox-SDK/Livox-SDK2.git](https://github.com/Livox-SDK/Livox-SDK2.git) && \
    cd Livox-SDK2 && \
    git checkout 6a940150d03248310038b55146c14115ac369e5d && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf Livox-SDK2 && \
    wget [https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh](https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh) && \
    sh install.sh && \
    rm install.sh && \
    git clone [https://github.com/zsh-users/zsh-autosuggestions](https://github.com/zsh-users/zsh-autosuggestions) ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    cd ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git checkout 85919cd559197824c88581634c0386121f618a81 && \
    cd && \
    git clone [https://github.com/zsh-users/zsh-syntax-highlighting.git](https://github.com/zsh-users/zsh-syntax-highlighting.git) ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
    cd ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
    git checkout 5eb677a281a8f0b02ab1f5a9f24c3de48a0a99ff && \
    cd

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

requirements.txt
paho-mqtt
pyserial-asyncio

3. Build the Docker Image
Navigate to the directory containing your Dockerfile and requirements.txt and run the build command. Replace image_name with your desired name for the image.

docker build -t image_name .

4. Run a Test Container
Use one of the following commands to run a container from the image you just built. These commands mount a local directory ~/shared_files to /root/shared_files inside the container for persistent storage and development.

With GPU Support
This command is for systems with an NVIDIA GPU.

docker run -it -v /tmp/.x11-unix:/tmp/.x11-unix \
 -v ~/shared_files:/root/shared_files \
 -v /dev:/dev --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
 -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE \
 --privileged --name "test"\
 --net=host --rm image_name zsh

Without GPU Support
Use this command for systems without a dedicated NVIDIA GPU.

docker run -it -v /tmp/.x11-unix:/tmp/.x11-unix \
 -v ~/shared_files:/root/shared_files \
 -v /dev:/dev \
 -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE \
 --privileged --name "test"\
 --net=host --rm image_name zsh

5. How to Run the Application
Once the container is running, you can start the ROS2 application. You will need to open multiple terminals to run all the necessary components. First, start the container if it's not already running.

sudo docker start ros2_ws_x86

Terminal 1: Bringup
Open a new terminal and attach to the running container to launch the robot bringup.

sudo docker exec -it ros2_ws_x86 zsh
ros2 launch indoor_robot robot_bringup_launch.py

Terminal 2: Navigation
Open a second terminal and attach to the container to launch the navigation stack.

sudo docker exec -it ros2_ws_x86 zsh
ros2 launch indoor_robot navigation2.launch.py params_file:=/root/shared_files/ros2_ws/src/indoor_robot/config/nav2_params_3d.yaml use_sim_time:=False

Terminal 3: UDP Receiver
Open a third terminal and attach to the container to run the UDP receiver node.

sudo docker exec -it ros2_ws_x86 zsh
ros2 run indoor_robot udp_receiver_node
