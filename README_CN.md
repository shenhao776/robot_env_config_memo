# Ubuntu 22.04 系统通用配置备忘录 📝

本备忘录为在特定硬件上设置 Ubuntu 22.04 系统提供了全面的指南，内容包括 Shell 自定义、基本工具安装、NVIDIA 驱动程序安装以及针对 GPU 支持的 Docker 配置。

  * **设备**: ROG Flow Z13 13.4英寸 (RTX 4060, Intel i9-13900H, 16GB RAM, 1TB SSD, GZ301VV-I9R4060)
  * 无 GPU 版本的配置，请参阅: [Ubuntu 22.04 系统通用配置备忘录 (无GPU版本)](./robot_config_without_gpu.md)

-----

## **ZSH**

本节介绍如何安装和配置 Zsh、Oh My Zsh 以及一些实用的插件。

```bash
sudo apt update
sudo apt install zsh git
ttps://www.google.com/search?q=
# 安装 Oh My Zsh
wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh
sh install.sh
rm install.sh

# 安装插件：命令自动建议、语法高亮
git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting

# 修改 .zshrc 以启用插件
sed -i 's/^plugins=(git)$/plugins=(git zsh-syntax-highlighting zsh-autosuggestions)/' ~/.zshrc

# 如果已安装 ROS2，修改 rosidl-argcomplete.zsh 以避免冲突
sed -i 's/^autoload -U +X compinit && compinit/# &/' /opt/ros/humble/share/rosidl_cli/environment/rosidl-argcomplete.zsh

source ~/.zshrc
```

-----

## **工具** 🛠️

安装一些必要的命令行工具。

```bash
sudo apt update
sudo apt install -y openssh-server vim net-tools
```

-----

## **忽略笔记本盒盖** 💻

配置系统在笔记本电脑盒盖时电脑不进入休眠模式。

1.  使用 Vim 打开配置文件：
    ```bash
    sudo vim /etc/systemd/logind.conf
    ```
2.  按 `i` 键进入插入模式。
3.  在文件末尾添加以下三行：
    ```
    HandleLidSwitch=ignore
    HandleLidSwitchExternalPower=ignore
    HandleLidSwitchDocked=ignore
    ```
4.  按 `Esc` 键，然后输入 `:wq` 并按 `Enter` 保存并退出。
5.  重启系统使更改生效。
    ```bash
    sudo reboot
    ```

-----

## **CUDA** 🚀

本节详细介绍在 Ubuntu 22.04 上安装 NVIDIA 驱动和 CUDA Toolkit 12.9 的过程。

  * **原始链接**: [CUDA Toolkit 12.9.0 下载页面](https://developer.nvidia.com/cuda-12-9-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_local)
  * **注意**: 建议在全新的操作系统上执行此操作，并且在安装 Ubuntu 系统时**不要**安装专有驱动程序。

### 1\. CUDA Toolkit 安装程序

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.9.0/local_installers/cuda-repo-ubuntu2204-12-9-local_12.9.0-575.51.03-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2204-12-9-local_12.9.0-575.51.03-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2204-12-9-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-9
```

### 2\. 驱动程序安装

```bash
# 安装专有内核模块
sudo apt-get install -y cuda-drivers
```

### 3\. 添加到环境变量

将以下内容添加到您的 `~/.zshrc` 或 `~/.bashrc` 文件中。

```bash
export PATH=/usr/local/cuda-12.9/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-12.9/lib64\
                      ${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```

### 4\. 重启并验证

重启您的系统。在 BIOS 中，确保显示模式设置为使用 NVIDIA 独立显卡。

```bash
# 检查 OpenGL 渲染器
# 结果应包含 "NVIDIA GeForce RTX 4060 Laptop GPU"
glxinfo | grep "OpenGL renderer"

# 使用 nvidia-smi 检查驱动状态
nvidia-smi
```

`nvidia-smi` 的输出应与下图类似，显示驱动程序和 CUDA 版本以及 GPU 状态。如果 `GPU-Util` 一直为 0%，则表示 GPU 未被用于计算。

```txt
Tue Jul 15 20:38:47 2025      
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 575.51.03            Driver Version: 575.51.03    CUDA Version: 12.9     |
|-----------------------------------------+------------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
|                                         |                        |               MIG M. |
|=========================================+========================+======================|
|   0  NVIDIA GeForce RTX 4060 ...    Off |   00000000:01:00.0 Off |                  N/A |
| N/A   53C    P4               7W /   35W |   1284MiB /   8188MiB |     10%      Default |
|                                         |                        |                  N/A |
+-----------------------------------------+------------------------+----------------------+
                                                                                         
+-----------------------------------------------------------------------------------------+
| Processes:                                                                              |
|  GPU   GI   CI        PID   Type   Process name                              GPU Memory |
|        ID   ID                                                              Usage      |
|=========================================================================================|
|    0   N/A  N/A      1305      G   /usr/lib/xorg/Xorg                            448MiB |
|    0   N/A  N/A      1671      G   /usr/bin/gnome-shell                          254MiB |
|    0   N/A  N/A      2488      G   gnome-control-center                            2MiB |
|    0   N/A  N/A      2687      G   ...ersion=20250714-180043.191000            359MiB |
|    0   N/A  N/A      3483      G   /usr/share/code/code                          138MiB |
+-----------------------------------------------------------------------------------------+
```

-----

## **Docker** 🐳

请遵循官方文档在 Ubuntu 上安装 Docker Engine。

  * **安装指南**: [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)

-----

## **在 Docker 中使用 GPU**

配置 Docker 以便在容器中使用 NVIDIA GPU。

```bash
# 安装 Docker Compose (可选，但推荐)
sudo curl -L "https://github.com/docker/compose/releases/download/v2.35.0/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# 添加 NVIDIA Container Toolkit 仓库
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 安装 NVIDIA Container Toolkit
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# 配置 Docker 使用 NVIDIA runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# --- 验证 ---
# 1. 在容器中运行 nvidia-smi
sudo docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu24.04 nvidia-smi

# 2. PyTorch CUDA 验证
sudo docker run --gpus all -it pytorch/pytorch:latest python -c "import torch; print('CUDA available:', torch.cuda.is_available()); print('CUDA device count:', torch.cuda.device_count())"
```

### 支持 GUI 的 Docker 运行示例

这是一个运行具有 GPU 加速和图形界面转发功能的容器的示例命令。

```bash
xhost +
docker run -it -v /tmp/.x11-unix:/tmp/.x11-unix \
               -v ~/shared_files:/root/shared_files \
               -v /dev:/dev --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all\
               -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE \
               --privileged --name "test"\
               --net=host --rm image_name zsh
```

-----

## **AMR2 Dockerfile 使用说明**

关于如何使用 AMR2 的 x86 Dockerfile，请参考独立的指南文档。

  * **链接**: [AMR2 Dockerfile (X86) 使用指南](./amr2_dockerfile.md)
