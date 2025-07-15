# Ubuntu 22.04 系统通用配置备忘录

## 设备信息
- **设备**：ROG Flow Z13 13.4英寸 RTX 4060 英特尔第13代i9-13900H 16GB 1TB GZ301VV-I9R4060


## ZSH 配置
```bash
sudo apt update
sudo apt install zsh
wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh
sh install.sh
rm install.sh

# 安装插件：自动建议、语法高亮
git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting

# 在 ~/.zshrc 中添加插件配置
# plugins=(git zsh-syntax-highlighting zsh-autosuggestions)

source ~/.zshrc
```


## 工具安装
```bash
sudo apt update
sudo apt install -y openssh-server vim net-tools
```


## 忽略合盖动作
```bash
sudo vim /etc/systemd/logind.conf

# 修改或添加以下配置
HandleLidSwitch=ignore
HandleLidSwitchExternalPower=ignore
HandleLidSwitchDocked=ignore 

# 重启生效
reboot
```


## CUDA 配置
**系统**：Ubuntu 22.04  
> *注：系统安装时未安装驱动，若已安装可能会出现错误（不确定）*

### 原始链接
[CUDA 12.9.0 下载存档](https://developer.nvidia.com/cuda-12-9-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_local)

#### 1. 安装 CUDA 工具包
```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600

wget https://developer.download.nvidia.com/compute/cuda/12.9.0/local_installers/cuda-repo-ubuntu2204-12-9-local_12.9.0-575.51.03-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2204-12-9-local_12.9.0-575.51.03-1_amd64.deb

sudo cp /var/cuda-repo-ubuntu2204-12-9-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-9
```

#### 2. 安装驱动
```bash
# 专有内核驱动
sudo apt-get install -y cuda-drivers
```
> 切换 NVIDIA 驱动内核模块版本参考：  
> [https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/#switching-between-driver-module-flavors](https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/#switching-between-driver-module-flavors)

#### 3. 配置环境变量（添加到 .zshrc 或 .bashrc）
```bash
export PATH=/usr/local/cuda-12.9/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-12.9/lib64\
                         ${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```

#### 4. 重启系统
> *注：在 BIOS 中，将显示模式设置为 Nvidia GPU*

#### 验证配置
```bash
# 检查 OpenGL 渲染器，结果应类似：OpenGL renderer string: NVIDIA GeForce RTX 4060 Laptop GPU/PCIe/SSE
glxinfo | grep "OpenGL renderer"

# 检查 nvidia-smi 输出，示例如下
nvidia-smi
```
> GPU-Util 表示 GPU 利用率。*如果始终为 0%，说明 GPU 未用于计算*

**nvidia-smi 输出示例**：
```txt
Tue Jul 15 20:38:47 2025       
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 575.51.03              Driver Version: 575.51.03      CUDA Version: 12.9     |
|-----------------------------------------+------------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
|                                         |                        |               MIG M. |
|=========================================+========================+======================|
|   0  NVIDIA GeForce RTX 4060 ...    Off |   00000000:01:00.0 Off |                  N/A |
| N/A   53C    P4              7W /   35W |    1284MiB /   8188MiB |     10%      Default |
|                                         |                        |                  N/A |
+-----------------------------------------+------------------------+----------------------+
                                                                                          
+-----------------------------------------------------------------------------------------+
| Processes:                                                                              |
|  GPU   GI   CI              PID   Type   Process name                        GPU Memory |
|        ID   ID                                                               Usage      |
|=========================================================================================|
|    0   N/A  N/A            1305      G   /usr/lib/xorg/Xorg                      448MiB |
|    0   N/A  N/A            1671      G   /usr/bin/gnome-shell                    254MiB |
|    0   N/A  N/A            2488      G   gnome-control-center                      2MiB |
|    0   N/A  N/A            2687      G   ...ersion=20250714-180043.191000        359MiB |
|    0   N/A  N/A            3483      G   /usr/share/code/code                    138MiB |
+-----------------------------------------------------------------------------------------+
```


## Docker 配置
### 安装参考链接
[Docker 官方 Ubuntu 安装指南](https://docs.docker.com/engine/install/ubuntu/)

### 在 Docker 中使用 GPU
```bash
# 安装 Docker Compose（可选但推荐）
sudo curl -L "https://github.com/docker/compose/releases/download/v2.35.0/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# 添加 NVIDIA Container Toolkit 源
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 安装 NVIDIA Container Toolkit
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# 配置 Docker
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 验证配置
sudo docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu24.04 nvidia-smi

# PyTorch 验证
sudo docker run --gpus all -it pytorch/pytorch:latest python -c "import torch; print('CUDA available:', torch.cuda.is_available()); print('CUDA device count:', torch.cuda.device_count())"
```

### 在 Docker 中使用图形界面（示例命令）
```bash
docker run -it -v /tmp/.x11-unix:/tmp/.x11-unix \
                -v ~/shared_files:/root/shared_files \
                -v /dev:/dev \
                --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
                -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE \
                --privileged --name "容器名称"\
                --net=host 镜像ID bash
```
