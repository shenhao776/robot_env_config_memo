# Ubuntu 22.04 System General Configuration Memo 📝

[中文版点这里](./README_CN.md)

-----
This memo provides a comprehensive guide for setting up an Ubuntu 22.04 system on the specified hardware, including shell customization, essential tools, NVIDIA driver installation, and Docker configuration for GPU support.

  * **Device**: ROG Flow Z13 13.4" (RTX 4060, Intel i9-13900H, 16GB RAM, 1TB SSD, GZ301VV-I9R4060)
  * For a non-GPU setup, see: [Ubuntu 22.04 system general configuration memo (no gpu)](./robot_config_without_gpu.md)

-----

## **ZSH**

This section covers the installation and configuration of Zsh with Oh My Zsh and useful plugins.

```bash
sudo apt update
sudo apt install zsh git

# Install Oh My Zsh
wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh
sh install.sh
rm install.sh

# Install some plugins: autosuggestions, highlighting
git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting

# Revise .zshrc to add plugins
sed -i 's/^plugins=(git)$/plugins=(git zsh-syntax-highlighting zsh-autosuggestions)/' ~/.zshrc

# If ROS2 was installed, revise rosidl-argcomplete.zsh to prevent conflicts
sed -i 's/^autoload -U +X compinit && compinit/# &/' /opt/ros/humble/share/rosidl_cli/environment/rosidl-argcomplete.zsh

source ~/.zshrc
```

-----

## **Tools** 🛠️

Install essential command-line utilities.

```bash
sudo apt update
sudo apt install -y openssh-server vim net-tools
```

-----

## **Ignore Lid Closure** 💻

Configure the system to not suspend when the laptop lid is closed.

1.  Open the configuration file with Vim:
    ```bash
    sudo vim /etc/systemd/logind.conf
    ```
2.  Press `i` to enter insert mode.
3.  Add the following lines to the end of the file:
    ```
    HandleLidSwitch=ignore
    HandleLidSwitchExternalPower=ignore
    HandleLidSwitchDocked=ignore
    ```
4.  Press `Esc`, then type `:wq` and press `Enter` to save and quit.
5.  Reboot the system for the changes to take effect.
    ```bash
    sudo reboot
    ```

-----

## **CUDA** 🚀

This section details the installation of the NVIDIA driver and CUDA Toolkit 12.9 on Ubuntu 22.04.

  * **Original Link**: [CUDA Toolkit 12.9.0 Download](https://developer.nvidia.com/cuda-12-9-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_local)
  * **Note**: It's recommended to perform this on a fresh OS install where proprietary drivers were **not** installed during the Ubuntu setup process.

### 1\. CUDA Toolkit Installer

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.9.0/local_installers/cuda-repo-ubuntu2204-12-9-local_12.9.0-575.51.03-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2204-12-9-local_12.9.0-575.51.03-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2204-12-9-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-9
```

### 2\. Driver Installer

```bash
# Install proprietary kernel modules
sudo apt-get install -y cuda-drivers
```

### 3\. Add to Environment Variables

Add the following lines to your `~/.zshrc` or `~/.bashrc` file.

```bash
export PATH=/usr/local/cuda-12.9/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-12.9/lib64\
                      ${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```

### 4\. Reboot and Verify

Reboot your system. In your BIOS, ensure the display mode is set to use the dedicated NVIDIA GPU.

```bash
# Check the OpenGL renderer
# The result should contain: "NVIDIA GeForce RTX 4060 Laptop GPU"
glxinfo | grep "OpenGL renderer"

# Check the driver status with nvidia-smi
nvidia-smi
```

The output of `nvidia-smi` should look similar to this, showing the driver and CUDA versions, and GPU status. If `GPU-Util` is always 0%, the GPU is not being used for computation.

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

Follow the official documentation to install Docker Engine on Ubuntu.

  * **Installation Guide**: [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)

-----

## **Run GPU in Docker**

Configure Docker to use the NVIDIA GPU within containers.

```bash
# Install Docker Compose (optional but recommended)
sudo curl -L "https://github.com/docker/compose/releases/download/v2.35.0/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# Add NVIDIA Container Toolkit repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install NVIDIA Container Toolkit
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Configure Docker to use the NVIDIA runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# --- Verification ---
# 1. Run nvidia-smi in a container
sudo docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu24.04 nvidia-smi

# 2. PyTorch CUDA verification
sudo docker run --gpus all -it pytorch/pytorch:latest python -c "import torch; print('CUDA available:', torch.cuda.is_available()); print('CUDA device count:', torch.cuda.device_count())"
```

### Docker Run Sample with GUI Support

This is an example command to run a container with GPU acceleration and GUI forwarding.

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

## **AMR2 Dockerfile Usage**

For instructions on using the specific AMR2 Dockerfile for x86 systems, refer to the separate guide.

  * **Link**: [AMR2 Dockerfile (X86) Usage Guide](./docker/amr2_dockerfile.md)
