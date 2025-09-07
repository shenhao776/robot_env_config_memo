# Ubuntu 22.04 System General Configuration Memo (No GPU) 📝

This guide provides setup instructions for an Ubuntu 22.04 system without a focus on GPU-specific configurations.

  * **Device**: ROG Flow Z13 13.4" (RTX 4060, Intel i9-13900H, 16GB RAM, 1TB SSD, GZ301VV-I9R4060)

-----

## **ZSH** 셸

This section covers the installation and configuration of the Zsh shell with Oh My Zsh and useful plugins.

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

Install some essential command-line utilities.

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

## **Install Docker** 🐳

Follow the official documentation to install Docker Engine on Ubuntu.

  * **Official Link**: [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)

-----

## **AMR2 Dockerfile Usage**

For instructions on using the specific AMR2 Dockerfile for x86 systems, refer to the separate guide.

  * **Reference Link**: [AMR2 Dockerfile (X86) Usage Guide](https://github.com/shenhao776/memo/blob/main/amr2_dockerfile.md)
