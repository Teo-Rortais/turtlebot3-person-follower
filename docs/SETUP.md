# Setting up the project 

To set up the project you need to flash the Jetson with JetPack SDK Manager, build and initialize the TurtbleBot3 and install ROS on both devices. Here is a list of all the hardware, and software I used :

### Equipement I used 

- Jetson Orin Nano \[8GB developer kit version\]
- TurtleBot 3 Burger Kit
- USB Camera (C920 HD Pro Webcam from logitech)
- Computer with the following softwares :
	- NVIDIA SDK Manager
	- Raspberry PI Imager
	- SD Card Formatter
- 2 Micro SD Cards
- Keyboard/Mouse/Screen
- Display Port cable
- Micro HDMI cable
- USB-A to USB-C cable
- Jumper Wire


## Flash Jetson with JetPack

First of all you need to format a micro SD card (you can use SD Card Formatter software) and insert it in the Jetson Orin Nano. Connect the port FC REC and GND with a jumper wire before turning the power on. If you need more guidance [here](images/jetson_recovery_mode.jpg) is an image that might help you.

Plug the USB-A to USB-C cable on the Jetson and a computer with the NVIDIA SDK Manager software. Launch the SDK Manager and connect. Since your Jetson is in recovery mode, you should be able to select it in the "Target Hardware" section. Choose "JetPack 6.0 (rev. 2)" in the "SDK Version" section and continue to step 2 (no need to download DeepStream).

![[sdk_manager_step_1.png]]

Accept the terms and conditions of the license agreements and continue until step 3.

![[sdk_manager_step_2.png]]

When the card is ready to be flashed, choose Pre-Config for the OEM Configuration and define a username and password. Then hit the flash button. 

At one point you will have a window asking you to enter an IP address, username and password. You will have to connect to your Jetson and configure the system. When it's ready, connect the Jetson to the network and open a terminal (CTRL+ALT+T). Enter this command :

``` bash
ifconfig
```

Find the IP address of the Jetson and come back to the SDK Manager. Fill out the informations needed and clic on the "Install" button (no need to set proxy). 

![[sdk_manager_step_3.png]]

When the installation is finished you can go back to your Jetson and enter this command in a terminal :

``` bash
sudo apt install nvidia-jetpack
```


## Build Jetson Inference Project

I strongly advise you install a web browser so you can copy and paste the command instead of manually enter them. You can install Firefox with this command :

``` bash
sudo snap install firefox
```

To use AI  we gonna build the Jetson Inference Project, you can follow the [guide]( https://github.com/dusty-nv/jetson-inference/blob/master/docs/building-repo-2.md) on the GitHub repository or just execute the commands bellow.

``` bash
sudo apt-get update
sudo apt-get install git cmake libpython3-dev python3-numpy
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig
```

To check if the AI works, you can run the object detection live camera demo. Just connect the USB camera to the Jetson and enter this command :

``` bash
detectnet.py /dev/video0
```

The first time you use this program is going to take time. The next use of it are going to be faster.


## Set up TurtleBot and install ROS Humble 

Remember that when building the TurtleBot, you can skip the part where you have to fix teh LDS-02, since it's our USB camera that is gonna take this spot.

### ROS on Jetson Orin Nano

You can follow the [guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) on the official website (remember to choose Humble at the top of the webpage) or you can just enter the following commands :

``` bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

``` bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

``` bash
sudo apt update
sudo apt upgrade
```

``` bash
sudo apt install ros-humble-desktop
```

``` bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> ~/.bashrc
source  ~/.bashrc
```

``` bash
sudo apt install ros-humble-dynamixel-sdk
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-turtlebot3
```


### Raspberry PI Setup

You can follow the [guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup) on the official website (remember to choose Humble at the top of the webpage) or you can just do the followings :

Insert a  formatted micro SD card in the computer with Raspberry PI Imager installed. Open the software and choose Raspberry PI 4 (Model B) for the model of the Raspberry PI. In "Operating System" choose : Other general purpose OS -> Ubuntu -> Ubuntu Server 22.04.4 LTS (64-bit). Finally choose your micro SD card in the "Storage" section and clic on "Write".

When your card is flashed, insert it in your TurtleBot's Raspberry. You can plug the micro HDMI cable on the HDMI 0 port and turn the power of the robot on. Connect on the ubuntu with the ID and password both being "*ubuntu*". Choose a new password and modify some files to connect to the wifi.

``` bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

The file should ressemble this :

``` yaml
network
	ethernets:
		eth0:
			dhcp4: yes
			optional: yes
	version: 2
	wifis:
		wlan0:
			dhcp4: yes
			dhcp6: yes
			access-points:
				YOUR_WIFI_SSID:
					password: YOUR_WIFI_PASSWORD
	
```

Save the file, then change the next file like so :

``` bash
sudo nano /etc/apt/apt.conf.d/20auto-upgrades
```

``` yaml
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
```

Enter the following commands to finalize the setup :

``` bash
systemctl mask systemd-networkd-wait-online.service
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
reboot
```

Find the IP address of your device :

``` bash
ip address
```

Then you can continue the guide from your Jetson by connecting to the robot with ssh. This way you will be able to copy paste the rest of the commands :

``` bash
ssh ubuntu@{IP Address of Raspberry PI}
```

Now we need to install ROS Humble on the TurtleBot also, so we gonna follow the same step as when we did for the Jetson :

``` bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

``` bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

``` bash
sudo apt update
sudo apt upgrade
```

``` bash
sudo apt install ros-humble-desktop
```

``` bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> ~/.bashrc
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
source  ~/.bashrc
```

Once the ROS set up is complete, we need to install and build some ROS packages on the TurtleBot :

``` bash
sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
sudo apt install ros-humble-hls-lfcd-lds-driver
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-dynamixel-sdk
sudo apt install libudev-dev
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
cd ~/turtlebot3_ws/src/turtlebot3
rm -r turtlebot3_cartographer turtlebot3_navigation2
cd ~/turtlebot3_ws/
colcon build --symlink-install --parallel-workers 1
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

``` bash
sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

There is some additional packages we will need for the project :

``` bash
sudo apt-get install ros-humble-v4l2-camera
sudo apt-get install ros-humble-image-transport-plugins
```


## OpenCR Setup

You can follow the [guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup) on the official website (remember to choose Humble at the top of the webpage) or you can just do the followings :

``` bash
sudo dpkg --add-architecture armhf
sudo apt update
sudo apt install libc6:armhf
echo 'export OPENCR_PORT=/dev/ttyACM0' >> ~/.bashrc
echo 'export OPENCR_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
rm -rf ./opencr_update.tar.bz2
```

``` bash
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xvf ./opencr_update.tar.bz2
cd ~/opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```


## Clone and build the person follower project

``` bash 
sudo apt install python3-colcon-common-extensions
git clone https://github.com/Teo-Rortais/turtlebot3-person-follower.git
cd turtlebot3-person-follower/ros2_ws
source install/setup.bash
colcon build
```


Now that the set up of the project is done you can go back to the main section to use the nodes you just installed.




  