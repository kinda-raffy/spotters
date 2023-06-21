# **Spotters 🐶 📷**

<!-- TODO: Make links to different sections. -->

The Spotters project aims to provide a comprehensive system for fully autonomous
navigation on the Boston Dynamics Spot platform.

## **Requirements**

This section outlines the prerequisites for installing and executing the Spotters project. These requirements should be met, and the requisite steps completed, before proceeding with installation.

### **Hardware**

Spotters has been designed for universal access and applicability across robotic platforms and camera technologies.

- A Boston Dynamics Spot robot. This is not technically required for the VSLAM section of the project, since this can be executed using just an iPhone camera for now, but the navigation stack is specifically tailored to the Spot platform.
- We require a monocular pinhole camera as the primary video input for our SLAM algorithm. Although our project will eventually support different platforms, for now an iOS 16 device should be used as the camera.
- An `x86-64` processor. Spotters may be able to run on `ARM64` processors, but we cannot make any guarantees.

We have partially completed integration of corresponding IMU (Internal Measurement Unit) data, so this capability remains on the horizon. This integration would substantially improve our mapping capabilities, but would require a camera device with accessible IMU data. All iPhones should qualify for use in this respect. This project was designed for relative computational efficiency when compared against existing navigation libraries; we have successfully demonstrated basic autonomous navigation from a simple laptop CPU. That being said, GPU integration remains a future goal of ours, among numerous others detailed [elsewhere](#extension).

### **Cameras**

Currently, the Spotters project is heavily decoupled across input, computation and action modules. As a result, additional input devices may be easily added with little modifications to the codebase.

#### **Base Requirements**

The chosen camera device must be able to stream clear images of resolution 640 by 480 pixels for reliable feature detection. Given that, once the IMU data has been integrated, SLAM utilises the distance traversed between frames to determine the relative position of features, a frame rate of around 30 frames per second is desirable; our SLAM algorithm performs noticeably worse at lower frame rates.

As the video stream is being used for localisation, the stream should have low latency to maintain realistic robot positioning.

#### **Spot SensorRelay**

The Spot SensorRelay is a custom made iOS application that ensures proper satisfaction of the given base requirements. The sensor relay may be launched outside of the Spot robotic environment, which detaches algorithm development from the robot. In additon, the stream may be readily changed to a wide angled lens, to allow for more thorough localisation across fast-paced movements.

The sensor relay contains IMU drivers developed in-house to enable the use of mobile MEMS Gyroscopes and mobile accelerometers with the ROS ecosystem. Unfortunately, IMU calibration requires thorough calibration using the Kalibr toolset, which is something the group ran out of time for in the last few days.

#### **Onboard cameras**

Due to hardware limitations, Spot's onboard cameras fail to meet the project's base camera requirements and may not be used for localisation.

#### **Spot Cam**

In cases where a Spot Cam is preferred over the SensorRelay app, slight modifications need to be made to exsiting launch files. This includes remapping the camera input topic in `spot_slam/launch/slam.launch` from `usb_cam/image_raw` to the correct spot cam topic. In addition, `spot_compose/launch/up.launch` should be modified to omit the launch of the `usb_cam` node. Steps relevant to Spot Cam and SensorRelay have been annotated during installing.

### **Software**

This section deals exclusively with the software requirements of the Spotters project, and their configuration.

#### **Ubuntu**

Spotters requires an installation of [Ubuntu 20.04](https://releases.ubuntu.com/focal/). We recommend a minimal installation, but this is not required. Once the operating system has been installed, execute the following commands to install the required software.

```bash
$ sudo apt update
$ sudo apt upgrade
$ sudo apt install python3-pip git wget
$ sudo pip3 install virtualenv
```

This project is **incompatible** with LibreOffice and Rhythmbox, so this software should be removed before continuing with installation.

```bash
$ sudo apt purge libreoffice*
$ sudo apt-get purge rhythmbox rhythmbox-plugins
```

#### **ROS and Build Tools**

The project has been tested on a full ROS Noetic installation, which needs to be acquired by either following the [website installation instructions](http://wiki.ros.org/noetic/Installation/Ubuntu) or using the commands outlined below.
```bash
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh
chmod +x ./ros_install_noetic.sh
./ros_install_noetic.sh
source ~/.bashrc
```

Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).

Catkin tools is used as Spotter's build system. Information on how to use this build system is available here: https://catkin-tools.readthedocs.io/en/latest/

## **Installation**

This section details the project installation and initialisation. Known errors and their solutions are outlined in the [installation issues](#installation-issues) section below. These steps outline every command required for a full installation, including navigating into relevant directories; we recommend executing them contiguously, inside a single terminal session. We assume you are using the `bash` shell, since this is the default in Ubuntu. First, clone the `spotters` repository into your home directory.

```bash
$ cd ~
$ git clone https://github.com/kinda-raffy/spotters
```

Next, create a virtual environment to house all of the project's dependencies. This virtual environment is required because Ubuntu uses the base Python installation internally, and overwriting its dependencies will corrupt the system.

```bash
$ cd spotters
$ python3 -m virtualenv --python=/usr/bin/python3 spot_venv
```

The following commands ensure the project's custom setup script is sourced automatically by every new terminal session. Once the first command has been executed successfully, either run the second or restart your terminal session. If you would prefer to skip this step entirely, you can source the script manually as required with the `$ source ~/spotters/.setup.bash` command.

```bash
$ echo "source ~/spotters/.setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Next, activate the Python virtual environment you created. This is not technically required yet, only later once we start installing dependencies, but it's generally good practice to work from the active virtual environment in this context, so consider this step mandatory. If this command does not work for any reason, try the alternative provided in the relevant [issues subsection](#failed-to-build-slam) below.

```bash
$ spot_env
```

Here we populate the submodules of the base `spotters` repository. We have used submodules to allow for continuously integrating updated versions of these repositories, so this project remains compatible with the most recent Boston Dynamics Spot SDK. For more information, see the appropriate [context section](#submodules). The `wspt` command is defined in our custom `.setup.bash` script: it navigates to the `spotters/workspace` directory. The `spot_ros` submodule houses the nested `spot_wrapper` submodule, which is populated by the final three commands.

```bash
$ git submodule init
$ git submodule update
$ wspt
$ cd src/spot_ros
$ git submodule init
$ git submodule update
```

Next, install the populated submodules as ROS dependencies. This makes them available as packages when developing, building, and running the project.

```bash
$ rosdep install --from-paths spot_driver spot_msgs spot_viz spot_description spot_cam --ignore-src -y
```

Now we install the `spot_wrapper` so it can be used as as an external Python library. The `-e` argument installs the wrapper from the local repository instead of a remote Python package index.

```bash
$ pip install -e spot_wrapper
```

Now we need to install the project requirements into the new virtual environment and onto our system. In the event of errors referencing problematic packages, refer to the [relevant error section](#python-packages-not-found). These are minor problems in our experience, but they are quite common and the solution is not obvious.

```bash
$ wspt
$ cd ..
$ pip install -r requirements.txt
```

The following dependency is a template library written in C++ that provides access to functions for linear algebra. These are utilised heavily in drawing the robot's location on the generated map using series of points.

```bash
$ sudo apt install libeigen3-dev
```

Pangolin allows us to visualise point cloud data alongside the tracked location of the robot. This functionality has since been more comprehensively integrated with RViz, the ROS visualisation and debugging software. Pangolin is still heavily intertwined within the source code of `spot_slam`, however, so for now consider this step mandatory, even though Pangolin will be removed as a dependency in the future. The following commands download, build and install Pangolin.

```bash
$ cd ~
$ mkdir -p dev/tools
$ cd dev/tools
$ git clone https://github.com/stevenlovegrove/Pangolin.git pangolin
$ cd pangolin
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```

Ensure OpenCV version `4.2.0` is installed. If a different version is installed, run `$ pip uninstall opencv-python` and install the correct version with the `$ pip install opencv-python==4.2.0.34` command. Multiple versions of OpenCV is heavily discouraged.

```bash
$ python3 -c "import cv2; print(cv2.__version__)"
```

Now we install some ROS packages required by the `spotters` project. The first provides critical octree functionality and the second loads a USB camera device into Linux as a ROS image stream. If the Spot SensorRelay is used, this is required. If Spot Cam is used instead, this step can be ignored. Note that the current repository will require [minor changes in the launch configurations](#cameras) to enable Spot Cam functionality.

```bash
$ sudo apt install ros-noetic-octomap-server ros-noetic-usb-cam
```

The following dependencies enable video streaming from the SensorRelay app to the primary compute server running ROS. SensorRelay encodes data from capture devices using a hardware accelerated encoding pipeline and pushes the data onto a local web server. Given that the primary compute server is on the same network, the compute server uses `gstreamer` to listen to the video broadcast, decode and convert the stream for processing, and transfer the data into a dummy webcam device in Linux. This dummy device is then used by the `ros-noetic-usb-cam` package to load it into ROS for processing.

```bash
$ sudo apt install v4l-utils v4l2loopback-dkms gstreamer1.0-tools
```

Finally, clean build the `spotters` library using the `cbuildspt` command, defined in our custom `.setup.bash` script and outlined [below](#utilities). Building requires a significant computational resources and the build will fail if your machine does not have enough memory. In the event of a build failure, refer to the [errors and known issues](#build-errors) below.

```bash
$ cbuildspt
```

This concludes the project installation; for a guide running the project, please see the [execution](#execution) section.

### **Installation Issues**

Any errors or warnings encountered during the installation process are documented here, with their corresponding solutions. This list is not exhaustive; we've likely only encountered a subset of possible issues during our own installations and builds. If you encounter a problem not documented here, please contact the owner of this repository. We will fix address the issue here.

#### **Python Packages Not Found**

If any errors like this occur during `pip install` they mean required Python packages are not supported by `pip` or are defective. Currently known packages include `PyKDL` and `sip` but you may encounter others. In our experience, the `apt` versions of these packages work perfectly. To resolve these errors, install the precompiled Python packages using `apt` instead of `pip` and copy the packages from your base interpreter into your `spotters` virtual environment. The required package name should be included in the error message.

```bash
$ sudo apt install python3-<python_package>
$ cp /usr/lib/python3/dist-packages/<python_package>* ~/spotters/spot_env/lib/<python_version>/site-packages/
```

It's important to note that this solution is not ideal, since the packages will not be tracked by any package manager. Thus, it's important to note down any packages installed this way for reference later, in case they need to be manually upgraded. Alternatively, a better solution would be to create a symlink between the files instead of performing a hard copy. This would leave the base installation's package manager in charge of the dependency. We do not recommend this method by default as we haven't tested it. We will update this command once it has been tested.

```bash
$ ln -s /usr/lib/python3/dist-packages/<python_package> ~/spotters/spot_env/lib/<python_version>/site-packages/<python_package> # Untested, don't use this.
```

#### **Failed to Build SLAM**

Building SLAM requires the dependencies installed in the `spot_venv` virtual environment. Before building workspace packages, always ensure this environment is active; if not, activate it as per the below.

```bash
$ cd ~/spotters
$ spot_env
```

If, for some reason, the latter command does not work, try this one, which specifies the activation script explicitly. If you are using a shell other than `bash` you may need to specify the corresponding script using this method. For example, users of the `fish` shell will need to source the `activate.fish` file; the standard `activate` script will not work.

```bash
$ source ~/spotters/spot_env/bin/activate
```

#### **Failed to Detect Successful Installation**

If this error occurs for any dependencies, manually install the relevant package using `pip` and ensure `spot_env` is activated. The only package known to raise this error is `transforms3d` and we have verified the solution in this case.

```bash
$ spot_env
$ pip install <python_package>
```

#### **Build Errors**

The build process is resource intensive, and you may encounter errors such as `c++: fatal error: Killed signal terminated program cc1plus` or similar. The process may also hang or cause your operating system to freeze. This error may be resolved by allocating more resources to the virtual machine. Our recommended build configuration uses 7 cores and 11 gigabytes of memory. If this is not possible, configure `catkin` tools in the `buildspt` and `cbuildspt` commands defined by `.setup.bash` to use less resources by using the `-j` or the `--mem-limit` [flags](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html#configuring-build-jobs). The former should be used to configure core count, and the latter for memory.

#### **Build Warnings**

Some dependencies use deprecated modules which could raise warnings during the build process. These are not critical in our experience. So, for the moment, we believe it safe to ignore these warnings.

## **Utilities**

As part of Spotters, we define a number of aliases that help to interact with the project. These commands should always be used where applicable, especially the build commands. We may update these and the corresponding sections of our documentation to allow for `spotters` to be installed anywhere in future, instead of just the home directory.

- `buildspt` performs a regular build of the `spotters` repository.
- `cbuildspt` clears all build files and rebuilds the repository from scratch. This is very resource-intensive.
- `spot_env` activates the `~/spotters/spot_venv` virtual environment.
- `wspt` switches to the `spotters` workspace directory, currently `~/spotters/workspace`.
- `sspt` sources `spotters` build packages, which allows these packages to be discoverable by the local ROS environment.

## **Context**

This section contains some contextual, but not strictly required, information about the project. For the moment, these subjects are limited. However, we expect them to multiply as the project grows in scope and capability.

### **Submodules**

Some of the project dependencies are installed as submodules in the repository. This is due to the rolling release of these dependencies, namely the `spot_ros` and `spot_wrapper` packages. Since the hardware and software of the Spot platform will change over time, these dependencies need to be continuously updated and integrated to enable Spotters to interface with the most recent Boston Dynamics SDK.

The `spot_ros` and `spot_wrapper` submodules are held in their respective forks under the Github user `kinda-raffy`, who owns this repository. Within these forks, there is a primary synchronization branch (which corresponds to the `master` or `main` branch of the source repository) and a secondary `dev` branch. Since we need to extend these dependencies ourselves, our version of these dependencies gets pushed to the `dev` branch. As the respective third-party maintainers develop and integrate updates into their base repositories, our forked repository is notified, and pull requests from the synchronisation branch are made to update our `dev` branch. This ensures our versions of these dependencies will remain updated as long as they are maintained.

By default, the Spotters submodules are latched to the `dev` branches of their respective remote repositories. The command `git submodule update --remote` will update the most recent commit on the remote `dev` branch. Due to the difficult nature of integrating dependencies altered by separate parties (updated by their creators and modified by us), some knowledge of Git submodules is required to work with the Spotters project.

## **Execution**

The execution process will differ slightly depending on the input devices used. The current process highlights the methodology required to use the sensor relay on iPhones. If the Spot Cam is being used, the [Connect to SensorRelay](#connect-to-sensorrelay) may be skipped.

### **Connect to SensorRelay**

Open the `Spot SensorRelay` on your iPhone device and, on your Ubuntu machine, create a dummy webcam device with the first command below. Next, use the second command to identify the device ID of the dummy webcam. The id will be listed in the `/dev/video[id]` format.

```bash
$ sudo modprobe v4l2loopback
$ v4l2-ctl --list-devices
```

Then, define a pipeline to decode and convert the broadcasted stream from the sensor relay into the dummy device.

```bash
$ gst-launch-1.0 souphttpsrc location=http://[ip]:[port]/rgb ! jpegdec ! videoconvert ! v4l2sink device=/dev/video[number]
```

A `new clock` message will be outputted upon successful execution of the pipeline. For all other outputs, refer to [Execution Issues](#execution-issues).

Before starting the Spotters project, ensure the video device id is correct in `spot_slam/launch/camera.launch`.

### **Start Spotters**

To run the Spotters project, use the following command to run all required nodes.

```bash
roslaunch spot_compose up.launch
```

### **Execution Issues**

These are the errors we have encountered during our testing and their fixes is not exhaustive, and will likely grow as the project grows.

#### **USB Cam Timeout**

Occurs when the network between the sensor relay and the primary compute server has been disrupted. To resolve this, restart the sensor relay app and relaunch the spotters project.

#### GStreamer connectivity issues

In cases where gstreamer fails to connect to the sensor relay broadcast, follow the given checklist.

- Ensure the sensor relay is running and broadcasting.
- Ensure the command is syntactically correct.
- Ensure the `ip` and `port` is correct.
- Ensure the primary compute server is on the same network as the sensor relay.

If the error persists after going through the above points, restart the sensor relay app. On iOS, this may be done by swiping up to the app drawer and flicking the sensor relay up from the screen. Afterwards, relaunch the app.

From our experience, all errors are resolved by restarting the app and re-running the pipeline.

In rare circumstances this does not resolve the error, remove the dummy camera device using `sudo modprobe -r v4l2loopback` and go through the [Connect to SensorRelay](#connect-to-sensorrelay) instructions again.

#### **Octomap Errors (RANSAC)**

Octomap uses a RANSAC technique to perform ground plane segmentation. This requires a large number of points to identify the ground plane. However, when the program initially starts, there is a lack aof registered features to perform this segmentation, and therefore a RANSAC related error may be shown. This error should resolve itself overtime, and preferably before the completing of the initialisation stage of the program.

#### **Remote/One-off Issues with Spotters**

In very rare cases, some nodes may fail with a non-descriptive error. Although this is unlikely and may not ever happen, it is good to know that the general practice of restarting the program will resolve most issues.

## **Architecture**

### **Network Overview**

![Spotters System Diagram]("/docs/images/System_Overview.png")

### **Primary Architecture**

