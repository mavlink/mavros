Installation
============

ROS2 installation is out of topic for that guide,
you should follow the [official installation instruction][kinst].

!!! note
    This guide tested on the ROS2 **Kilted** release.
    But it is likely that your release will have quite similar set of commands.


Dependencies
------------

Most of the ROS dependencies are supported and installed by `rosdep`, including external
libraries such as [ASIO][asio], [Eigen][eigen] and [GeographicLib][geolib].

Since **GeographicLib requires certain datasets** so to fulfill
certain calculations, these need to be installed manually by the user.

!!! warning
    The geoid dataset is mandatory to allow the conversion between heights in order to respect ROS API.
    Not having the dataset available will shutdown the UAS node.


Binary package (deb)
--------------------

ROS repository has binary packages for Ubuntu amd64 and aarch64.
Just use `apt` for installation:

```shell
sudo apt install ros-kilted-mavros
```

Then install GeographicLib datasets using helper script:

```shell
sudo ros2 run mavros install_geographiclib_datasets.sh
```

Or alternative one-liner script:
```
curl -LsSf https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh | sudo bash
```


Build from source
-----------------

Use `vcs` utility for retrieving sources and `colcon` tool for build.

### Install `vcs` tool and `rosinstall_generator`
```shell
sudo apt install -y python3-vcstool python3-rosinstall-generator python3-osrf-pycommon
```

### Create the workspace (_unneeded if you already have it_)
```shell
source /opt/ros/kilted/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Get MAVLink and MAVROS repos
```shell
rosinstall_generator --format repos mavlink | tee /tmp/mavlink.repos
rosinstall_generator --format repos --upstream mavros | tee -a /tmp/mavros.repos
```

!!! note
    `--upstream` - tells to use upstream git instead of git-buildpackage one.

!!! note
    You may use `--upstream-development` to pull latest sources (`ros2` branch).

### Clone repos to workspace
```shell
vcs import src < /tmp/mavlink.repos
vcs import src < /tmp/mavros.repos
```

### Install dependencies
```shell
rosdep update
rosdep install --from-paths src --ignore-src -y
```

### Install GeographicLib datasets
```shell
sudo ./src/mavros/scripts/install_geographiclib_datasets.sh
```

### Build
```shell
colcon build
```

### Update environment
```shell
source ./install/setup.bash
```

!!! node
    Make sure that you use `setup.bash` or `setup.zsh` from workspace.
    Otherwise `ros2 run` can't find nodes.


[kinst]: https://docs.ros.org/en/kilted/Installation.html
[asio]: https://think-async.com/Asio/
[eigen]: https://eigen.tuxfamily.org/
[geolib]: https://geographiclib.sourceforge.io/
