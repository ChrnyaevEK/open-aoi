**IN DEVELOPMENT!**

# Welcome to Open AOI!
This is a ROS2 powered Automated Optical Inspection framework, developed as part of my master thesis at **BUT Brno** university. Project aims to provide development environment for PCB optical inspection tasks in production. Project targets **Raspberry Pi** platform yet it is possible to use any other general purpose computer. Project use **docker** and **Nice GUI** for frontend, general architecture is described in `design.drawio` file. 

![System overview](/assets/documentation/system_high_level_overview.png)

# Deployment  guide
Open AOI should be deployed to a computer with Bastler camera close to production line. Connect camera to PC, setup light and continue with system setup.
1. Install `Ubuntu Server 22.04` on target machine.
2. Follow Docker [installation guide](https://docs.docker.com/) and install Docker and Docker compose.
3. Run system with following command from project root folder.
```
docker-compose up -d
```

# Development guide
Basic system configuration is provided by `.env` file from root directory.
```
MYSQL_ROOT_PASSWORD=<SQL root password>
MYSQL_DATABASE=<preferred database name>
MYSQL_USER=<Name for new SQL user>
MYSQL_PASSWORD=<Password for new SQL user>

SIMULATION=<1|0, is 1 (allow) system will run in simulation mode>

STORAGE_SECRET=<Really strong secret string>

AOI_OPERATOR_INITIAL_PASSWORD=<Initial password for operator>
AOI_ADMINISTRATOR_INITIAL_PASSWORD=<Initial password for administrator>
```

## ROS2 services
This is a guide to start development of new Open AOI ROS2 node (handle inspection procedures). In case you just want to run system, skip this section. Development guide was tested on Ubuntu operation system. 
**ROS2**: foxy, **Python**: 3.8. Other versions were not tested, but may eventually also work (make sure to update ROS2 distro and python version in package as it was referenced by name). 

### Development installation
1. Follow ROS2 [installation guide](https://docs.ros.org/) and install ROS2 (bare version is enough).
2. Follow ROS2 Bridge [installation guide](https://wiki.ros.org/rosbridge_suite) and install ROS2 bridge.
3. Follow Docker [installation guide](https://docs.docker.com/) and install Docker and Docker compose (required to run database).
4. Make sure **`python3` is python global system interpreter** and **`pip3` is available**. Check python version and update `aoi.setup.bash` with your python version (default is python3.8). Run next commands to build colcon workspace and install python dependencies. Python dependencies will be installed to colcon `./install` folder and will not interfere  with global packages (will be available only after workspace is sourced, same fashion as `rclpy` package from ROS2).
```
bash aoi.build.bash  # Build workspace 
bash aoi.setup.bash  # Install python dependencies
```

Test installation
```
bash aoi.launch.bash  #  Launch AOI services and ROS2 Bridge
```

### Docker instance
Alternatively one may want to run AOI ROS2 services in docker. Docker container is with AOI ROS2 is described by `dockerfile.aoi-ros2`. Use `docker compose aoi-ros2 --build` for build. Container expose port `9090` with ROS2 Bridge. Bridge API for python is realized by `roslibpy` (part of bridge suite), basic setup is available in playground at `playground/ros2.ipynb`.

## Open AOI support python package
Development and deployment of `open_aoi` package follow [official python packaging guide](https://packaging.python.org/en/latest/tutorials/packaging-projects/). Package source code is located at `./open_aoi`.