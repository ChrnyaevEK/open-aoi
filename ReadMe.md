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

## ROS2 nodes
This is a guide to start development of new Open AOI ROS node (handle inspection procedures). In case you just want to run system, skip this section. Development guide was tested on Ubuntu operation system.

1. Follow ROS2 [installation guide](https://docs.ros.org/) and install ROS2 (bare version is enough).
2. Follow Docker [installation guide](https://docs.docker.com/) and install Docker and Docker compose (required to run database).
3. **Using python system interpreter** create virtual environment. 
```
python -m venv ./venv  # Create
source venv/bin/activate # Activate
```
4. Install python requirements.
```
python -m pip install -r requirements.txt
```

## Open AOI support python package
Development and deployment of `open_aoi` package follow [official python packaging guide](https://packaging.python.org/en/latest/tutorials/packaging-projects/). Package source code is located at `./open_aoi`.