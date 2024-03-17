# Welcome to Open AOI!
This is a support package for [Open AOI project](https://github.com/ChrnyaevEK/open-aoi-system).

Open AOI is ROS2 powered Automated Optical Inspection framework, developed as part of my master thesis at **BUT Brno** university. Project aims to provide development environment for PCB optical inspection tasks in production. Project targets **Raspberry Pi** platform yet it is possible to use any other general purpose computer. Project use **docker** and **Nice GUI** for frontend.


*Please refer main project repository for details.*

## Run web service
Start web server with the following command.
```
python -m open_aoi_web_interface.main
```

## Run GPIO service
```
python -m open_aoi_gpio_interface.main
```
