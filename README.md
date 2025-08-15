# Geo Transformer

A ROS 2 package that provides services for converting between **geodetic coordinates** (latitude, longitude, altitude) and **local Cartesian coordinates (x, y, z)** using [GeographicLib](https://index.ros.org/d/geographiclib/).  

This is useful for robotics applications where GPS data needs to be mapped into a local reference frame.

---
Table of Content:
- [Geo Transformer](#geo-transformer)
  * [Features](#features)
  * [Services](#services)
  * [Dependencies](#dependencies)
  * [Build Instructions](#build-instructions)
  * [Usage](#usage)
    + [Expected log output:](#expected-log-output)
    + [Example Calls](#example-calls)
  * [Project Structure](#project-structure)
  * [Docker Support](#docker-support)
    + [Build Docker Image](#build-docker-image)
    + [Run Container](#run-container)
  * [Video Demonstration](#video-demonstration)
  * [Contributing](#contributing)


## Features

- Set and query a local origin
- Convert **Lat/Lon/Alt → Local (x, y, z)**
- Convert **Local (x, y, z) → Lat/Lon/Alt**
- Input validation & error handling
- gTest based tests

---

## Services

The node provides the following services:

| Service Name              | Type                                              | Description |
|----------------------------|---------------------------------------------------|-------------|
| `/local_coordinate/set`    | `geo_transformer_interfaces/srv/SetOrigin`        | Set the origin latitude, longitude, altitude |
| `/local_coordinate/get`    | `geo_transformer_interfaces/srv/GetOrigin`        | Get the currently set origin |
| `/from_ll`                 | `geo_transformer_interfaces/srv/FromLL`           | Convert latitude/longitude/altitude → local (x,y,z) |
| `/to_ll`                   | `geo_transformer_interfaces/srv/ToLL`             | Convert local (x,y,z) → latitude/longitude/altitude |

---

## Dependencies

- ROS 2 (tested on Humble)
- GeographicLib
- `rclcpp` (C++ ROS 2 client library)
- Custom service definitions in `geo_transformer_interfaces
- Docker`

---

## Build Instructions

Clone the repo into your ROS 2 workspace:

```bash
mkdir ~/ros2_ws && cd ~/ros2_ws/
git clone https://github.com/ashok-arora/ros2_geo_transformer.git
```

## Usage

```
ros2 run geo_transformer geo_transformer_node
```

### Expected log output:
```
[INFO] [geo_transformer_node]: GeoTransformer node started.
```

### Example Calls
1. Set Origin
```
ros2 service call /local_coordinate/set geo_transformer_interfaces/srv/SetOrigin "{latitude: 52.0, longitude: 13.0, altitude: 35.0}"
```
2. Get Origin
```
ros2 service call /local_coordinate/get geo_transformer_interfaces/srv/GetOrigin "{}"
```

3. Convert Geodetic → Local
```
ros2 service call /from_ll geo_transformer_interfaces/srv/FromLL "{latitude: 52.0001, longitude: 13.0001, altitude: 36.0}"
```

4. Convert Local → Geodetic

```
ros2 service call /to_ll geo_transformer_interfaces/srv/ToLL "{x: 7.432, y: 11.217, z: 1.000}"
```

## Project Structure

```
geo_transformer/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── geo_transformer_node.cpp
└── include/
    └── geo_transformer/
        └── ...
geo_transformer_interfaces/
├── CMakeLists.txt
├── package.xml
└── srv/
    ├── SetOrigin.srv
    ├── GetOrigin.srv
    ├── FromLL.srv
    └── ToLL.srv
```

## Docker Support

You can build and run the package inside a Docker container to avoid setting up ROS 2 locally.

### Build Docker Image
```bash
docker build -t my_ros2_image .
```

### Run Container
```
# --- terminal 1 ---

docker run -it --rm --name ros2_container my_ros2_image

ros2 run geo_transformer geo_transformer_node

# --- terminal 2 ---

docker exec -it ros2_container bash

source /opt/ros/humble/setup.sh && source ./install/setup.sh 

ros2 service call /local_coordinate/set geo_transformer_interfaces/srv/SetOrigin "{latitude: 12.9716, longitude: 77.5946, altitude: 900.0}" 

ros2 service call /local_coordinate/get geo_transformer_interfaces/srv/GetOrigin "{}"

ros2 service call /from_ll geo_transformer_interfaces/srv/FromLL "{latitude: 12.9716, longitude: 77.5946, altitude: 900.0}"

ros2 service call /to_ll geo_transformer_interfaces/srv/ToLL "{x: 10.0, y: 20.0, z: 5.0}
```

## Video Demonstration
[final_recording.webm](https://github.com/user-attachments/assets/be10945b-6cc2-4228-82fe-be5cf8e59ca8)


## Contributing

If you encounter any bugs, have questions, or would like to request a new feature, please open an issue or submit a pull request on the GitHub repository.
