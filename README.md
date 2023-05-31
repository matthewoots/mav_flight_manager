# Mavlink Flight Manager

To create a flight manager framework with **mavsdk** and **fastdds**, using **OMG idl** generated ROS2 types 

## Installation and Information

### Generation of ROS2 Fast-DDS message types
Do look into the `external/msgs_idl` submodule to understand more

### Submodules
1. **sockpp** - release v0.8.1
2. **fmt** - release 9.1.0
4. **yaml-cpp** - commit 0e6e28d

### Dependencies
```bash
# Download mavsdk
cd
git clone https://github.com/mavlink/MAVSDK.git mavsdk
cd mavsdk
git checkout v1.4.13
git submodule update --init --recursive
cmake -DCMAKE_BUILD_TYPE=Debug -Bbuild/default -H.
cmake --build build/default -j4
sudo cmake --build build/default --target install
```

### Setup
```bash
git clone git@github.com:matthewoots/mav_flight_manager.git --recursive
cd mav_flight_manager
cmake -Bbuild -H.
cmake --build build -j4
cd build
# run the executable
```

### Extra
To know how to generate `mavlink` messages to be used in c++ modules
```bash
git clone git@github.com:mavlink/mavlink.git
cd mavlink
git checkout 1.0.12
git submodule update --init --recursive
python3 -m pymavlink.tools.mavgen --lang=C++11 --wire-protocol=2.0 --output=generated/include/mavlink/v2.0 message_definitions/v1.0/common.xml
```
