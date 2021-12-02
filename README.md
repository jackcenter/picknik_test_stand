# Picknik Test Stand

## About

**Packages**
- Bringup
- Camera Calibration (future)
- Description
- Kinematic Calibration
- Moveit Config

## Install

First, create a workspace for the package. Alternatively you may use an existing melodic workspace:
```bash
export TESTSTAND_WS=~/ws_test_stand
mkdir -p $TESTSTAND_WS/src
```

Initialize wstool and clone the repository into the source folder:
```bash
cd $TESTSTAND_WS
wstool init src
cd $TESTSTAND_WS/scr
git clone https://github.com/jackcenter/picknik_test_stand.git
```

Download the workspace dependencies. Currently I need to change `realsense-ros/realsense2_camera/CMakeLists.txt` to look for version 2.48 base on what's installed on my machine.

```bash
cd $TESTSTAND_WS
wstool merge -t src src/picknik_test_stand.gi/upstream.repos
wstool update -t src
```

## Use

Simulation
```bash
roslaunch picknik_test_stand_bringup picknik_test_stand.launch
```
Hardware
```
roslaunch picknik_test_stand_bringup picknik_test_stand.launch simulate:=false robot_ip:=xx.x.x.xxx
```

Arguments
- robot_ip: your robot's IP address
- simulate: true or false
- end_effector: mock_grappler, calibration_plate, none


## Contributing

## License