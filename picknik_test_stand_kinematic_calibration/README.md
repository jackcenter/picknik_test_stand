# Picknik Test Stand Kinematic Calibration
These instructions were developed specifically for the UR5 as a template to follow for a generic robot. The calibration is based on the [robot_calibration](https://github.com/mikeferguson/robot_calibration) package by Mike Fergusion which was used for the Fetch Robot in the [fetch_calibration](https://github.com/fetchrobotics/fetch_ros/tree/indigo-devel/fetch_calibration) package which contains examples of its use. This package offers a ROS node to calibrate joint angle and robot frame offsets and store them in an updated URDF. To use the package, the following steps will need to be completed.

# Install the workspace
Running the calibration on the UR5 requires the drivers for the arm (Universal_Robots_ROS_Driver), the description of the arm (fmauch_universal_robot), the description of the camera used (realsense-ros), and the calibration package (robot_calibration). Specific packages used for a generic robot will differ, but generally a method to bring up the arm and 3D camera is required.

As an example, the ur5_calibration package can be created by executing the following.
1. Create the workspace directory

```bash
export CALIB_WS=~/ws_calibration
mkdir -p $CALIB_WS/src
```
2. Initialize the workspace
```bash
cd $CALIB_WS
wstool init src
cd src
git clone https://github.com/jackcenter/ur5_calibration.git
```
3. Merge in required workspaces
```bash
cd $CALIB_WS
wstool merge -t src src/ur5_calibration/upstreams.repos
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r -y
```
4. Build the package
```bash
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
catkin build
```

# Setup the calibration
## Create a launch file for bringing up the calibration environment.
This should include launching the arm, the calibration camera publishing a PointCloud2, and the environment obstacles. The coresponding launch file for this package is `test_stand_bringup.launch`. Align the camera by adjusting it's position and the urdf values so the point cloud lines up approximatley with the real world.

**Note:** The robot_calibration package requires an ordered point cloud. RealSense by default publishes an unordered point cloud. The following settings should be used when launching a RealSense camera and can be found in `test_stand_bringup.launch`:

```xml
<!-- Launch RealSense -->
<include file="$(find realsense2_camera)/launch/rs_camera.launch" >
  <arg name="filters" value="pointcloud" />
  <arg name="ordered_pc" value="true" />
</include>
```

## Print a configuration checkerboard
To generate a checkerboard, the [calib.io](https://calib.io/pages/camera-calibration-pattern-generator) pattern generator can be used (or any other pattern generator). The parameters used in this demonstration are:

| Parameter | Value |
| --- | --- |
| Target Type | Checkerboard |
| Board Width |  216 |
| Board Height | 279 |
| Rows | 5 |
| Columns | 7 |

**Note:** the grid generated here is 5x7. The openCV checkerboard finder expects a paramter for the number of *inner edges* on the checkerboard which is 4x6 (5-1 x 7-1). and **not** the number of cells. This parameter is set in `config/capture_checkerboards.yaml` which will be created in the next step. The minimum number of inner edges the checkerboard finder needs is 3, which means the minimum dimension for either the rows or columns of the checkerboard is 4 (ie a 4x4 grid).

## Create data capture configuration file
Create a new file: `config/capture_checkerboards.yaml`. This file will be used to specify the kinematic chains and the feature being used for calibration.

```yaml
chains:
  - name: arm                                     # Name used by robot_calibration for the arm chain.
    topic: /move_group/display_planned_path       # This is for automated motion which is not required.
    joints:                                       # These should be the ordered joint names from the urdf.
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    planning_group: arm                           # The move group in MoveIt
duration: 2.0                                     # The duration used during automated calibration
features:
  checkerboard_finder:
    type: robot_calibration/CheckerboardFinder    # For using a checkerboard during calibration 
    topic: /camera/depth/color/points             # The depth camera color topic
    camera_info_topic: /camera/depth/camera_info  # The topic where details specific to the camera are published
    camera_sensor_name: camera                    # Name used by robot_calibration for the camera chain
    chain_sensor_name: arm                        # Chain with the checkerboard attached to the tip
    points_x: 6                                   # Number of inner edges on the calibration checkerboard (x cells minus 1)
    points_y: 4                                   # Number of inner edges on the calibration checkerboard (y celss minus 1)
    square_size: 0.035                            # Edge length of a single checkerboard cell
```
## Generate a configuration optimization configuration file

Create another new file: `config/calibrate.yaml`. This file specifies which joints will be optimized. It helps to have the camera and checkerboard positions well-approximated before attempting the calibration on all of the joints. To do this, you can adjust the free params in three steps:

1. Set only the checkerboard board position as a free frame (an example can be found in `config/calibrate_checkerboard.yaml`).
    - Run the calibration.
    - Convert the A, B, and C numbers to roll, pitch, and yaw using `rosrun robot_calibration to_rpy A B C`.
    - Add the x, y, z, roll, pitch, and yaw values to the `callibrate.yaml` under `free_frames_initial_values` (example below). 
2. Set the checkerboard and camera positions as free frames (an example can be found in 'config/calibrate_camera.yaml'). Alternativley, running a separate camera calibration process would work here.
    - Run the calibration.
    - Convert the A, B, and C numbers to roll, pitch, and yaw using `rosrun robot_calibration to_rpy A B C`. The values for both the camera and the checkerboard can be kept during this step as it is likely the checkerboard position was also refined.
    - Adjust the camera position in the urdf according to the output. The output values should be treated as offsets. This can be done directly in the urdf or as part of a `camera_calibration.yaml` file (which is what is done in this package; an example can be found in `picknik_test_stand_description/config/camera_calibration.yaml` in conjunction with `picknik_test_stand_description/urdf/picknik_test_stand.urdf.xacro`).
3. Run the calibration including the checkerboard, camera, and joints as free frames (an example can be found in `config/calibrate.yaml`). 
    - Run the calibration.
    - Convert the A, B, and C numbers to roll, pitch, and yaw using `rosrun robot_calibration to_rpy A B C` as before. 
    - Apply the offsets to the urdf.

Manually adding the values to the urdf is tedious so it will be a good idea to implement these offsets from a `kinematic_calibration.yaml` that automatically applies the changes. However, this functionality hasn't been implemented yet.
 
```yaml

verbose: true
base_link: base_link                    # the base link of the urdf
models:
  - name: arm                           # Name used by robot_calibration for the arm chain.
    type: chain                         # Type of model from robot_calibration.
    frame: wrist_3_link                 # The tip (last link) of the chain.
  - name: camera                        # Name used by robot_calibration for the camera
    type: camera3d                      # Type of model from robot_calibration
    frame: camera_color_frame           # Frame for the rgb package
    topic: /camera/depth/color/points   # Topic where the depth color points are being published.
free_params:                            # Joint positions to be optimized.
  - shoulder_pan_joint
  - shoulder_lift_joint
  - elbow_joint
  - wrist_1_joint
  - wrist_2_joint
  - wrist_3_joint
free_frames:                            # Additional frames included in the optimization.
  - name: camera_color_joint            # Included because it is unlikely its position is perfectly known.
    x: true
    y: true
    z: true
    roll: true
    pitch: true
    yaw: true
  - name: checkerboard                  # Virtual link (not in the URDF) adjusted by the optimizer.
    x: true
    y: true
    z: true
    roll: true
    pitch: true
    yaw: true
free_frames_initial_values:             # Estimated initial poses
  - name: checkerboard
    x: 0.172357
    y: -0.114575
    z: -0.0130403
    roll: -3.07854
    pitch: 0.0277346
    yaw: 3.13444
error_blocks:                           # Error blocks to be optimized
  - name: hand_eye                       # Minimize error between arm and camera projections of the checkerboard 
    type: chain3d_to_chain3d
    model_a: camera
    model_b: arm
  - name: restrict_camera                # Restrict the camera frame from being optimized too far from its initial position 
    type: outrageous
    param: camera_color_joint
    joint_scale: 0.0
    position_scale: 0.1
    rotation_scale: 0.1
```
## Create a launch file for the calibration node
This file will load the two configuration files as rosparams and launch the *manual* robot calibration node. The launch file used for the example is in `launch/capture_checkerboards.launch`. 

```xml
<!-- Clear robot_calibration parameter -->
<rosparam command="delete" param="robot_calibration" />

<!-- Launch the calibration -->
<node pkg="robot_calibration" type="calibrate" name="robot_calibration"
    args="--manual"
    output="screen"
    launch-prefix="gdb -ex run --args" >
  <rosparam file="$(find ur5_calibration)/config/capture_checkerboards.yaml" command="load" />
  <rosparam file="$(find ur5_calibration)/config/calibrate.yaml" command="load" />
</node>
```

## Run the calibration
1. Launch the environment which should include the arm, the camera, and the workspace obstacles:
```bash
roslaunch ur5_calibration test_stand_bringup.launch simulate=false robot_ip:=xx.x.x.xxx
```
2. In a separate terminal, launch the calibration node:
```bash
roslaunch ur5_calibration capture_checkerboards.launch
```
3. Follow the onscreen instructions. 

The robot will need to be moved into a sufficient number of unique poses (where all joints are moved) to allow the calibration to converge. During data collection in manual mode, the operator will need to mannually adjust the arm and press `enter` once it's in position to capture the pose. Once complete with the operator selceted poses, type `done` into the console and the optimization will begin.

Saving the poses in the srdf to be played back with MoveIt is reccomended to allow for faster and more repeatable calibration in the event the solution does not converge and the calibration process needs to start over. There is not a way to regain poses after the optimization is started at the end of the data collection phase.

## Apply the changes
Does the robot support "calibration" tags?

