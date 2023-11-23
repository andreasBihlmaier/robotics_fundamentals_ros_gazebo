[ROSCon DE 2023](https://roscon2023.de/) talk "Learning Robotics Fundamentals with ROS 2 and modern Gazebo"

[Slides available here](https://docs.google.com/presentation/d/16ZQMB7OVc1fkPRG1K7rQIWCd5zYWTj1hTaeI2rAjZSo/edit?usp=drive_link)

Setup:
* Follow the official installation instructions at https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html.
* In the step "Install ROS 2 packages", install the packages `ros-humble-desktop` and `ros-dev-tools`.
* In addition, install the following packages
  ```
  sudo apt install \
    python3-colcon-common-extensions \
    ros-humble-ign-ros2-control \
    ros-humble-plotjuggler-ros \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ros-gz-sim-demos \
    ros-humble-ros-ign-gazebo \
    ros-humble-rqt-joint-trajectory-controller \
    ros-humble-rqt-tf-tree
  ```

M1:
* Start (modern) Gazebo:
  ```
  ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf -r"
  ```
* Spawn a simple mobile robot in Gazebo:
  ```
  ros2 run ros_gz_sim create -file $(pwd)/vehicle_blue.sdf -z 0.325
  ```
* Create a bridge between ROS and Gazebo:
  ```
  ros2 run ros_gz_bridge parameter_bridge \
    '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
    '/model/vehicle_blue/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
  ```
* Start PlotJuggler
  ```
  ros2 run plotjuggler plotjuggler
  ```
* Add plots:
  * ROS2 Topic Subscriber -> Select `/model/vehicle_blue/pose`
  * Drag and drop `/model/vehicle_blue/pose/empty/vehicle_blue/translation/x` from Timeseries List to the plot area
  * Split the plot vertical twice
  * Add Custom Series -> Input timeseries: Same `x` as above; Function library: `backward_difference_derivative`; New name: `v`; Create New Timeseries
  * Drag and drop `v` from Custom Series to the middle plot and to the bottom plot
  * Apply filter to data on the bottom plot -> Derivative
  * Now the top plot shows the position x, the middle plot shows the velocity v, and the bottom plot shows the acceleration a.
* Command the mobile robot to move forward:
  ```
  ros2 topic pub --once /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist '
  linear:
    x: 0.1
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0'
  ```

P1:
* Start Gazebo:
  ```
  ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$(pwd)/falling_world.sdf"
  ```
* Create ROS-Gazebo bridge:
  ```
  ros2 run ros_gz_bridge parameter_bridge '/model/sphere/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
  ```
* Record pose of falling sphere:
  ```
  ros2 bag record /model/sphere/pose
  ```
* Unpause Gazebo
* Wait for sphere to hit the ground. Stop recording.
* Open bag file in PlotJuggler

P2:
* Start Gazebo:
  ```
  ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$(pwd)/slippery_slope.sdf"
  ```
* Create ROS-Gazebo bridge:
  ```
  ros2 run ros_gz_bridge parameter_bridge \
    '/model/sphere/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' \
    '/model/cylinder/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
  ```
* Record pose of falling sphere:
  ```
  ros2 bag record /model/sphere/pose /model/cylinder/pose
  ```
* Unpause Gazebo
* Wait for both objects to have rolled down the inclined plane. Stop recording.
* Open bag file in PlotJuggler

R1:
* Start Gazebo:
  ```
  ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$(pwd)/stiff_one_armed_bandit.sdf"
  ```
* Create ROS-Gazebo bridge:
  ```
  ros2 run ros_gz_bridge parameter_bridge \
    '/world/stiff_one_armed_bandit/model/double_pendulum_with_base/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model' \
    '/model/double_pendulum_with_base/joint/upper_joint/cmd_force@std_msgs/msg/Float64]gz.msgs.Double' \
    '/model/double_pendulum_with_base/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
  ```
* Unpause Gazebo
* Add `/world/stiff_one_armed_bandit/model/double_pendulum_with_base/joint_state/lower_joint/position` to PlotJuggler
* Add `/model/double_pendulum_with_base/pose/double_pendulum_with_base/double_pendulum_with_base/lower_link/translation/y` and `z` as XY plot to PlotJuggler

R2:
* Start Gazebo:
  ```
  ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$(pwd)/damped_stiff_one_armed_bandit.sdf -r"
  ```
* Create ROS-Gazebo bridge:
  ```
  ros2 run ros_gz_bridge parameter_bridge \
    '/world/damped_stiff_one_armed_bandit/model/double_pendulum_with_base/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model' \
    '/model/double_pendulum_with_base/joint/upper_joint/cmd_force@std_msgs/msg/Float64]gz.msgs.Double' \
    '/model/double_pendulum_with_base/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
  ```
* Setup plots in PlotJuggler, similar to R1.
* Set torque on `upper_joint`:
  ```
  ros2 topic pub --once /model/double_pendulum_with_base/joint/upper_joint/cmd_force std_msgs/msg/Float64 'data: 3.0'
  sleep 3
  ros2 topic pub --once /model/double_pendulum_with_base/joint/upper_joint/cmd_force std_msgs/msg/Float64 'data: 0.0'
  ```
* Set friction coefficient of the `upper_joint` to zero and set the damping coefficient to 0.5:
  ```
  <joint name="upper_joint" type="revolute">
    <parent>base</parent>
    <child>upper_link</child>
    <axis>
      <xyz>1.0 0 0</xyz>
      <dynamics>
        <damping>0.5</damping>
        <!-- <friction>1.0</friction> -->
      </dynamics>
    </axis>
  </joint>
  ```
* Restart Gazebo and repeat the torque command. Observe the difference in the plots.

R3:
* Start Gazebo:
  ```
  ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$(pwd)/damped_stiff_one_armed_bandit.sdf -r"
  ```
* Create ROS-Gazebo bridge:
  ```
  ros2 run ros_gz_bridge parameter_bridge \
    '/world/one_armed_bandit/model/double_pendulum_with_base/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model' \
    '/model/double_pendulum_with_base/joint/upper_joint/cmd_force@std_msgs/msg/Float64]gz.msgs.Double' \
    '/model/double_pendulum_with_base/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' \
    '/joint_lower_joint_cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
  ```
* Setup plots in PlotJuggler, similar to R1.
* Set torque on `upper_joint`:
  ```
  ros2 topic pub --once /model/double_pendulum_with_base/joint/upper_joint/cmd_force std_msgs/msg/Float64 'data: 50.0'
  ```
* Set joint position on `lower_joint`:
  ```
  ros2 topic pub --once /joint_lower_joint_cmd_pos std_msgs/msg/Float64 'data: 0.0'
  ```
* Change the joint position of `lower_joint` and observe the resulting settling position of the pendulum.
* You can also vary the `lower_joint` position more continuously:
  ```
  for q in $(seq 0 0.1 6.28); do
    ros2 topic pub --once /joint_lower_joint_cmd_pos std_msgs/msg/Float64 "data: ${q}"
  done
  ```

C1:
* Start Gazebo along with a simulated UR5e robot:
  ```
  ros2 launch ur_simulation_ignition ur_sim_control.launch.py
  ```
* Plot the robot's joint efforts via the `/joint_states` topic.
* Command trajectories to the simulated robot:
  ```
  ros2 launch ur_bringup test_joint_trajectory_controller.launch.py
  ```