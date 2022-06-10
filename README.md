# dream_track
## 의존성 페키지 수정사항
### launch 파일
<details>
<summary> turtlebot3_bringup/turtlebot3_robot.launch </summary>
<div markdown="1">

  카메라 제거
```
<launch>
  <arg name="multi_robot_name" default=""/>
  <arg name="set_lidar_frame_id" default="base_scan"/>
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>

  <include file="$(find turtlebot3_bringup)/launch/turtlebot3_core.launch">
    <arg name="multi_robot_name" value="$(arg multi_robot_name)"/>
  </include>
  <include file="$(find turtlebot3_bringup)/launch/turtlebot3_lidar.launch">
    <arg name="set_frame_id" value="$(arg set_lidar_frame_id)"/>
  </include>
  <node pkg="turtlebot3_bringup" type="turtlebot3_diagnostics" name="turtlebot3_diagnostics" output="screen"/>

  <group if = "$(eval model == 'waffle_pi')">
    <include file="$(find turtlebot3_bringup)/launch/turtlebot3_rpicamera.launch"/>
  </group>
</launch>
```

</div>
</details>
  
<details>
<summary> turtlebot3_slam/turtlebot3_slam.launch </summary>
<div markdown="1">

rviz 실행 제거. navigation과 동시 사용시 충돌
```
<launch>
  <!-- Arguments -->
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
  <arg name="slam_methods" default="gmapping" doc="slam type [gmapping, cartographer, hector, karto, frontier_exploration]"/>
  <arg name="configuration_basename" default="turtlebot3_lds_2d.lua"/>
  <arg name="open_rviz" default="true"/>

  <!-- TurtleBot3 -->
  <include file="$(find turtlebot3_bringup)/launch/turtlebot3_remote.launch">
    <arg name="model" value="$(arg model)" />
  </include>

  <!-- SLAM: Gmapping, Cartographer, Hector, Karto, Frontier_exploration, RTAB-Map -->
  <include file="$(find turtlebot3_slam)/launch/turtlebot3_$(arg slam_methods).launch">
    <arg name="model" value="$(arg model)"/>
    <arg name="configuration_basename" value="$(arg configuration_basename)"/>
  </include>
</launch>
```

</div>
</details>

<details>
<summary> turtlebot3_navigation/turtlebot3_navigation.launch </summary>
<div markdown="1">
 
  map_server 제거
```  
<launch>
  <!-- Arguments -->
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
  <arg name="map_file" default="$(find turtlebot3_navigation)/maps/map.yaml"/>
  <arg name="open_rviz" default="true"/>
  <arg name="move_forward_only" default="false"/>

  <!-- Turtlebot3 -->
  <include file="$(find turtlebot3_bringup)/launch/turtlebot3_remote.launch">
    <arg name="model" value="$(arg model)" />
  </include>

  <!-- Map server -->
  <!-- <node pkg="map_server" name="map_server" type="map_server" args="$(arg map_file)"/> -->

  <!-- AMCL -->
  <include file="$(find turtlebot3_navigation)/launch/amcl.launch"/>

  <!-- move_base -->
  <include file="$(find turtlebot3_navigation)/launch/move_base.launch">
    <arg name="model" value="$(arg model)" />
    <arg name="move_forward_only" value="$(arg move_forward_only)"/>
  </include>

  <!-- rviz -->
  <group if="$(arg open_rviz)"> 
    <node pkg="rviz" type="rviz" name="rviz" required="true"
          args="-d $(find turtlebot3_navigation)/rviz/turtlebot3_navigation.rviz"/>
  </group>
</launch>
```
  
</div>
</details>


### 파라미터
<details>
<summary> turtlebot3_slam/config/gmapping_params.yaml </summary>
<div markdown="1">
  
```
throttle_scans: 1 # (int, default: 1) Process 1 out of every this many scans (set it to a higher number to skip more scans)
map_update_interval: 1.0 #(float, default: 5.0) #How long (in seconds) between updates to the map. Lowering this number updates the occupancy grid more often, at the expense of greater computational load.
maxUrange: 80.0 #(float, default: 80.0) The maximum usable range of the laser. A beam is cropped to this value.
sigma: 0.05 # (float, default: 0.05) The sigma used by the greedy endpoint matching
kernelSize: 1 # (int, default: 1) The kernel in which to look for a correspondence
lstep: 0.05 # (float, default: 0.05) The optimization step in translation
astep: 0.05 # (float, default: 0.05) The optimization step in rotation
iterations: 5 # (int, default: 5) The number of iterations of the scanmatcher
lsigma: 0.075 # (float, default: 0.075) The sigma of a beam used for likelihood computation
ogain: 3.0 # (float, default: 3.0) Gain to be used while evaluating the likelihood, for smoothing the resampling effects
lskip: 0.0 # (int, default: 0) Number of beams to skip in each scan. Take only every (n+1)th laser ray for computing a match (0 = take all rays)
minimumScore: 0.0 # (float, default: 0.0) Minimum score for considering the outcome of the scan matching good. Can avoid jumping pose estimates in large open spaces when using laser scanners with limited range (e.g. 5m). Scores go up to 600+, try 50 for example when experiencing jumping estimate issues.
srr: 0.1 # (float, default: 0.1) Odometry error in translation as a function of translation (rho/rho)
srt: 0.2 # (float, default: 0.2) Odometry error in translation as a function of rotation (rho/theta)
str: 0.1 # (float, default: 0.1) Odometry error in rotation as a function of translation (theta/rho)
stt: 0.2 # (float, default: 0.2) Odometry error in rotation as a function of rotation (theta/theta)
linearUpdate: 1.0 # (float, default: 1.0) Process a scan each time the robot translates this far
angularUpdate: 0.5 # (float, default: 0.5) Process a scan each time the robot rotates this far
temporalUpdate: 1.0 # (float, default: -1.0) Process a scan if the last scan processed is older than the update time in seconds. A value less than zero will turn time based updates off.
resampleThreshold: 0.5 # (float, default: 0.5) The Neff based resampling threshold
particles: 30 # (int, default: 30) Number of particles in the filter
xmin: -50.0 # (float, default: -100.0) Initial map size (in metres)
ymin: -50.0 # (float, default: -100.0) Initial map size (in metres)
xmax: 50.0 # (float, default: 100.0) Initial map size (in metres)
ymax: 50.0 # (float, default: 100.0) Initial map size (in metres)
delta: 0.05 # (float, default: 0.05) Resolution of the map (in metres per occupancy grid block)
llsamplerange: 0.01 # (float, default: 0.01) Translational sampling range for the likelihood
llsamplestep: 0.01 # (float, default: 0.01) Translational sampling step for the likelihood
lasamplerange: 0.005 # (float, default: 0.005) Angular sampling range for the likelihood
lasamplestep: 0.005 # (float, default: 0.005) Angular sampling step for the likelihood
transform_publish_period: 0.05 # (float, default: 0.05) How long (in seconds) between transform publications. To disable broadcasting transforms, set to 0.

# ~occ_thresh (float, default: 0.25)
# Threshold on gmapping's occupancy values. Cells with greater occupancy are considered occupied (i.e., set to 100 in the resulting sensor_msgs/LaserScan). New in 1.1.0.
# ~maxRange (float)
# The maximum range of the sensor. If regions with no obstacles within the range of the sensor should appear as free space in the map, set maxUrange < maximum range of the real sensor <= maxRange.
```

</div>
</details>  
  
<details>
<summary> turtlebot3_navigation/global_costmap_params.yaml </summary>
<div markdown="1">
  
  static_map true -> false
```  
global_costmap:
  global_frame: map
  robot_base_frame: base_footprint

  update_frequency: 10.0
  publish_frequency: 10.0
  transform_tolerance: 0.5

  static_map: false
```

</div>
</details>


---  
  
##토글 코드
<details>
<summary> 토글 </summary>
<div markdown="1">
  
 내용

</div>
</details>
  
