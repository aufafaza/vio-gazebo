# Learning Visual Inertial Odometry using Realsense T265 Plugin 
make sure you have dependencies in check, i.e. any ros2 distro (mine is jazzy), mavros, mavconn, and tf2. 

to run, open two terminals. 

terminal 1: 
`ros2 launch uav_sim uav_sim.launch.xml use_swarm:=true/false` 

terminal 2: 
`ros2 run vision t265_subscriber` 

## rviz2 
to visualize the odometry of the camera, use rviz2 and do: 
1. TF, under "rviz_default_plugins", select TF. This shows all the coordinate frames as axes so you can see the tree connecting.
2. Odometry, select Odometry, then in its properties set Topic to /camera/pose/sample.
3. Image, select Image, set Topic to /camera/fisheye1/image_raw. This opens as a separate small window/panel, not in the 3D view.

# Todo
1. ENU to NED conversion (currently the drone spins frantically due to the mismatch 
something i want to do: try openvins and not rely on the t265 black box vio 
