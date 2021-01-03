# Visual-Servoing
Image-based visual servoing in eye-in-hand configuration for Universal Robot 5 using Microsoft Kinect V2 camera.

#### System Specifications:
- OS: Ubuntu 18.04 LTS
- Processor : Intel® Core™ i7-9700K @ 3.6 GHz 
- RAM : 64 GB
- ROS: Melodic
- Gazebo Version:  9.16.0

## Project Brief:
In this work, an image-based visual servoing technique is developed which uses the images from a RGB camera in eye-in-hand configuration with occlusion handling.The proposed method chooses a set of correctly extracted image features, and obtains an estimate of all the image features from the correctly extracted image features.   The estimation procedure makes it possible to track image features even  when  occlusion  occurs.   A  velocity  controller  is  developed  for  a  6 -DoF robotic manipulator which minimises the error between current and desired imagefeatures.   At each control loop,  end-effector velocity is estimated from image interaction matrix and it is used to generate joint velocities from robot jacobian. An extensive set of experiments are carried out to indicate the feasibility of the proposed approach. Finally, a dexterous manipulation capability is shown by using the robot to play ping-pong.

## How to run the experiments in simulation?
#### Tracking in 2D without Kalman filter:
- Launch the ros-gazebo environment:
`roslaunch ur_gazebo my_simple_environment.launch`
- Launch the CAMShift tracker
`rosrun camshift_tracker camshift_tracker`
- Launch the ROS Controller for UR5
`rosrun ur5_visual_servoing gazebo_ibvs_two_features.cpp`

#### Tracking in 2D with Kalman filter and occlusion:
- Launch the ros-gazebo environment:
`roslaunch ur_gazebo my_simple_environment_occlusion.launch`
- Launch the CAMShift tracker
`rosrun camshift_tracker camshift_kalman_tracker`
- Launch the ROS Controller for UR5
`rosrun ur5_visual_servoing gazebo_ibvs_two_features.cpp`

#### Tracking in 3D with Kalman filter and occlusion:
- Launch the ros-gazebo environment:
`roslaunch ur_gazebo my_simple_environment_occlusion.launch`
- Launch the CAMShift tracker
`rosrun camshift_tracker camshift_kalman_tracker`
- Launch the ROS Controller for UR5
`rosrun ur5_visual_servoing gazebo_ibvs_three_features.cpp`


#### Ping Pong:
- Launch the ros-gazebo environment with ping-pong table and paddle at end-effector:
`roslaunch ur_gazebo my_world.launch`
- Launch the CAMShift tracker
`rosrun camshift_tracker camshift_kalman_tracker`
- Launch the ROS Controller to play ping-pong:
`rosrun ur5_visual_servoing gazebo_ibvs_three_features.cpp`


For results and future details please refer the project report.









