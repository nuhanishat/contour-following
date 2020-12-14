# contour-following

## Setup 

### Kinova (to use kinova_msgs for commanding the arm)

- Go to the kinova-ros repo to follow the installation instructions, here:
  https://github.com/Kinovarobotics/kinova-ros/blob/master/README.md#installation

- Go to the following directory and put all files in the kinova_demo_files here:

      /kinova-ros/kinova_demo/nodes/kinova_demo/
      
### Marker Tracking with OpenCV and cv_bridge

- Install opencv and cv_bridge for ros using the following commands
**Note: We have to use python2 to use this**

    sudo apt-get install python-opencv
    sudo apt-get install python-cv-bridge

#### USB Camera Topic

- Install a usb camera package in order use a webcam as camera

	   sudo apt-get install ros-melodic-usb-cam

- Put the arm_control package inside the src folder of your catkin_ws and catkin_make
  **Note: This package contains the camera feedback nodes**

- Open the *cam_launch.launch* file in *arm_control/launch* directory and change the value in the following line to your usb camera device id:
		<-param name="video_device" value="/dev/video2"->
    
- Then roslaunch it to run the camera image node using the following command:
      
      roslaunch arm_control cam_launch.launch

#### Camera Caliberation
- Take 30 pictures of the checkerboard pattern in different orientation.
- Open *camera_cal.py* and change **line 23** to the directory you saved the checkerboard images.
- Then run *camera_cal.py* to generate your camera matrices



## Running

### Marker Tracking

#### USB Camera Topic

- Roslaunch it to run the camera image node using the following command:
      
      roslaunch arm_control cam_launch.launch
      
#### Trajectory and PID

2. Run the file **generate_path.py** to create the trajectory files. This will generate two files, *path.txt* and *velocity.npy*. 
  - path.txt : Contains the trajectory waypoints 
  - velocity.npy : Contains the velocity of trajectory needed for derivative setpoint of PID
  
3. To run the PID controller simulation 
- Simply run the file PID_test.py in
  
  
