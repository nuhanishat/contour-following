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

#### Aruco Markers
- Generate Markers from here: 
	https://chev.me/arucogen/
- Edit the file *marker_tracking.py* so that the marker size and dictionary variables are set to the properties of your marker. 
- Print out 2 markers, one for the fixed robot base and another for the end-effector


## Running

The general algorithm looks like:
- Get current position from marker tracking
- Feed it to the PID controller
- Command the output velocity as twist command

***Please note that the inidividual components (marker tracking, PID and commanding arm using kinova_msgs) have been tested and are working, but the PID controller needs more work (issues with saturation and limited arm publish rate) before implementation in real world so they haven't been integrated yet due to time limitations***

### Marker Tracking

#### USB Camera Topic

- Roslaunch it to run the camera image node using the following command:
      
      roslaunch arm_control cam_launch.launch
      
#### Trajectory and PID

- Run the file **generate_path.py** to create the trajectory files. This will generate two files, *path.txt* and *velocity.npy*. 
  - path.txt : Contains the trajectory waypoints 
  - velocity.npy : Contains the velocity of trajectory needed for derivative setpoint of PID
  
-  To run the PID controller simulation 
	- Simply run the file **PID_test.py***
	**Note: This is written in python3 unlike the rest of the project to utilize the turtle gui**

- *KinovaControl.py* contains the methods that can be used to command position and velocity. They can take inputs from the PID controller. You can only run this on the real arm because the kinova_gazebo simulation doesn't support it (It has no jacobian solver so you can only command joint position/velocity but not cartesian). 

#### Marker Tracking

- Run the file **marker_tracking.py** to test the marker tracking. This utilizes the aruco marker library but the pose estimation is very noisy so I have utilized this and added my own code to do pixel tracking instead. 

- Use the following command. Make sure the usb camera node is running already (as mentioned earlier)

		rosrun arm_tracking marker_tracking.py


  
  
