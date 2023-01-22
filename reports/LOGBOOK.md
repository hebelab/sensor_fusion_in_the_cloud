# Sensor Fusion in the Cloud: Logbook

---

*Author: Ahmet Yusuf Şirin*

*Student Number: 21947641*

---

This logbook documents the progress and milestones of our sensor fusion project for a legged robot, which aims to improve the robot's perception and localization capabilities using a combination of stereo camera and LiDAR sensors. The project is being carried out using a combination of Github, MATLAB, Python, and ROS programming environments, and we are working closely with our supervisor and a team of fellow students to achieve our goals. The logbook entries include details on our research, experimentation, and implementation efforts, as well as any challenges and successes we encounter along the way. Our ultimate goal is to develop a robust sensor fusion algorithm that can be integrated into the robot's control system to enhance its performance in real-world environments. The methodology is to research and experiment on various sensor fusion algorithms and eventually implement the most suitable one for our project and also we are keeping a logbook for the progress of our project and documenting our every step.

## Summer Study
The below logs contain the works that we study on until the fall semester.

### 2022.06.01
* Project proposal and concept development
* Discussed project idea with supervisor and received approval
* Formed project team and assigned roles and responsibilities
* Began literature review on sensor fusion and legged robot systems

### 2022.06.15
* Reviewed various sensor technologies and selected ZED stereo camera and Velodyne 3D LiDAR for the further usage

### 2022.07.15
* Began working on sensor fusion algorithm using a complementary filter and input signals matched to keep the sensor fusion simple
* Worked on generating alpha generation function to calculate alpha value for each pixel
* Began testing the algorithm on collected sensor data


## Fall-Term Study
The below logs contain the works that we study on the fall term.

### 2022.08.01
* Continued talking on design and the sensor fusion algorithm in theory
* Kept a logbook and discussed progress and issues with team members and supervisor regularly

### 2022.09.01
* Conducted thorough testing and evaluation of the Unitree system
* Prepared report and presentation on the project progress and results

### 2022.09.21
* First meeting 
* Robot Review 
* Initial objective is assigned to learning robot, SDK’s API’s and robot controls. 

### 2022.10.08
* Researching internal structure of the robot 

    ![Interconnection of Unitree Computers](static\Go1CommFram_E.png "Interconnection of Unitree Computers")

    * STM32 – Driving joint motors 
    * Raspberry Pi – unitree_legged_sdk (robot controls) 
    * Raspberry pi Wireless Module – Connecting directly to Raspberry Pi wirelessly 
    * Nvidia Jetson Nano 1 – unitree_camera_sdk (Head Cameras) 
    * Nvidia Jetson Nano 2 – unitree_camera_sdk (Side Cameras) 
    * Nvidia Jetson Nano (Main) – unitree_camera_sdk (Bottom Camera) 
    * Ethernet Switch – Connecting internal devices together. It has a port to connect from outside 
* Robot is hanged to development stand. 
* Robot should be in idle mode to hang! 
* We failed to connect to robot using ethernet port. 
* We connected to the robot using wifi module connected to the Raspberry Pi 

### 2022.10.10
* We write program using unitree_legged_sdk to control robot. 
* high_cmd object is used in unitree_legged_sdk 
* for detailed information check unitree_legged_sdk/include/comm.h 
* keyboard_controller.py 
* speed adjustment 
* back forward left and right motion 
* yaw turn 

### 2022.10.12
We try to get access on the Unitree robot SDKs but we cannot because of lack of documentation.

* Attempted to access Unitree robot SDKs but encountered issues due to lack of documentation
* Research on alternative methods to access and utilize sensor data

### 2022.10.15
* Investigated various online resources such as Stack Overflow and GitHub for solutions to streaming video with Gstreamer and OpenCV
* Attempted to utilize Gstreamer tools for accessing sensor data

We research the required tools on the Internet. The below hyperlinks will be helpful for us:

| Topic | Hyperlink |
| ----- | -------- |
| Gstreamer stream is not working with OpenCV | https://stackoverflow.com/questions/70753630/gstreamer-stream-is-not-working-with-opencv |
| Streaming Gstreamer video to a different address using OpenCV with Python | https://stackoverflow.com/questions/71820489/streaming-gstreamer-video-to-a-different-address-using-opencv-with-python |
| How to find the device index of external camera for Gstreamer | https://stackoverflow.com/questions/53940019/how-to-find-the-device-index-of-external-camera-for-gstreamer |
| TheImagingSource/tiscamera#436 | https://github.com/TheImagingSource/tiscamera/issues/436 |
| OpenCV_GStreamer | https://github.com/simondlevy/OpenCV_GStreamer |
| Cannot open OpenCV videocapture with Gstreamer pipeline | https://forums.developer.nvidia.com/t/cannot-open-opencv-videocapture-with-gstreamer-pipeline/181639 |
| Gstreamer tools | https://gstreamer.freedesktop.org/documentation/tutorials/basic/gstreamer-tools.html?gi-language=c |
| Gstreamer tutorials | https://gstreamer.freedesktop.org/documentation/tutorials/index.html?gi-language=c |

### 2022.10.22
* Unitree_camera_sdk 
* Point cloud server app is using the cameras 
* We are able to get the image from cameras 
* Rectangular Image 
* Depth Frame 
* Depth Video 

![Unitree Supersensory: Under Cam](static\super_sensory_down_cam.jpeg "Unitree Supersensory: Under Cam")
![Stereo Depth Frame](static\StereoDeptFrame.png "Stereo Depth Frame")

### 2022.10.26
* Successfully established communication between the legged robot and ROS
* Began collecting sensor data from a fixed position for further analysis
* There is only a number of sensors available. We also cannot find the <ros2udp_motion_mode_adv.cpp> file that is developed but not opened for us.

---
**NOTE**

.ros/log/8ded59d2-3e3b-11ec-934f-e45f0153901d/rosout.log:4:1636118632.074689849 INFO /ros2udp_motion_mode_adv [/home/pi/Unitree/autostart/utrack/catkin_utrack/src/utrack/a2_ros2udp_adv/src/ros2udp_motion_mode_adv.cpp:1546(main)] [topics: /rosout] [ROS2UDP] Node ros2udp has started.

---

* We tried to get Inertial Measurement Unit data from robot using unitree_legged_sdk with python 
* All data are empty or filled with zeros. 
* We tried to get the data using ROS but the result was same. ROS messages under desired topics were filled with zeros. 

### 2022.11.01
* Making the priliminary design for the interim report and talking about making test also in Extended Kalman Filter and another methods
* Discussed and decided on using a complementary filter for sensor fusion algorithm
* Began research on edge preservation method as an aid to increase sensor fusion performance

### 2022.11.03
* A straight walking script is implemented using unitree_legged_sdk and python to perform inertial measurements and position estimation 

![Straigth Walk Test](static\straigth_walk.png "Straigth Walk Test")

### 2022.11.25
* Talked about the reports and the project to the supervisor and peers
* Received feedback about our plans and suggestions for further improvements and future work
* Started talking on data collection from the selected sensors in different formats
* Began experimenting with different methods to match the sampled data from the two sensors such as dynamic Papoulis–Gerchberg algorithm and linear interpolation
* Experienced with different data rates and rotation speeds of the lidar to optimize resolution and data rates

### 2022.12.09
* General Project meeting 
* Next objective – Sensor fusion – Stereo Camera data & Lidar Data 
* A high frequency 3D LiDAR with enhanced measurement density via Papoulis-Gerchberg 

### 2022.12.10
We collect the sensor data (for Lidar and Stereo camera) from a fixed position and we try to similarize their domain and sampling numbers to work on them. we use Jupyter Notebook for this purposes.

* Continued collection and manipulation of sensor data from Lidar and stereo camera
* Utilized Jupyter Notebook for data analysis and pre-processing
* Began talking the actual development of sensor fusion algorithm using complementary filter and edge preservation methods
* Continuously updating the progress and results in the logbook.
* Research about point cloud data types 
* Collecting data for sensor fusion from both ZED and Lidar. We collected data on both sensors with different qualities to be able to compare the fusion performance

    * Getting 3D lidar Data from Velodyne VLP16 lidar
    * Observing data type 

    ![Lidar Selfie](static\lidar_selfie.png "Lidar Selfie")

    * Getting Depth frame form ZED stereo camera 
    * Observing data type 

### 2022.12.12
* Second interim report. 
* Control Algorithm for sensor fusion 

![Alpha Function](static\AlphaFunction.jpeg "Alpha Function")
![Complementary Filter](static\CompFiltBD.jpeg "Complementary Filter")

### 2022.12.14
* Inertial Measurement data, odometry data, range of obstacle data provided from both cameras and ultrasonic distance sensors were obtained from robot using unitree_legged_sdk and C++ 
* For detailed information, please look at unitree_legged_sdk/include/comm.h “high_cmd” class. 

### 2022.12.15
* Obtaining both lidar and stereo camera point clouds in the same frame. We use the Python language especially the Jupyter Notbook environment for implementations
* The data that we will make the fusion is prepared roughly

![Data Preprocessing](static\image_reduction.jpeg "Data Preprocessing")

### 2022.12.20
* Developed a pipeline to upsample the lidar data using dynamic Papoulis-Gerchberg algorithm and convert it to * cartesian form
* Integration of stereo camera and lidar data using the complementary filter and edge preservation methods
* Testing the sensor fusion algorithm with sample data and evaluating its performance

### 2023.01.05
* Further optimization of the sensor fusion algorithm by adjusting parameters and implementing additional methods
* Testing the algorithm on the legged robot and gathering experimental results
* Preparing for final term report and report submission.

### 2023.01.21
* Completing the final exams and rearranging the methods that we have discussed till now
* The developed algorithm on the MATLAB is used for the real collected data. Firstly i tried for sample data that is randomly generated. This was not enogh to compare the results end determining the performance

### 2023.01.22
* Writing the final term report
* Hard studying on hands-on coding for the preliminary design (this might not be fast enough to be ready for the first term)


## Spring-Term
Comming soon...