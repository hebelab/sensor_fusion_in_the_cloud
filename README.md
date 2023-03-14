# Sensor Fusion in the Cloud
A workspace for all codes, reports, and further files of our graduate project.

## Development

### Jupyter Notebook

Please clear output products before committiong and pushing files.

### DVC

`Pipfile` at the root directory is reserved for the DVC tool. To install required packages:

```bash
pipenv update
```

For the first setup of the DVC in this project I follow the below steps.

```bash
dvc init
dvc remote add --default gdrive://1xw2gfQyT5gwDVAv3DRXnSHcHIkuVPe2W
dvc add data/image/ data/lidar/ data/ZED/
git add data/.gitignore data/ZED.dvc data/image.dvc data/lidar.dvc
```

For updating the `data` directory only you have to do is:

```bash
dvc add data/image/ data/lidar/ data/ZED/
git add data/.gitignore data/ZED.dvc data/image.dvc data/lidar.dvc
```

For pushing/pulling:

```bash
dvc push
dvc pull
```

## Objectives 
The purpose of the graduation project is for the student to have a major design experience based on the skills and knowledge acquired in earlier course work, with multiple realistic constraints. In this first course in the two-course sequence, the student starts a project that involves designing a system, component, or process to meet desired needs. At the end of this course, the student develops possible solutions and identifies design choices and/or design parameters.

## Project description
This project aims to develop new sensor fusion algorithms in the cloud via wireless communication. Sensor fusion can be defined as integrating signals from multiple sources to get a unique representation of the environment. This allows more spatial coverage, robustness, and improved resolution. Our goal is to implement and compare state-of-the-art sensor fusion techniques, such as Kalman filtering and Bayesian networks, to work in the cloud via wireless transmission of the sensor data to a central server. We will analyze the impact of communication bandwidth and delays on the sensor fusion performance.

The project will involve the following main tasks:

1. Implementing sensor fusion algorithms to combine 3D LiDAR and ZED stereo camera.
2. Testing the algorithms for state estimation on a four-legged mobile robot that we have in the lab.
3. Comparing the performance of different sensor fusion algorithms.
4. Evaluating the impact of communication bandwidth and delay on the state estimation performance of the robot.

## Fields of background research needed to complete the project
The project involves sensor fusion, software development, and adaptive Kalman filtering. Thus, students should be willing to learn programming languages, such as C/C++, Python, and Robot Operating System during the project in addition to their knowledge from the regular course load of the first three years.

Design constraints and the requirements of the project

1. The sensor fusion algorithm should work at least at 10 Hz, but preferably at 25Hz.
2. We need to test at least three sensor fusion algorithms on the actual robot.
3. An effective and easy-to-operate GUI needs to be designed for visualization of the results.
4. The weight of the sensor system, which includes the 3D LiDAR and Stereo Camera and peripheral of them, must be less than 5 kg because of the payload limit of the robots.

## Different possible solutions to the problem
(Extended) Kalman filters, adaptive Kalman filters, and dynamic Bayesian networks could be preferred for implementation. The ROS platform allows programming in C, C++, and Python, so the students can prefer the one they are most comfortable with. However, we encourage C++. Virtual delays and bandwidth limitations could be added to test different scenarios.

## Initial prototype required
In the initial prototype, the LiDAR and Stereo Camera should be placed on our Unitree Go1 Edu robot for testing. The robot should walk through a 25-meter-long corridor and should be able to do state estimation by combining sensor data in the cloud.

Possible iterations and tests on this prototype

1. Data transfer protocols can be improved to better utilize the available communication bandwidth.
2. The robot could be tested outside for state estimation and mapping by sensor fusion.
3. Students can use different wireless communication technologies to test their performance, such as WiFi, LTE, and bluetooth.
