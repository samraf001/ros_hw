# Robotic_HW
## Context
The design of our cells in Machina Labs has evolved over the past years. Currently, each of our cells has two articulated industrial robots on rails (a total of 7 axes) and a frame with hydraulic clamps. For the parts to form correctly, we must exert and maintain a dynamic force during the forming in a very accurate location in space. Currently, each robot is equipped with a load cell. See a quick video about our process [here](https://www.youtube.com/watch?v=iqYMprTEXRI). We are using ROS2 to collect the data from the network and control the robots in real-time. As a robotic engineer, we keep developing different modules for our network to add features to the system.  
 
## Objective
The goal of This project is to build a ROS2 network that collects data from 3-DOF sensors and makes the filtered data available as a ROS service and topic. Since we cannot send a real sensor to all of our applicants, we made a simple simulator (sensor.py) that mimics the behavior of a real sensor but with random data. 
- The first task is to make a custom service for 3-DOF sensor 
- The second task is to make a ROS2 service server that continuously reads data from the sensor and has the latest filter data available for the client service that you make. 
- Finally, please make a simple client that calls two of these services and publishes them to a topic at 500Hz. Please keep in mind that your service servers can run slower than 500Hz. 
- You can define a second server in the simulator to modify the code and run two at the same time.
- You can check the example.py to see how to make calls to the sensor

## Grading Criteria
- We’re looking for code that is clean, readable, performant, and maintainable.
- The developer must think through the process of deploying and using the solution and provide the necessary documentation. 
- The sensor samples with 2000Hz, and you can request a specific number of samples in each call. Each call also has a ~1ms delay on top of the sampling time. We would like to hear your thought on picking the number of samples that you read in each call. 

## Submission
To submit the assignment, do the following:

1. Navigate to GitHub's project import page: [https://github.com/new/import](https://github.com/new/import)

2. In the box titled "Your old repository's clone URL", paste the homework repository's link: [https://github.com/Machina-Labs/robotic_hw](https://github.com/Machina-Labs/robotic_hw)

3. In the box titled "Repository Name", add a name for your local homework (ex. `Robotic_soln`)

4. Set the privacy level to "Public", then click "Begin Import" button at bottom of the page.

5. Develop your homework solution in the cloned repository and push it to GitHub when you're done. Extra points for good Git hygiene.

6. Send us the link to your repository.

## ROS2
Install instructions (Ubuntu): [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

ROS2 tutorials: [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)


**Solution**

**ROS 2 3-DOF Sensor Data Pipeline**

Overview
This ROS 2 project facilitates the acquisition, processing, and publication of 3-DOF sensor data. It consists of two primary packages:

ros_hw_interfaces: Defines the custom service interface ThreeDOF.srv.

ros_hw: Implements the service server (service_server.py) and the data client (data_client.py).

The system is designed to request raw and cleaned sensor data via TCP, process it, and publish the results on ROS 2 topics.

**Features**
Service Server: Connects to a TCP sensor, retrieves data, processes it, and serves it via ROS 2 services.

Data Client: Periodically requests data from the service server and publishes it to ROS 2 topics.

Custom Service Interface: ThreeDOF.srv defines the request and response structure for data acquisition.

**Prerequisites**

Operating System: Ubuntu 22.04

ROS 2 Distribution: Humble Hawksbill

Python: 3.10 or higher

Packages:

rclpy

std_msgs

numpy

Ensure that the ROS 2 environment is properly sourced:

bash
Copy
Edit
source /opt/ros/humble/setup.bash
Installation

**1. Create a ROS 2 Workspace:**

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

**Clone the Repositories:**

git clone <repository_url_for_ros_hw_interfaces>
git clone <repository_url_for_ros_hw>

**2. Build the Workspace:**

cd ~/ros2_ws
colcon build

**3. Source the Workspace:**

source install/setup.bash

**Usage**


Running the Service Server

Start the service server to connect to the TCP sensor and provide data services:

`ros2 run ros_hw service_server`

Running the Data Client

The data client requests data from the service server and publishes it to ROS 2 topics. You can specify the publishing frequency using the --ros-args parameter:


`ros2 run ros_hw data_client --ros-args -p publish_frequency:=10.0`

Replace 10.0 with your desired frequency in Hz.

**Monitoring Published Topics**

To monitor the published data:

`ros2 topic echo /raw_data`

`ros2 topic echo /cleaned_data`

**Adjusting Parameters at Runtime**

You can adjust the publishing frequency at runtime using:

`ros2 param set /data_client publish_frequency 20.0`

**Custom Service Interface: ThreeDOF.srv**

Located in ros_hw_interfaces/srv/ThreeDOF.srv, this service interface is defined as:

int32 num_samples
---
float32[] x
float32[] y
float32[] z

Request: Number of samples to retrieve.

Response: Arrays of x, y, and z axis data.

**System Architecture**

The system comprises:

Service Server (service_server.py):

Connects to the TCP sensor.

Retrieves and processes raw data.

Provides two services:

get_raw_3dof_data

get_cleaned_3dof_data

Data Client (data_client.py):

Periodically requests data from the service server.

Publishes raw and cleaned data to:

/raw_data

/cleaned_data

**Performance Considerations**

Sampling Rate: The sensor samples at 2000 Hz.

Service Call Delay: Each service call introduces approximately 1 ms delay.

Optimal Sample Size: To balance performance and data freshness, requesting around 10 samples per call is recommended.

