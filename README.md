# float32multiarray_to_pointcloud2

Description
=================================

This package aims to convert data from type std_msgs/Float32MultiArray to sensor_msgs/PointCloud2. This conversion is necessary so that it is possible to collect the Point Cloud of the CoppeliaSim simulator behind the ROS Interface 1 and view it on rviz. The message comes from the simulator in the format std_msgs/Float32MultiArray and is then converted by this package to sensor_msgs/PointCloud2.

File layout
--------------------------

* coppelia_sim_scene: Contains the scene to be performed in the CoppeliaSim simulator;
* rviz_config: Contains Rviz configuration to display PointCloud data;
* launch: Contains the launch ROS to run the package.
* src: Contains source code

How to use
-------------------------

To use the package, compile and then run the launch, as follows.


```
$ cd ~/catkin_ws/src
$ git clone https://github.com/marco-teixeira float32multiarray_to_pointcloud2.git
$ cd ~/catkin_ws
$ catkin_make
```

After compiling, run

```
$ roslaunch float32multiarray_to_pointcloud2 float32multiarray_to_pointcloud2.launch
```

"float32multiarray_to_pointcloud2.launch" parameters
------------------------


1. ** frame_id **
   - Type: String
   - Default value: "/camera"
   - Description: Frame_id to which the output topic should be registered 
   
   
2. ** input_topic **
   - Type: String
   - Default value: "/Float32MultiArray_in"
   - Description: Input topic name, must be of type "std_msgs/Float32MultiArray"
   
3. ** output_topic **
   - Type: String
   - Default value: "/cloud_out"
   - Description: Output topic name, after conversion. It will be of type "sensor_msgs/PointCloud2"

4. ** near_clip **
   - Type: double
   - Default value: "0.001"
   - Description: Minimum actuation distance of the sensor. Must be equal to the distance configured in the simulator
   
5. ** far_clip **
   - Type: double
   - Default value: "3.5"
   - Description: Maximum operating distance of the sensor. Must be equal to the distance configured in the simulator


6. ** view_angle **
   - Type: double
   - Default value: "90"
   - Description: The sensor opening angle must be the same configured in the simulator

7. ** height **
   - Type: int
   - Default value: "480"
   - Description: Point cloud height configured in the simulator
   
8. ** width **
   - Type: int
   - Default value: "640"
   - Description: Width of the point cloud configured in the simulator
   
   
Configuration of the system used
--------------------------------

* Ubuntu 18.04.4 (http://releases.ubuntu.com/18.04.4/)
* ROS Melodic (http://wiki.ros.org/melodic/Installation/Ubuntu)
* CoppeliaSim simulator V4_1_0 (https://www.coppeliarobotics.com/downloads)

![alt text](https://github.com/marco-teixeira/figures/blob/master/fig_float32multiarray_to_pointcloud2/fig1.png)
![alt text](https://github.com/marco-teixeira/figures/blob/master/fig_float32multiarray_to_pointcloud2/fig2.png)
![alt text](https://github.com/marco-teixeira/figures/blob/master/fig_float32multiarray_to_pointcloud2/fig3.png)
![alt text](https://github.com/marco-teixeira/figures/blob/master/fig_float32multiarray_to_pointcloud2/fig4.png)
