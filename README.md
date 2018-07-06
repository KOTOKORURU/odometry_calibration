# odometry_calibration
using direct linear method to calibrate the odometry estimation, by sloving A^T *A x = A^T *b by Cholesky decompose

usage step: 
1.mkdir odom_ws
cd ./odom_ws

2.mkdir src
mv  <calib_odom>(the file path of the "calib_odom") ./

cd ..

3.catkin_init_workspace

4.source ./devel/setup.bash

5.catkin_make

6.goto the launch path and execute roslaunch  calib_odom odomCalib.launch

7.then uncompress the bag file

8.rosrun rviz rivz 
set the fix_frame as odom and add three tpic:odom_path_pub,calib_path_pub,scan_path_pub,and change the color whatever you like

9.goto the odom_ws/bag,uncompress the bag file
and execute: rosbag play --clock odom.bag

10.and you will see two  trajectory(one from odometry,one from scan)

11.after the bag go to the end
you can open a terminal and public an topic:rostopic pub /calib_flag std_msgs/Empty "{}"

the calibrated trajectory will gained as well the calibration matrix

final result:
![image](https://github.com/KOTOKORURU/odometry_calibration/blob/master/IMG_0024.JPG)
