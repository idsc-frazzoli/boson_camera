FLIR Boson ROS Wrapper [deprecated see zauron-interface]
=============================
ROS Wrapper camera interface for the FLIR Boson 640 being used for the Zauron. This is an initial work-in-progress project.
This repository provides minimal functionalities to stream and republish over ROS the infrared camera [FLIR Boson 640](https://www.flir.com/products/boson/).

Getting started
------------------
0. Install requirements: [catkin_simple](https://github.com/catkin/catkin_simple) and [OpenCV](https://www.opencv.org/)
1. Build the package with `catkin build boson_camera`
2. Connect the FLIR Boson 640 camera.

   Make sure that your system has read access to the device, which should be listed as `/dev/ttyACM0`
   
   If not you can give access to the device by invoking `sudo chmod a+rwx /dev/ttyACM0` command
3. Try to find the unique ID that your Boson 640 has by using these commands:
   ```$xslt
   $ cd /dev/v4l/by-id
   $ ls
   ```
   Your device ID should look somewhat like this:
   ```$xslt
   /dev/v4l/by-id/usb-FLIR_Boson_XXXXX-video-index 
   ```
   Update the launch file accordingly
4. Run the code `roslaunch boson_camera boson640.launch`

#### Examples

<img src="resources/images/ex_1.png" width="300"> <img src="resources/images/ex_2.png" width="300">

-----------------------------
Boson SDK Documentation
-----------------------------
[Boson SDK Documentation](https://drive.google.com/open?id=1fuXUIu_wzB4zuVmTPbtUhoiKg0WnqEHm)

-----------------------------
Contribution
-----------------------------
Contributions, enhancement and everything else is welcome, open issues, open pull requests, or simply contact us.
