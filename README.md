**arena_camera**

arena_camera node publishes image data collected from Lucid Vision Labs Triton GigE cameras in /lucid_vision/camera_"X"
/image_raw topic as a ROS message(
sensor_msgs/Image). This node can connect to multiple camera devices discovered by Lucid Vision Labs' ArenaSDK. In order
to use this node, you need to install Lucid Vision Labs' ArenaSDK.

### Installation
This camera driver has been tested on ROS2 Galactic(Ubuntu 20.04) and ROS2 Humble(Ubuntu 22.04).

1. Download ArenaSDK from [here](https://thinklucid.com/downloads-hub/).
2. Install ArenaSDK.
3. Before connecting to the camera you need to set your IP address, according to the documentation of Triton Cameras your IP
   address can be set to 169.254.0.1.

4. Clone this driver to <your_workspace>/src/.

5. Check the Serial number of devices and change the serial number values in parameter files.
   (Serial number writes on the box of the camera) The serial number is different for each product.

   If you have only one camera work on this param file :
   <your_ws>/src/arena_camera/param/lucid_vision_camera.param.yml If you have more than one camera work on this param
   file :
   <your_ws>/src/arena_camera/param/multi_camera.param.yml

6. Build your code with the following command.

   `colcon build `

7. Source the directory and run the executable with following command.

   `ros2 launch arena_camera test_node_container.launch.py`

   7.1 You can check whether data is flowing or not and what is the rate of it with following commands.

   `ros2 topic hz /lucid_vision/<your_camera>/image_raw`

8. Open another terminal and run Rviz2 with following command.
   `rviz2`

   8.1 Add the /lucid_vision/<your_camera>/image_raw topic to Display panel. With following "Add">"By topic" section.

   8.2 Set your "Fixed Frame" as "lucid_vision".

   8.3 Set your "Reliability Policy" to "Best Efford".  (Best effort works in UDP, Reliable works in TCP/IP)

   8.4 Be sure that the visibility button is checked.

### Camera Settings
Camera settings can be made in two ways;

1. Use default device settings.
   1. Change camera settings with LUCID's ArenaView on a Windows computer.
   2. Load settings to the camera device.
   3. Run this driver using `use_default_device_settings` parameter as a `true`.

2. Change camera setting with rqt_reconfigure.

   1. Open the terminal and run rqt_reconfigure with the following command.
   
      `ros2 run rqt_reconfigure rqt_reconfigure`

   2. Change your camera settings with rqt_reconfigure GUI. Choose your camera from the list and change your settings.
      Choose desired exposure, gain and gamma values. You can also change the FPS of the image.
      (You can change your settings with ROS2 parameters too. You can find the parameters in the param file.)

   3. Dump your camera settings with the following command.
   
      `ros2 param dump /arena_camera_node --output-dir <your_workspace>/src/arena_camera/param/`
      
      Run the camera node with the new param file.
      `ros2 run arena_camera arena_camera_node_exe --ros-args --params-file <your_workspace>/src/arena_camera/param/arena_camera_node.yaml`