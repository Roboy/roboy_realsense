# roboy_realsense
This repo uses the real sense as a webcam to detect aruco markers. The pose of the detected markers is published. Additionally you can stream the camera and visualize the marker pose in rviz.
# dependencies
install librealsense from official repo, make sure the camera is working
```
#!/bin/bash 
# install librealsense ros package
sudo apt install ros-kinetic-librealsense
# install aruco detect
sudo apt install ros-kinectic-aruco-detect

```
