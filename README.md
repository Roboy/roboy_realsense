# roboy_realsense
This repo uses the real sense as a webcam to detect aruco markers. The pose of the detected markers is published. Additionally you can stream the camera and visualize the marker pose in rviz.
# dependencies
install librealsense from official repo, make sure the camera is working
```
sudo apt-get install libudev-dev pkg-config libgtk-3-dev
git clone https://github.com/IntelRealSense/librealsense
cd librealsense && mkdir build && cd build && cmake .. && sudo make install -j5 && cd .. && sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && udevadm trigger
```
```
#!/bin/bash 
# install librealsense ros package
sudo apt install ros-kinetic-librealsense
# install aruco detect
sudo apt install ros-kinectic-aruco-detect

```
### marker ids
use [this](http://terpconnect.umd.edu/~jwelsh12/enes100/markergen.html) page for marker id generation. You might have to adjust the marker size in the header.
