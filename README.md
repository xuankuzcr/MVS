# MVS 

## A driver for all kinds of MVS cameras based on ROS operation system. 

### Dependence installation
```
tar zxvf MVS-1.0.0_x86_64.tar.gz
chmod +x setup.sh
./setup.sh
```
### How to build and run
```
cd
mkdir -p catkin_ws/src
cp ~/MVS ~/catkin_ws/src/
cd catkin_ws && catkin_make -j4
roslaunch MVS MSV.launch

```
### Visualization
```
rosrun image_view image_view image:=/image/raw
```
### Parameters
The MVS industrial camera's parameters can be adjusted directly by the **camera.yaml** file in **MVS/config/**. <br>
**All parameter explanations are shown below:**

* width: 700            `图像宽度` 
* height: 500           `图像高度`
* FrameRateEnable: true `帧率使能`
* FrameRate: 30         `帧率`
* ExposureTime: 10000   `曝光时间`
* GammaEnable: true     `伽马使能`
* Gamma: 0.7            `伽马因子`
* Gain: 5               `亮度5`
* SaturationEnable: true`饱和度使能`
* Saturation: 128       `饱和度`
* offset_x: 50          `图像原点x方向偏置(在不同分辨率下需要对应修改offset，为了使物体在图像正中间)`
* offset_y: 50          `图像原点y方向偏置`

### Read from camera or video
In image_fromation.cpp, ifdef **CAMERA_READ**, we will use true camera directly by usb. <br>
ifdef **VIDEO_READ**, the video is avaliable to be read. You can set the option `video_path` in camera.yaml.

