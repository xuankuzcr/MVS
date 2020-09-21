# MVS 

## A driver for all kinds of MVS cameras based on ROS operation system. 

The MVS industrial camera's parameters can be adjusted directly by the camera.yaml file in MVS/config/. <br>
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
