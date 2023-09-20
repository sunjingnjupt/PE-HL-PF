# PE-HL-PF
 Dynamic Vehicular Pose Estimation  with Heuristic L-Shape Fitting and Grid-based Particle Filter
### The example vedio for the vehicle pose estimation can be found on the [Youtube](https://www.youtube.com/watch?v=FMX1Pya6qwg).
This reposetary contains the implement detais abount the paper" Dynamic Vehicular Pose Estimation  with Heuristic L-Shape Fitting and Grid-based Particle Filter".
## Introduction



### The following image denotes the illustration of the process for the geometric shape classifier. (a) shows the processing of endpoints extraction and width calculation, (b) shows the step of cluster classifying according to extracted contours, the solid blue points in each segment represent contours. (c) shows a real-world traffic scene on the road, there are various clusters on it, the cluster in black rectangle represents a symmetrical-like cluster.
<div align=center><img width="99%" height="100%" alt="Geometric classifier" src="https://github.com/sunjingnjupt/PE-CHL-v2/blob/main/image/classifier.jpg"/></div>

### platform and thirdparty
*platform*: ubuntu14.04 or higher ubuntu version., CMakeList 2.8.3 or higher.
*third party*:  
[Qt5.6: ](https://download.qt.io/new_archive/qt/5.6/5.6.3/)  you should change [CMakeList.txt](https://github.com/sunjingnjupt/PE-HL-PF/blob/main/CMakeLists.txt) content Qt5_DIR to Qt install dir.
[OPENCV3： ](https://pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/), you should change [CMakeList.txt](https://github.com/sunjingnjupt/PE-HL-PF/blob/main/CMakeLists.txt) content OpenCV_DIR to Opencv install dir.
[QGLVIEWER](https://github.com/GillesDebunne/libQGLViewer) is a powerful and high-performance point cloud image visualization
```
sudo apt-get install libqglviewer-dev libboost-all-dev libeigen3-dev freeglut3-dev libgl1-mesa-dev libgtest-dev2
```
when all dependencies are met, type this in root code dir， you will get a bin exec file to run.
```
mkdir build && cd build && cmake ..
```
### datasets
we used kitti dataset, example data can be done here. [link](https://www.cvlibs.net/datasets/kitti/raw_data.php?type=road)， Register and download the data for this section *2011_10_03_drive_0047 (3.3 GB)*.  
change data to this format  
dataset/velodyne  
dataset/image_2  
dataset should change [param.h](https://github.com/sunjingnjupt/PE-HL-PF/blob/main/src/groundRemove/include/param.h) kitti_velo_dir, this var will be use in [mainwindow.cpp => onOpenFolderToRead](https://github.com/sunjingnjupt/PE-HL-PF/blob/main/src/qt/widgets/mainwindow.cpp)


