# Leg-KILO: Robust Kinematic-Inertial-Lidar Odometry for Dynamic Legged Robots

**Currently, this version is only tested in the legkilo-dataset; a lighter and more accurate version, Leg-KILO 2.0, will be released before 2025.**

<p align='center'>
    <img src="https://github.com/ouguangjun/Leg-KILO/blob/main/figure/overview0305.jpg" alt="drawing" width="600"/>
</p>

**Abstract** This paper presents  a robust multi-sensor fusion  framework, Leg-KILO (Kinematic-Inertial-Lidar Odometry). When lidar-based SLAM is applied to legged robots, high-dynamic motion (e.g., trot gait) introduces frequent foot impacts, leading to IMU degradation and lidar motion distortion. Direct use of IMU measurements can cause significant drift, especially in the z-axis direction. To address these limitations,  we tightly couple leg odometry, lidar odometry, and loop closure module based on graph optimization. For leg odometry, we propose a kinematic-inertial odometry using an on-manifold error-state Kalman filter, which incorporates the constraints from our proposed contact height detection to reduce height fluctuations. For lidar odometry,  we present an adaptive scan slicing and splicing method to alleviate the effects of high-dynamic motion. We further propose a robot-centric incremental mapping system that enhances map maintenance efficiency. Extensive experiments are conducted in both indoor and outdoor environments, showing that Leg-KILO has lower drift performance compared to other state-of-the-art lidar-based methods, especially during high-dynamic motion. To benefit the legged robot community, a lidar-inertial dataset containing leg kinematic data and the code  are released.

<p align='center'>
    <img src="https://github.com/ouguangjun/kilo-dataset/blob/main/figure/map_dog.jpg" alt="drawing" width="500"/>
</p>

<p align='center'>
    <img src="https://github.com/ouguangjun/kilo-dataset/blob/main/figure/dog01.jpg" alt="drawing" width="200"/>
    <img src="https://github.com/ouguangjun/kilo-dataset/blob/main/figure/dog02.jpg" alt="drawing" width="200"/>
</p>

# News
- **`2024.07.31`:** The code is released.
- **`2024.07.20`:** The paper is accepted by RA-L 2024!

# Dataset
Related datasets have been released in [link](https://github.com/ouguangjun/legkilo-dataset)

# Video
The related video can be watched on [Youtube](https://youtu.be/6O74De5BLeQ). 

<a href="[https://youtu.be/HyLNq-98LRo](https://youtu.be/6O74De5BLeQ)" target="_blank"><img src="https://github.com/ouguangjun/Leg-KILO/blob/main/figure/youtube.png" 
alt="leg-kilo" width="500"  /></a>


# Prerequisites
Currently our code is tested on 

- Ubuntu 18.04
- ROS melodic
- gtsam 4.0.3
- pcl 1.8
- opencv 3.2
- [unitree_legged_msgs](https://github.com/unitreerobotics/unitree_ros_to_real) (has included in the project)

You can refer to the configuration process of [LIOSAM](https://github.com/TixiaoShan/LIO-SAM).

We provide the ubuntu18.04 installation process, other versions may require some changes. We recommend that you install the ros desktop version, as this contains most of the required dependencies.

Ros

```
apt-get update
sudo apt-get install ros-melodic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Gtsam

```
sudo apt-get install software-properties-common
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

# Run

```
cd ~/legkilo_ws/src
git clone https://github.com/ouguangjun/Leg-KILO.git
cd ..
catkin_make
```

Download the dataset from [link](https://github.com/ouguangjun/legkilo-dataset)

```
source devel/setup.bash
roslaunch legkilo run.launch
rosbag play xxxx.bag
```


# Acknowledgments

Our project is developed on [LIOSAM](https://github.com/TixiaoShan/LIO-SAM) and retains the part of lidar optimization and loop closure detection , thanks very much to the authors for their excellent open source work. And also thanks to  [fast lio](https://github.com/hku-mars/FAST_LIO) for the ikd-tree,   and [A1-QP-MPC-Controller](https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller).

# License

The code is under BSD 3-Clause License, the same as [LIOSAM](https://github.com/TixiaoShan/LIO-SAM). 

# Todo list

- [ ] With Docker configuration
- [ ] Test on other ubuntu version
- [ ] The matlab code for evaluation
- [ ] Test on other open source legged robot's datasets
