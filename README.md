# 自己写点儿注释
> 2022/5/12 

## 与ORB-SLAM2的一些区别
> 虽然代码相对ORB2改动比较大，但是整体还是能看出来是ORB-SLAM2的框架
> - 线程方面是，取消了回环检测重定位的部分，`tracking`与`localmap`线程改为了`tracking`和`localtion`线程
>     - `tracking`对应论文中的前端部分
>     - `location`对应论文中的后端部分
> - 添加了先验地图加载与keyframe的feature做数据关联，并加入优化中
>   - 数据关联入口在设置frame为keyframe的部分
>   - 优化项的添加主要在location中，本质上还都是些重投影误差 
## 注释
> 主要对照这作者的这篇论文加了些注释 H. Huang, H. Ye, Y. Sun and M. Liu, "GMMLoc: Structure Consistent Visual Localization With Gaussian Mixture Models," in IEEE Robotics and Automation Letters, vol. 5, no. 4, pp. 5043-5050, Oct. 2020, doi: 10.1109/LRA.2020.3005130.
> 找到的部分用`// * 对应论文 xxxx `做了注释可以直接搜索
- 优化项的构建
- GMM的数据关联
- ~~没找到其中【公式1】和之前的概率公式的实现部分，可能主要是概念呢吧~~
## ~~吐槽~~
~~下面这两个玩意不是一模一样吗?!~~
- ~~[/gmmloc/src/modules/localization_opt.cpp](/gmmloc/src/modules/localization_opt.cpp)~~
- ~~[/gmmloc/gmmloc/src/modules/localization_gmm.cpp](/gmmloc/gmmloc/src/modules/localization_gmm.cpp)~~

# GMMLoc

[![Build Status](https://travis-ci.org/HyHuang1995/gmmloc.svg?branch=master)](https://travis-ci.org/github/HyHuang1995/gmmloc)
[![LICENSE](https://img.shields.io/badge/license-GPL%20(%3E%3D%202)-informational)](https://github.com/HyHuang1995/gmmloc/blob/master/LICENSE)

Dense Map Based Visual Localization. [[project]](https://sites.google.com/view/gmmloc/)

## Paper and Video

Related publication:
```latex
@article{huang2020gmmloc,
  title={GMMLoc: Structure Consistent Visual Localization with Gaussian Mixture Models},
  author={Huang, Huaiyang and Ye, Haoyang and Sun, Yuxiang and Liu, Ming},
  journal={IEEE Robotics and Automation Letters},
  volume={5},
  number={4},
  pages={5043--5050},
  year={2020},
  publisher={IEEE}
}
```

Demo videos:

<a href="https://www.youtube.com/watch?v=Ul4-H33uwx4" target="_blank"><img src="https://www.ram-lab.com/image/gmmloc_v103.gif" alt="v103" height="240" border="10" style="margin-right:10em"/></a>
<a href="https://www.youtube.com/watch?v=Ul4-H33uwx4" target="_blank"><img src="https://www.ram-lab.com/image/hyhuang_iros2020_cover.png" 
alt="gmmloc" height="240" border="10" /></a>

## Prerequisites

We have tested this library in Ubuntu 18.04. Prerequisites for installation:

1. [ROS](http://wiki.ros.org/melodic/Installation) (melodic)

2. [OpenCV3](https://docs.opencv.org/3.4.11/d7/d9f/tutorial_linux_install.html)
```
apt-get install libopencv-dev
```
3. miscs:
```
apt-get install python-wstool python-catkin-tools 
```
4. [evo](https://github.com/MichaelGrupp/evo) (optional)
```
pip install evo --upgrade --no-binary evo
```

## Installation
Initialize a workspace:

```
mkdir -p /EXAMPLE/CATKIN/WORK_SPACE
cd /EXAMPLE/CATKIN/WORK_SPACE

mkdir src
catkin init
catkin config --extend /opt/ros/melodic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
```

Clone the code:
```
cd src
git clone git@github.com:hyhuang1995/gmmloc.git
```

If using SSH keys for github, prepare the dependencies via:
```
wstool init . ./gmmloc/gmmloc_ssh.rosinstall
wstool update
```

or using https instead:
```
wstool init . ./gmmloc/gmmloc_https.rosinstall
wstool update
```

上面的依赖安装不好使用，那个开源没提供`gmmloc_https.rosinstall`都是`.gmmloc_https.rosinstall`的隐藏文件，所以自己复制一份不是隐藏文件的形式。


Compile with:
```
catkin build gmmloc_ros
```

## Running Examples
We provide examples on EuRoC Vicon Room sequences. For example, to run the demo on V1_03_difficult:

1. Download the [sequence](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) (ASL Format)

2. Replace the [**/PATH/TO/EUROC/DATASET/**](https://github.com/HyHuang1995/gmmloc/blob/770eadc99229eff17f2f613e969e4e9c10499496/gmmloc_ros/launch/v1.launch#L25) in [v1.launch](https://github.com/HyHuang1995/gmmloc/blob/master/gmmloc_ros/launch/v1.launch) with where the sequence is decompressed:
```
<param name="data_path" value="/PATH/TO/EUROC/DATASET/$(arg seq)/mav0/" />
```

3. Launch:
```
roslaunch v1.launch seq:=V1_03_difficult
```

## Evaluation
If evo is installed, we provide script for evaluating on Vicon Room sequences.
```
roscd gmmloc_ros
./scripts/evaluate_euroc.sh
```
and the results would be saved to **gmmloc_ros/expr**.
By default, we follow the evaluation protocol of [DSO](https://vision.in.tum.de/research/vslam/dso) to perform evaluation without multi-threading. If you would like to run the script in online mode, uncomment [this line](https://github.com/HyHuang1995/gmmloc/blob/770eadc99229eff17f2f613e969e4e9c10499496/gmmloc_ros/scripts/evaluate_euroc.sh#L60) in the script:
```
rosparam set /gmmloc/online True
```

## Credits

Our implementation is built on top of [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2), we thank Raul et al. for their great work.
