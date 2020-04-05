# IMLS-SLAM

这是一个基于IMLS-ICP原理实现的2D的激光slam的帧间匹配算法.

IMLS-SLAM的论文位于src/papers文件下,简述了该算法的原理.

## 求解大致如下过程:

1. bag包存放的是激光和里程计的数据, bag包的下载[地址](https://pan.baidu.com/s/1HrvG9WegHcT41T_RodpEjA ) 提取码:nrqk   请将其存放在     

   imlsMatcherProject/src/bag包下面

2. imlsDebug类中会读取bag包中的数据, 每当读取到激光的数据后,将会调用回调函数, 在回调函数中将会进行激光的帧间匹配,激光的帧间匹配的结果将会发布到rviz中进行显示.  里程计的回调函数只是发布的路径这里并没有利用里程计的数据进行激光雷达运动畸变的去除.因此效果没有那么好.

3. 激光回调函数中回调用IMLSICPMatcher类的Mather方法进行帧间匹配得到位姿增量. Matcher方法在求解的时候使用点线ICP方法, 该方法的原理位于src/papers中An_ICP_variant_using_a_point-to-line_metric这篇论文中.

   ## 程序编译过程

   程序主要依赖Eigen库, CSM库, PCL库,OpenCV库

   程序依赖于ros系统运行, 如果你的ros系统是kinetic版本请将CMakeLists中所有的ros版本换成kinetic

   Eigen库安装:   sudo apt-get install libeigen3-dev

   CSM库: sudo apt-get install ros-kinetic-csm  // 同时将CMakeLists中的所有melodic替换成kinetic

   注: 如果使用melodic版本的csm并没有发布, 需要使用源码进行编译

```c
git clonehttps://github.com/AndreaCensi/csm.git
cd csm
sudo apt-get install libgsl-dev
sudo ./install_quickstart.sh
```

即可完成安装, 需要注意一下安装结束的头文件和库文件的路径, 然后将CMakeLists中关于csm库的换成这个路径

​	PCL库安装, 参考[博客](https://blog.csdn.net/qq_40022890/article/details/100786946)

​	OpenCV库的安装, [参考](https://github.com/zhoupengwei/SLAM/blob/master/OpenCV4.0%E5%AD%A6%E4%B9%A0%E7%AC%94%E8%AE%B0.md) 

​		安装一个消息包, 这个消息包需要将里面的ros版本换成你的ros系统版本, 如果是kinetic版本忽略

​		cd 	imlsMatcherProject/src/champion_nav_msgs

​		sudo bash install.sh

​		至此, 可以返回工作空间,进行编译

​	

```
cd  imlsMatcherProject
catkin_make
// 打开另外两个终端, 一个运行roscore, 一个运行rviz进行可视化
// 编译完成以后
source devel/setup.bash
rosrun imlsMatcher imlsMatcher_node
```

在rviz中选择add, 窗口选择by topic 添加两条path, 一条激光帧间匹配得到的, 一个是里程计的path

topic名称: imls_path_pub_, odom_path_pub_

注意: rviz中fix frame请输入odom,  所有发布的path都是以里程计作为参考系

如果顺利的话, 你将看到如下结果	

![Image](https://github.com/zhoupengwei/SLAM/blob/master/img-folder/imls-icp.png)







