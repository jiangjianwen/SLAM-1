# SLAM算法学习笔记

## 视觉slam

### 第一部分数学基础

[在slam问题的求解中,经常会遇到一些矩阵的求解,尤其以对矩阵的SVD分解最为常见,参考博客](https://blog.csdn.net/billbliss/article/details/78579308)



[李群常用于表达相机的位姿变换,由于旋转矩阵对与加法不封闭,导致在优化相机位姿时常常无法求导,因此主要的办法是对李代数进行求导,然后通过指数映射到录李群所对应的流行中.高博写了几篇关于这部分的数学博客](https://www.cnblogs.com/gaoxiang12/p/5137454.html#4500852)



g2o优化库

g2o是在SLAM邻域中广为使用的图优化库,主要完成对视觉里程计的优化和后端优化,

[g2o源码存放于github中,下面是链接](https://github.com/RainerKuemmerle/g2o)

相关几篇参考博文如下:

[高博:](https://www.cnblogs.com/gaoxiang12/p/5304272.html)



## 激光slam























