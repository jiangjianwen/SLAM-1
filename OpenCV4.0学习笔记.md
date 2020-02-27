# OpenCV4.0学习笔记

## 1.安装指导

ubuntu18.04 安装OpenCV

1. 首先安装所需要的依赖项

   sudo apt-get install build-essential libgtk2.0-dev libvtk5-dev libjpeg-dev libtiff4-dev libjasper-dev libopenexr-dev libtbb-dev

2. 下载linux安装包进入[网址](https://opencv.org/downloads.html)点击4.0以上版本的Sources进行下载

3. 进入下载好的安装包所在目录,

   打开终端

   解压: tar -zxvf opencv-4.0.0.zip

   进入解压后的目录: cd opencv-4.0.0

   新建文件夹:mkdir build

   编译:cd build

   ​          cmake ..

   ​			make -j4

   ​		   sudo make install

4. 测试是否安装成功

   mkdir opencv-test

   cd opencv-test

   gedit DisplayImage.cpp

   编辑如下代码:

   ```c++
   #include <stdio.h>  
   #include <opencv2/opencv.hpp>  
   using namespace cv;  
   int main(int argc, char** argv )  
   {  
       if ( argc != 2 )  
       {  
           printf("usage: DisplayImage.out <Image_Path>\n");  
           return -1;  
       }  
       Mat image;  
       image = imread( argv[1], IMREAD_COLOR );  
       if ( !image.data )  
       {  
           printf("No image data \n");  
           return -1;  
       }  
       namedWindow("Display Image", WINDOW_AUTOSIZE );  
       imshow("Display Image", image);  
       waitKey(0);  
       return 0;  
   }  
   ```

   创建Cmake编译文件

   gedit CMakeLists.txt

   输入如下内容:

   ```cmake
   cmake_minimum_required(VERSION 2.8) 
   
   project( DisplayImage ) 
   
   find_package( OpenCV REQUIRED ) 
   
   add_executable( DisplayImage DisplayImage.cpp ) 
   
   target_link_libraries( DisplayImage ${OpenCV_LIBS} )
   ```

   编译

   cmake .

   make

   运行

   此时opencv-test文件夹中已经产生了可执行文件DisplayImage, 随便找一张图片放在这个文件中用于演示,以假设图像名为opencv.png.运行

   ./DisplayImage opencv.png

   结果

   将会在屏幕上面显示刚刚放在目录下面的图像,表示使用成功.

   ![Image](https://github.com/zhoupengwei/SLAM/blob/master/img-folder/opencv.png)

## 2.常用API

### CalcOpticalFlowPyrLK()函数

该函数CalcOpticalFlowPyrLK()主要利用Lucas-Kanade光流实现图像之间的追踪

由于避免计算描述子和描述子的匹配因此该光流法可以回避掉计算和匹配描述子所带来的时间,而光流本身的计算要小于描述子的计算与匹配

函数接口如下:

```c++
void cv::calcOpticalFlowPyrLK	(	
InputArray 	prevImg,
InputArray 	nextImg,
InputArray 	prevPts,
InputOutputArray 	nextPts,
OutputArray 	status,
OutputArray 	err,
Size 	winSize = Size(21, 21),
int 	maxLevel = 3,
TermCriteria 	criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
int 	flags = 0,
double 	minEigThreshold = 1e-4 
)		
```

参数:

·prevImg: buildOpticalFlowPyramid构造的第一个8位输入图像或者金字塔.

·nextImg: 与prevImg相同大小或者相同类型的第二个输入图像或者金字塔

·prevPts: 需要找到流的2D点的矢量(vector of points for which the flow needs to be found;); 点坐标必须是单精度浮点数.

·nextPts: 输出二维点的矢量(具有单精度浮点坐标),包含第二个图像中输入特征的计算新位置,当传递OPTFLOW_USE_INITIAL_FLOW标志时，向量必须与输入中的大小相同.

·status ：输出状态向量（无符号字符）;如果找到相应特征的流，则向量的每个元素设置为1，否则设置为0.

·err ：输出错误的矢量; 向量的每个元素都设置为相应特征的错误，错误度量的类型可以在flags参数中设置; 如果未找到流，则未定义错误（使用status参数查找此类情况).

·winSize: 每个金字塔等级的搜索窗口的winSize大小

·maxLevel: 基于0的最大金字塔等级数;如果设置为0,则不使用金字塔(单级), 如果设置为1,则使用两个级别,以此类推;如果将金字塔传递给输入,那么算法将使用与金字塔一样多的级别,但是不超过maxLevel.

·criteria: 指定迭代搜索算法的终止条件(在指定的最大迭代次数criteria.maxCount之后或者当搜索窗口移动小于criteria.epsilon时).

·flags: 操作标志:

​		1.OPTFLOW_USE_INITIAL_FLOW使用初始统计,存储在nextPts中; 如果未设置标志,则将prevPts复制nextPts		并将其视为估计

​		2.OPTFLOW_LK_GET_MIN_EIGENVALS使用最小特征值作为误差测量(参见minEigThreshold描述);如果没有		设置标志,则将原稿周围的色块和移动点之间的L1距离除以窗口的像素数,作为误差测量

·minEigThreshold: 算法计算光流方程的2X2正常矩阵的最小特征值,除以窗口中的像素数;如果此值小于minEigThreshold,则过滤掉相应的功能功能并且不处理其流程,因此它允许删除坏点并获得性能提升

函数使用:

使用LK的目的是跟踪特征点.从第一幅图像中提取FAST角点.然后使用LK光流来跟踪它们.

源码

