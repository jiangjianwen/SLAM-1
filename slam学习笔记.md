# SLAM算法学习笔记

## 视觉slam

### 第一部分数学基础

在slam问题的求解中,经常会遇到一些矩阵的求解,尤其以对矩阵的SVD分解最为常见.

[参考博客](https://blog.csdn.net/billbliss/article/details/78579308)

李群常用于表达相机的位姿变换,由于旋转矩阵对与加法不封闭,导致在优化相机位姿时常常无法求导,因此主要的办法是对李代数进行求导,然后通过指数映射到录李群所对应的流行中.高博写了几篇关于这部分的数学推导.

[博客](https://www.cnblogs.com/gaoxiang12/p/5137454.html#4500852)



g2o优化库

g2o是在SLAM邻域中广为使用的图优化库,主要完成对视觉里程计的优化和后端优化,g2o源码存放于github中.

[链接](https://github.com/RainerKuemmerle/g2o)

相关几篇参考博文如下:

[高博](https://www.cnblogs.com/gaoxiang12/p/5304272.html)

g20编程步骤

![Image](https://github.com/zhoupengwei/SLAM/blob/master/img-folder/g2o.png)

高博十四讲g2o[拟合曲线源码](https://github.com/gaoxiang12/slambook2/blob/master/ch6/g2oCurveFitting.cpp)

下面是学习的时候自己添加的一些注释方便以后查看

```c++
using Block =  g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > ;  // 每个误差项优化变量维度为3，误差值维度为1

// 第1步：创建一个线性求解器LinearSolver
Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); 

// 第2步：创建BlockSolver。并用上面定义的线性求解器初始化
Block* solver_ptr = new Block( linearSolver );      

// 第3步：创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );

// 第4步：创建终极大boss 稀疏优化器（SparseOptimizer）
g2o::SparseOptimizer optimizer;     // 图模型
optimizer.setAlgorithm( solver );   // 设置求解器
optimizer.setVerbose( true );       // 打开调试输出

// 第5步：定义图的顶点和边。并添加到SparseOptimizer中
CurveFittingVertex* v = new CurveFittingVertex(); //往图中增加顶点
v->setEstimate( Eigen::Vector3d(0,0,0) );
v->setId(0);
optimizer.addVertex( v );
for ( int i=0; i<N; i++ )    // 往图中增加边
{
  CurveFittingEdge* edge = new CurveFittingEdge( x_data[i] );
  edge->setId(i);
  edge->setVertex( 0, v );                // 设置连接的顶点
  edge->setMeasurement( y_data[i] );      // 观测数值
  edge->setInformation( Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma) ); // 信息矩阵：协方差矩阵之逆
  optimizer.addEdge( edge );
}

// 第6步：设置优化参数，开始执行优化
optimizer.initializeOptimization();
optimizer.optimize(100);
```

综上,总结出编写g2o进行优化的主要步骤如下:

1.  创建一个线性求解器LinearSolver

   增量方程的形式:

   
   $$
   H△X=-b
   $$
   (这部分数学知识参考)[博客](https://www.cnblogs.com/gaoxiang12/p/5244828.html)

2. 创建BlockSolver. 并且使用上面定义的线性求解器进行初始化

3. 创建总求解器solver, 并从GN, LM, DogLeg 中选一个, 再用上述块求解器BlockSolver初始化

4. 创建总求解器稀疏优化器(SparseOptimizer), 并用已定义求解器作为求解方法

5. 定义图的顶点和边. 并添加到SparseOptimizer中

6. 设置优化参数,开始执行优化.

## 激光slam























