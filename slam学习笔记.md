# SLAM算法学习笔记

## 视觉slam

### 第一部分数学基础

在slam问题的求解中,经常会遇到一些矩阵的求解,尤其以对矩阵的SVD分解最为常见.

[参考博客](https://blog.csdn.net/billbliss/article/details/78579308)

李群常用于表达相机的位姿变换,由于旋转矩阵对与加法不封闭,导致在优化相机位姿时常常无法求导,因此主要的办法是对李代数进行求导,然后通过指数映射到录李群所对应的流行中.高博写了几篇关于这部分的数学推导.

[博客](https://www.cnblogs.com/gaoxiang12/p/5137454.html#4500852)

贝叶斯定理参考[博客](https://blog.csdn.net/jiangjiang_jian/article/details/81346797)

#### g2o优化库

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

创建一个线性求解器LinearSolver

增量方程的形式:


$$
H△X=-b
$$
(这部分数学知识参考)[博客](https://www.cnblogs.com/gaoxiang12/p/5244828.html)

1. 创建BlockSolver. 并且使用上面定义的线性求解器进行初始化
2. 创建总求解器solver, 并从GN, LM, DogLeg 中选一个, 再用上述块求解器BlockSolver初始化
3. 创建总求解器稀疏优化器(SparseOptimizer), 并用已定义求解器作为求解方法
4. 定义图的顶点和边. 并添加到SparseOptimizer中
5. 设置优化参数,开始执行优化.



##### g2o提供的顶点

(1). 李代数的位姿

```c++
class VertexSE3Expmap : public BaseVertex<6, SE3Quat>
```

继承于BaseVertex这个模版类

需要设置的模版参数:

参数6: SE3Quat类型为6维, 三维旋转, 三维平移

参数SE3Quat: 该类型旋转在前, 平移在后, 注意: 类型内部使用的其实是四元数,不是李代数

该顶点需要设置的参数:

```c++
g2o::VertexSE3Example * vSE3 = new g2o::VertexSE3Expmap();
// 1. 设置待优化位姿 (这里是粗略位姿)
vSE3->setEstimate(converter::toSE3Quat(pKFi->GetPose()));
// 2.设置id号
vSE3->setid(pKFi->mnId);
// 3.设置是否固定, 第一帧固定
vSE3->setFixed(pKFi->mnId==0);
```

(2). 空间点的位置

```c++
class VertexSBAPointXYZ : public BaseVertex<3, Vector3d>
```

该顶点需要设置的参数:

```c++
g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
//【1】设置待优化空间点3D位置XYZ
vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
//【2】设置Id号
vPoint->setId(id);
//【3】是否边缘化（以便稀疏化求解）
vPoint->setMarginalized(true);
```



##### g2o提供的边edge

(1). Point-Pose二元边(Point_xyz-SE3边)

既要优化MapPoints的位置, 又要优化相机的位姿

```c++
class EdgeSE3PojectXYZ : public BaseBinaryEdge<2, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>
```

继承于BaseBinaryEdge这个二元边模版类

需要设置的模版参数:

参数2: 观测值(这里是3D点在像素坐标系下的投影坐标)的维度

参数Vector: 观测值类型, piex.x, piexl.y

参数VertexSBAPointXYZ: 第一个顶点类型

参数VertexSE3Expmap: 第二个顶点类型

该边需要设置的参数:

```c++
g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

//【1】设置第一个顶点，注意该顶点类型与模板参数第一个顶点类型对应
e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
//【2】设置第二个顶点
e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
//【3】设置观测值，类型与模板参数对应
e->setMeasurement(obs);
const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
//【4】设置信息矩阵，协方差
e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

//【5】设置鲁棒核函数
g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
e->setRobustKernel(rk);
rk->setDelta(thHuberMono);

//【6】设置相机内参
e->fx = pKFi->fx;
e->fy = pKFi->fy;
e->cx = pKFi->cx;
e->cy = pKFi->cy;
```

(2). Pose一元边(SE3)

仅优化相机位姿,为了构造出投影方程. 需要按下面的方式把MapPoints的位置作为常量加入

```c++
class EdgeSE3ProjectXYZOnlyPose : public BaseUnaryEdge<2, Vector2d, VertexSE3Expmap>
```

该类继承于BaseUnaryEdge这是一个一元边模版类, 需要设置的模版参数加上

该边需要设置的参数:

```c++
g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

// 注意这里只设置一个顶点，其它一样
e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
e->setMeasurement(obs);
const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
e->setRobustKernel(rk);
rk->setDelta(deltaMono); /** @attention 设置阈值，卡方自由度为2，内点概率95%对应的临界值*/

e->fx = pFrame->fx;
e->fy = pFrame->fy;
e->cx = pFrame->cx;
e->cy = pFrame->cy;

/** @attention 需要在这里设置<不做优化>的MapPoints的位置*/
cv::Mat Xw = pMP->GetWorldPos();
e->Xw[0] = Xw.at<float>(0);
e->Xw[1] = Xw.at<float>(1);
e->Xw[2] = Xw.at<float>(2);
```

(3). Pose-Pose二元边(SE3-SE3边)

优化变量是相机相邻两个关键帧位姿, 约束来自对这两个关键帧位姿变换的测量(里程计, IMU等)

```c++
class G2O_TYPES_SBA_API EdgeSE3Expmap ; public BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap>
```

需要设置的参数如下:

```c++
Se2 measure_se2 = pMsrOdo->se2;
// 【1】里程计测量的协方差
g2o::Matrix3D covariance = toEigenMatrixXd(pMsrOdo->info).inverse(); 

// 【2】将里程计测量转换成g2o::SE3Quat类型
Eigen::AngleAxisd rotz(measure_se2.theta, Eigen::Vector3d::UnitZ());
g2o::SE3Quat relativePose_SE3Quat(rotz.toRotationMatrix(), Eigen::Vector3d(measure_se2.x, measure_se2.y, 0));

// 【3】将`里程计测量协方差`转换为`相机坐标系下协方差`
// 注意：g2o::SE3Quat是旋转在前，平移在后
g2o::Matrix6d covariance_6d = g2o::Matrix6d::Identity();
covariance_6d(0,0) = covariance(2,2);
covariance_6d(0,4) = covariance(2,0); covariance_6d(0,5) = covariance(2,1);
covariance_6d(4,0) = covariance(0,2); covariance_6d(5,0) = covariance(1,2);
covariance_6d(3,3) = covariance(0,0);
covariance_6d(4,4) = covariance(1,1);
covariance_6d(1,1) = 0.00001;
covariance_6d(2,2) = 0.01;
covariance_6d(5,5) = 0.0001;

g2o::Matrix6d Info = g2o::Matrix6d::Identity();
Info = covariance_6d.inverse();

// 【4】构造边
g2o::EdgeOnlineCalibration* e = new g2o::EdgeOnlineCalibration;
e->vertices()[0] = optimizer.vertex(id0);
e->vertices()[1] = optimizer.vertex(id1);
e->setMeasurement(relativePose_SE3Quat);
e->setInformation(Info);
optimizer.addEdge(e);
```

上面的比较麻烦的协方差的计算，准确的协方差计算可以参考论文《Odometry-Vision-Based Ground Vehicle Motion Estimation With SE(2)-Constrained SE(3) Poses》中的处理.



##### g2o优化

```c++
//【1】指定pose维度为6, landmark维度为3
typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;
//【2】线性方程求解器，使用CSparse分解
//    还有g2o::LinearSolverDense，使用cholesky分解
Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); 
//【3】矩阵块求解器
Block* solver_ptr = new Block ( linearSolver );     
//【4】梯度下降方法，从GN, LM, DogLeg 中选
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
// 图模型
g2o::SparseOptimizer optimizer;
//【5】设置求解器
optimizer.setAlgorithm ( solver );
......
......
......
//【6】打开调试输出
optimizer.setVerbose ( true );
optimizer.initializeOptimization();
//【7】指定迭代次数:100次
optimizer.optimize ( 100 );
```

g2o中经常使用的BlockSolver_6_3, BlockSolver_7_3是以下重定义:

```c ++
  // solver for BA/3D SLAM
  typedef BlockSolver< BlockSolverTraits<6, 3> > BlockSolver_6_3;  
  // solver fo BA with scale
  typedef BlockSolver< BlockSolverTraits<7, 3> > BlockSolver_7_3; 

```

检查outliner

优化完成后, 对每一条边都进行检查,剔除误差较大的边(认为是错误的边), 并设置为setLevel为0, 即下次不再对该边进行优化,第二次完成后, 会对连接误差比较大的边,在关键帧中剔除对该MapPoint的观测, 在MapPoint中剔除对该关键帧的观测,具体实现可以参考orb-slam源码Optimizer::LocalBundleAdjustment

```c++
optimizer.optimize ( 100 );
// 优化完成后，进行Edge的检查
for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
{
    g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];

    if(pMP->isBad())
        continue;

    // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
    // 第二个判断条件，用于检查构成该边的MapPoint在该相机坐标系下的深度是否为正？
    if(e->chi2()>5.991 || !e->isDepthPositive())
    {
        e->setLevel(1);// 不优化
    }

    // 因为剔除了错误的边，所以下次优化不再使用核函数
    e->setRobustKernel(0);
}
```

加入对相机内参的优化

```c++
g2o::CameraParameters* camera = new g2o::CameraParameters (
    K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
);
// 设置Id号为0即可
camera->setId ( 0 );
optimizer.addParameter ( camera );
```

实践自定义PoseGraph进行优化



### 第二部分 SLAM系统

#### 前端

#### 后端

#### 1.滤波器后端

后端(Backend)

​		从带噪声的数据估计内在状态---状态估计问题

​		Estimated the inner state from noisy data

渐进式(Incremental)      

​		a.保持当前状态的估计, 在加入新信息时, 更新已有的估计(滤波)

​		b.线性系统+高斯噪声=卡尔曼滤波器

​		c.非线性系统+高斯噪声+线性近似 = 扩展卡尔曼

​		d.非线性系统+非高斯噪声+非参数化=粒子滤波器(slam中很少使用,所需要的粒子的数量是指数级增长的)

​		e.Sliding window fiter & mutiple stats Kalman(MSCKF) 滑动窗口

批量式(Batch)

​		给定一定规模的数据, 计算该数据下的最优估计(优化)



高斯分布的线性变换





## 激光slam























