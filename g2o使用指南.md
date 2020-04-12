# g2o编程手册

g2o是在SLAM邻域中广为使用的图优化库,主要完成对视觉里程计的优化和后端优化,g2o源码存放于github中.

[链接](https://github.com/RainerKuemmerle/g2o)

## 1.安装指南

g2o是一个cmake工程, 因此首先需要安装一些依赖项

```bash
sudo apt-get update
sudo apt-get install qt5-qmake qt5-default libqglviewer-dev-qt5 libsuitesparse-dev libcxsparse3 libcholmod3
git clone https://github.com/RainerKuemmerle/g2o.git
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

完成安装之后g20的头文件将位于/usr/local/g2o下面, 库文件位于/usr/local/lib

## 2.g2o框架

![Image](https://github.com/zhoupengwei/SLAM/blob/master/img-folder/g2o_class.png)

上面这张图, 是g2o的类结构框架图, 整个图的核心是SparseOptimizer, 它是一个optimizableGraph,

从而也是一个超图

超图包含了许多顶点(HyperGraph::Vertex)和边(HyperGraph::Edge). 继承自BaseVertex, 也就是

OptimizableGraph::Vertex,边可以继承自BaseUnaryEdge（单边）, BaseBinaryEdge（双边）或

BaseMultiEdge（多边），都被称为OptimizableGraph::Edge



整个图的核心SparseOptimizer 包含一个优化算法（OptimizationAlgorithm）的对象。

OptimizationAlgorithm是通过OptimizationWithHessian 来实现的。

其中迭代策略可以从Gauss-Newton（高斯  牛顿法，简称GN）, Levernberg-Marquardt（简称LM法）, Powell's dogleg 三者中间选择一个（常用的是GN和LM）.



OptimizationWithHessian 内部包含一个求解器（Solver），这个Solver实际是由一个BlockSolver组成的。

这个BlockSolver有两个部分，一个是SparseBlockMatrix ，用于计算稀疏的雅可比和Hessian矩阵；

一个是线性方程的求解器（LinearSolver），它用于计算迭代过程中最关键的一步HΔx=−b，

LinearSolver有几种方法可以选择：PCG, CSparse, Choldmod，



编程步骤如下: 

![Image](https://github.com/zhoupengwei/SLAM/blob/master/img-folder/g2o.png)

第一步:  创建一个线性求解器LinearSolver

```c++
LinearSolverCholmod      // 使用sparse cholesky分解法。继承自LinearSolverCCS
LinearSolverCSparse        // 使用CSparse法。继承自LinearSolverCCS
LinearSolverPCG                 // 使用preconditioned conjugate gradient 法，继承自LinearSolver
LinearSolverDense             //  使用dense cholesky分解法。继承自LinearSolver
LinearSolverEigen              //   依赖项只有eigen，使用eigen中sparse Cholesky 求解
```

第二步: 创建BlockSolver. 并且使用上面的线性求解器进行初始化

```c++
using BlockSolverPL = BlockSolver< BlockSolverTraits<p, l> >;
// p表示pose的维度必须是流形manifold下的最小表示, 1表示landmark的维度
```

第三步: 创建总求解器solver. 并从GN, LM, DogLeg中选一个, 在用上述块求解器BlockSolver初始化

```c++
// 三种方法任意选择其中一种
g2o::OptimizationAlgorithmGaussNewton;
g2o::OptimizationAlgorithmLevenberg ;
g2o::OptimizationAlgorithmDogleg ;
```

第四步:  创建最终的稀疏优化器(SparseOptimizer), 并用已定义求解器作为求解方法

​				创建稀疏优化器

```c++
				g2o::SparseOptimizer    optimizer;
```

​				使用前面定义好的求解器作为求解方法:

```c++
				SparseOptimizer::setAlgorithm(OptimizationAlgorithm* algorithm)
```

​				其中的setVerbose是设置优化过程输出信息用的

```c++
				SparseOptimizer::setVerbose(bool verbose)
```

第五步: 定义图中的顶点和边

第六步: 设置优化参数, 开始执行优化

​				初始化:

```c++
			SparseOptimizer::initializeOptimization(HyperGraph::EdgeSet& eset)
```

​				设置迭代次数, 然后就开始执行图优化了

```c++
		SparseOptimizer::optimize(int iterations, bool online)
```

下面是高博视觉slam十四讲中使用g2o作曲线拟合的例子

```c++
#include <iostream>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>
// 曲线方程: y = exp(ax^2+bx+c) + w     其中w是高斯分布的噪声
// 曲线模型的顶点, 模版参数: 优化变量维度和数据类型
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 重置
    void setToOriginImpl() override  {
        _estimate << 0, 0, 0;
    }
    // 更新
    void oplusImpl(const double * update) override{
        _estimate += Eigen::Vector3d(update);
    }
    // 存盘和读盘: 留空
    bool read(std::istream & in) override {}
    bool write(std::ostream & out) const override {}
};

// 误差模型 模版参数: 观测值维度, 类型, 连接顶点类型
// BaseUnaryEdge是一个一元边即自己连接自己
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>{
public:
    double _x;  // x值, y值为_measurement
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}
    // 计算曲线模型误差
    virtual void computeError() override{
        const CurveFittingVertex * v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - std::exp(abc(0, 0)*_x*_x + abc(1, 0)*_x + abc(2, 0));
    }
    // 计算雅克比矩阵
    virtual void linearizeOplus() override {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex*>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        double y = std::exp(abc[0]*_x*_x + abc[1]*_x + abc[2]);
        _jacobianOplusXi[0] = -_x * _x * y;
        _jacobianOplusXi[1] = -_x * y;
        _jacobianOplusXi[2] = -y;
    }

    virtual bool read(std::istream& in){}
    virtual bool write(std::ostream& out) const {}
};

int main() {
    double ar = 1.0, br = 2.0, cr = 1.0;    // 真实数据值
    double ae = 2.0, be = -1.0, ce = 5.0;   // 估计参数值
    int N = 100;                            // 数据点
    double wSigma = 1.0;                    // 噪声Sigma值
    double invSigma = 1.0 / wSigma;
    cv::RNG rng;                            // OpenCV随机数产生器

    std::vector<double> xData, yData;       // 数据
    for (int i = 0; i < N; i++){
        double x = i / 100.0;
        xData.push_back(x);
        yData.push_back(std::exp(ar*x*x + br*x + cr) + rng.gaussian(wSigma*wSigma));
    }

    // 构建图优化, 先设定g2o
    // 声明误差项类型和求解器类型
    using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>; // 每个误差项优化变量维度为3, 误差值维度为1
    using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>; // 线性求解器类型

    // 梯度下降法, 可以从GN, LM, Dogleg中选
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
            g2o::make_unique<BlockSolverType >(g2o::make_unique<LinearSolverType >())
            );
    // 创建一个稀疏优化器(SparseOptimizer)
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);     // 设置求解器
    optimizer.setVerbose(true);  // 打开调试输出
    // 接下来向图中添加顶点和边
    // 向图中增加顶点
    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(ae, be, ce));  // 设置估计值
    v->setId(0);   // 顶点的id标识, 方便后面和边连接
    optimizer.addVertex(v);

    // 往图中增加边
    for (int i = 0; i < N; i++){
        CurveFittingEdge *edge = new CurveFittingEdge(xData[i]);  // 每次创建一条边, 边的预测值就是前面的xData
        edge->setId(i);
        edge->setVertex(0, v);         // 设置连接的顶点 由于这仅仅是一个一元边, 只连接了一个顶点因此第一个参数永远是0
        edge->setMeasurement(yData[i]);  // 观测数据
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (wSigma * wSigma));  // 信息矩阵: 协方差矩阵之逆
        optimizer.addEdge(edge);

    }

    // 执行优化
    std::cout << "start optimization " << std::endl;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "solve time cost = " << time_used.count() << " seconds. " << std::endl;
    // 输出优化值
    Eigen::Vector3d abcEstimate = v->estimate();
    std::cout << "estimated model: " << abcEstimate.transpose() << std::endl;

    std::cout << "Hello, World!" << std::endl;
    return 0;

```



## 3.g2o顶点

g20的顶点最重要的就是类结构的顶点图, 如下图所示:

![Image](https://github.com/zhoupengwei/SLAM/blob/master/img-folder/g2oindex.png)

HyperGraph::Vertex是一个抽象的类, 必须通过派生来使用

OptimizableGraph继承自HyperGraph, 但是也是一个非常底层的类

最重要的顶点类是BaseVertex, 位于g2o/core/base_vertex.h中

```c++
* Templatized BaseVertex
 * D  : minimal dimension of the vertex, e.g., 3 for rotation in 3D
 * T  : internal type to represent the estimate, e.g., Quaternion for rotation in 3D
 */
  template <int D, typename T>
  class BaseVertex : public OptimizableGraph::Vertex;  
```

D是int类型的, 表示vertex的最小维度, 例如3D空间中旋转是3维的. 对应D就是3

T表示待估计的vertex数据类型, 比如用如果使用四元数表示旋转, 那就是Quaternion类型

D和T声明如下:

```c++
// D并非是顶点(状态变量)的维度, 而是其在流行空间(manifold)的最小表示
static const int Dimension = D;           ///< dimension of the estimate (minimal) in the manifold space
// T表示顶点(状态变量的类型)
typedef T EstimateType;
EstimateType _estimate;
```

不同的场景将会有使用不同的顶点类型, g2o提供了常用的顶点类型以供使用

```c++
VertexSE2 : public BaseVertex<3, SE2>;  // 2维空间位姿(x, y, Θ)
VertexSE3 : public BaseVertex<6, Isometry3>;   // 3维空间位姿(x,y,z,qx,qy,qz)
VertexPointXY : public BaseVertex<2, Vector2>;
VertexPointXYZ : public BaseVertex<3, Vector3>;

VertexSE3Expmap : public BaseVertex<6,  SE3Quat>;
```

当上面的顶点不能满足使用时, 可以考虑通过继承的方式重新定义顶点, 一般需要考虑重写以下函数:

```c++
virtual bool read(std::istream& is);  // 读取文件
virtual bool write(std::ostream& os) const ;  // 写文件
// 上述两个函数如果进行实际的操作, 继承的时候就声明一下就可以
virtual void oplusImpl(const number_t* update); // 顶点更新函数,重要
virtual void setToOriginImpl();       // 顶点重置函数, 设定被优化变量的原始值
```

oplusImpl: 顶点更新函数, 主要用于优化过程中增量△x的计算, 当根据增量方程计算出增量以后就是通过这个

函数对估计值进行调整的,如高博拟合曲线中的

```c++
// 更新
    void oplusImpl(const double * update) override{
        _estimate += Eigen::Vector3d(update);
    }
```

自定义节点一般使用下面的格式:

```c++
class myVertex : public g2o::BaseVertex<Dim, Type>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 使用Eigen字节对齐声明空间
    myVertex(){}
    virtual bool read(std::istream& in) override{}
    virtual bool write(std::ostream& out) const override {}
    virtual void setOriginImpl(){
        _estimate = Type();
    }
    virtual void oplusImpl(const double* update) override{
        _estimate += /*update*/
    }
}
```

高博的十四讲是直接将更新量加上去的_estimate += Eigen::Vector3d(update);  就是x+△x;

这是因为顶点的类型是Eigen::Vector3d(a,b,c), 属于向量, 可以通过加法直接来进行更行.

但是在三维slam中李代数表示的位姿VectorSE3Expmap是不能够直接相加的, 这个空间没有加法

g2o的官网给出了这个更新方式如下:

```c++
**
 * \brief SE3 Vertex parameterized internally with a transformation matrix
 and externally with its exponential map
 */
class G2O_TYPES_SBA_API VertexSE3Expmap : public BaseVertex<6, SE3Quat>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexSE3Expmap();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  virtual void setToOriginImpl() {
    _estimate = SE3Quat();
  }

  virtual void oplusImpl(const number_t* update_)  {
    Eigen::Map<const Vector6> update(update_);
    setEstimate(SE3Quat::exp(update)*estimate());
  }
};
```

 BaseVertex<6, SE3Quat> 6表示内部优化变量的维度是6维

SE3Quat表示优化变量的类型, 使用的是g2o定义的相机位姿类型: SE3Quat

定义为于: se3quat.h文件中, 内部使用了四元数表达旋转, 然后加上位移来存储位姿, 同时支持李代数的运算,

比如对数映射(log函数), 李代数上的增量(update)

这里不能直接进行相加的原因是变换矩阵不满足加法封闭, 这里相机的位姿使用的是顶点类VertexSE3Expmap

来表达, 而没有使用旋转矩阵和平移矩阵, 原因是旋转矩阵是有约束的矩阵, 它必须是正交矩阵且行列式为1. 使用它

作为优化变量就会引入额外的约束条件, 从而增大优化的复杂度. 而将旋转矩阵转换为李群-李代数之间的转换关系

转换为李代数表示, 就可以将位姿估计问题变成无约束的优化问题, 降低求解难度.



三维空间点的是一个向量也可以直接进行相加

```c++
class G2O_TYPES_SBA_API VertexSBAPointXYZ : public BaseVertex<3, Vector3>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexSBAPointXYZ();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  virtual void setToOriginImpl() {
    _estimate.fill(0);
  }

  virtual void oplusImpl(const number_t* update_)  {
    Eigen::Map<const Vector3> v(update_);
    _estimate += v;
  }
};
```



向图中添加顶点,setEstimate(type)函数来设定初始值, setId(int) 定义节点编号

```c++
 // 向图中增加顶点
    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(ae, be, ce));  // 设置估计值
    v->setId(0);   // 顶点的id标识, 方便后面和边连接
    optimizer.addVertex(v);
```

// 添加VertexSBAPointXYZ

```c++
int index = 1;
for (const Point3f p : points_3d){
    g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
    point->setId(index++);
    point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
    point->setMarginalized(true);
    optimizer.addVertex(point);
}
```



## 4.g2o边

g2o的边主要定义在源码的core/hyper_graph.h   core/optimizable_graph.h  core/base_edge.h

BaseUnaryEdge表示一元边,  BaseBinaryEdge表示二元边, BaseMultiEdge表示多元边

一元边表示一条边只连接一个顶点,  两元边表示一条边连接两个顶点, 多元边可以理解为一条边可以连接多个顶点.

```c++
参数的主要区别: D, E, VertexXi, VertexXj, 含义如下:
D 是int型, 表示测量值的维度(dimension)
E 表示测量值的数据类型
VertexXi, VertexXj 分别表示不同的顶点类型    
```

例如常用边表示三维点投影到图像平面的重投影误差, 就可以设置参数如下:

```c++
BaseBinaryEdge<2, Vector2D, VertexSBAPointXYZ, VertexSE3Expmap>
```

含义为: 第一个2表示测量值是2维的, 也就是图像像素坐标x, y的差值

​               对应的测量值的类型就是Vector2D

​				两个顶点表示优化变量分别是三维点VertexSBAPointXYZ, 和李群位姿VertexSE3Expmap



当g2o中的边的类型不能满足使用的时候, 就需要自己定义边的类: 主要需要重载以下几个函数:

```c++
virtual bool read(std::istream& is);
virtual bool write(std::ostream& os);
virtual void computeError();
virtual void linearizeOplus();
```

read, write分别表示读盘, 存盘函数, 一般情况下不需要进行读/写操作的话, 仅仅就声明一下就可以

computeError函数: 非常重要, 是使用当前顶点的值计算的预测值与真实的测量值之间的误差,

linearizeOplus函数: 非常重要, 是在当前顶点的值下, 该误差函数对优化变量的偏导数,雅克比矩阵

还有几个重要的函数

```c++
_measurement: 观测值
_error: 存储computeError()函数计算的误差值
_vertices[]: 存储顶点信息,  例如二元边的话, _vertices[]的大小为2, 存储顺序和调用setVertex(int,  vertex)是设定的int有关(0或者1)
setId(int): 定义节点的编号(决定了在H矩阵中的位置)
setMeasurement(type)  函数来定义观测值
setVertex(int, vertex)  来定义顶点
setInformation()   信息矩阵, 用来定义协方差矩阵的逆    
```

定义g2o的边, 套路如下:

```c++
class myEdge : public g2o::BaseBinaryEdge<errorDim, errorType, Vertex1Type, Vertex2Type>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    myEdge() {}
    virtual bool read(istream& in) {}
    virtual bool write(ostream& out) const {}
    virtual void computeError() override{
        // ...
        _error = _measurement - Something;
    }
    virtual void linearizeoplus() override{
        // ....
        _jacobianOplusXi(pos, pos) = something;
        /*
        * _jocobianOplusXj(pos, pos) = something
        */
    }
private:
    // data
}
```

高博十四讲中曲线拟合的边定义如下

```c++
// 误差模型 模版参数: 观测值维度, 类型, 连接顶点类型
// BaseUnaryEdge是一个一元边即自己连接自己
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>{
public:
    double _x;  // x值, y值为_measurement
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}
    // 计算曲线模型误差
    virtual void computeError() override{
        const CurveFittingVertex * v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - std::exp(abc(0, 0)*_x*_x + abc(1, 0)*_x + abc(2, 0));
    }
    // 计算雅克比矩阵
    virtual void linearizeOplus() override {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex*>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        double y = std::exp(abc[0]*_x*_x + abc[1]*_x + abc[2]);
        _jacobianOplusXi[0] = -_x * _x * y;
        _jacobianOplusXi[1] = -_x * y;
        _jacobianOplusXi[2] = -y;
    }

    bool read(std::istream& in) override{}
    bool write(std::ostream& out) const override {}
};
```



3D-2D点的PnP问题, 最小化重头应误差, 使用的是二元边. 

```c++
// 继承自BaseBinaryEdge类, 观测值是2维, 类型是Vector2D, 顶点分别是三维点, 李群位姿
class G2O_TYPES_SBA_API EdgeProjectXYZ2UV : public  BaseBinaryEdge<2, Vector2, VertexSBAPointXYZ, VertexSE3Expmap>{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // 1. 默认构造函数
    EdgeProjectXYZ2UV();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;
	// 2. 计算误差
    void computeError()  {
        // 李群相机位姿v1
      const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
        // 顶点v2
      const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        // 相机内参
      const CameraParameters * cam
        = static_cast<const CameraParameters *>(parameter(0));
      Vector2 obs(_measurement);
        // 误差计算, 测量值减去估计值, 也就是重投影误差obs-cam
        // 估计值计算方法是T*p, 得到相机坐标系坐标, 然后在利用camera2pixel()函数得到像素坐标
      _error = obs-cam->cam_map(v1->estimate().map(v2->estimate()));
    }
	// 3. 线性增量函数, 也就是雅克比矩阵J的计算方法
    virtual void linearizeOplus();
    // 4. 相机参数
    CameraParameters * _cam;
};
```

 _error = obs-cam->cam_map(v1->estimate().map(v2->estimate()));  误差 = 预测 - 投影

cam_map函数功能是把相机坐标系下三维点(输入)用内参转换为图像坐标(输出), 具体代码如下:

```c++
Vector2  CameraParameters::cam_map(const Vector3 & trans_xyz) const {
  Vector2 proj = project2d(trans_xyz);
  Vector2 res;
  res[0] = proj[0]*focal_length + principle_point[0];
  res[1] = proj[1]*focal_length + principle_point[1];
  return res;
}
```

.map函数是将世界坐标系下三维点变换到相机坐标系, 函数在types/sim3/sim3.h

```c++
 Vector3 map (const Vector3& xyz) const {
        return s*(r*xyz) + t;
      }
```

代码

```c++
v1->estimate().map(v2->estimate());
```

使用v1估计的pose把v2代表的三维点, 变换到相机坐标系下面



向图中添加边

高博曲线拟合的例子如下

```c++
// 往图中增加边
    for (int i = 0; i < N; i++){
        CurveFittingEdge *edge = new CurveFittingEdge(xData[i]);  // 每次创建一条边, 边的预测值就是前面的xData
        edge->setId(i);
        edge->setVertex(0, v);         // 设置连接的顶点 由于这仅仅是一个一元边, 只连接了一个顶点因此第一个参数永远是0
        edge->setMeasurement(yData[i]);  // 观测数据
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (wSigma * wSigma));  // 信息矩阵: 协方差矩阵之逆
        optimizer.addEdge(edge);

    }

```

对于曲线拟合来说,  edge->setMeasurement(yData[i]);  // 观测数据, 观测值就是实际观测到的数据点. 

对于视觉slam来说通常就是观测点的特征点坐标

下面是一个二元边, 需要用边连接来个顶点

```c++
 // 设置边
    index = 1;
    for ( const Point2f p:points_2d )
    {
        auto* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, vertexPose );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );  //设置观测值
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }
```

上面的setVertex中的0, 1不能随便互换

_vertices[i]里面的i就是上面设置的0和1, 边的类型g2o::EdgeProjectXYZUV定义, 来前面一样, 

```c++
class G2O_TYPES_SBA_API EdgeProjectXYZ2UV
    // 李群相机位姿
const VertexSE3Expmap* v1 = static_cast<const VertexSExmap*> (_vertices[1]);	
// 顶点v2
const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
```

_vertices[0] 对应的就是VertexSBAPointXYZ类型的点, 也就是三维点,

_vertices[1]  对应的就是VertexSE3Expmap类型的顶点, 就是是pose的位姿,

因此前面1对应的就应该是pose, 0对应的应该就是三维点.







































