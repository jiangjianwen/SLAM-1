# Ceres官网学习笔记

## 安装

配置环境Ubuntu18.04

1.首先安装所需要的依赖项:

```bash
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev 
```

2.下载Ceres安装包:

```bash
git clone https://github.com/ceres-solver/ceres-solver
cd cd ceres-solver
mkdir build
cd build
cmake ..
make
sudo make install
```

如果最新的包编译不过可以clone下面的源码包

```bash
git clone https://github.com/zhoupengwei/SLAM/tree/master/Slambook2/Sources/ceres-solver
```

3.使用 在CmakeLists中加入

```cmake
find_package( Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )
add_executable(ceresCurveFitting main.cpp)
target_link_libraries(ceresCurveFitting  ${CERES_LIBRARIES} )
```



## 官网学习教程

Ceres是一个广泛使用的最小二乘问题求解库.

使用Ceres只需要按照一定的步骤定义待求解的优化问题,然后交给求解其进行求解.

[官网](http://ceres-solver.org/nnls_tutorial.html)

[源程序](https://github.com/zhoupengwei/SLAM/tree/master/Slambook2/ceres_example)



## 示例一

Ceres可以解决边界约束鲁棒非线性最小二乘法优化问题,常常使用如下表达式进行表示

参考翻译[博客](https://blog.csdn.net/wzheng92/article/details/79634069)

损失函数这里应该理解成核函数,引入核函数的目的是为了减少异常值对损失函数的影响,如:在视觉slam中进行图

优化时,    如果把一条错误的边加入到整个系统中,可是优化算法却不知道这是一条不合法的边,因此这时候\\优化算

法会把所有的数据都当成误差来处理,在算法看来是相机突然观测到了一次不可能产生的数据.这时候

在图优化中会有一条误差很大的边,它的梯度也很大,意味着调整与它相关的变量会使目标函数下降更多.所以算法   

将试图优先调整这条边所连接的节点估计值,是他们顺应这条边的无理要求.由于这条边的误差真的很大,往往会  

抹平其他正确边的影响,使优化算法专注于调整一个错误的值.

​		出现这种问题的原因时,当误差很大时,二范数增长的太快,于是就有了核函数的存在.核函数保证每条边的误差不

会大的没边而掩盖其他的边.具体的方式是,把原先误差的二范数度量替换成一个增长没有那么快的函数,同时保证自

己的光滑性质(使得整个优化过程结果更加稳健),所以又叫做鲁棒核函数(Robust Kernel)

![Image](https://github.com/zhoupengwei/SLAM/blob/master/img-folder/robust.png)

注: 参考视觉slam后端优化



```c++
#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// A templated cost functor that implements the residual r = 10 -
// x. The method operator() is templated so that we can then use an
// automatic differentiation wrapper around it to generate its
// derivatives.
// 使用ceres需要重载operator()已完成函数对象的定义,其中模版特化部分将由ceres自动完成
struct CostFunctor {
    template <typename T> bool operator()(const T* const x, T* residual) const {
        residual[0] = 10.0 - x[0]; // 残差的计算方式
        return true;
    }
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // The variable to solve for with its initial value. It will be
    // mutated in place by the solver.
    double x = 0.5;
    const double initial_x = x;

    // Build the problem.
    Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    // 构建代价函数,注意这里的new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    // 第一个1,表示残差的维度,第二个1表示待优化变量的维度信息,这里需要和上面定义的类中的维度信息统一,
    // 使用自动求导, 也可以自己定义雅克比矩阵的形式
    CostFunction* cost_function =
            new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, NULL, &x);

    // Run the solver!
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x
              << " -> " << x << "\n";
    return 0;
}

```

## 示例二

数值法和解析法求导,参考[博客](https://blog.csdn.net/wzheng92/article/details/79699783?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task)

### 数值法求导

```c++
#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// A cost functor that implements the residual r = 10 - x.
// 数值求导这里就不需要使用模版类了
struct CostFunctor {
    bool operator()(const double* const x, double* residual) const {
        residual[0] = 10.0 - x[0];
        return true;
    }
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // The variable to solve for with its initial value. It will be
    // mutated in place by the solver.
    double x = 0.5;
    const double initial_x = x;

    // Build the problem.
    Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // numeric differentiation to obtain the derivative (jacobian).
    CostFunction* cost_function =
            new NumericDiffCostFunction<CostFunctor, CENTRAL, 1, 1> (new CostFunctor);
    problem.AddResidualBlock(cost_function, NULL, &x);

    // Run the solver!
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x
              << " -> " << x << "\n";
    return 0;
}


```

### 解析法求导

解析法求导可以自己定义雅克比矩阵的形式,在一些问题中,如果我们准确知道雅克比矩阵的形式就可以使用这种方法,



## Problem类

Ceres的求解过程包括构建最小二乘和求解最小二乘问题两部分,其中构建最小二乘问题的相关方法均包含在

Ceres::Problem类中. 涉及到的主要成员函数包括Problem::AddResidualBlock()和

Problem::AddParameterBlock().

### Problem::AddResidualBlock()

Problem::AddResidualBlock()主要用于向Problem类传递残差模块的信息,函数原型如下, 传递的参数主要包括代

价函数模块, 损失函数模块和参数模块

```c++
ResidualBlockId Problem::AddResidualBlock(CostFunction *cost_function, 
										  LossFunction *loss_function, 
										  const vector<double *> parameter_blocks)
										  
ResidualBlockId Problem::AddResidualBlock(CostFunction *cost_function, 
										  LossFunction *loss_function,
										  double *x0, double *x1, ...)

```

·代价函数: 包含了参数块的维度信息,内部使用仿函数定义误差函数的计算方式.AddResidualBlock()函数会检查

传入的参数模块是否和代价函数模块中定义的维数一致, 维度不一致时程序会强制退出.

·损失函数: 用于处理参数中含有野值的情况,避免错误量对估计的影响,常用参数包括HuberLoss, CauchyLoss等.

·参数模块: 待优化的参数,可一次性传入所有参数的指针容器vector<double\# * >>或者依次传入参数的指针double\# * >.

### Problem::AddParameterBlock()

用户在调用AddResidualBlock()时其实已经隐式地向Problem传递了参数模块,但在一些情况下,需要用户显示地

向Problem传入参数模块(通常出现在需要对参数进行重新参数化的情况). Ceres提供了

Problem::AddParameterBlock( )`函数用于用户显式传递参数模块：

```c++
void Problem::AddParameterBlock(double *values, int size)

void Problem::AddParameterBlock(double *values, int size, LocalParameterization *local_parameterization)

```

其中，第一种函数原型除了会增加一些额外的参数检查之外，功能上和隐式传递参数并没有太大区别。第二种函数

原型则会额外传入`LocalParameterization`参数，用于重构优化参数的维数，重点看`LocalParameterization`类

### LocalParameterization

LocalParameterization类的作用是解决非线性优化中的过参数化问题。所谓过参数化，即待优化参数的实际自由

度小于参数本身的自由度。例如在SLAM中，当采用四元数表示位姿时，由于四元数本身的约束（模长为1），实

际的自由度为3而非4。此时，若直接传递四元数进行优化，冗余的维数会带来计算资源的浪费，需要使用Ceres预

先定义的QuaternionParameterization对优化参数进行重构：

```c++
problem.AddParameterBlock(quaternion, 4);// 直接传递4维参数

ceres::LocalParameterization* local_param = new ceres::QuaternionParameterization();
problem.AddParameterBlock(quaternion, 4, local_param)//重构参数，优化时实际使用的是3维的等效旋转矢量

```

自定义LocalParameterization

`LocalParaneterization`本身是一个虚基类，详细定义如下。用户可以自行定义自己需要使用的子类，或使用Ceres预先定义好的子类。

```c++
class LocalParameterization {
 public:
  virtual ~LocalParameterization() {}
  //
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const = 0;//参数正切空间上的更新函数
  virtual bool ComputeJacobian(const double* x, double* jacobian) const = 0; //雅克比矩阵
  virtual bool MultiplyByJacobian(const double* x,
                                  const int num_rows,
                                  const double* global_matrix,
                                  double* local_matrix) const;//一般不用
  virtual int GlobalSize() const = 0; // 参数的实际维数
  virtual int LocalSize() const = 0; // 正切空间上的参数维数
};
```

述成员函数中，需要我们改写的主要为`GlobalSize()`、`ComputeJacobian()`、`GlobalSize()`和`LocalSize()`，以ceres预

先定义好的`QuaternionParameterization`为例具体说明，类声明如下：

```c++
class CERES_EXPORT QuaternionParameterization : public LocalParameterization {
 public:
  virtual ~QuaternionParameterization() {}
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x,
                               double* jacobian) const;
  virtual int GlobalSize() const { return 4; }
  virtual int LocalSize() const { return 3; }
};
```

·可以看到，`GlobalSize()`的返回值为4，即四元数本身的实际维数；由于在内部优化时，ceres采用的是旋转矢

量，维数为3，因此`LocalSize()`的返回值为3。

·重载的`Plus`函数给出了四元数的更新方法，接受参数分别为优化前的四元数`x`，用旋转矢量表示的增量`delta`，

以及更新后的四元数`x_plus_delta`。函数首先将增量由旋转矢量转换为四元数，随后采用标准四元数乘法对四元数

进行更新。



























