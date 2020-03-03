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

源程序

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





















