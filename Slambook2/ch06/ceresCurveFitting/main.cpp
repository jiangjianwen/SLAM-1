/*
 * 使用Ceres进行求解问题需要按照下面几个步骤进行配置
 * 1.定义每个参数块.参数快通常为平凡的变量,但是在slam里面也可以定义成四元数,李代数这种特殊的结构.
 *   如果是向量,那么需要为每个参数块分配一个double数组来存储变量的值
 * 2.定义残差的计算方式.残差块通常关联若干个参数,对它们进行一些自定义的计算,然后返回残差值.
 *   Ceres对它们求平方和之后,作为目标函数的值
 * 3.残差块往往也需要定义雅克比的计算方式.在Ceres中,你可以使用它提供的"自动求导功能功能",也可以手动
 *   指定雅克比的计算过程.如果使用自动求导,那么残差块需要按照特定的写法书写:残差的计算过程应该是一个
 *   带模版的括号运算符.
 * 4.把所有的参数块和残差块加入Ceres定义的Problem对象中,调用Solve函数求解即可.求解之前,我们可以传入一些配置信息
 *   例如迭代次数,终止条件等,也可以使用默认的配置
 *
 * */

#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

// 代价函数的计算模型
// 代价函数也就是仿函数 由于重载了()运算符,使得其能够具有和函数一样的调用行为.
// 误差函数中的参数包括已知参数和待优化参数两部分,待优化参数由Problem::AddResidualBlock()统一添加和管理,而已知参数则在
// 仿函数创建时通过构造函数传入,若优化问题没有已知参数,则不需要编写构造函数.
struct CurveFittingCost{
    const double _x, _y;    // x,y数据
    CurveFittingCost(double x, double y) : _x(x), _y(y) {}
    // 操作符()是一个模版方法,返回值为bool类型,接受参数为待优化变量和残差变量.待优化变量的传入方式应该和
    // Problem::AddResidualBlock()一致,即若Problem::AddResidualBlock()中一次性传入变量指针,
    // 若Problem::AddResidualBlock()变量是依次传入,则此处也应该依次传入,并且要保证变量的传入顺序是一致的.
    // 同时需要注意的是,该操作符的输入和输出变量统一为模版类型T,一般统一转换为double类型的数组
    // 残差的计算 abc 表示模型参数有三维
    template<typename T>
    bool operator()(const T * const abc, T * residual) const{
        // y-exp(ax^2 + bx + c)
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
        return true;
    }
};

int main() {
    double ar = 1.0, br = 2.0, cr = 1.0;    // 真实数据值
    double ae = 2.0, be = -1.0, ce = 5.0;   // 估计参数值
    int N = 100;      // 数据点
    double wSigma = 1.0;                    // 噪声数据的标准差
    double invSigma = 1.0 / wSigma;
    cv::RNG rng;
    std::vector<double> xData, yData;
    for (int i = 0; i < N; i++){
        double x = i / 100.0;
        xData.push_back(x);
        yData.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(wSigma * wSigma));
    }
    double abc[3] = {ae, be, ce};
/*
 * ceres官方极力推荐使用自动求导的方式AutoDiffCostFunction
 * ceres::AutoDiffCostFunction<CostFunctor, int residualDim, int paramDim>(CostFunctor* functor);
 * 模版参数依次为仿函数(functor)类型CostFunction, 残差维数residualDim和参数维数paramDim. 接收参数类型为仿函数指针CostFunctor*.
 *
 * */
    // 构建最小二乘问题

    ceres::Problem problem;
    for (int i = 0; i < N; i++){
        // 向问题中添加误差项
        // 使用自动求导,模板参数:误差类型,输出维度,输入维度,维数要与前面struct中一致
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 3>(
                new CurveFittingCost(xData[i], yData[i])
                ),
                                 nullptr, abc);
        problem.AddParameterBlock(abc, 3); // 参数重复调用将会被忽略 如果使用同一指针但是size和上面的维度不一样将会报错
    }
    //配置求解器
    ceres::Solver::Options options; // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;    // 输出到cout

    ceres::Solver::Summary summary;     // 优化信息
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);  // 开始进行求解
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "solve time cost  = " << time_used.count() << " seconds " << std::endl;

    // 输出结果
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "estimated a,b,c = ";
    for (auto a : abc){
        std::cout << a << " ";
    }
    std::cout << std::endl;

    std::cout << "Hello, World!" << std::endl;
    return 0;
}