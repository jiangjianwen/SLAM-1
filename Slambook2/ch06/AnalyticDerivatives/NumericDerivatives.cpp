/*
 * 本示例摘自Ceres Tutorial http://www.ceres-solver.org/nnls_tutorial.html#numeric-derivatives
 * 数值求导法(Numeric Derivative) 在某些情况下，像在Hello World中一样定义一个代价函数是不可能的。
 * 比如在求解残差值（residual）的时候调用了一个库函数，而这个库函数的内部算法你根本无法干预。
 * 在这种情况下数值微分算法就派上用场了。用户定义一个CostFunctor来计算残差值，
 * 并且构建一个NumericDiffCostFunction数值微分代价函数。比如对于 f(x)=10−xf(x)=10−x 对应函数体如下
 * */
#include <iostream>
#include <ceres/ceres.h>
#include <glog/logging.h>

struct CostFunctor{
    bool operator()(const double * const x, double * residual) const {
        residual[0] = 10.0 - x[0];
        return true;
    }
};
int main(int argc, char ** argv) {

    google::InitGoogleLogging(argv[0]);
    // 初始值
    double x = 0.5;
    const double initial_x = x; // 优化初始值, 优化的结果将会保存到这个变量当中去
    // 构建问题
    ceres::Problem problem;
    // 定义代价函数使用数值求导 其中ceres::CENTRAL参数指定用来怎样求导数
    ceres::CostFunction* costFunction = new ceres::NumericDiffCostFunction<CostFunctor, ceres::CENTRAL, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(costFunction, nullptr, &x);

    // 求解
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x: " << initial_x << " -> " << x << std::endl;

    std::cout << "Hello, World!" << std::endl;
    return 0;
}