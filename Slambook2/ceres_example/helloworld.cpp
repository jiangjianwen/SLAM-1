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
