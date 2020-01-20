//
// Created by zpw on 2020/1/16.
//

/*
 * 本程序演示解析法求导(AnalyticDerivatives) http://www.ceres-solver.org/nnls_tutorial.html#analytic-derivatives
 * 有些时候,应用自动求解算法时是不可能的,不如在某些情况下,计算导数的时候,使用闭合解(closed form,也被称为解析解)会比
 * 使用自动微分算法中的链式法则(chain rule)更加有效率.
 * 解析解就是一些严格的数学公式,给出任意的自变量就可以求出其因变量,也就是问题的解.他人可以利用这些公式计算各自的问题.所谓的解析解是一种
 * 包含:分式,三角函数,指数,对数甚至是无限级数等基本函数的解的形式
 * 数值解是采用某种计算方法,如有限元的方法,数值逼近,插值的方法得到的解.别人只能利用数值计算的结果,而不能随意给出自变量并计算值.
 * 当无法由微积分技巧球的解析解时,这时便只能利用数值分析的方式来求得其数值解了.
 * */
#include <vector>
#include <ceres/ceres.h>
#include <glog/logging.h>

// 残差的维数, 第一个参数的维数
class QuadraticCostFunction : public ceres::SizedCostFunction<1, 1>{
public:
    virtual ~QuadraticCostFunction() {}
    // parameters 输入参数数组 residuals 输出残差数组 jacobians输出雅克比行列式
    virtual bool Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const{
        double x = parameters[0][0];
        // f(x) = 10 - x
        residuals[0] = 10 - x;
        // f'(x) = -1
        if(jacobians != NULL && jacobians[0] != NULL){
            jacobians[0][0] = -1;
        }
        return true;
    }
};

int main(int argc, char ** argv){
    google::InitGoogleLogging(argv[0]);
    double x = 0.5;
    const double initial_x = x;

    ceres::Problem problem;
    ceres::CostFunction* costFunction = new QuadraticCostFunction;
    problem.AddResidualBlock(costFunction, NULL, &x);

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";

    return 0;
}

