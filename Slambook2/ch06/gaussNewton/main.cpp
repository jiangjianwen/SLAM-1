//本程序主要完成手写高斯牛顿的实例 深刻的理解优化的过程
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
int main() {
    double ar = 1.0, br = 2.0, cr = 1.0;    // 真实的数据值
    double ae = 2.0, be = -1.0, ce = 5.0;   // 估计参数值
    int N = 100;                            // 数据点
    double wSigma = 1.0;                    // 噪声的Sigma值 图像的标准差
    double invSigma = 1.0 / wSigma;
    cv::RNG rng;                            // OpenCV 随机数产生器
    std::vector<double> xData, yData;       // 数据
    for (int i = 0; i < N; i++){
        double x = i / 100.0;
        xData.push_back(x);
        yData.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(wSigma * wSigma));
    }

    // 开始GaussNewton迭代
    int iteration = 100;    // 设置迭代次数
    double cost = 0, lastCost = 0;  // 本次迭代和上一次迭代的cost
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    for (int iter = 0; iter < iteration; iter++){
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();    // 高斯牛顿近似的Hessian
        Eigen::Vector3d b = Eigen::Vector3d::Zero();    // Bias
        cost = 0;

        for (int i = 0; i < N; i++){
            double xi = xData[i], yi = yData[i];    // 第i个数据点
            double error = yi - exp(ae * xi * xi + be * xi + ce);
            Eigen::Vector3d J;  // 雅克比矩阵
            J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);
            J[1] = -xi * exp(ae * xi * xi + be * xi + ce);
            J[2] = -exp(ae * xi * xi + be * xi + ce);

            H += invSigma * invSigma * J * J.transpose();
            b += -invSigma * invSigma * error * J;

            cost += error * error;  // 误差需要平方防止正负抵消
        }
        // 求解线性方程组
        Eigen::Vector3d dx = H.ldlt().solve(b);
        if (isnan(dx[0])){
            std::cout << "result is nan!  " << std::endl;
            break;
        }
        // 如果迭代以后的误差比原来的误差值还大的话就是说明已经优化到了最小值了
        if (iter > 0 && cost >= lastCost){
            std::cout << "cost: " << cost << " >= last cost: " << lastCost << ", break" << std::endl;
            break;
        }

        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        lastCost = cost;
        std::cout << "total cost: " << cost << ", \t\t update: " << dx.transpose() << "\t\t estimated params: " <<
        ae << ", " << be << ", " << ce << std::endl;
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "solve time cost = " << time_used.count() << " seconds." << std::endl;
    std::cout << "estimated abc = " << ae << ", " << be << ", " << ce << std::endl;

    std::cout << "Hello, World!" << std::endl;
    return 0;
}
















