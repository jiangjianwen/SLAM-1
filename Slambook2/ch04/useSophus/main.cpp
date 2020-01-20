#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "sophus/se3.hpp"

using namespace Eigen;

// 程序演示Sophus库的主要使用
int main() {
    // 沿Z轴旋转90度的旋转矩阵
    Matrix3d R = AngleAxisd(M_PI/2, Vector3d(0,0,1)).toRotationMatrix();

    // 或者使用四元数来构造
    Quaterniond q(R);
    Sophus::SO3d SO3_R(R);       // Sophus::SO3 可以直接从旋转矩阵构造
    Sophus::SO3d SO3_q(q);       // 也可以通过四元数直接构造
    // 这样构造出来的结果是一样的
    std::cout << "SO(3) from matrix:\n" << SO3_R.matrix() << std::endl;
    std::cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << std::endl;
    std::cout <<  "they are equal " << std::endl;

    //使用对数映射获得它的李代数
    Vector3d so3 = SO3_R.log();
    std::cout << "so3 = " << so3.transpose() << std::endl;
    // hat 为向量到反对称矩阵
    std::cout << "so3 hat = \n" << Sophus::SO3d::hat(so3) << std::endl;
    // 相对地 vee 为反对称到向量
    std::cout << "so3 hat vee = " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << std::endl;

    // 增量扰动模型的更新
    Vector3d update_so3(1e-4, 0, 0); // 假设更新量有这么多
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    std::cout << "SO3 updated  = \n " << SO3_updated.matrix() << std::endl;

    std::cout << "-----------------***************************------------------------------" << std::endl;
    // 对SE(3) 的操作大同小异
    Vector3d t(1, 0, 0);    // 沿x轴平移1
    Sophus::SE3d SE3_Rt(R, t);     // 从R t 构造SE(3)
    Sophus::SE3d SE3_qt(q, t);     // 从q t 构造SE(3)
    std::cout << "SE3 from R t = \n" << SE3_Rt.matrix() << std::endl;
    std::cout << "SE3 from q t = \n" << SE3_qt.matrix() << std::endl;

    // 李代数se(3) 是一个六维向量
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    Vector6d se3 = SE3_Rt.log();
    // 在输出的时候在Sophus中 se(3) 的平移在前 旋转在后
    std::cout << "se3 = " << se3.transpose() << std::endl;

    // 同样有hat 和 vee 两个运算符
    std::cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << std::endl;
    std::cout << "se3 hat vee = \n" << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << std::endl;

    // 演示更新
    Vector6d update_se3;    // 更新量
    update_se3.setZero();
    update_se3(0, 0) = 1e-4d;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    std::cout << "SE3 updated = " << std::endl << SE3_updated.matrix() << std::endl;


    std::cout << "Hello, World!" << std::endl;
    return 0;
}