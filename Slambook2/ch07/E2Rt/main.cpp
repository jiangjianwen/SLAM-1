
// 本程序演示如何从Essential矩阵计算R,t
// 第五节课习题三

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.hpp>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    Eigen::JacobiSVD<Matrix3d> svd(E, ComputeFullU|ComputeFullV); // 矩阵的SVD分解
    Vector3d sigma = svd.singularValues();  // svd分解出来的sigma是3X1的向量就是奇异值
    Matrix3d SIGMA; // 将向量sigma调整成矩阵SIGMA 得到奇异值矩阵
    SIGMA << (sigma(0, 0) + sigma(1, 0))/2, 0, 0,
             0, (sigma(0, 0) + sigma(1, 0))/2, 0,
             0, 0, 0;
    cout << "SIGMA = \n" << SIGMA << endl;

    // END YOUR CODE HERE

    // set t1, t2, R1, R2
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;

    Matrix3d R1;
    Matrix3d R2;

    // 得到SVD分解以后U,V的值
    Matrix3d U = svd.matrixU(); // U的值
    Matrix3d V = svd.matrixV(); // V的值
    // 定义旋转矩阵
    Matrix3d R_z1 = AngleAxisd(M_PI/2, Vector3d(0, 0, 1)).toRotationMatrix(); // 通过旋转向量构造旋转矩阵,沿Z轴旋转90°
    Matrix3d R_z2 = AngleAxisd(-M_PI/2, Vector3d(0, 0, 1)).toRotationMatrix(); // 定义旋转矩阵沿Z轴旋转-90°

    // cout << U.rows() << "  " << U.cols() << endl;
    // 根据公式计算t1, t2, R1, R2的值
    t_wedge1 = U*R_z1*SIGMA*U.transpose();
    t_wedge2 = U*R_z2*SIGMA*U.transpose();
    R1 = U*R_z1.transpose()*V.transpose();
    R2 = U*R_z2.transpose()*V.transpose();
    // END YOUR CODE HERE

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << Sophus::SO3d::vee(t_wedge1) << endl; // vee表示反对称矩阵到向量
    cout << "t2 = " << Sophus::SO3d::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;

    return 0;
}