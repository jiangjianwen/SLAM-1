#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;
int main() {
    // 本程序演示Eigen库几何模块的使用方法
    // Eigen/Geometry库提供了各种旋转和平移的表示
    // 3D旋转矩阵直接使用Matrix3d或者Matrix3f
    Matrix3d rotationMatrix = Matrix3d::Identity();     // 直接创建一个单位矩阵
    // std::cout << rotationMatrix << std::endl;
    // 旋转向量使用AngleAxis,它底层不直接是Matrix 但是运算可以当做是矩阵进行运算 因为重载了运算符
    AngleAxisd rotationVector(M_PI/4, Vector3d(0, 0, 1));   // 沿着Z轴旋转了45°
    std::cout.precision(3);
    std::cout << "roration Matrix = \n" << rotationVector.matrix() << std::endl; // 使用matrix()转换成矩阵进行输出

    // 也可以直接赋值
    rotationMatrix = rotationVector.toRotationMatrix();

    // 使用AngleAxis可以进行坐标变换
    Vector3d v(1, 0,0 );
    Vector3d v_rotated = rotationVector * v;
    std::cout << "(1, 0, 0) after rotation (by angle axis) = " << v_rotated.transpose() << std::endl;

    // 或者使用旋转矩阵
    v_rotated = rotationMatrix * v;
    std::cout << "(1, 0, 0 ) after rotation (by matrix) = " << v_rotated.transpose() << std::endl;

    // 欧拉角:可以直接将旋转矩阵直接转换成欧拉角
    Vector3d eulerAngles = rotationMatrix.eulerAngles(2, 1, 0); // ZYX顺序 即roll pitch yaw 顺序
    std::cout << "eulerAngle is yaw pitch roll = " << eulerAngles.transpose() << std::endl;

    // 欧式变换矩阵使用Eigen::Isometry
    Isometry3d T = Isometry3d::Identity();  // 虽然称为3D 但是实质上是4*4矩阵
    T.rotate(rotationVector);               // 使用旋转向量进行旋转
    T.pretranslate(Vector3d(1, 3, 4));  // 把平移向量设置成(1, 3, 4)
    std::cout << "Transform matrix = \n" << T.matrix() << std::endl;

    // 使用变换矩阵进行坐标变换
    Vector3d vTransformed = T*v;
    std::cout << "vTransformed = " << vTransformed.transpose() << std::endl;

    // 对于仿射变换和射影变换, 使用Eigen::Affine3d和Eigen::Projective3d

    // 四元数
    // 可以直接将旋转向量AngleAxis赋值给四元数 反之亦然
    Quaterniond q = Quaterniond(rotationVector);
    // 这里注意coeffs的顺序是(x,y,z,w), w为实部 前三者为虚部
    std::cout << "quaternion from rotation vector = " << q.coeffs().transpose() << std::endl;

    // 也可以直接将旋转矩阵赋给它
    q = Quaterniond(rotationMatrix);
    std::cout << "quaternion from matrix = " << q.coeffs().transpose() << std::endl;

    // 使用四元数旋转一个向量,使用重载的乘法即可
    v_rotated = q * v;  // 注意数学上是 qvq-1
    std::cout << "(1,0,0) after rotation = " << v_rotated.transpose() << std::endl;

    // 用常规向量乘法表示 则表示如下
    std::cout << "should be equal to " << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << std::endl;

    std::cout << "Hello, World!" << std::endl;
    return 0;
}