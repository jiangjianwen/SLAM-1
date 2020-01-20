/*
 * 求解不同坐标系之间的位姿变换
 * */
#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

int main() {

    Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);
    // 注意如果直接定义四元数的话 在使用之前要进行归一化处理
    // 由于旋转矩阵是正交矩阵 如果四元数没有进行归一化处理 则由它所生成的旋转矩阵就不是正交的
    // 不满足R转置 = R逆
    q1.normalize();
    q2.normalize();
    Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);
    Vector3d p1(0.5, 0, 0.2);

    Isometry3d T1w(q1), T2w(q2);
    T1w.pretranslate(t1);
    T2w.pretranslate(t2);

    Vector3d p2 = T2w * T1w.inverse() * p1;
    //std::cout << std::endl << p2.transpose() << std::endl;





    AngleAxisd rotationVecotr(0, Vector3d(0,0,1));
    Quaterniond q3 = Quaterniond(rotationVecotr);
    Quaterniond q4 = Quaterniond(rotationVecotr);
    Vector3d t3(-1,0,0), t4(0,-1,0);
    Isometry3d Tr1W(q3),Tr2W(q4);
    Tr1W.pretranslate(t3);
    Tr2W.pretranslate(t4);
    Vector3d CamberPoint(2,2,0);
    Vector3d LaserPoint = Tr2W * Tr1W.inverse() * CamberPoint;
    std::cout << "\n The point in laser coordinate is: " << LaserPoint.transpose() << std::endl;









    //std::cout << "Hello, World!" << std::endl;
    return 0;
}