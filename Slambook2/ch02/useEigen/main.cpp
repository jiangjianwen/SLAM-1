#include <iostream>
#include <ctime>
#include <Eigen/Core>   // Eigen库的核心部分
#include <Eigen/Dense>  // 稠密矩阵的代数运算(逆 特征值等)

using namespace Eigen;

#define MATRIX_SIZE 50

int main() {
    // Eigen 中所有向量和矩阵都是Eigen::Matrix,他是一个模版类.前三个参数分别是 数据类型, 行, 列
    // 声明一个2*3的float矩阵
    Matrix<float, 2, 3> matrix_23;

    // 同时Eigen提供了许多的内置类型,不过底层仍然是Eigen::Matrix 类
    // 例如, Vector3d实质上是Eigen::Matrix<double, 3, 1> 即是三维向量
    Vector3d v_3d;
    // 等价于
    Matrix<float, 3, 1> vd_3d;

    // Matrix3d 实质上是Eigen::Matrix<double, 3, 3>
    Matrix3d matrix3D = Matrix3d::Zero();   // 直接初始化为0

    // 当如果不确定矩阵的大小 可以使用动态大小的矩阵
    Matrix<double, Dynamic, Dynamic> matrixDynamic;
    // 也可以使用Eigen库的动态矩阵
    MatrixXd matrixXd;

    // 下面是对矩阵进行初始化
    matrix_23 << 1, 2, 3, 4, 5, 6;
    // 输出
    std::cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << std::endl;


    // 使用()访问矩阵中的元素
    std::cout << "print matrix 2x3 " << std::endl;
    for (int i = 0; i < matrix_23.rows(); i++){
        for (int j  = 0; j < matrix_23.cols(); j ++){
            std::cout << matrix_23(i, j) << "\t";
            std::cout << std::endl;
        }
    }

    // 矩阵和向量相乘(实际上仍是矩阵和矩阵之间的运算)
    v_3d << 3, 2, 1;
    vd_3d << 4, 5, 6;

    // 但是在Eigen里面不能混用两种不同种类的类型 像这样是错的
    // Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;
    // 如果想要相乘则需要显示的类型转换
    Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    std::cout << "[1,2,3;4,5,6] * [3,2,1]\n" << result.transpose() << std::endl;

    // 同样 不可以搞错矩阵的维度信息 提示错误YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES
    // Eigen::Matrix<double, 2, 3> resultWrongDimension = matrix_23.cast<double>() * v_3d;

    // 一些矩阵运算
    // 四则运算可以直接使用+-*/
    matrix3D = Matrix3d::Random();  // 随机数矩阵
    std::cout << "random matrix: \n" << matrix3D << std::endl;
    std::cout << "transpose: \n" << matrix3D.transpose() << std::endl;  // 转置
    std::cout << "sum: " << matrix3D.sum() << std::endl;                // 各个元素求和
    std::cout << "trace: " << matrix3D.trace() << std::endl;            // 迹
    std::cout << "time 10: \n" << 10 * matrix3D << std::endl;           // 数乘
    std::cout << "inverse: \n" << matrix3D.inverse() << std::endl;      // 逆
    std::cout << "det: " << matrix3D.determinant() << std::endl;        // 行列式

    // 特征值
    // 实对称矩阵可以保证对角化成功
    SelfAdjointEigenSolver<Matrix3d> eigenSolver(matrix3D.transpose() * matrix3D);
    std::cout << "Eigen values = \n" << eigenSolver.eigenvalues() << std::endl;       // 输出特征值
    std::cout << "Eigen vectors = \n" << eigenSolver.eigenvectors() << std::endl;     // 输出特征向量

    // 求解方程
    // 求解matrix_NN * x = v_Nd 方程 Ax = b
    // N的大小在前面的宏定义处
    // 直接对就矩阵进行求逆运算是最直接的但是 这样的做法将会导致运算速度很慢 通常对于维度较大的矩阵来说 常常使用Qr分解来完成

    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrixNN = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrixNN = matrixNN * matrixNN.transpose(); // 半正定
    Matrix<double, MATRIX_SIZE, 1> vNd = MatrixXd::Random(MATRIX_SIZE, 1);

    clock_t timeStart = clock();    // 计时开始
    // 直接求逆
    Matrix<double, MATRIX_SIZE, 1> x = matrixNN.inverse() * vNd;
    std::cout << "time of normal inverse is " << 1000 * (clock() - timeStart) / static_cast<double>(CLOCKS_PER_SEC) << " ms " << std::endl;
    std::cout << "\nx by inverse() = " << x << std::endl;
    // 通常使用矩阵分解来提高运算速度 例如使用Qr分解速度将会快很多
    timeStart = clock();
    x = matrixNN.colPivHouseholderQr().solve(vNd);
    std::cout << "time of Qr decomposition is " << 1000 * (clock() - timeStart) / static_cast<double>(CLOCKS_PER_SEC) << " ms " << std::endl;
    std::cout << "\nx by Qr = " << x << std::endl;

    // 对于一个正定矩阵 还可以使用cholesky来进行分解 cholesky这个方法要求A必须是一个正交矩阵
    timeStart = clock();
    x = matrixNN.ldlt().solve(vNd);
    std::cout << "time of ldlt decomposition is " << 1000 * (clock() - timeStart) / static_cast<double>(CLOCKS_PER_SEC) << " ms " << std::endl;

    std::cout << "\n\n------------------------*********----------------\n\n" << std::endl;
    MatrixXd A_pre = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    MatrixXd A = A_pre.transpose()*A_pre;   // 使A为正定对称矩阵时 才能使用cholesky分解成功
    MatrixXd B = VectorXd::Random(MATRIX_SIZE);
    VectorXd x_1 = A.colPivHouseholderQr().solve(B); // 使用QR分解进行求解
    VectorXd y_1 = A.ldlt().solve(B);   // 调用cholesky分解进行求解

    std::cout << "\nA*x=B 方程的解为 " << x_1 << std::endl;
    std::cout << "\nA*y=B 方程的解为 " << y_1 << std::endl;


    std::cout << "Hello, World!" << std::endl;
    return 0;


















}