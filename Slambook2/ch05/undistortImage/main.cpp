#include <iostream>
#include <opencv2/opencv.hpp>


int main(int argc, char ** argv) {
    // 去畸变的参数如下
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    // 相机内参
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 247.375;
    cv::Mat image;
    image = cv::imread(argv[1], 0);
    if (image.empty()){
        std::cout << "can not find this image!!!! " << std::endl;
        return -1;
    }
    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1); // 直接创建一个灰度图

    // 计算去除畸变以后的图像
    for (int v = 0; v < rows; v++){
        for (int u = 0; u < cols; u++){
            // 按照公式, 计算点(u,v) 对应到畸变图像中的坐标
            double x = (u - cx) / fx, y = (v - cy) / fy; // 首次坐标先转换到图像坐标系的原点处
            double r = sqrt(x*x + y*y);
            double x_distorted = x * ( 1 + k1 * r*r + k2 *r *r * r*r) + 2 * p1*x * y +
                    p2 * (r*r + 2*x*x);
            double y_distorted = y * (1 + k1 * r * r + k2 * r* r*r*r) + p1 * (r*r + 2*y*y) + 2*p2*x*y;
            double u_distorted = fx * x_distorted + cx;
            double v_distorted = fy * y_distorted + cy;
            // 赋值 (最邻近插值)
            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows){
                image_undistort.at<uchar>(v, u) = image.at<uchar>(static_cast<int>(v_distorted), static_cast<int>(u_distorted));

            }else{
                image_undistort.at<uchar>(v, u) = 0;
            }
        }
    }




    // 画出去畸变以后的图像
    cv::imshow("distorted", image);
    cv::imshow("undistorted", image_undistort);
    cv::waitKey(0);

    std::cout << "Hello, World!" << std::endl;
    return 0;
}