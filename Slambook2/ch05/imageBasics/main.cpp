#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::cout;
using std::endl;
int main(int argc, char ** argv) {

    cv::Mat image;
    image = cv::imread(argv[1]);
    if (image.empty()){
        std::cerr << "can not open this image " << std::endl;
        return -1;
    }
    // 文件顺利读取 首先输出一些基本信息
    std::cout << "图像的宽 " << image.cols << "高为 " << image.rows << ", 通道数是 " << image.channels() << std::endl;
    cv::imshow("image", image);
    cv::waitKey(0);

    // 判断image的类型
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3){
        std::cout << "无法打开此图像 " << std::endl;
        return 0;
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    for (size_t y = 0; y < image.rows; y++){
        unsigned char * row_ptr = image.ptr<unsigned char>(y);
        for (size_t x = 0; x < image.cols; x++){
            unsigned char *data_ptr = &row_ptr[x * image.channels()];
            for (int c = 0; c != image.channels(); c++){
                unsigned char data = data_ptr[c];
            }
        }
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "访问图像一共使用时间 " << time_used.count() << " 秒" << std::endl;

    // 直接复制并不会导致复制图像 对这个图像进行赋值操作会直接改变原图像的内容
    cv::Mat image_another  = image;
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0);
    cv::imshow("image", image);

    // 可以使用clone() 函数这样会复制数据
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("image", image);
    cv::waitKey(0);


    cv::destroyAllWindows();

    std::cout << "Hello, World!" << std::endl;
    return 0;
}