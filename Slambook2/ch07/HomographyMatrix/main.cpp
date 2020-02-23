/****************************
 * 实现虚拟广告牌的效果。
 * 提供两张图，一张是“计算机视觉life”公众号的logo，另外一张是带广告牌的原图，请用单应矩阵实现将原图中广告牌替换为提供的logo的效果。
 * 利用OpenCV函数，通过鼠标点击来选择要替换的广告牌的四个顶点。
 *
****************************/
#include <iostream>
#include <opencv2/opencv.hpp>

struct userdata{
    cv::Mat im;
    std::vector<cv::Point2f> points;
};
void mouseHandler(int event, int x, int y, int flags, void * data_ptr){
    if (event == cv::EVENT_LBUTTONDOWN){
        userdata * data = ((userdata *) data_ptr);
        cv::circle(data->im, cv::Point(x, y), 3, cv::Scalar(0, 255, 255), 5, cv::LINE_AA);
        cv::imshow("Image", data->im);
        if (data->points.size() < 4){
            data->points.push_back(cv::Point2f(x, y));
        }
    }
}
int main() {

    cv::Mat image = cv::imread("cvlife.jpg");
    cv::Size size = image.size();

    //记录图像信息的处理机能
    std::vector<cv::Point2f> pts_src;
    pts_src.push_back(cv::Point2f(0, 0));
    pts_src.push_back(cv::Point2f(size.width - 1, 0));
    pts_src.push_back(cv::Point2f(size.width - 1, size.height - 1));
    pts_src.push_back(cv::Point2f(0, size.height - 1));

    // 读入广告牌
    cv::Mat im_dst = cv::imread("ad.jpg");

    cv::Mat im_temp = im_dst.clone();
    userdata data;
    data.im = im_temp;
    cv::imshow("Image", im_temp);
    std::cout << "Click on four corners of a billboard and then press ENTER " << std::endl;
    cv::setMouseCallback("Image", mouseHandler, &data);
    cv::waitKey(0);
    cv::Mat h = cv::findHomography(pts_src, data.points);   // 计算单应矩阵
    cv::warpPerspective(image, im_temp, h, im_temp.size());
    cv::imshow("im_temp", im_temp);
    cv::imwrite("cv.jpg", im_temp);
    cv::Point pts_dst[4];
    for (int i = 0; i < 4; i++){
        pts_dst[i] = data.points[i];
    }
    cv::fillConvexPoly(im_dst, pts_dst, 4, cv::Scalar(0), cv::LINE_AA); //把其余的地方填充成黑色.
    im_dst = im_dst + im_temp;



    cv::imshow("Image", im_dst);
    cv::waitKey(0);


    std::cout << "Hello, World!" << std::endl;
    return 0;
}