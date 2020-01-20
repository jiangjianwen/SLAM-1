/*
 * 运动中的相机拍摄连续的两张照片,首先完成特征点的匹配,
 * 根据两帧图像对应的匹配点,计算基础矩阵,并利用该矩阵绘制出前
 * 10个特征点对应的极线.
 * */

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
int main() {
    cv::Mat rgb1 = cv::imread("rgb1.ppm");
    cv::Mat rgb2 = cv::imread("rgb2.ppm");



    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor;

    detector = cv::ORB::create();
    descriptor = cv::ORB::create();

    // 首先检测特征点
    std::vector<cv::KeyPoint> kp1, kp2;
    detector->detect(rgb1, kp1);
    detector->detect(rgb2, kp2);

    // 计算描述子
    cv::Mat desp1, desp2;
    descriptor->compute(rgb1, kp1, desp1);
    descriptor->compute(rgb2, kp2, desp2);

    //匹配描述姿
    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher;
    matcher.match(desp1, desp2, matches);
    std::cout << "Find total " << matches.size() << " matches. " << std::endl;

    // 筛选匹配对
    std::vector<cv::DMatch> goodMatches;
    double minDis = 9999;
    for (auto & matche : matches){
        if (matche.distance < minDis){
            minDis = matche.distance;
        }
    }
    for (auto & matche : matches){
        if (matche.distance < 10 * minDis){
            goodMatches.push_back(matche);
        }
    }

    // 记录选择的匹配特征点的索引向量
    // 首先查询每副图像特征点的索引
    // 将索引指定的特征点转换成2D点
    std::vector<cv::Point2f> pts1, pts2;
    for (size_t i=0; i < goodMatches.size(); i++){
        pts1.push_back(kp1[goodMatches[i].queryIdx].pt);
        pts2.push_back(kp2[goodMatches[i].trainIdx].pt);
    }

    // 首先计算基础矩阵
    cv::Mat fundamental_matrix;
    fundamental_matrix = cv::findEssentialMat(pts1, pts2, cv::FM_8POINT);
    std::cout << "fundamental_matrix is " << std::endl << fundamental_matrix << std::endl;

    // 通过使用基础矩阵, 在对应的图像上绘制外极线

    cv::Mat image_match;
    cv::drawMatches(rgb1, kp1, rgb2, kp2, goodMatches, image_match);

    // 绘制出前十个匹配点对应的极线
    std::vector<cv::Vec<float, 3>> epipolar1;
    std::vector<cv::Vec<float, 3>> epipolar2;
    cv::computeCorrespondEpilines(pts1, 1, fundamental_matrix, epipolar1);
    cv::computeCorrespondEpilines(pts2, 2, fundamental_matrix, epipolar2);

    cv::RNG rng = cv::theRNG(); // 用于随机生成RGB的颜色
    for (int i =0; i < 10; i++){
        cv::Scalar color(rng(256), rng(256), rng(256));
        cv::circle(rgb1, pts1[i], 5, color);
        cv::line(rgb1, cv::Point(0, -epipolar1[i][2]/epipolar1[i][1]),
                cv::Point(rgb1.cols, -(epipolar1[i][0]*rgb1.cols+epipolar1[i][2])/epipolar1[i][1]), color);
        cv::circle(rgb2, pts2[i], 5, color);
        cv::line(rgb2, cv::Point(0, -epipolar2[i][2]/epipolar2[i][1]),
                cv::Point(rgb2.cols, -(epipolar2[i][0]*rgb2.cols+epipolar2[i][2])/epipolar2[i][1]), color);


    }

    cv::imshow("good matches", image_match);
    cv::imshow("rgb1", rgb1);
    cv::imshow("rgb2", rgb2);

    cv::waitKey(0);
    return 0;
}