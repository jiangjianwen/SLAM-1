/*
 * 提取图像中的ORB特征
 * */
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

int main(int argc, char** argv) {
    if (argc != 3){
        std::cout << "usage: feature extraction image1 image2 " << std::endl;
        return -1;
    }
    // 读取图像
    cv::Mat image1 = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::Mat image2 = cv::imread(argv[2], cv::IMREAD_COLOR);
    if (image1.empty() || image2.empty()){
        std::cout << "can not find image.." << std::endl;
        return -1;
    }
    // 初始化
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2; // 记录关键点周围的描述子
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    // 通过使用汉明距离来实现两个图像之间的匹配 ORB特征一般使用Hamming距离来进行匹配
    // 汉明距离是指图形的描述子有多少位是不一样的
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    // 第一步: 检测Oriented FAST角点的位置
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    descriptor->detect(image1, keypoints1);
    descriptor->detect(image2, keypoints2);

    // 第二步: 根据角点位置计算BRIEF描述子
    descriptor->compute(image1, keypoints1, descriptors1);
    descriptor->compute(image2, keypoints2, descriptors2);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "extract ORB cost = " << time_used.count() << std::endl;

    cv::Mat image_out1;
    cv::drawKeypoints(image1, keypoints1, image_out1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("ORB features", image_out1);

    // 第三步: 计算两幅图像中的BRIEF描述子进行匹配, 使用Hamming距离,这里可能得到的最小距离会非常小
    // 由于它会为每个点都找到一个匹配,而有些点在另外一副图像中是提取不到的,因此这其中的唷一些点会存在一些误匹配的情况
    std::vector<cv::DMatch> matches;
    t1 = std::chrono::steady_clock::now();
    matcher->match(descriptors1, descriptors2, matches);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    // 第四步进行匹配点对的筛选
    // 计算该容器内汉明距离的最小距离和最大距离
    auto min_max = minmax_element(matches.begin(), matches.end(), [](const cv::DMatch& m1, const cv::DMatch&m2)
                    { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    std::cout << "--Max dist: " << max_dist << std::endl;
    std::cout << "--Min dist: " << min_dist << std::endl;

    // 当描述子之间的距离大于两倍的最小距离时, 即认为匹配有误. 但是有时最小距离会非常小,所以要设置一个经验值30最为下限
    std::vector<cv::DMatch> good_mathches;
    for (int i = 0; i < descriptors1.rows; i++){
        if (matches[i].distance <= std::max(2 * min_dist, 30.0)){
            good_mathches.push_back(matches[i]);
        }
    }

    // 第五步: 绘制匹配结果
    cv::Mat image_match;
    cv::Mat image_goodmatch;
    cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, image_match);
    cv::drawMatches(image1, keypoints1, image2, keypoints2, good_mathches, image_goodmatch);
    cv::imshow("all matches", image_match);
    cv::imshow("good matches", image_goodmatch);



    cv::waitKey(0);
    return 0;
}