#include <iostream>
#include "DBoW3/DBoW3.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

// 根据data目录下的十张图训练字典, 第一幅图像与第十副图像是在同一个地点
int main() {

    // read the image
    cout << "reading images.... " << endl;
    vector<Mat> images;
    for (int i = 0; i < 10; i++){
        string path = "./data/" + to_string(i+1) + ".png";
        images.push_back(imread(path));
    }
    // 检测ORB特征
    cout << "detecting ORB features ... " << endl;
    Ptr<Feature2D> detector = ORB::create();
    vector<Mat> descriptors;
    for (Mat& image : images){
        vector<KeyPoint> keypoints;
        Mat descriptor;
        detector->detectAndCompute(image, Mat(), keypoints, descriptor);
        descriptors.push_back(descriptor);
    }

    // 创建一个字典
    cout << "creating vocabulary .... " << endl;
    DBoW3::Vocabulary vocabulary;
    vocabulary.create(descriptors);
    cout << "vocabulary info: " << vocabulary << endl;
    vocabulary.save("vocabulary.yml.gz");
    cout << "done" << endl;


    return 0;
}