#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>
#include <sophus/se3.hpp>
#include <pangolin/pangolin.h>

using TrajectoryType = std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d >>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

// 在pangolin 中画图 写好 无需调整
void showPointCloud(const std::vector<Vector6d, Eigen::aligned_allocator<Vector6d >> &pointcloud);
int main() {

    // 用来保存深度图和彩色图
    std::vector<cv::Mat> colorImage, depthImages;
    TrajectoryType poses;
    std::ifstream fin("./pose.txt");
    if (!fin){
        std::cerr << "请在有pose.txt 的目录下面运行此程序 " << std::endl;
        return -1;
    }
    for (int i =0; i < 5; i++){
        boost::format fmt("./%s/%d.%s");    // 图像文件格式
        colorImage.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        depthImages.push_back(cv::imread((fmt % "depth" % (i + 1) % "pgm").str(), -1)); // 使用-1 读取原始图像
        double data[7] = { 0 };
        for (auto & d : data){
            fin >> d;
        }
        Sophus::SE3d pose(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(pose);
    }
    // 计算点云拼接
    // 相机内参
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    std::vector<Vector6d, Eigen::aligned_allocator<Vector6d >> pointcloud;
    pointcloud.reserve(1000000);
    for (int i = 0; i < 5; i++){
        std::cout << "转换图像中: " << i + 1 << std::endl;
        cv::Mat color = colorImage[i];
        cv::Mat depth = depthImages[i];
        Sophus::SE3d  T = poses[i];
        for (int v = 0 ; v < color.rows; v++){
            for (int u =0; u < color.cols; u++){
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                if (d == 0) continue;
                Eigen::Vector3d point;
                point[2] = static_cast<double>(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointWorld = T * point;

                Vector6d p;
                p.head<3>() = pointWorld;
                p[5] = color.data[v * color.step + u * color.channels()];
                p[4] = color.data[v * color.step + u * color.channels() + 1];
                p[3] = color.data[v * color.step + u * color.channels() + 2];
                pointcloud.push_back(p);
            }
        }
    }

    std::cout << "点云共有 " << pointcloud.size() << " 个点 " << std::endl;
    showPointCloud(pointcloud);

    std::cout << "Hello, World!" << std::endl;
    return 0;
}

void showPointCloud(const std::vector<Vector6d, Eigen::aligned_allocator<Vector6d >> &pointcloud){
    if (pointcloud.empty()){
        std::cerr << "Point cloud is empty !! " << std::endl;
        return;
    }
    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;

}
