#include <iostream>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>

using namespace std;
// Eigen::aligned_allocator<Eigen::Vector2d>使用Eigen库的对齐方式来初始化矩阵
using VecVector2d = vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;
// 相机内参
double fx = 718.756, fy = 718.856, cx = 607.1928, cy = 185.2175;
// baseline
double baseline = 0.573;
// file paths
string left_file = "./left.png";
string disparity_file = "./disparity.png";
boost::format fmt_others("./%06d.png");

// 重定义一些类型
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix26d = Eigen::Matrix<double, 2, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

// 雅克比矩阵
class JacobianAccumulator{
public:
    JacobianAccumulator(
            const cv::Mat &img1_,
            const cv::Mat &img2_,
            const VecVector2d &px_ref_,
            const vector<double> depth_ref_,
            Sophus::SE3d &T21_
            ) : img1(img1_), img2(img2_), px_ref(px_ref_), depth_ref(depth_ref_), T21(T21_){
        projection = VecVector2d(px_ref.size(), Eigen::Vector2d(0, 0));
    }
    /// accumulate jacobians in a range
    void accumulate_jacobian(const cv::Range &range);

    /// get hessian matrix
    Matrix6d hessian() const { return H; }

    /// get bias
    Vector6d bias() const { return b; }

    /// get total cost
    double cost_func() const { return cost; }

    /// get projected points
    VecVector2d projected_points() const { return projection; }

    /// reset h, b, cost to zero
    void reset() {
        H = Matrix6d::Zero();
        b = Vector6d::Zero();
        cost = 0;
    }
private:
    const cv::Mat &img1;
    const cv::Mat &img2;
    const VecVector2d &px_ref;
    const vector<double> depth_ref;
    Sophus::SE3d &T21;
    VecVector2d projection;  // projected points 投影点

    std::mutex hession_mutex;
    Matrix6d H = Matrix6d::Zero();
    Vector6d b = Vector6d::Zero();
    double cost = 0;
};

/**
 * pose estimation using direct method 直接法利用多层光流
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3d &T21
);

/**
 * pose estimation using direct method 直接法使用单层光流
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3d &T21
);

// bilinear interpolation 双线性插值获取图像浮点数位置的像素值
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    // boundary check
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= img.cols) x = img.cols - 1;
    if (y >= img.rows) y = img.rows - 1;
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}

int main(int argc, char **argv) {

    // 这两幅图像可以确定left.png图像中任意一点像素的深度值,也就可以获得三维坐标
    cv::Mat left_img = cv::imread(left_file, 0);
    cv::Mat disparity_img = cv::imread(disparity_file, 0);

    // 随机在第一幅图像中选取像素点,并计算出这些像素点所对应位置的3位坐标
    cv::RNG rng;
    int nPoints = 2000;
    int boarder = 20;
    VecVector2d  pixels_ref;
    vector<double> depth_ref;

    // generate pixels in ref and load depth data
    for (int i = 0; i < nPoints; i++) {
        int x = rng.uniform(boarder, left_img.cols - boarder);  // don't pick pixels close to boarder
        int y = rng.uniform(boarder, left_img.rows - boarder);  // don't pick pixels close to boarder
        int disparity = disparity_img.at<uchar>(y, x);
        double depth = fx * baseline / disparity; // you know this is disparity to depth
        depth_ref.push_back(depth);
        pixels_ref.push_back(Eigen::Vector2d(x, y));
    }

    // 使用这些信息估计01-05.png图像的位姿
    Sophus::SE3d T_cur_ref;  // 从第一幅图像到第二幅图像的相机的位姿变换

    for (int i = 1; i < 6; i++){
        cv::Mat img = cv::imread((fmt_others % i).str(), 0);
        // 单层直接法
        // DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
        // 多层直接法
        DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
    }

    return 0;
}

void JacobianAccumulator::accumulate_jacobian(const cv::Range &range){
    // parameters
    const int half_patch_size = 4;
    int cnt_good = 0;
    Matrix6d hessian = Matrix6d::Zero();
    Vector6d bias = Vector6d::Zero();
    double cost_tmp = 0;
    for (size_t i = range.start; i < range.end; i++){
        // 计算在第二幅图像中的重投影
        // 首先计算第一幅图像像素点的空间坐标
        Eigen::Vector3d point_ref = depth_ref[i] * Eigen::Vector3d((px_ref[i][0] - cx) / fx, (px_ref[i][1] - cy) / fy, 1);
        // 通过位姿矩阵将点转换到第二幅图像下面
        Eigen::Vector3d point_cur = T21 * point_ref;
        if (point_cur[2] < 0){ // 如果深度值小于0,就说明深度值不合法
            continue;
        }
        // 通过空间点计算对应的像素坐标
        float u = fx * point_cur[0] / point_cur[2] + cx, v = fy * point_cur[1] / point_cur[2] + cy;
        if (u < half_patch_size || u > img2.cols - half_patch_size || v < half_patch_size ||
            v > img2.rows - half_patch_size){
            continue;
        }
        // 把投影点保存到projection中
        projection[i] = Eigen::Vector2d(u, v);
        double X = point_cur[0], Y = point_cur[1], Z = point_cur[2],
               Z2 = Z * Z, Z_inv = 1.0 / Z, Z2_inv = Z_inv * Z_inv;
        cnt_good++;
        for (int x = -half_patch_size; x <= half_patch_size; x++){
            for (int y = -half_patch_size; y <= half_patch_size; y++){
                // 计算根据当前的R和t得到对应在图像二中的像素与图像一中像素的差别
                double error = GetPixelValue(img1, px_ref[i][0] + x, px_ref[i][1] + y) -
                               GetPixelValue(img2, u + x, v + y);
                Matrix26d J_pixel_xi;  // 像素坐标对扰动模型的雅克比矩阵, 对应书中220页8.19的后半部分
                Eigen::Vector2d J_img_pixel; // 像素的梯度值,对应书中220页8.19的前半部分,这里使用会灰度值之差除以2作为梯度值

                J_pixel_xi(0, 0) = fx * Z_inv;
                J_pixel_xi(0, 1) = 0;
                J_pixel_xi(0, 2) = -fx * X * Z2_inv;
                J_pixel_xi(0, 3) = -fx * X * Y * Z2_inv;
                J_pixel_xi(0, 4) = fx + fx * X * X * Z2_inv;
                J_pixel_xi(0, 5) = -fx * Y * Z_inv;

                J_pixel_xi(1, 0) = 0;
                J_pixel_xi(1, 1) = fy * Z_inv;
                J_pixel_xi(1, 2) = -fy * Y * Z2_inv;
                J_pixel_xi(1, 3) = -fy - fy * Y * Y * Z2_inv;
                J_pixel_xi(1, 4) = fy * X * Y * Z2_inv;
                J_pixel_xi(1, 5) = fy * X * Z_inv;

                J_img_pixel = Eigen::Vector2d(
                        0.5 * (GetPixelValue(img2, u + 1 + x, v + y) - GetPixelValue(img2, u - 1 + x, v + y)),
                        0.5 * (GetPixelValue(img2, u + x, v + 1 + y) - GetPixelValue(img2, u + x, v - 1 + y))
                );

                // 相乘得到总的雅克比矩阵
                Vector6d J = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose();

                // 根据最小二乘法得到对应的hession, bias
                hessian += J * J.transpose();
                bias += -error * J;
                cost_tmp += error * error;

            }
        }
        if (cnt_good){
            // set hessian, bias and cost
            unique_lock<mutex> lck(hession_mutex);  // 用于对资源进行上锁和解锁.
            H += hessian;
            b += bias;
            cost += cost_tmp / cnt_good;
        }

    }
}

void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3d &T21){

    const int iterations = 10;
    double cost  = 0, lastCost = 0;
    auto t1 = chrono::steady_clock::now();
    JacobianAccumulator jaco_accu(img1, img2, px_ref, depth_ref, T21);

    for (int iter = 0; iter < iterations; iter++){
        jaco_accu.reset();
        cv::parallel_for_(cv::Range(0, px_ref.size()),
                          std::bind(&JacobianAccumulator::accumulate_jacobian, &jaco_accu, std::placeholders::_1));
        Matrix6d H = jaco_accu.hessian();
        Vector6d b = jaco_accu.bias();

        // 求解更新,将新值放入到更新中
        Vector6d update = H.ldlt().solve(b);
        T21 = Sophus::SE3d::exp(update) * T21;
        cost = jaco_accu.cost_func();

        if (std::isnan(update[0])){
            // 有些时候当图像是全黑或者全白就会计算不出来梯度信息
            cout << "update is nan " << endl;
            break;
        }
        if (iter > 0 && cost > lastCost){
            cout << "cost increased:  " << cost << ", " << lastCost << endl;
            break;
        }
        if (update.norm() < 1e-3){
            // converge
            break;
        }
        lastCost = cost;
        cout << "iteration: " << iter << ", cost " << cost << endl;
    }

    cout << "T21 = \n" << T21.matrix() << endl;
    auto t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "direct method for signal layer: " << time_used.count() << endl;

    // 画出极线
    cv::Mat img2_show;
    cv::cvtColor(img2, img2_show, cv::COLOR_GRAY2BGR);
    VecVector2d  projection = jaco_accu.projected_points();
    for (size_t i = 0; i < px_ref.size(); i++){
        auto p_ref = px_ref[i];
        auto p_cur = projection[i];
        if (p_cur[0] > 0 && p_cur[1] > 0){
            cv::circle(img2_show, cv::Point2f(p_cur[0], p_cur[1]), 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_show, cv::Point2f(p_ref[0], p_ref[1]), cv::Point2f(p_cur[0], p_cur[1]), cv::Scalar(0, 250, 0));
        }
    }
    cv::imshow("current", img2_show);
    cv::waitKey(0);
}

void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3d &T21){
    // 多层光流法
    // 设置参数
    int pyamids = 4;  // 金字塔的层数
    double pyramid_scale = 0.5;  // 金字塔每次缩小原图的0.5倍
    double scales[] = {1.0, 0.5, 0.25, 0.125}; // 图像的缩放比例

    // 构建金字塔
    vector<cv::Mat> pyr1, pyr2;   // 图像金字塔
    for (int i = 0; i < pyamids; i++){
        if (i == 0){
            // 金字塔的第一层就是原始图像
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        }
        else{
            cv::Mat img1_pyr, img2_pyr;
            /*
            OpenCV图像缩放使用的函数是：resize
            void resize(InputArray src, OutputArray dst, Size dsize, double fx=0, double fy=0, int interpolation=INTER_LINEAR )
            参数含义：
            InputArray src     -原图像
            OutputArray dst    -输出图像
            Size dsize         -目标图像的大小
            double fx=0        -在x轴上的缩放比例
            double fy=0        -在y轴上的缩放比例
            int interpolation  -插值方式，有以下四种方式

            INTER_NEAREST      -最近邻插值
            INTER_LINEAR  -双线性插值 (缺省使用)
            INTER_AREA    -使用象素关系重采样，当图像缩小时候，该方法可以避免波纹出现。当图像放大时，类似于 INTER_NN 方法。
            INTER_CUBIC   -立方插值。

            说明：dsize与fx和fy必须不能同时为零
            */
            // 每一层的原图就是前一层 图像坐标系列对应x轴
            cv::resize(pyr1[i - 1], img1_pyr, cv::Size(pyr1[i-1].cols * pyramid_scale, pyr1[i-1].rows * pyramid_scale));
            cv::resize(pyr2[i - 1], img2_pyr,
                       cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
            pyr1.push_back(img1_pyr);
            pyr2.push_back(img2_pyr);
        }
    }
    double fxG = fx, fyG = fy, cxG = cx, cyG = cy;
    // 下面首先将关键点转换到对应的金字塔的层数中开始进行单层直接发
    for (int level = pyamids - 1; level >= 0; level--){
        VecVector2d px_ref_pyr;  // 用于设置对应层的关键点
        for (auto &px : px_ref){
            px_ref_pyr.push_back(scales[level] * px);  // 缩放比例已经存储好
        }
        // 相机的内参在不同的缩放比例下面也将会发生改变
        fx = fxG * scales[level];
        fy = fyG * scales[level];
        cx = cxG * scales[level];
        cy = cyG * scales[level];
        DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);

    }

}