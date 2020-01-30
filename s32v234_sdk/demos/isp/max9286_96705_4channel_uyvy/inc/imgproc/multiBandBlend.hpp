//
// Created by Mario on 19-5-7.
//
// TODO: 以两层为起点
// TODO: 暂时基于输入图像为1280*720创建5层金字塔
// TODO: 忽略了OPENCV中的 UMAT OPENCL加速
//
// 金字塔大小：
// e:   80*45
// d:  160*90
// c:  320*180
// b:  640*360
// a: 1280*720
//

#pragma once

#define NUM_PYRAMID 2
#define NUM_IMAGE 4
#include <opencv2/core.hpp>
#include <opencv2/opencv_modules.hpp>
#include <vector>
#include <numeric>
#include <chrono>
#include <math.h>

/**
 * 将lut表转换到mat
 * @param lut
 * @param lutMat
 */
void lut2MatConverter(const LUT& lut, cv::Mat& lutMat)
{

    int width = lut.header.bev_img_width;
    int height = lut.header.bev_img_height;

    cv::Mat mat{ cv::Size(width, height), CV_32FC1 };
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int i = y * width + x;
            mat.at<uchar>(y, x) = lut.table[i].wt_fusion;
        }
    }
    lutMat = mat.clone();
}

/**
 * 对img，生成levels层高斯金字塔
 * @param img
 * @param levels
 * @return
 */
std::vector<cv::Mat> gaussianPyramid(cv::Mat img, int levels)
{
    std::vector<cv::Mat> _gaussianPyramid;
    _gaussianPyramid.emplace_back(img);
    cv::Mat currentImage = img;
    //cout << currentImage.size() << endl;
    for(int i = 1; i < levels; i++)
    {
        cv::Mat downsampleImage;
        cv::pyrDown(currentImage, downsampleImage); // Blurs an image and downsamples it.
        _gaussianPyramid.emplace_back(downsampleImage);
        //showImage("currentImage", currentImage);
        //cout << downsampleImage.size() << endl;
        currentImage = downsampleImage;
    }
    return _gaussianPyramid;
}

/**
 * 对单个摄像头的图像生成Laplace金字塔[2019-05-07, Mario]
 * @param images
 * @param lapPyr
 */
void createLaplacePyr(const cv::Mat& images, std::vector<cv::Mat>& Pyr){

    std::cout << "\033[33m" << "====== Debug Laplace ======" << std::endl;
    std::cout << "Origin Size: " << images.cols << "x" << images.rows << std::endl;

    assert(images.depth() == CV_8U);

    int num_levels = NUM_PYRAMID-1;

    // 表示降采样后的图像
    cv::Mat downNext;
    // 表示当前层的图像
    cv::Mat current = images;
    // 调用pyrDown函数，对img图像进行高斯模糊和降采样，得到downNext
    cv::pyrDown(images, downNext);

    std::cout << "\033[33m" << "downNext Size: " << downNext.cols << "x" << downNext.rows << "\033[0m" << std::endl;

    // 遍历金字塔的各个层
    for(int j = 1; j < num_levels; ++j)
    {
        // 表示对当前图像进行升采样得到的图像
        cv::Mat lvl_up;
        // 表示对当前图像进行降采样得到的图像
        cv::Mat lvl_down;

        // 调用pyrDown函数，得到lvl_down
        cv::pyrDown(downNext, lvl_down);

        // 调用pyrUp函数，得到lvl_up, expand运算
        cv::pyrUp(downNext, lvl_up, current.size());

        cv::subtract(current, lvl_up, Pyr[j-1], cv::noArray(), CV_8U);

        // 更新current
        current = downNext;
        // 更新downNext
        downNext = lvl_down;

        std::cout << "\033[33m" << "downNext Size: " << downNext.cols << "x" << downNext.rows << "\033[0m" << std::endl;

    }

    //得到金字塔的顶层图像
    {
        cv::Mat lvl_up;
        cv::pyrUp(downNext, lvl_up, current.size());

        cv::subtract(current, lvl_up, Pyr[num_levels-1], cv::noArray(), CV_8U);

        downNext.convertTo(Pyr[num_levels], CV_8U);
    }

//    imshow("Source Images", images);
//    cv::waitKey(1000);
//    for(int k = 0; k < NUM_PYRAMID; k ++){
//        imshow("Lapalce Images", Pyr[k]);
//        cv::waitKey(3000);
//    }

}

/**
 * 对四个摄像头, 对于单层Laplace金字塔，根据LUT表进行加权融合
 * @param Pyr               固定层金字塔的4个相机图片
 * @param birdviewPyr       固定层融合的俯视图
 * @param lut               日常LUT
 * @param level             第几层金字塔
 */
void linearCombination(const std::vector<cv::Mat>& Pyr, cv::Mat& birdviewPyr, const LUT& lut, const int level){

    int pyrFactor = pow(2,level-1);

    // 根据level缩放俯视图大小
    int lutWidth = lut.header.bev_img_width;
    int birdviewWidth = 800 / pyrFactor;
    int birdviewHeight = lut.header.bev_img_height / pyrFactor;

    std::cout << "\033[33m" << "LINEARCOMBINATION: pyrFactor: " << pyrFactor << "\033[0m" << std::endl;
    std::cout << "\033[33m" << "LINEARCOMBINATION: Pyr Size: " << Pyr[0].cols << "x" << Pyr[0].rows << "\033[0m" << std::endl;
    std::cout << "\033[33m" << "LINEARCOMBINATION: birdviewHeight Size: " << birdviewWidth << "x" << birdviewHeight << "\033[0m" << std::endl;
    std::cout << "\033[33m" << "LINEARCOMBINATION: pyrFactor: " << pyrFactor << "\033[0m" << std::endl;
    std::cout << "\033[33m" << "LINEARCOMBINATION: lut.header.bev_img_width : " << lut.header.bev_img_width  << "\033[0m" << std::endl;


    // TODO: delete birdview definition
    cv::Mat_<cv::Vec3b> birdview = cv::Mat_<cv::Vec3b>::zeros(cv::Size(birdviewWidth, birdviewHeight));

    for (int y = 0; y < birdviewHeight; ++y)
    {
        for (int x = 0; x < birdviewWidth; ++x)
        {
//            int idx = (y*pyrFactor) * (birdviewWidth*pyrFactor)   + x * pyrFactor;
            int idx = (y*pyrFactor) * (lutWidth) + x * pyrFactor;
            auto& p = lut.table[idx];

            int x1 = p.point_pos1.x / pyrFactor;
            int y1 = p.point_pos1.y / pyrFactor;

            int x2 = p.point_pos2.x / pyrFactor;
            int y2 = p.point_pos2.y / pyrFactor;

            double w = (double)p.wt_fusion / 255.0;

            birdview(y, x) = Pyr[p.pic1].at<cv::Vec3b>(y1, x1) * w + Pyr[p.pic2].at<cv::Vec3b>(y2, x2) * (1.0 - w);
        }
    }

//    cv::imshow("level", birdview);
//    cv::waitKey(3000);

    birdviewPyr = birdview.clone();

}

/**
 * 对四个摄像头, 对于单层Laplace金字塔，根据LUT的amsk图进行加权融合
 * @param Pyr               固定层金字塔的4个相机图片
 * @param birdviewPyr       固定层融合的俯视图
 * @param lut               日常LUT
 * @param level             第几层金字塔
 */
void linearCombination(const std::vector<cv::Mat>& Pyr, cv::Mat& birdviewPyr, const LUT& lut, const cv::Mat& lutPyr, const int level){

    int pyrFactor = pow(2,level-1);

    // 根据level缩放俯视图大小
    int lutWidth = lut.header.bev_img_width;
    int birdviewWidth = 800 / pyrFactor;
    int birdviewHeight = lut.header.bev_img_height / pyrFactor;

    std::cout << "\033[33m" << "LINEARCOMBINATION: pyrFactor: " << pyrFactor << "\033[0m" << std::endl;
    std::cout << "\033[33m" << "LINEARCOMBINATION: Pyr Size: " << Pyr[0].cols << "x" << Pyr[0].rows << "\033[0m" << std::endl;
    std::cout << "\033[33m" << "LINEARCOMBINATION: birdviewHeight Size: " << birdviewWidth << "x" << birdviewHeight << "\033[0m" << std::endl;
    std::cout << "\033[33m" << "LINEARCOMBINATION: pyrFactor: " << pyrFactor << "\033[0m" << std::endl;
    std::cout << "\033[33m" << "LINEARCOMBINATION: lut.header.bev_img_width : " << lut.header.bev_img_width  << "\033[0m" << std::endl;


    // TODO: delete birdview definition
    cv::Mat_<cv::Vec3b> birdview = cv::Mat_<cv::Vec3b>::zeros(cv::Size(birdviewWidth, birdviewHeight));

    for (int y = 0; y < birdviewHeight; ++y)
    {
        for (int x = 0; x < birdviewWidth; ++x)
        {
//            int idx = (y*pyrFactor) * (birdviewWidth*pyrFactor)   + x * pyrFactor;
            int idx = (y*pyrFactor) * (lutWidth) + x * pyrFactor;
            auto& p = lut.table[idx];

            int x1 = p.point_pos1.x / pyrFactor;
            int y1 = p.point_pos1.y / pyrFactor;

            int x2 = p.point_pos2.x / pyrFactor;
            int y2 = p.point_pos2.y / pyrFactor;

//            double w = (double)p.wt_fusion / 255.0;
            double w = (double)lutPyr.at<uchar>(y,x) / 255.0;

            birdview(y, x) = Pyr[p.pic1].at<cv::Vec3b>(y1, x1) * w + Pyr[p.pic2].at<cv::Vec3b>(y2, x2) * (1.0 - w);
        }
    }

//    cv::imshow("level", birdview);
//    cv::waitKey(3000);

    birdviewPyr = birdview.clone();

}

/**
 * 根据各层金字塔的俯视图，融合生成最终俯视图
 * @param birdviewPyrs
 * @param birdImage
 */
void restoreImageFromLaplacePyr(std::vector<cv::Mat>& birdviewPyrs, cv::Mat& birdImage){

    cv::Mat tmp;
    // 遍历拉普拉斯金字塔的各层
    for (int i = NUM_PYRAMID - 1; i > 0; --i)
    {

        std::cout << "\033[31m" << "restoreImageFromLaplacePyr: birdviewPyrs[i] Size: " << birdviewPyrs[i].cols << "x" << birdviewPyrs[i].rows << "\033[0m" << std::endl;
        std::cout << "\033[31m" << "restoreImageFromLaplacePyr: birdviewPyrs[i - 1] Size: " << birdviewPyrs[i - 1].cols << "x" << birdviewPyrs[i - 1].rows << "\033[0m" << std::endl;

        // expand运算
        cv::pyrUp(birdviewPyrs[i], tmp, birdviewPyrs[i - 1].size());
        cv::add(tmp, birdviewPyrs[i - 1], birdviewPyrs[i - 1]);
    }
    birdImage = birdviewPyrs[0].clone();
}

/**
 * 多频融合主函数
 * @param images
 * @param lut
 * @param birdImage
 */
void multiBandBlend(cv::Mat* images, const LUT& lut, cv::Mat& birdImage)
//cv::Mat multiBandBlend(const cv::Mat_<T>* images, const LUT& lut)
{

    // 创建Laplace金字塔
    std::vector<std::vector<cv::Mat>> lapPyrs;      // 第一个vector大小为4维，表示4个摄像机； 第二个vector大小为5维数组，表示5层金字塔
    for(int i = 0; i < NUM_IMAGE; ++i){
        std::vector<cv::Mat> lapPyr;
        lapPyr.resize(NUM_PYRAMID);
        createLaplacePyr(images[i], lapPyr);
        lapPyrs.emplace_back(lapPyr);
    }

    // 创建权重高斯金字塔
    cv::Mat lutMat;
    lut2MatConverter(lut, lutMat);
    std::vector<cv::Mat> lutPyrs;
    lutPyrs = gaussianPyramid(lutMat, NUM_PYRAMID);

    // 对于每一层Laplace金字塔，根据LUT表进行加权融合
    std::vector<cv::Mat> birdviewPyrs;              // birdviewPyrs一共5维，每张大小1080*810
    // 遍历每一层金字塔，每层金字塔会根据4个摄像头生成一张俯视拼接图
    for(int i = 0; i < NUM_PYRAMID; ++i){

        // 生成4个相机金字塔的输入,维数是4
        std::vector<cv::Mat> lapPyrLayer;

        for(int j = 0; j < NUM_IMAGE; ++j){
            lapPyrLayer.emplace_back(lapPyrs[j][i]);
        }
        // 这一层金字塔合成的俯视拼接图
        cv::Mat birdviewPyr;
        // 根据lut表生成俯视拼接图
#if 1
        linearCombination(lapPyrLayer, birdviewPyr, lut, i+1);
#else
        linearCombination(lapPyrLayer, birdviewPyr, lut, lutPyrs[i], i+1);
#endif
        birdviewPyrs.emplace_back(birdviewPyr);
    }

    // 根据加权融合好的Laplace金字塔，生成最终俯视图结果
    restoreImageFromLaplacePyr(birdviewPyrs, birdImage);
}
