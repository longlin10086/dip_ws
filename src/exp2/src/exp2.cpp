#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"
#include <math.h>
#include <cv_bridge/cv_bridge.h>

enum CameraState
{
    COMPUTER = 0,
    ZED,
    REALSENSE,
    IMAGE
};
CameraState state = IMAGE;
#define pi 3.1415926
using namespace cv;

//空域均值滤波函数
void meanFilter(Mat &input)
{

    //生成模板
    int T_size = 9; 
    //int T_size = 3;                                   // 模板大小
    Mat Template = Mat::zeros(T_size, T_size, CV_64F); // 初始化模板矩阵
    /*** 第一步：在此处填充均值滤波模板 ***/
    Template = Mat::ones(T_size, T_size, CV_64F) / (T_size * T_size);

    // 卷积
    Mat output = Mat::zeros(input.size(), CV_64F);
    Mat out = output;

    /*** 第二步：填充模板与输入图像的卷积代码 ***/    
    cv::filter2D(input, output, -1, Template);


    output.convertTo(output, CV_8UC1);
    imshow("mean_filtered_image", output);

    // cv::blur(input, out, cv::Size(9, 9));
    // out.convertTo(out, CV_8UC1);
    // imshow("out", out);
}
// 空域高斯滤波器函数
void gaussianFilter(Mat &input, double sigma)
{

    //利用高斯函数生成模板
    int T_size = 9;                                    // 模板大小
    Mat Template = Mat::zeros(T_size, T_size, CV_64F); // 初始化模板矩阵
    int center = round(T_size / 2);                    // 模板中心位置
    double sum = 0.0;
    
    for (int i = 0; i < T_size; i++)
    {
        for (int j = 0; j < T_size; j++)
        {

            /*** 第三步：在此处填充高斯滤波模板元素计算代码 ***/

            int x = i - center;
            int y = j - center;
            Template.at<double>(i, j) = exp(-(x * x + y * y) / (2 * sigma * sigma)) / (2 * CV_PI * sigma * sigma);
            sum += Template.at<double>(i, j); //用于归一化模板元素
        }
    }


    for (int i = 0; i < T_size; i++)
    {
        for (int j = 0; j < T_size; j++)
        {

            /*** 第四步：在此处填充模板归一化代码 ***/

            Template.at<double>(i, j) /= sum; // 归一化模板，使得所有元素之和为1

        }
    }
    // 卷积
    Mat output = Mat::zeros(input.size(), CV_64F);

    /*** 第五步：同第二步，填充模板与输入图像的卷积代码 ***/ 

    int offset = T_size / 2; // 定义模板的偏移量
    for (int i = offset; i < input.rows - offset; i++)
    {
        for (int j = offset; j < input.cols - offset; j++)
        {
            double sum = 0.0;
            for (int m = -offset; m <= offset; m++)
            {
                for (int n = -offset; n <= offset; n++)
                {
                    sum += input.at<uchar>(i + m, j + n) * Template.at<double>(m + offset, n + offset);
                }
            }
            output.at<double>(i, j) = sum;
        }
    }

     // cv::GaussianBlur(input, output, cv::Size(T_size, T_size), sigma, 0);

    output.convertTo(output, CV_8UC1);
    imshow("spatial_filtered_image", output);
}
// 锐化空域滤波
void sharpenFilter(Mat &input)
{

    //生成模板
    int T_size = 3;                                    // 模板大小
    Mat Template = Mat::zeros(T_size, T_size, CV_64F); // 初始化模板矩阵
    /*** 第六步：填充锐化滤波模板 ***/   
    Template.at<double>(0, 1) = -1;
    Template.at<double>(1, 0) = -1;
    Template.at<double>(1, 1) = 5;
    Template.at<double>(1, 2) = -1;
    Template.at<double>(2, 1) = -1;

    // 卷积
    Mat output = Mat::zeros(input.size(), CV_64F);

    /*** 第七步：同第二步，填充模板与输入图像的卷积代码 ***/    

    int offset = T_size / 2; // 定义模板的偏移量
    for (int i = offset; i < input.rows - offset; i++)
    {
        for (int j = offset; j < input.cols - offset; j++)
        {
            double sum = 0.0;
            for (int m = -offset; m <= offset; m++)
            {
                for (int n = -offset; n <= offset; n++)
                {
                    sum += input.at<uchar>(i + m, j + n) * Template.at<double>(m + offset, n + offset);
                }
            }
            output.at<double>(i, j) = sum;
        }
    }



    output.convertTo(output, CV_8UC1);
    imshow("sharpen_filtered_image", output);
}
// 膨胀函数
void Dilate(Mat &Src)
{
    Mat Dst = Src.clone();
    Dst.convertTo(Dst, CV_64F);

    // Mat binarySrc;
    // threshold(Src, binarySrc, 128, 255, THRESH_BINARY); // 使用阈值128进行二值化处理

    Dst.convertTo(Dst, CV_64F);

    /*** 第八步：填充膨胀代码 ***/    

    int T_size = 3; // 定义膨胀操作的模板大小
    int offset = T_size / 2;
    
    for (int i = offset; i < Src.rows - offset; i++)
    {
        for (int j = offset; j < Src.cols - offset; j++)
        {
            double maxVal = -1;
            for (int m = -offset; m <= offset; m++)
            {
                for (int n = -offset; n <= offset; n++)
                {
                    double val = Src.at<uchar>(i + m, j + n);
                    if (val > maxVal)
                    {
                        maxVal = val;
                    }
                }
            }
            Dst.at<double>(i, j) = maxVal; // 将最大值赋给目标图像
        }
    }

    // cv::dilate(Src, Dst, Mat());



    Dst.convertTo(Dst, CV_8UC1);
    imshow("dilate", Dst);
}
// 腐蚀函数
void Erode(Mat &Src)
{
    Mat Dst = Src.clone();
    Dst.convertTo(Dst, CV_64F);

    // 将图像二值化处理
    // Mat binarySrc;
    // threshold(Src, binarySrc, 128, 255, THRESH_BINARY); // 使用阈值128进行二值化处理

    /*** 第九步：填充腐蚀代码 ***/    

    int T_size = 3; // 定义腐蚀操作的模板大小
    int offset = T_size / 2;
    
    for (int i = offset; i < Src.rows - offset; i++)
    {
        for (int j = offset; j < Src.cols - offset; j++)
        {
            double minVal = 256; // 初始化为超过像素可能的最大值
            for (int m = -offset; m <= offset; m++)
            {
                for (int n = -offset; n <= offset; n++)
                {
                    double val = Src.at<uchar>(i + m, j + n);
                    if (val < minVal)
                    {
                        minVal = val;
                    }
                }
            }
             Dst.at<double>(i, j) = minVal; // 将最小值赋给目标图像
        }
    }

    // cv::erode(Src, Dst, Mat());

    Dst.convertTo(Dst, CV_8UC1);
    imshow("erode", Dst);
}

Mat frame_msg;
void rcvCameraCallBack(const sensor_msgs::Image::ConstPtr& img)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
    frame_msg = cv_ptr->image;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exp2_node"); // 初始化 ROS 节点
    ros::NodeHandle n;
    ros::Subscriber camera_sub;
    VideoCapture capture;

    Mat frIn;                                        // 当前帧图片

    if(state == COMPUTER)
    {
        capture.open(0);     
        if (!capture.isOpened())
        {
            printf("电脑摄像头没有正常打开\n");
            return 0;
        }
        waitKey(1000);
    }
    else if(state == ZED)
    {
        capture.open(4);     
        if (!capture.isOpened())
        {
            printf("ZED摄像头没有正常打开\n");
            return 0;
        }
        waitKey(1000);
    }
    else if(state == REALSENSE)
    {
        camera_sub = n.subscribe("/camera/color/image_raw",1,rcvCameraCallBack);
        waitKey(1000);
    }
    else if (state == IMAGE) {
        std::string image_path = samples::findFile("/home/longlin/WorkSpace/dip_ws/monica.jpeg");
        frIn = imread(image_path, IMREAD_COLOR);
        if(frIn.empty())
        {
            std::cout << "Could not read the image: " << image_path << std::endl;
            return 0;
        }
        imshow("img", frIn);
        waitKey(100);
    }

    ros::Rate loop_rate(10); // 设置循环频率为10Hz
    while (ros::ok())
    {

        if(state == COMPUTER)
        {
            capture.read(frIn);
            if (frIn.empty())
            {
                printf("没有获取到电脑图像\n");
                continue;
            }
        }
        else if(state == ZED)
        {
            capture.read(frIn);
            if (frIn.empty())
            {
                printf("没有获取到ZED图像\n");
                continue;
            }
            frIn = frIn(cv::Rect(0,0,frIn.cols/2,frIn.rows));//截取zed的左目图片
        }
        else if(state == REALSENSE)
        {
            if(frame_msg.cols == 0)
            {
                printf("没有获取到realsense图像\n");
                ros::spinOnce();
                continue;
            }
            frIn = frame_msg;
        }
        else if (state == IMAGE) {
            waitKey(1000);
        }

        Mat frame = frIn;
        cvtColor(frame, frame, COLOR_BGR2GRAY, 0);
        imshow("original_image", frame);
        //空域均值滤波
	    meanFilter(frame);
	
        // 空域高斯滤波
        double sigma = 2.5;
        gaussianFilter(frame, sigma);

        //空域锐化滤波
        sharpenFilter(frame);

        // 膨胀函数
        Dilate(frame);

        // 腐蚀函数
        Erode(frame);

        ros::spinOnce();
        waitKey(5);
        loop_rate.sleep();
    }
    return 0;
}
