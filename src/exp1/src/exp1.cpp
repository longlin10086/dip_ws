#include <stdlib.h>

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "iostream"
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/Twist.h"


ros::Publisher vel_pub;

enum CameraState
{
    COMPUTER = 0,
    ZED,
    REALSENSE,
    IMAGE
};
CameraState state = IMAGE;

#define N 255 //灰度level
using namespace std;
using namespace cv;

//getHistImage()--画图像直方图
Mat getHistImage( Mat hist)
{
    Scalar color(172, 172, 100);//划线颜色
    Scalar Background(255,255,255);//背景颜色
    int thickness = 2;	//划线宽度
    int histss[256] = {0};

    /*** 第一步：下面计算不同灰度值的像素分布 ***/
    Mat g_hist;
    bool uniform = true, accumulate = false;
    int hist_size = 256; 
    float range[] = { 0, 256 };  
    const float* histRange = { range };

    calcHist(&hist, 1, 0, Mat(), g_hist, 1, &hist_size, &histRange, uniform, accumulate);

    int histHeight = 500;  // 直方图高度
    normalize(g_hist, g_hist, 0, histHeight, NORM_MINMAX, -1, Mat());

    int histSize = 500;
    Mat histImage(histSize, histSize, CV_8UC3, Background );//绘制背景

    int bin_w = cvRound( (double) histSize/hist_size );

    for (int h = 0; h < 256; h++) {

    /*** 第二步：画出像素的直方图分布 ***/
        line(histImage,
            Point((h - 1)*bin_w, histSize - cvRound(g_hist.at<float>(h - 1))),
            Point(h*bin_w, histSize - cvRound(g_hist.at<float>(h))),
            color, thickness);
    }
    return histImage;
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
	ros::init(argc, argv, "exp1_node"); // 初始化 ROS 节点
    ros::NodeHandle n;
    ros::Subscriber camera_sub;
	VideoCapture capture;

    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    Mat frame;//当前帧图片

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
        frame = imread(image_path, IMREAD_COLOR);
        if(frame.empty())
        {
            std::cout << "Could not read the image: " << image_path << std::endl;
            return 0;
        }
        imshow("img", frame);
        waitKey(1000);
    }


	int Grayscale[N];//灰度级
	int Grayscale2[N];//均衡化以后的灰度级
	float Gray_f[N];//频率
	int Gray_c[N];//累计密度
    ros::Rate loop_rate(10); // 设置循环频率为10Hz
	while (ros::ok())
	{
        if(state == COMPUTER)
        {
            capture.read(frame);
            if (frame.empty())
            {
                printf("没有获取到电脑图像\n");
                continue;
            }
        }
        else if(state == ZED)
        {
            capture.read(frame);
            if (frame.empty())
            {
                printf("没有获取到ZED图像\n");
                continue;
            }
            frame = frame(cv::Rect(0,0,frame.cols/2,frame.rows));//截取zed的左目图片
        }
        else if(state == REALSENSE)
        {
            if(frame_msg.cols == 0)
            {
                printf("没有获取到realsense图像\n");
                ros::spinOnce();
                continue;
            }
			frame = frame_msg;
        }
        else if (state == IMAGE) {
            waitKey(1000);
        }


		Mat frIn = frame; 
		Mat New;
		cvtColor(frIn,frIn,COLOR_RGB2GRAY,0);

    	/*** 第三步：直方图均衡化处理 ***/
	
        equalizeHist(frIn, New);
	
		Mat last = getHistImage(New);
		Mat origi= getHistImage(frIn);
		imshow("his",last);//均衡化后直方图
		imshow("origi",origi);//原直方图
		imshow("Histed",New);//均衡化后图像
		imshow("Origin",frIn);//原图像


    	/*** 第四步：参考demo程序，添加让小车原地旋转代码 ***/
        geometry_msgs::Twist vel_msg;
        // 设置线速度为0，确保小车不向前或向任何方向移动
        vel_msg.linear.x = 0.0;
        vel_msg.linear.y = 0.0;
        vel_msg.linear.z = 0.0;

        // 设置角速度，z方向表示绕z轴旋转，小车将原地旋转
        vel_msg.angular.x = 0.0;
        vel_msg.angular.y = 0.0;
        vel_msg.angular.z = 1.0; // 角速度值可调节旋转速度

        // 发布消息，让小车原地旋转
        vel_pub.publish(vel_msg);


        ros::spinOnce(); // 处理回调函数
        waitKey(5);
        loop_rate.sleep(); // 控制循环速率
	
	}
	return 0;
}
