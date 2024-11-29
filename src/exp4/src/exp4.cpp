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


using namespace cv;
using namespace std;
void Gaussian(const Mat &input, Mat &output, double sigma)
{
    if (output.rows != input.rows || output.cols != input.cols || output.channels() != input.channels())
        return;
    int kernel_size = 9;
    double gaussian_kernel[kernel_size][kernel_size];

    /*** 第一步：结合实验二，在此处填充高斯滤波代码 ***/
    // 1. 计算高斯核
    int center = kernel_size / 2;
    double sum = 0.0; // 用于归一化
    
    // 计算高斯核的每个元素
    for(int i = 0; i < kernel_size; i++) {
        for(int j = 0; j < kernel_size; j++) {
            double x = i - center;
            double y = j - center;
            // 二维高斯函数公式
            gaussian_kernel[i][j] = exp(-(x*x + y*y)/(2*sigma*sigma)) / (2*M_PI*sigma*sigma);
            sum += gaussian_kernel[i][j];
        }
    }
    
    // 归一化高斯核
    for(int i = 0; i < kernel_size; i++) {
        for(int j = 0; j < kernel_size; j++) {
            gaussian_kernel[i][j] /= sum;
        }
    }
    
    // 2. 应用高斯滤波
    int channels = input.channels();
    int height = input.rows;
    int width = input.cols;
    int radius = kernel_size / 2;
    
    // 对每个像素进行卷积运算
    for(int y = radius; y < height-radius; y++) {
        for(int x = radius; x < width-radius; x++) {
            // 处理每个通道
            for(int c = 0; c < channels; c++) {
                double sum = 0.0;
                
                // 应用高斯核
                for(int ky = -radius; ky <= radius; ky++) {
                    for(int kx = -radius; kx <= radius; kx++) {
                        if(channels == 1) {
                            // 单通道图像
                            sum += input.at<uchar>(y+ky, x+kx) * 
                                  gaussian_kernel[ky+radius][kx+radius];
                        } else {
                            // 多通道图像
                            sum += input.at<Vec3b>(y+ky, x+kx)[c] * 
                                  gaussian_kernel[ky+radius][kx+radius];
                        }
                    }
                }
                
                // 将结果写入输出图像
                if(channels == 1) {
                    output.at<uchar>(y, x) = saturate_cast<uchar>(sum);
                } else {
                    output.at<Vec3b>(y, x)[c] = saturate_cast<uchar>(sum);
                }
            }
        }
    }
}

void BGR2HSV(const Mat &input, Mat &output)
{
    if (input.rows != output.rows ||
        input.cols != output.cols ||
        input.channels() != 3 ||
        output.channels() != 3)
        return;

	for(int i = 0; i < input.rows; i++)
	{
		for (int j = 0; j < input.cols; j++)
		{

        /*** 第二步：在此处填充RGB转HSV代码 ***/
        int max_bgr = max(max(input.at<Vec3b>(i, j)[0], input.at<Vec3b>(i, j)[1]), input.at<Vec3b>(i, j)[2]);
        int min_bgr = min(min(input.at<Vec3b>(i, j)[0], input.at<Vec3b>(i, j)[1]), input.at<Vec3b>(i, j)[2]);
        
        if (max_bgr == 0)
        {
            output.at<Vec3b>(i, j)[0] = 0;
            output.at<Vec3b>(i, j)[1] = 0;
            output.at<Vec3b>(i, j)[2] = 0;
        }
        else
        {
            double k=(input.at<Vec3b>(i, j)[2]*2-input.at<Vec3b>(i, j)[1]-input.at<Vec3b>(i, j)[0])/2/
            sqrt(pow((input.at<Vec3b>(i, j)[2]-input.at<Vec3b>(i, j)[1]),2)+(input.at<Vec3b>(i, j)[2]-input.at<Vec3b>(i, j)[0])*(input.at<Vec3b>(i, j)[1]-input.at<Vec3b>(i, j)[0]));
            double theta=acos(k)/3.1415926535*180;
            double sum_bgr=input.at<Vec3b>(i, j)[0]+input.at<Vec3b>(i, j)[1]+input.at<Vec3b>(i, j)[2];
            output.at<Vec3b>(i, j)[0] = (int)(sum_bgr/3);  //I
            output.at<Vec3b>(i, j)[1] = (int) (255*(1-min_bgr*3/sum_bgr));//S
            
            if (input.at<Vec3b>(i, j)[1] >= input.at<Vec3b>(i, j)[0])
                output.at<Vec3b>(i, j)[2] = (int) (
                        theta/360*255);//H
            else
                output.at<Vec3b>(i, j)[2] = (int) (
                        255-theta/360*255);
        }
        }
    }
}


void ColorSplitManual(const Mat &hsv_input, Mat &grey_output, const string window)
{
    static int hmin = 0;
    static int hmax = 20;
    static int smin = 100;
    static int smax = 255;
    static int vmin = 50;
    static int vmax = 255;
    createTrackbar("Hmin", window, &hmin, 255);
    createTrackbar("Hmax", window, &hmax, 255);
    createTrackbar("Smin", window, &smin, 255);
    createTrackbar("Smax", window, &smax, 255);
    createTrackbar("Vmin", window, &vmin, 255);
    createTrackbar("Vmax", window, &vmax, 255);

    /*** 第三步：在此处填充阈值分割代码代码 ***/
    
    // 确保grey_output是单通道图像
    if(grey_output.empty() || grey_output.size() != hsv_input.size() || grey_output.type() != CV_8UC1) {
        grey_output = Mat::zeros(hsv_input.size(), CV_8UC1);
    }

    // 遍历每个像素进行颜色分割
    for(int i = 0; i < hsv_input.rows; i++) {
        for(int j = 0; j < hsv_input.cols; j++) {
            // 获取当前像素的HSV值
            Vec3b hsv = hsv_input.ptr<Vec3b>(i)[j];
            
            // 判断是否在阈值范围内
            if((hsv[0] >= hmin && hsv[0] <= hmax) && 
               (hsv[1] >= smin && hsv[1] <= smax) && 
               (hsv[2] >= vmin && hsv[2] <= vmax)) {
                grey_output.at<uchar>(i, j) = 255;  // 在范围内设为白色
            } else {
                grey_output.at<uchar>(i, j) = 0;    // 在范围外设为黑色
            }
        }
    }
}

void ColorSplitManualRGB(const Mat &hsv_input, Mat &grey_output_red, Mat &grey_output_yellow, Mat &grey_output_green, const string window)
{
    // RGB三色的HSV阈值
    static int red_hmin = 0, red_hmax = 10;        // 红色范围1
    static int red_hmin2 = 160, red_hmax2 = 180;   // 红色范围2
    static int yellow_hmin = 20, yellow_hmax = 40;
    static int green_hmin = 50, green_hmax = 90;
    
    static int smin = 90;     // 所有颜色共用较高的饱和度下限
    static int smax = 255;
    static int vmin = 90;     // 所有颜色共用较高的亮度下限
    static int vmax = 255;

    // 创建滑动条
    createTrackbar("Red Hmin", window, &red_hmin, 180);
    createTrackbar("Red Hmax", window, &red_hmax, 180);
    createTrackbar("Yellow Hmin", window, &yellow_hmin, 180);
    createTrackbar("Yellow Hmax", window, &yellow_hmax, 180);
    createTrackbar("Green Hmin", window, &green_hmin, 180);
    createTrackbar("Green Hmax", window, &green_hmax, 180);
    createTrackbar("Smin", window, &smin, 255);
    createTrackbar("Vmin", window, &vmin, 255);

    // 确保输出图像是正确的大小和类型
    if(grey_output_red.empty() || grey_output_red.size() != hsv_input.size() || grey_output_red.type() != CV_8UC1) {
        grey_output_red = Mat::zeros(hsv_input.size(), CV_8UC1);
        grey_output_yellow = Mat::zeros(hsv_input.size(), CV_8UC1);
        grey_output_green = Mat::zeros(hsv_input.size(), CV_8UC1);
    }

    // 遍历图像进行颜色分割
    for(int i = 0; i < hsv_input.rows; i++) {
        for(int j = 0; j < hsv_input.cols; j++) {
            Vec3b hsv = hsv_input.ptr<Vec3b>(i)[j];
            uchar h = hsv[0];
            uchar s = hsv[1];
            uchar v = hsv[2];

            // 检查饱和度和亮度是否满足条件
            if(s >= smin && s <= smax && v >= vmin && v <= vmax) {
                // 红色检测（考虑到红色在HSV空间中横跨0/180）
                if((h >= red_hmin && h <= red_hmax) || (h >= red_hmin2 && h <= red_hmax2)) {
                    grey_output_red.at<uchar>(i, j) = 255;
                } else {
                    grey_output_red.at<uchar>(i, j) = 0;
                }

                // 黄色检测
                if(h >= yellow_hmin && h <= yellow_hmax) {
                    grey_output_yellow.at<uchar>(i, j) = 255;
                } else {
                    grey_output_yellow.at<uchar>(i, j) = 0;
                }

                // 绿色检测
                if(h >= green_hmin && h <= green_hmax) {
                    grey_output_green.at<uchar>(i, j) = 255;
                } else {
                    grey_output_green.at<uchar>(i, j) = 0;
                }
            } else {
                grey_output_red.at<uchar>(i, j) = 0;
                grey_output_yellow.at<uchar>(i, j) = 0;
                grey_output_green.at<uchar>(i, j) = 0;
            }
        }
    }

    // 对每个颜色的结果进行形态学操作来去除噪声
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    
    // 红色
    morphologyEx(grey_output_red, grey_output_red, MORPH_OPEN, kernel);
    morphologyEx(grey_output_red, grey_output_red, MORPH_CLOSE, kernel);
    
    // 黄色
    morphologyEx(grey_output_yellow, grey_output_yellow, MORPH_OPEN, kernel);
    morphologyEx(grey_output_yellow, grey_output_yellow, MORPH_CLOSE, kernel);
    
    // 绿色
    morphologyEx(grey_output_green, grey_output_green, MORPH_OPEN, kernel);
    morphologyEx(grey_output_green, grey_output_green, MORPH_CLOSE, kernel);

        // 创建一个彩色输出图像用于绘制轮廓
    Mat contour_output = Mat::zeros(hsv_input.size(), CV_8UC3);

    // 红色轮廓检测与标记
    vector<vector<Point>> contours_red;
    vector<Vec4i> hierarchy_red;
    findContours(grey_output_red, contours_red, hierarchy_red, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    drawContours(contour_output, contours_red, -1, Scalar(0, 0, 255), 2); // 使用红色绘制红色区域的轮廓

    // 黄色轮廓检测与标记
    vector<vector<Point>> contours_yellow;
    vector<Vec4i> hierarchy_yellow;
    findContours(grey_output_yellow, contours_yellow, hierarchy_yellow, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    drawContours(contour_output, contours_yellow, -1, Scalar(0, 255, 255), 2); // 使用黄色绘制黄色区域的轮廓

    // 绿色轮廓检测与标记
    vector<vector<Point>> contours_green;
    vector<Vec4i> hierarchy_green;
    findContours(grey_output_green, contours_green, hierarchy_green, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    drawContours(contour_output, contours_green, -1, Scalar(0, 255, 0), 2); // 使用绿色绘制绿色区域的轮廓

    // 显示结果
    imshow(window + " Contours", contour_output);
}

void ColorSplitAuto(const Mat &hsv_input, Mat &bgr_output, vector<vector<Point>> &contours, int hmin, int hmax, int smin, int smax, int vmin, int vmax)
{
    int rw = hsv_input.rows;
	int cl = hsv_input.cols;
    Mat color_region(rw, cl, CV_8UC1);

    /*** 第五步：利用已知的阈值获取颜色区域二值图 ***/
    // 创建用于存储HSV各个通道的阈值图像
    Mat hsv_mask;
    
    // 设定HSV阈值范围
    Scalar lower_bound(hmin, smin, vmin);
    Scalar upper_bound(hmax, smax, vmax);
    
    // 利用inRange函数获取特定颜色区域的掩码
    inRange(hsv_input, lower_bound, upper_bound, color_region);
    
    // 可选：使用形态学操作来去除噪点和填充空洞
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(color_region, color_region, MORPH_OPEN, kernel);
    morphologyEx(color_region, color_region, MORPH_CLOSE, kernel);



    /* 获取多边形轮廓 */
    vector<Vec4i> hierarchy;
	findContours(color_region, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	vector<vector<Point>> lines(contours.size());
    /* 利用多项式近似平滑轮廓 */
	for(int i = 0; i < contours.size(); i++)
	{
		approxPolyDP(contours[i], lines[i],9,true);
	}
	drawContours(bgr_output, lines, -1,Scalar(0, 0, 255), 2, 8);
}


void GetROI(const Mat &input, Mat &output, const vector<vector<Point>> &contour)
{
    /* 第六步：补充获取颜色区域代码，可使用drawContours函数 */
    // 创建与输入图像同样大小的掩码图像
    Mat mask = Mat::zeros(input.size(), CV_8UC1);
    
    // 在掩码图像上绘制轮廓（填充轮廓内部）
    drawContours(mask, contour, -1, Scalar(255), FILLED);
    
    // 将输入图像拷贝到输出图像
    output = Mat::zeros(input.size(), input.type());
    
    // 使用掩码提取ROI区域
    input.copyTo(output, mask);

}

int CountROIPixel(const Mat &input)
{
	int cnt = 0;

    /* 第七步：补充获取颜色区域像素个数的代码 */
    /* 第七步：补充获取颜色区域像素个数的代码 */
    // 遍历图像的每个像素
    for(int i = 0; i < input.rows; i++)
    {
        for(int j = 0; j < input.cols; j++)
        {
            // 如果是单通道图像
            if(input.channels() == 1)
            {
                if(input.at<uchar>(i,j) > 0)
                {
                    cnt++;
                }
            }
            // 如果是三通道图像
            else if(input.channels() == 3)
            {
                Vec3b pixel = input.at<Vec3b>(i,j);
                // 如果像素不是全黑(0,0,0)，就计数
                if(pixel[0] > 0 || pixel[1] > 0 || pixel[2] > 0)
                {
                    cnt++;
                }
            }
        }
    }



    return cnt;
}


/*** 第四步：在第三步基础上修改各颜色阈值 ***/
//{hmin, hmax, smin, smax, vmin, vmax}

// 红色阈值 (注意红色在HSV空间中横跨了0/180度的分界线)
int red_thresh[6] = {
    156,    // hmin（也可以用0）
    180,    // hmax
    100,    // smin
    255,    // smax
    50,     // vmin
    255     // vmax
};

// 绿色阈值
int green_thresh[6] = {
    35,     // hmin
    85,     // hmax
    50,     // smin
    255,    // smax
    50,     // vmin
    255     // vmax
};

// 蓝色阈值
int blue_thresh[6] = {
    90,     // hmin
    130,    // hmax
    100,    // smin
    255,    // smax
    50,     // vmin
    255     // vmax
};

// 黄色阈值
int yellow_thresh[6] = {
    15,     // hmin
    35,     // hmax
    100,    // smin
    255,    // smax
    100,    // vmin
    255     // vmax
};

Mat frame_msg;
void rcvCameraCallBack(const sensor_msgs::Image::ConstPtr& img)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
    frame_msg = cv_ptr->image;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exp4_node"); // 初始化 ROS 节点
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 
    ros::Subscriber camera_sub;
    VideoCapture capture;
    Mat frIn;
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


        // 空域高斯滤波
        Mat filter(frIn.size(), CV_8UC3);
        Gaussian(frIn, filter, 3);
        imshow("filter",filter);

        // RGB转HSV
        Mat hsv(frIn.size(), CV_8UC3);
        BGR2HSV(filter, hsv);
        imshow("hsv",hsv);

        // // 手动颜色分割
        // Mat grey(frIn.rows, frIn.cols, CV_8UC1);
        // ColorSplitManual(hsv, grey, "hsv_split");
        // imshow("split", grey);

        // 创建窗口
        string window_name = "Color Threshold Controls";
        namedWindow(window_name);
        // 创建输出图像
        Mat red_result, yellow_result, green_result;
        ColorSplitManualRGB(hsv, red_result, yellow_result, green_result, window_name);
        
        // 显示结果
        imshow("Red Detection", red_result);
        imshow("Yellow Detection", yellow_result);
        imshow("Green Detection", green_result);
        
        int colors = 0;
        int maxs_color_num = 0;
        /* 目标颜色检测 */

	    Mat tmp_line = frIn.clone();
	    Mat tmp_roi = Mat::zeros(frIn.size(), CV_8UC3);
        vector<vector<Point>> contours_r;
        	ColorSplitAuto(hsv, tmp_line, contours_r, red_thresh[0], red_thresh[1], red_thresh[2],
				   red_thresh[3], red_thresh[4], red_thresh[5]);
	    GetROI(frIn, tmp_roi, contours_r);
	    int red_color_num = CountROIPixel(tmp_roi);

        // Yellow color detection
        vector<vector<Point>> contours_y;
        tmp_line = frIn.clone();
        tmp_roi = Mat::zeros(frIn.size(), CV_8UC3);
        ColorSplitAuto(hsv, tmp_line, contours_y, yellow_thresh[0], yellow_thresh[1], yellow_thresh[2],
                       yellow_thresh[3], yellow_thresh[4], yellow_thresh[5]);
        GetROI(frIn, tmp_roi, contours_y);
        int yellow_color_num = CountROIPixel(tmp_roi);

        // Green color detection
        vector<vector<Point>> contours_g;
        tmp_line = frIn.clone();
        tmp_roi = Mat::zeros(frIn.size(), CV_8UC3);
        ColorSplitAuto(hsv, tmp_line, contours_g, green_thresh[0], green_thresh[1], green_thresh[2],
                       green_thresh[3], green_thresh[4], green_thresh[5]);
        GetROI(frIn, tmp_roi, contours_g);
        int green_color_num = CountROIPixel(tmp_roi);

        /* 第八步：结合给出的检测红颜色的代码框架，给出控制小车运动的代码 */
        maxs_color_num = max({red_color_num, yellow_color_num, green_color_num});
        

        vector<vector<Point>> *dominant_contours;
        if (maxs_color_num == red_color_num) {
            dominant_contours = &contours_r;
        } else if (maxs_color_num == yellow_color_num) {
            dominant_contours = &contours_y;
        } else {
            dominant_contours = &contours_g;
        }

        // Find the largest contour's center
        if (!dominant_contours->empty()) {
            int largest_contour_idx = 0;
            double largest_area = 0;
            for (size_t i = 0; i < dominant_contours->size(); i++) {
                double area = contourArea((*dominant_contours)[i]);
                if (area > largest_area) {
                    largest_area = area;
                    largest_contour_idx = i;
                }
            }

            // Calculate centroid of largest contour
            Moments mu = moments((*dominant_contours)[largest_contour_idx]);
            Point2f center(mu.m10/mu.m00, mu.m01/mu.m00);

            // Determine movement based on position
            float image_center = frIn.cols / 2.0f;
            float position_threshold = frIn.cols / 6.0f;  // Adjust this value to change sensitivity

            if (abs(center.x - image_center) < position_threshold) {
                // Target is centered - move forward
                colors = 0;
            } else if (center.x < image_center - position_threshold) {
                // Target is to the left - turn left
                colors = 2;
            } else if (center.x > image_center + position_threshold) {
                // Target is to the right - turn right
                colors = 3;
            }
        }






        geometry_msgs::Twist vel;
        vel.linear.x = 0;
        vel.linear.y = 0;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;
        if(maxs_color_num)
        {
            switch(colors)
            {
                case 0:
                    vel.linear.x = 0.5;
                    break;
                case 1:
                    vel.linear.x = -0.5;
                    break;
                case 2:
                    vel.angular.z = 0.4;
                    break;
                case 3:
                    vel.angular.z = -0.4;
                    break;
            }
        }
        vel_pub.publish(vel);




        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}