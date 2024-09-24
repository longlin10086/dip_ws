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
    REALSENSE
};
CameraState state = REALSENSE;


using namespace cv;
using namespace std;
void Gaussian(const Mat &input, Mat &output, double sigma)
{
    if (output.rows != input.rows || output.cols != input.cols || output.channels() != input.channels())
        return;
    int kernel_size = 9;
    double gaussian_kernel[kernel_size][kernel_size];

    /*** 第一步：结合实验二，在此处填充高斯滤波代码 ***/




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

        

        }
    }
}


void ColorSplitManual(const Mat &hsv_input, Mat &grey_output, const string window)
{
	static int hmin = 0;
	static int hmax = 255;
	static int smin = 0;
	static int smax = 255;
	static int vmin = 0;
	static int vmax = 255;
	createTrackbar("Hmin", window, &hmin, 255);
	createTrackbar("Hmax", window, &hmax, 255);
	createTrackbar("Smin", window, &smin, 255);
	createTrackbar("Smax", window, &smax, 255);
	createTrackbar("Vmin", window, &vmin, 255);
	createTrackbar("Vmax", window, &vmax, 255);

    /*** 第三步：在此处填充阈值分割代码代码 ***/




}

void ColorSplitAuto(const Mat &hsv_input, Mat &bgr_output, vector<vector<Point>> &contours, int hmin, int hmax, int smin, int smax, int vmin, int vmax)
{
    int rw = hsv_input.rows;
	int cl = hsv_input.cols;
    Mat color_region(rw, cl, CV_8UC1);

    /*** 第五步：利用已知的阈值获取颜色区域二值图 ***/



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


}

int CountROIPixel(const Mat &input)
{
	int cnt = 0;

    /* 第七步：补充获取颜色区域像素个数的代码 */




    return cnt;
}


/*** 第四步：在第三步基础上修改各颜色阈值 ***/
//{hmin, hmax, smin, smax, vmin, vmax}
int red_thresh[6] = {0};
int green_thresh[6] = {0};
int blue_thresh[6] = {0};
int yellow_thresh[6] = {0};

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
	ros::Publisher vel_pub;
    ros::Subscriber camera_sub;
    VideoCapture capture;
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
    

    Mat frIn;
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


        // 空域高斯滤波
        Mat filter(frIn.size(), CV_8UC3);
        Gaussian(frIn, filter, 3);
        imshow("filter",filter);

        // RGB转HSV
        Mat hsv(frIn.size(), CV_8UC3);
        BGR2HSV(filter, hsv);
        imshow("hsv",hsv);

        // 手动颜色分割
        Mat grey(frIn.rows, frIn.cols, CV_8UC1);
        ColorSplitManual(hsv, grey, "hsv_split");
        imshow("split", grey);
        
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

        /* 第八步：结合给出的检测红颜色的代码框架，给出控制小车运动的代码 */






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