#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/***************函数声明，相关参数自行修改***************/
Mat EdgeDetector(Mat input, int threshold, Mat &grad_x, Mat &grad_y);
Mat HoughLines(Mat input);
Mat HoughCircles(Mat input,Mat output,Mat &grad_x, Mat &grad_y);

Mat raw;

int main(int argc, char *argv[])
{
        Mat raw_line = imread("./src/exp3/data/lane.png");
        Mat raw_circle = imread("./src/exp3/data/circle.png");
        while (waitKey(10))
        {
                /***************读取图像***************/
                // raw = imread("./src/exp3/data/lane.png");

                if (!raw_line.data || !raw_circle.data)
                {
                        cout << "error" << endl;
                        break;
                }

                imshow("raw_line", raw_line);
                imshow("raw_circle", raw_circle);
                Mat gray_line,gray_circle;
                cvtColor(raw_line, gray_line, COLOR_BGR2GRAY);
                cvtColor(raw_circle, gray_circle, COLOR_BGR2GRAY);

                /*************** 边缘检测 ***************/
                Mat grad_x, grad_y;
                Mat edges_line = EdgeDetector(gray_line, 60, grad_x, grad_y);  // 50为阈值
                imshow("Edges Line", edges_line);

                /*************** 霍夫线变换 ***************/
                Mat hough_lines_output = HoughLines(edges_line);
                imshow("Hough Lines", hough_lines_output);

                /*************** 霍夫圆变换 ***************/
                Mat output_circle;
                Mat edges_circle = EdgeDetector(gray_circle, 60, grad_x, grad_y);
                cvtColor(gray_circle, output_circle, COLOR_GRAY2BGR);  // 转换为彩色图像以显示结果
                Mat hough_circles_output = HoughCircles(edges_circle, output_circle, grad_x, grad_y);
                imshow("Hough Circles", hough_circles_output);
        }
        return 0;
}
/***************下面实现EdgeDetector()函数***************/

Mat EdgeDetector(Mat input, int threshold, Mat &grad_x, Mat &grad_y)
{
    // 高斯滤波
    Mat blur;
    GaussianBlur(input, blur, Size(3, 3), 0, 0);

    // Sobel 核
    int sobel_x[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
    int sobel_y[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};
    
    // 创建初始化为0的Mat对象，使用CV_64F来确保精度
    grad_x = Mat::zeros(input.size(), CV_64F);
    grad_y = Mat::zeros(input.size(), CV_64F);
    Mat output = Mat::zeros(input.size(), CV_8UC1);

    // 遍历像素并执行卷积操作
    for (int i = 1; i < input.rows - 1; i++)
    {
        for (int j = 1; j < input.cols - 1; j++)
        {
            double sum_x = 0.0;
            double sum_y = 0.0;
            
            // 计算卷积结果
            for (int m = -1; m <= 1; m++)
            {
                for (int n = -1; n <= 1; n++)
                {
                    sum_x += static_cast<double>(blur.at<uchar>(i + m, j + n)) * sobel_x[m + 1][n + 1];
                    sum_y += static_cast<double>(blur.at<uchar>(i + m, j + n)) * sobel_y[m + 1][n + 1];
                }
            }
            
            grad_x.at<double>(i, j) = sum_x;
            grad_y.at<double>(i, j) = sum_y;
            
            // 计算梯度幅值
            double magnitude = sqrt(sum_x * sum_x + sum_y * sum_y);
            
            // 阈值判断
            if (magnitude > threshold)
            {
                output.at<uchar>(i, j) = 255;  // 边缘
            }
            else
            {
                output.at<uchar>(i, j) = 0;    // 非边缘
            }
        }
    }

    return output;
}


/***************下面实现HoughLines()函数***************/

Mat HoughLines(Mat input)
{
    int width = input.cols;
    int height = input.rows;
    int maxDist = sqrt((width * width + height * height));
    int thetaBins = 360; // 增加角度分辨率
    int rBins = maxDist * 2; // r离散化
    double thetaStep = CV_PI / thetaBins; // theta步长
    double rStep = 2 * maxDist / rBins;   // r步长

    // 初始化霍夫空间
    Mat houghSpace = Mat::zeros(rBins, thetaBins, CV_32SC1);

    // 遍历边缘图像
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if (input.at<uchar>(i, j) == 255)
            {
                for (int k = 0; k < thetaBins; k++)
                {
                    double theta = k * thetaStep;
                    double r = j * cos(theta) + i * sin(theta);
                    int rIndex = round((r + maxDist) / rStep);
                    if (rIndex >= 0 && rIndex < rBins)
                    {
                        houghSpace.at<int>(rIndex, k)++;
                    }
                }
            }
        }
    }

    Mat output;
    cvtColor(input, output, COLOR_GRAY2BGR);

    // 计算自适应阈值
    double minVal, maxVal;
    minMaxLoc(houghSpace, &minVal, &maxVal);
    int threshold = maxVal * 0.5; // 使用最高值的50%作为阈值

    double diagLength = sqrt(width * width + height * height); // 图像对角线长度
    for (int i = 0; i < rBins; i++)
    {
        for (int j = 0; j < thetaBins; j++)
        {
            if (houghSpace.at<int>(i, j) > threshold)
            {
                double r = (i * rStep) - maxDist;
                double theta = j * thetaStep;
                double a = cos(theta);
                double b = sin(theta);
                double x0 = a * r;
                double y0 = b * r;

                // 根据对角线长度调整线段绘制
                Point pt1(cvRound(x0 + diagLength * (-b)), cvRound(y0 + diagLength * (a)));
                Point pt2(cvRound(x0 - diagLength * (-b)), cvRound(y0 - diagLength * (a)));
                line(output, pt1, pt2, Scalar(0, 0, 255), 1, LINE_AA);
            }
        }
    }

    return output;
}



/***************下面实现HoughCircles()函数***************/

Mat HoughCircles(Mat input, Mat output, Mat &grad_x, Mat &grad_y)
{
    int width = input.cols;
    int height = input.rows;

    // 将灰度图转换为彩色图
    cvtColor(input, output, COLOR_GRAY2BGR);  // 确保 output 是彩色图像

    // 1. 初始化圆心累加器
    int maxRadius = 100; // 假设最大圆半径为100
    Mat accum = Mat::zeros(height, width, CV_32SC1);

    // 2. 遍历边缘图中的所有非零像素点，沿梯度方向投票
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (input.at<uchar>(y, x) == 255) // 是边缘点
            {
                double gx = grad_x.at<double>(y, x);
                double gy = grad_y.at<double>(y, x);
                double magnitude = sqrt(gx * gx + gy * gy);

                // 计算梯度方向的单位向量
                if (magnitude != 0)
                {
                    double dx = gx / magnitude;
                    double dy = gy / magnitude;

                    // 沿梯度方向找到圆心 (a, b)
                    for (int r = 5; r < maxRadius; r++) // 从小半径到大半径
                    {
                        int a = cvRound(x - r * dx); // 圆心x
                        int b = cvRound(y - r * dy); // 圆心y

                        // 检查圆心是否在图像范围内
                        if (a >= 0 && a < width && b >= 0 && b < height)
                        {
                            accum.at<int>(b, a)++; // 圆心累加器
                        }
                    }
                }
            }
        }
    }

    // 3. 找到最可能的圆心，并统计可能的半径
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (accum.at<int>(y, x) > 100) // 圆心累加值大于阈值
            {
                cout << "Detected circle center at (" << x << ", " << y << "), Accumulator value: " << accum.at<int>(y, x) << endl;
                // 针对这个圆心，统计边缘点到圆心的距离，计算半径
                vector<int> radiusVotes(maxRadius, 0); // 半径累加器
                for (int i = 0; i < height; i++)
                {
                    for (int j = 0; j < width; j++)
                    {
                        if (input.at<uchar>(i, j) == 255) // 是边缘点
                        {
                            int r = cvRound(sqrt(pow(j - x, 2) + pow(i - y, 2))); // 计算半径
                            if (r < maxRadius)
                            {
                                radiusVotes[r]++;
                            }
                        }
                    }
                }

                // 找到最大的半径
                int bestRadius = max_element(radiusVotes.begin(), radiusVotes.end()) - radiusVotes.begin();

                // 在图像上绘制圆（粉色）
                circle(output, Point(x, y), bestRadius, Scalar(255, 0, 255), 2);  // 粉色圆形

                // 绘制圆心（绿色）
                circle(output, Point(x, y), 3, Scalar(0, 255, 0), -1);  // 绿色圆心
            }
        }
    }

    return output;
}


