#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/***************函数声明，相关参数自行修改***************/
Mat EdgeDetector(Mat input, Mat output);
Mat HoughLines(Mat input);
Mat HoughCircles(Mat input);

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

                /****************调用边缘检测函数****************/
                // EdgeDetecto();
                // imshow();

                /***************调用霍夫线变换***************/
                // HoughLines();
                // imshow();

                /***************调用霍夫圆变换***************/
                // HoughCircles();
                // imshow();
        }
        return 0;
}
/***************下面实现EdgeDetector()函数***************/

/***************下面实现HoughLines()函数***************/

/***************下面实现HoughCircles()函数***************/
