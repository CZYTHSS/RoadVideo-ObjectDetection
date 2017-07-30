#define _CRT_SECURE_NO_DEPRECATE
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
#include <math.h>

using namespace std;
using namespace cv;

void extract_video();	//函数内可以修改读取文件名，输出文件名，编码格式以及帧数范围
void FindLine(Mat source, int arg);
void ObjectDetect(VideoCapture cap);
void FindRoadEdge(VideoCapture cap);
int main()
{
	//extract_video();

	VideoCapture cap("data/clip_8.mp4");

	if (!cap.isOpened()) {
		cerr << "can't open the video file!" << endl;
		return -1;
	}
	
	FindRoadEdge(cap);
	//ObjectDetect(cap);
	//FindLine(cap, 1);
	return 0;
}

void extract_video() {
	//【1】读入视频
	VideoCapture capture("data/4-12-17-A.MP4");

	VideoWriter writer("data/A_cut_test.avi", CV_FOURCC('M', 'J', 'P', 'G'), 29, Size(1920, 1080));//注意此处视频的尺寸大小要与真实的一致
																							  //【2】循环显示每一帧
	int i = 0;
	char name[50];
	while (1)
	{
		Mat frame;//定义一个Mat变量，用于存储每一帧的图像
		capture >> frame;  //读取当前帧
		i++;
		//若视频播放完成，退出循环
		if (frame.empty())
		{
			break;
		}

		if (i>0 && i < 1000)
		{
			sprintf(name, "pictures\\%d.jpg", i);//输出到上级目录的output文件夹下
			imwrite(name, frame);//输出一张jpg图片到工程目录下
			writer << frame;
			sprintf(name, "%d", i);
			putText(frame, name, Point(0, 20), FONT_HERSHEY_SIMPLEX,
				0.6, Scalar(0, 255, 0));
			imshow("读取视频", frame);  //显示当前帧
			writer << frame;
			waitKey(10);  //延时30ms
		}
	}
	return;
}


/* This is a standalone program. Pass an image name as the first parameter
of the program.  Switch between standard and probabilistic Hough transform
by changing arg from 1 to 0 and back */
void FindLine(Mat source, int arg)
{
	namedWindow("src", WINDOW_NORMAL);
	Mat src, dst, color_dst, frame1, frame2, diff;
	
	src = source.clone();
	
	Canny(src, dst, 50, 200, 3);
	cvtColor(dst, color_dst, CV_GRAY2BGR);
	//imshow("color_dst", color_dst);
	//waitKey(10000);

	if (arg == 0) {
		vector<Vec2f> lines;
		HoughLines(dst, lines, 1, CV_PI / 180, 100);

		for (size_t i = 0; i < lines.size(); i++)
		{
			float rho = lines[i][0];
			float theta = lines[i][1];
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			Point pt1(cvRound(x0 + 1000 * (-b)),
				cvRound(y0 + 1000 * (a)));
			Point pt2(cvRound(x0 - 1000 * (-b)),
				cvRound(y0 - 1000 * (a)));
			line(color_dst, pt1, pt2, Scalar(0, 0, 255), 3, 8);
		}
	}
	else {
		vector<Vec4i> lines;
		HoughLinesP(dst, lines, 1, CV_PI / 180, 80, 30, 10);
		for (size_t i = 0; i < lines.size(); i++)
		{
			line(color_dst, Point(lines[i][0], lines[i][1]),
				Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 3, 8);
		}
	}

	namedWindow("Source", WINDOW_NORMAL);
	imshow("Source", src);

	namedWindow("Detected Lines", WINDOW_NORMAL);
	imshow("Detected Lines", color_dst);

	waitKey(0);
}

void ObjectDetect(VideoCapture cap)
{
	Mat frame1, frame2, diff;
	cap >> frame2;

	namedWindow("video", WINDOW_NORMAL);

	for (;;) {
		frame1 = frame2.clone();
		cap >> frame2;
		if (frame2.empty())
			break;
		//diff = frame1 - frame2;
		Mat gray_diff, gray1, gray2;
		cvtColor(frame1, gray1, COLOR_RGB2GRAY);
		cvtColor(frame2, gray2, COLOR_RGB2GRAY);
		//cvtColor(diff, gray_diff, COLOR_RGB2GRAY);

		threshold(gray1, gray1, 15, 255, CV_THRESH_OTSU);
		//threshold(gray2, gray2, 100, 255, CV_THRESH_BINARY);

		gray_diff = gray1 - gray2;
		//threshold(gray_diff, gray_diff, 15, 255, CV_THRESH_OTSU);
		GaussianBlur(gray_diff, gray_diff, Size(3, 5), 0, 0);
		threshold(gray_diff, gray_diff, 15, 255, CV_THRESH_BINARY);

		gray1 = gray1 / 2;
		imshow("video", gray1);
		int key_value = waitKey(15);
		if (key_value != 255)
		{
			if (key_value == 27) break;		//press ESC to break
			else {
				key_value = waitKey(100000);
				if (key_value == 27) break;
			}
		}
	}
}

void FindRoadEdge(VideoCapture cap)
{
	
	Mat background, src, current;
	
	for (int i = 1; i < 600; i++) {
		cap >> src;
		if (i == 1) {
			background = src.clone();
			current = src.clone();
		}
		else {
			background = (current * (i - 1) + src) / i;
			current = background.clone();
		}
		cout << i << endl;
		//imshow("video", background);
		//waitKey(1000);
		/*if (waitKey(20) != 255) {
			int n = waitKey(10000);
			if (n == 27) break;
		}*/
	}
	imwrite("background.png", background);
	
	background = imread("background.png");
	namedWindow("video", WINDOW_NORMAL);
	/*imshow("video", background);
	waitKey(0);*/

	Mat gray_bg;
	cvtColor(background, gray_bg, CV_RGB2GRAY);
	imshow("video", gray_bg);
	waitKey(0);
	threshold(gray_bg, gray_bg, 0, 255, CV_THRESH_OTSU);
	imshow("video", gray_bg);
	waitKey(0);

	Mat result;
	result = gray_bg.clone();
	int nr = gray_bg.rows;
	int nc = gray_bg.cols*gray_bg.channels();
	//cout << nr << endl << nc;
	for (int i = 1; i<nr - 1; i++)
	{
		const uchar* up_line = gray_bg.ptr<uchar>(i - 1);//指向上一行
		const uchar* mid_line = gray_bg.ptr<uchar>(i);//当前行
		const uchar* down_line = gray_bg.ptr<uchar>(i + 1);//下一行
		uchar* cur_line = result.ptr<uchar>(i);
		for (int j = 1; j<nc - 1; j++)
		{
			int threshold = 15;
			int count = 0;
			for (int n = i; n < nr - 1 && n < i + threshold; n++) {
				if (gray_bg.ptr<uchar>(n)[j] == 0) count++;
			}
			if (mid_line[j] == 255 && count > threshold - 2) cur_line[j] = 0;
			else cur_line[j] = 255;
			if (i < nr / 5) cur_line[j] = 255;
			if (j > nc / 2) cur_line[j] = 255;
		}
	}     // 把图像边缘像素设置为255
	result.row(0).setTo(Scalar(255));
	result.row(result.rows - 1).setTo(Scalar(255));
	result.col(0).setTo(Scalar(255));
	result.col(result.cols - 1).setTo(Scalar(255));

	imshow("video", result);
	waitKey(0);

	FindLine(result,1);
	//for (;;) {
	//	frame1 = frame2.clone();
	//	cap >> frame2;
	//	if (frame2.empty())
	//		break;
	//	//diff = frame1 - frame2;
	//	Mat gray_diff, gray1, gray2;
	//	cvtColor(frame1, gray1, COLOR_RGB2GRAY);
	//	cvtColor(frame2, gray2, COLOR_RGB2GRAY);
	//	//cvtColor(diff, gray_diff, COLOR_RGB2GRAY);

	//	threshold(gray1, gray1, 15, 255, CV_THRESH_OTSU);
	//	//threshold(gray2, gray2, 100, 255, CV_THRESH_BINARY);

	//	gray_diff = gray1 - gray2;
	//	//threshold(gray_diff, gray_diff, 15, 255, CV_THRESH_OTSU);
	//	GaussianBlur(gray_diff, gray_diff, Size(3, 5), 0, 0);
	//	threshold(gray_diff, gray_diff, 15, 255, CV_THRESH_BINARY);

	//	gray1 = gray1 / 2;
	//	imshow("video", gray1);
	//	int key_value = waitKey(15);
	//	if (key_value != 255)
	//	{
	//		if (key_value == 27) break;		//press ESC to break
	//		else {
	//			key_value = waitKey(100000);
	//			if (key_value == 27) break;
	//		}
	//	}
	//}
}
