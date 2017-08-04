#define _CRT_SECURE_NO_DEPRECATE
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
#include <math.h>
#include "utils.h"
#include "objectdetection.h"
#define PI 3.14159265


using namespace std;
using namespace cv;

void extract_video();	//函数内可以修改读取文件名，输出文件名，编码格式以及帧数范围
vector<Vec2f> FindLine(Mat source, int arg);
vector<Vec2f> FindRoadEdge(VideoCapture cap);
vector<Vec2f> LineExtract(vector<Vec4i> lines, Mat img);		//从Hough Transform中提取的线段数据中找出代表路基的两条

bool Vec4isort(Vec4i i, Vec4i j) {
	int a = (i[1] < i[3] ? i[1] : i[3]);
	int b = (j[1] < j[3] ? j[1] : j[3]);
	if (a < b) return 1;
	else return 0;
}

bool Vec2fsort(Vec2f i, Vec2f j) {
	if (i[1] < j[1]) return 1;
	else return 0;
}


int main()
{
	//extract_video();
	string data_path = "data/clip_9.mp4";

	VideoCapture cap(data_path);

	if (!cap.isOpened()) {
		cerr << "can't open the video file!" << endl;
		return -1;
	}
	
	//检测马路边界，确定监控范围
	vector<Vec2f> edges = FindRoadEdge(cap);		//这里返回检测部分的路边界，以极坐标表示，edges[0],edges[1]分别是上下边界。edges[0][0]是r, edges[0][1]是theta。 edges[1]同理
	Mat img = imread("background.png");
	DrawLine(edges[0], img);
	DrawLine(edges[1], img);

	Point p1, p2;
	for (int i = 0; i <= 1; i++) {
		int width = img.cols;
		float rho = edges[i][0];
		float theta = edges[i][1];
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		Point pt1(0,
			cvRound(y0 + x0 * (1 / tan(theta))));
		Point pt2(width,
			cvRound(y0 - (1 / tan(theta) * (width - x0))));
		//line(img, pt1, pt2, Scalar(255), 3, 8);
		if (i == 0) p1 = pt1;
		else p2 = pt1;
	}
	Vec2f perspective_point = CalPerspectivePoint(edges);
	Point p0(0, 0);
	p0.y = p1.y + (p2.y - p1.y) * 0.66;
	Point p4(perspective_point[0], perspective_point[1]);
	line(img, p0, p4, Scalar(255), 3, 8);
	Point p01(0, 0);
	p01.y = p1.y + (p2.y - p1.y) * 0.31;
	line(img, p01, p4, Scalar(255), 3, 8);

	BoundingBox box1;
	box1.initBox(0, edges, perspective_point, img, "up");
	box1.drawBox(img);

	//绘制RoadEdge Detection结果显示
	namedWindow("edges", WINDOW_NORMAL);
	imshow("edges", img);
	waitKey(0);

	//侦测行人以及车辆
	ObjectDetect(data_path, edges);

	return 0;
}


/* This is a standalone program. Pass an image name as the first parameter
of the program.  Switch between standard and probabilistic Hough transform
by changing arg from 1 to 0 and back */
vector<Vec2f> FindLine(Mat source, int arg)
{
	//namedWindow("src", WINDOW_NORMAL);
	Mat src, dst, color_dst, frame1, frame2, diff;
	
	src = source.clone();
	
	Canny(src, dst, 50, 200, 3);
	cvtColor(dst, color_dst, CV_GRAY2BGR);
	//imshow("color_dst", color_dst);
	//waitKey(10000);
	Mat edge(color_dst.rows, color_dst.cols, CV_8UC3, Scalar(0, 0, 0));
	vector<Vec2f> edges;

	if (arg == 0) {
		vector<Vec2f> lines;		//Vec2f定义:typedef Vec<float, 2> Vec2f;
		HoughLines(dst, lines, 1, CV_PI / 180, 160);

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
		vector<Vec4i> lines;		//Vec4i定义：typedef Vec<int, 4> Vec4i;
		HoughLinesP(dst, lines, 1, CV_PI / 180, 80, 30, 30);
		for (size_t i = 0; i < lines.size(); i++)
		{
			//line(color_dst, Point(lines[i][0], lines[i][1]),Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 2, 8);
			//cout << "(" << lines[i][0] << "," << lines[i][1] << ")," << "(" << lines[i][2] << "," << lines[i][3] << ")" << endl;	//查看每一条线坐标
		}
		//cout << lines.size();
		edges = LineExtract(lines, edge);
	}

	//namedWindow("Source", WINDOW_NORMAL);
	//imshow("Source", src);

	

	//imwrite("data/RoadEdge8.png", edge);

	waitKey(0);
	return edges;
}



vector<Vec2f> FindRoadEdge(VideoCapture cap)
{
	
	Mat background, src, current;
	
	//找出一个平均背景
	//for (int i = 1; i < 600; i++) {
	//	cap >> src;
	//	if (i == 1) {
	//		background = src.clone();
	//		current = src.clone();
	//	}
	//	else {
	//		background = (current * (i - 1) + src) / i;
	//		current = background.clone();
	//	}
	//	cout << i << endl;
	//	//imshow("video", background);
	//	//waitKey(1000);
	//	/*if (waitKey(20) != 255) {
	//		int n = waitKey(10000);
	//		if (n == 27) break;
	//	}*/
	//}
	//imwrite("background.png", background);
	
	background = imread("background.png");
	//namedWindow("video", WINDOW_NORMAL);


	Mat gray_bg;
	cvtColor(background, gray_bg, CV_RGB2GRAY);
	//imshow("video", gray_bg);
	waitKey(0);
	threshold(gray_bg, gray_bg, 0, 255, CV_THRESH_OTSU);
	//imshow("video", gray_bg);
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
			if (i < nr / 5 || i > nr * 0.85) cur_line[j] = 255;
			if (j > nc / 2) cur_line[j] = 255;
		}
	}     // 把图像边缘像素设置为255
	result.row(0).setTo(Scalar(255));
	result.row(result.rows - 1).setTo(Scalar(255));
	result.col(0).setTo(Scalar(255));
	result.col(result.cols - 1).setTo(Scalar(255));

	//imshow("video", result);
	waitKey(0);

	vector<Vec2f> edges = FindLine(result,1);
	return edges;
}

vector<Vec2f> LineExtract(vector<Vec4i> lines, Mat img)
{
	cout << lines.size() << endl;
	sort(lines.begin(), lines.end(), Vec4isort);
	Vec4d up, down;		//indicate the upside edge and downside edge
	up = lines[0];
	down = lines[lines.size() - 1];
	cout << "(" << up[0] << "," << up[1] << ")," << "(" << up[2] << "," << up[3] << ")" << endl;
	cout << "(" << down[0] << "," << down[1] << ")," << "(" << down[2] << "," << down[3] << ")" << endl;
	Vec2d v_up((up[2] - up[0]), (up[3] - up[1]));
	Vec2d v_down((down[2] - down[0]), (down[3] - down[1]));	//将up down两条线变成两个二位向量，便于计算夹角。
	vector<int> up_flags, down_flags;
	
	//计算上边界
	for (size_t i = 1; i < lines.size() - 1; i++) {
		double param = 3.5;
		double theta1, theta2;	//theta1表示当前线段与up/down方向的夹角的cos，theta2表示当前线段起点与up/down起点的连线与up/down线段的夹角的cos
		Vec2d temp((lines[i][2] - lines[i][0]), (lines[i][3] - lines[i][1]));
		theta1 = (temp[0] * v_up[0] + temp[1] * v_up[1])/(sqrt(v_up[0] * v_up[0] + v_up[1] * v_up[1]) * sqrt(temp[0] * temp[0] + temp[1] * temp[1]));
		if (theta1 > cos(param * PI / 180.0)) {
			temp[0] = lines[i][2] - up[0];
			temp[1] = lines[i][3] - up[1];
			theta2 = (temp[0] * v_up[0] + temp[1] * v_up[1]) / (sqrt(v_up[0] * v_up[0] + v_up[1] * v_up[1]) * sqrt(temp[0] * temp[0] + temp[1] * temp[1]));
			if (theta2 > cos(param * PI / 180.0)) {
				up_flags.push_back(i);
			}
		}
	}

	//计算下边界
	for (size_t i = 1; i < lines.size() - 1; i++) {
		double param = 3.5;
		double theta1, theta2;	//theta1表示当前线段与down/down方向的夹角的cos，theta2表示当前线段起点与up/down起点的连线与up/down线段的夹角的cos
		Vec2d temp((lines[i][2] - lines[i][0]), (lines[i][3] - lines[i][1]));
		theta1 = (temp[0] * v_down[0] + temp[1] * v_down[1]) / (sqrt(v_down[0] * v_down[0] + v_down[1] * v_down[1]) * sqrt(temp[0] * temp[0] + temp[1] * temp[1]));
		if (theta1 > cos(param * PI / 180.0)) {
			temp[0] = lines[i][2] - down[0];
			temp[1] = lines[i][3] - down[1];
			theta2 = (temp[0] * v_down[0] + temp[1] * v_down[1]) / (sqrt(v_down[0] * v_down[0] + v_down[1] * v_down[1]) * sqrt(temp[0] * temp[0] + temp[1] * temp[1]));
			if (theta2 > cos(param * PI / 180.0)) {
				down_flags.push_back(i);
			}
		}
	}

	for (size_t i = 0; i < up_flags.size(); i++) {
		line(img, Point(lines[up_flags[i]][0], lines[up_flags[i]][1]),
			Point(lines[up_flags[i]][2], lines[up_flags[i]][3]), Scalar(0, 0, 255), 2, 8);
	}
	for (size_t i = 0; i < down_flags.size(); i++) {
		line(img, Point(lines[down_flags[i]][0], lines[down_flags[i]][1]),
			Point(lines[down_flags[i]][2], lines[down_flags[i]][3]), Scalar(0, 0, 255), 2, 8);
	}

	//namedWindow("Detected Lines", WINDOW_NORMAL);
	//imshow("Detected Lines", img);
	waitKey(0);
	Mat dst;

	cvtColor(img, dst, CV_RGB2GRAY);
	vector<Vec2f> v2f_lines;		//Vec2f定义:typedef Vec<float, 2> Vec2f;
	HoughLines(dst, v2f_lines, 1, CV_PI / 180, 180);


	/*imshow("Detected Lines", img);
	waitKey(0);*/
	cout << endl << "Hough Transform:" << endl;
	for (size_t i = 0; i < v2f_lines.size(); i++) {
		cout << v2f_lines[i][0] << "," << v2f_lines[i][1] << endl;
	}

	vector<Vec2f> up_lines, down_lines;
	for (size_t i = 0; i < v2f_lines.size(); i++) {
		if (v2f_lines[i][0] < 500) up_lines.push_back(v2f_lines[i]);
		else down_lines.push_back(v2f_lines[i]);
	}
	sort(up_lines.begin(), up_lines.end(), Vec2fsort);
	sort(down_lines.begin(), down_lines.end(), Vec2fsort);
	Vec2f up_edge, down_edge;
	if (up_lines.size() == 0) cerr << "ERROR: up road edge failed to detect!" << endl;
	else if (up_lines.size() % 2 == 1) {
		up_edge = up_lines[up_lines.size() / 2];
	}
	else {
		up_edge = up_lines[(up_lines.size() - 1) / 2];
	}

	if (down_lines.size() == 0) cerr << "ERROR: down road edge failed to detect!" << endl;
	else if (down_lines.size() % 2 == 1) {
		down_edge = down_lines[down_lines.size() / 2];
	}
	else {
		down_edge = down_lines[(down_lines.size() - 1) / 2];
	}

	DrawLine(up_edge, img);
	DrawLine(down_edge, img);
	//imshow("Detected Lines", img);
	waitKey(0);

	vector<Vec2f> edges;
	edges.push_back(up_edge);
	edges.push_back(down_edge);
	return edges;

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