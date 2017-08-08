#include "utils.h"

Vec2f CalPerspectivePoint(vector<Vec2f> edges)
{
	float x, y;
	float r1, theta1, r2, theta2;
	r1 = edges[0][0];
	theta1 = edges[0][1];
	r2 = edges[1][0];
	theta2 = edges[1][1];

	x = (r2 * sin(theta2) - r1 * sin(theta1) + r2 * (1 / tan(theta2)) * cos(theta2) - r1 * (1 / tan(theta1)) * cos(theta1)) / ((1 / tan(theta2)) - (1 / tan(theta1)));
	y = (0 - (1 / tan(theta2))) * (x - r2 * cos(theta2)) + r2 * sin(theta2);
	
	return Vec2f(x, y);
}


void DrawLine(Vec2f edge, Mat img) {
	int width = img.cols;
	float rho = edge[0];
	float theta = edge[1];
	double a = cos(theta), b = sin(theta);
	double x0 = a*rho, y0 = b*rho;
	Point pt1(0, 
		cvRound(y0 + x0 * (1 / tan(theta))));
	Point pt2(width,
		cvRound(y0 - (1/tan(theta) * (width - x0))));
	line(img, pt1, pt2, Scalar(255), 3, 8);
}

void ReadFrames(VideoCapture cap, vector<Mat> &frames, int &frame_num)
{
	cout << "Reading Video Frames:" << endl;
	frame_num = 0;
	Mat frame;
	for (;;) {
		cap >> frame;
		if (frame.empty())
			break;
		Mat temp;
		frame.copyTo(temp);
		frames.push_back(temp);
		frame_num++;
		if (frame_num % 50 == 0) cout << frame_num << endl;
	}
}

void BoundingBox::initBox(int xl, vector<Vec2f> edges, Vec2f pers_point, Mat src, String flag)
{
	pp = pers_point;
	img = src;
	//初始化au, bu,在这里求出的au, bu是路中间黄线的参数
	Point pu, pd;
	for (int i = 0; i <= 1; i++) {
		int width = src.cols;
		float rho = edges[i][0];
		float theta = edges[i][1];
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		Point pt1(0,
			cvRound(y0 + x0 * (1 / tan(theta))));
		Point pt2(width,
			cvRound(y0 - (1 / tan(theta) * (width - x0))));
		//line(img, pt1, pt2, Scalar(255), 3, 8);
		if (i == 0) pu = pt1;
		else pd = pt1;
	}
	//找出画面最左侧的两个点p0, p01分别作为两个车道的分割点
	Point p0(0, 0);
	p0.y = pu.y + (pd.y - pu.y) * 0.66;
	Point p01(0, 0);
	p01.y = pu.y + (pd.y - pu.y) * 0.31;
	Point pp(pers_point[0], pers_point[1]);

	au = (float)(p0.y - pp.y) / (float)(p0.x - pp.x);
	bu = (float)p0.y - au * (float)p0.x;

	//
	p1.x = xl; 
	p3.x = xl;
	int img_width = img.cols;
	if (flag == "down") {
		//求出下边界线参数,给ad, bd 赋值
		ad = (float)(pd.y - pp.y) / (float)(pd.x - pp.x);
		bd = (float)pd.y - ad * (float)pd.x;
		//给p1,p3赋值
		p1.y = au * p1.x + bu;
		p3.y = ad * p3.x + bd;

		float scale = (float)abs(p3.y - p1.y) / (float)abs(pd.y - p0.y);
		float w = box_width * scale;		//the width of this current bounding box

		//给p2, p4赋值
		p2.x = p1.x + w;
		p4.x = p2.x;
		p2.y = au * p2.x + bu;
		p4.y = ad * p4.x + bd;
	}
	else if (flag == "up") {
		ad = au;
		bd = bu;

		//求出上边界线参数,给au, bu 赋值
		au = (float)(p01.y - pp.y) / (float)(p01.x - pp.x);
		bu = (float)p01.y - au * (float)p01.x;
		//给p1,p3赋值
		p1.y = au * p1.x + bu;
		p3.y = ad * p3.x + bd;

		float scale = (float)abs(p3.y - p1.y) / (float)abs(p0.y - p01.y);
		float w = box_width * scale;		//the width of this current bounding box

										//给p2, p4赋值
		p2.x = p1.x + w;
		p4.x = p2.x;
		p2.y = au * p2.x + bu;
		p4.y = ad * p4.x + bd;
	}
}

void BoundingBox::drawBox(Mat img)
{
	line(img, p1, p2, Scalar(200), 3, 8);
	line(img, p2, p4, Scalar(200), 3, 8);
	line(img, p3, p4, Scalar(200), 3, 8);
	line(img, p3, p1, Scalar(200), 3, 8);
}

void BoundingBox::modifyBox()
{
	if (type == car) {
		box_width = 480;
	}
	else if (type == truck) {
		box_width = 800;
	}
}
