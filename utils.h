#pragma once
#define _CRT_SECURE_NO_DEPRECATE
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
#include <math.h>

#define PI 3.14159265

using namespace std;
using namespace cv;

Vec2f CalPerspectivePoint(vector<Vec2f> edges);

void DrawLine(Vec2f edge, Mat img);

class BoundingBox {
public:
	Point p1, p2, p3, p4;	//按照从上至下从左至右的顺序为p1,2,3,4
	float au, bu, ad, bd;	//a,b用来表示boundingBox的上下边界, y = ax + b; u -- up, d -- down, m -- middle
	float box_width;
	Mat img;

	void initBox(int x0, vector<Vec2f> edges, Vec2f pers_point, Mat img, String flag);
	BoundingBox() {
		box_width = 480;
	}

	void drawBox(Mat img);
};

void ReadFrames(VideoCapture cap, vector<Mat> &frames, int &frame_num);