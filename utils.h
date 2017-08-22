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

class Trajectory {
public:
	int frame;	//frame number
	int x, y;	//mass center of the tracked object
	Trajectory() {};
	Trajectory(int f, int _x, int _y) {
		frame = f;
		x = _x;
		y = _y;
	}
};

class BoundingBox {
public:
	Point p1, p2, p3, p4;	//按照从上至下从左至右的顺序为p1,2,3,4
	float au, bu, ad, bd, am, bm;	//a,b用来表示boundingBox的上下边界, y = ax + b; u -- up, d -- down, m -- middle
	float box_width;
	Mat img;
	Vec2f pp;	//perspective point
	enum vehicle { car, truck };
	vehicle type;
	vector<Trajectory> trajectory;

	int max_speed;	//the max speed we assume a car can move in one frame; (unit: px)
	int max_extend;	//max extra pixel length we need to search besides bounding box

	void initBox(int x0, vector<Vec2f> edges, Vec2f pers_point, Mat img, String flag);
	BoundingBox() {
		box_width = 380;
		max_speed = 20;
		max_extend = 20;
	}

	void drawBox(Mat img);
	void modifyBox();
	void resize();
};




void ReadFrames(VideoCapture cap, vector<Mat> &frames, int &frame_num);