#pragma once
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
#include "matching.h"
#define PI 3.14159265


using namespace std;
using namespace cv;

struct MouseParams
{
	vector<Point2f> points;
};

struct Lines
{
	float au, bu, am, bm, ad, bd;	//�ֱ������·�ϱ߽磬���ߣ��±߽�ֱ�߷��̲���
};

IplImage* src = 0;
void on_mouse(int event, int x, int y, int flags, void* param)
{
	MouseParams* mp = (MouseParams*)param;
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);//����ṹ��ʼ��  

	if ((event == CV_EVENT_LBUTTONDOWN) && (flags))//�����������¼�����  
	{
		CvPoint pt = cvPoint(x, y);//��ȡ��ǰ��ĺ�������ֵ  
		Point2f P(pt.x, pt.y);
		mp->points.push_back(P);
		char temp[16];
		sprintf(temp, "(%d,%d)", pt.x, pt.y);//��ӡ��ǰ����ֵ  
		cvPutText(src, temp, pt, &font, cvScalar(255, 255, 255, 0)); //��ͼ���д�ӡ��ǰ����ֵ   
		cvCircle(src, pt, 2, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);//����ͼ��ǰ������»�Բ  
		cvShowImage("src", src);
	}
}


//click points for road edge and road middle line, two points each.
Lines ClickPoints()
{
	MouseParams mp;
	Lines lines;
	src = cvLoadImage("background.png", 1); //����ͼ�� 

	cvNamedWindow("src", CV_WINDOW_NORMAL);//�½�����  
	cvSetMouseCallback("src", on_mouse, (void*)&mp);  //ע�������Ӧ�ص����� 
	cvShowImage("src", src);
	cvWaitKey(0);
	cvDestroyAllWindows();//�������д���  
	cvReleaseImage(&src);//�ͷ�ͼ������  

	for (int i = 0; i < mp.points.size(); i += 2) {
		float a, b;
		a = (mp.points[i].y - mp.points[i + 1].y) / (mp.points[i].x - mp.points[i + 1].x);
		b = (mp.points[i].x * mp.points[i + 1].y - mp.points[i + 1].x * mp.points[i].y) / (mp.points[i].x - mp.points[i + 1].x);
		if (i == 0) {
			lines.au = a;
			lines.bu = b;
		}
		else if (i == 2) {
			lines.am = a;
			lines.bm = b;
		}
		else if (i == 4) {
			lines.ad = a;
			lines.bd = b;
		}
	
	}

	Mat img;
	img = imread("background.png");
	line(img, mp.points[0], mp.points[1], Scalar(0, 255, 255), 2);
	line(img, mp.points[2], mp.points[3], Scalar(0, 255, 255), 2);
	line(img, mp.points[4], mp.points[5], Scalar(0, 255, 255), 2);
	namedWindow("lines", WINDOW_NORMAL);
	imshow("lines", img);
	waitKey(0);
	return lines;
}