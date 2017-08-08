#pragma once
#define _CRT_SECURE_NO_DEPRECATE
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
#include <math.h>
#include <fstream>
#include "utils.h"
#define PI 3.14159265

using namespace std;
using namespace cv;

void VehicleTracking(vector<Mat> frames, vector<Vec2f> edges, int frame_num);