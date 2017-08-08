#include "traking.h"


float seg_ratio = 3.0;	//this is used when segmentate the car from gray_diff image
int min_thresh = 800;	//the min_thresh when segmentate the car from gray_diff image
int max_gap = 300;	//determine the gap bewteen cars

void VehicleTracking(vector<Mat> frames, vector<Vec2f> edges, int frame_num)
{
	Vec2f pers_point = CalPerspectivePoint(edges);
	ifstream fin;
	fin.open("vehicle_frames.txt");
	vector<Vec2i> vehicles;
	while(!fin.eof()) {
		int temp1,temp2;
		fin >> temp1;
		fin >> temp2;
		if (fin.eof()) break;
		Vec2i t(temp1, temp2);
		vehicles.push_back(t);
	}
	fin.close();

	namedWindow("diff", WINDOW_NORMAL);
	vector<BoundingBox> boxes;
	for (int i = 0; i < vehicles.size(); i++) {
		Mat frame1, frame2;
		int last_frame = vehicles[i][1];
		frames[last_frame].copyTo(frame1);
		frames[last_frame + 1].copyTo(frame2);

		Mat diff = frame1 - frame2;
		Mat gray_diff;
		cvtColor(diff, gray_diff, COLOR_RGB2GRAY);
		imshow("diff", gray_diff);
		waitKey(0);
		BoundingBox temp;
		temp.initBox(0, edges, pers_point, frame1, "down");

		//统计车道上的竖向像素和
		vector<int> pixels;
		for (int x = 0; x < gray_diff.cols; x++) {
			int sum = 0;
			for (int y = (int)(temp.au * x + temp.bu); y < (int)(temp.ad * x + temp.bd); y++) {
				sum += gray_diff.at<uchar>(y, x);
			}
			sum = sum * (temp.p3.y - temp.p1.y) / ((temp.ad * x + temp.bd) - (temp.au * x + temp.bu));
			pixels.push_back(sum);
		}


		int summation = 0;
		for (int j = 0; j < pixels.size(); j++) {
			summation += pixels[j];
		}
		int avg_sum = summation / pixels.size();
		int thresh = (avg_sum > min_thresh) ? avg_sum : min_thresh;

		int p[3];
		for (int j = 0; j < pixels.size(); j++) {
			pixels[j] = (pixels[j] > thresh) ? 1 : 0;
			if (j <= 2) p[j] = pixels[j];
			else {
				int q[3] = {p[0], p[1], p[2]};
				if (q[1] < q[0]) {
					int t = q[1];
					q[1] = q[0];
					q[0] = t;
				}
				if (q[2] < q[1]) {
					int t = q[1];
					q[1] = q[2];
					q[2] = t;
				}
				if (q[1] < q[0]) {
					int t = q[1];
					q[1] = q[0];
					q[0] = t;
				}
				pixels[j - 2] = q[1];
				p[0] = p[1];
				p[1] = p[2];
				p[2] = pixels[j];
			}
		}

		Vec2i gap(0,0);
		int previous_num;
		for (int j = 0; j < pixels.size(); j++) {
			if (j == 0) previous_num = pixels[j];
			else if (previous_num == 1 && pixels[j] == 0) gap[0] = j;
			else if (previous_num == 0 && pixels[j] == 1) {
				gap[1] = j;
				if (gap[1] - gap[0] >= max_gap) {
					if (gap[0] < 500) {
						temp.type = BoundingBox::car;
						temp.modifyBox();
					}
				}
			}
			previous_num = pixels[j];
		}


		ofstream fout;
		fout.open("pixels.txt");
		for (int j = 0; j < pixels.size(); j++) {
			fout << pixels[j] << endl;
		}
		fout.close();

	}



	namedWindow("video", WINDOW_NORMAL);
	for (int i = 1; i < frame_num; i++) {
		Mat frame2, frame1, diff;
		frames[i - 1].copyTo(frame1);
		frames[i].copyTo(frame2);
		if (frame2.empty())
			break;

		if (i % 50 == 0) cout << i << endl;
		bool in_range;
		for (int n = 0; n < vehicles.size(); n++) {
			if (i - 1 >= vehicles[n][0] && i - 1 <= vehicles[n][1]) {
				in_range = true;
				break;
			}
			else in_range = false;
		}
		//if (in_range == false) continue;

		diff = frame1 - frame2;
		Mat gray_diff, gray1, gray2;
		//cvtColor(frame1, gray1, COLOR_RGB2GRAY);
		//cvtColor(frame2, gray2, COLOR_RGB2GRAY);
		cvtColor(diff, gray_diff, COLOR_RGB2GRAY);

		//threshold(gray1, gray1, 15, 255, CV_THRESH_OTSU);
		//threshold(gray2, gray2, 100, 255, CV_THRESH_BINARY);

		//gray_diff = gray1 - gray2;
		//threshold(gray_diff, gray_diff, 15, 255, CV_THRESH_OTSU);
		GaussianBlur(gray_diff, gray_diff, Size(3, 5), 0, 0);
		//threshold(gray_diff, gray_diff, 10, 255, CV_THRESH_BINARY);

		//DrawLine(edges[0], gray_diff);
		//DrawLine(edges[1], gray_diff);
		Vec2f perspective_point = CalPerspectivePoint(edges);
		BoundingBox box1;
		box1.initBox(0, edges, perspective_point, gray2, "down");
		vector<Point2f> corners;

		Mat crop_gray;
		crop_gray.create(gray2.size(), gray2.type());
		int margin = 0;
		for (int y = 0; y < gray2.rows; y++) {
			if (y < box1.p1.y - margin - 50 || y > box1.p3.y + margin) continue;
			for (int x = 0; x < gray2.cols; x++) {
				if (box1.au * x + box1.bu - margin <= y && box1.ad * x + box1.bd + margin >= y) crop_gray.at<uchar>(y, x) = gray2.at<uchar>(y, x);
			}
		}

		goodFeaturesToTrack(crop_gray, corners, 50, 0.05, 30);

		for (int m = 0; m < corners.size(); m++) {
			circle(crop_gray, corners[m], 5, Scalar(255));
		}

		box1.drawBox(crop_gray);
		//逐帧播放视频
		imshow("video", gray_diff);
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
