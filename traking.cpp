#include "traking.h"


float seg_ratio = 3.0;	//this is used when segmentate the car from gray_diff image
int min_thresh = 800;	//the min_thresh when segmentate the car from gray_diff image
int max_gap = 300;	//determine the gap bewteen cars

void VehicleTracking(vector<Mat> frames, vector<Vec2f> edges, int frame_num, Mat H)
{
	Vec2f pers_point = CalPerspectivePoint(edges);

	//读取vehicle_frame.txt中储存的车辆分类信息
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

	//为每个车辆确定BoundingBox以及车辆种类
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
		//imshow("diff", gray_diff);
		//waitKey(0);
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
						break;
					}
					else if (gap[0] >= 500) {
						temp.type = BoundingBox::truck;
						temp.modifyBox();
						break;
					}
				}
			}
			previous_num = pixels[j];
		}
		temp.trajectory.push_back(calCenter(gray_diff, temp, last_frame));

		boxes.push_back(temp);

	}
	//上述代码找到判定为车辆的位置并生成BoundingBox
	
	//逐帧计算center
	/*for (int j = 0; j < boxes.size(); j++) {
		Mat f1, f2;
		frames[boxes[j].trajectory[0].frame].copyTo(f1);
		frames[boxes[j].trajectory[0].frame + 1].copyTo(f2);
		Mat diff = f1 - f2;
		Mat gray_diff;
		cvtColor(diff, gray_diff, COLOR_RGB2GRAY);
		circle(f2, Point(boxes[j].trajectory[0].x, boxes[j].trajectory[0].y), 5, Scalar(255), 2);
		imshow("diff", f2);
		waitKey(0);
	}*/		//测试代码，看看质心画在哪了


	for (int j = 0; j < boxes.size(); j++) {
		boxes[j].trajectory[0].y = boxes[j].trajectory[0].x * boxes[j].am + boxes[j].bm;
		for (int n = boxes[j].trajectory[0].frame + 1; n < frames.size(); n++) {
			boxes[j].resize();
			Mat f1, f2, diff, gray_diff, g2;
			frames[n].copyTo(f2);
			frames[n - 1].copyTo(f1);
			diff = f1 - f2;
			cvtColor(diff, gray_diff, COLOR_RGB2GRAY);
			Trajectory t = calCenter(gray_diff, boxes[j], n);
			t.y = t.x * boxes[j].am + boxes[j].bm;
			boxes[j].trajectory.push_back(t);
			if (boxes[j].p2.x >= f1.cols) break;

			cvtColor(f2, g2, COLOR_RGB2GRAY);
			//Canny(g2, g2, 50, 150);
			threshold(gray_diff, gray_diff, 25, 200, THRESH_BINARY);
			boxes[j].drawBox(f2);
			circle(f2, Point(boxes[j].trajectory.back().x, boxes[j].trajectory.back().y), 5, Scalar(255), 2);


			imshow("diff", f2);
			int key_value = waitKey(10);
			if (key_value != 255)
			{
				if (key_value == 27) break;		//press ESC to break
				else {
					key_value = waitKey(100000);
					if (key_value == 27) break;
				}
			}
			cout << "j = " << j << ";  n = " << n << endl;
		}
	}
	//将轨迹存储在txt文件中
	ofstream ftr;
	ftr.open("trajectory/tra.txt");
	ftr << "test:" << endl;
	for (int j = 0; j < boxes.size(); j++) {
		vector<Point2f> ori_traj(boxes[j].trajectory.size()), trans_traj(boxes[j].trajectory.size());
		for (int n = 0; n < boxes[j].trajectory.size(); n++) {
			ori_traj[n].x = boxes[j].trajectory[n].x;
			ori_traj[n].y = boxes[j].trajectory[n].y;
		}
		perspectiveTransform(ori_traj, trans_traj, H);
		ftr << "car 1:" << endl;
		for (int n = 0; n < boxes[j].trajectory.size(); n++) {
			ftr << "(" << boxes[j].trajectory[n].frame << "," << trans_traj[n].x << "," << trans_traj[n].y << ") ";
		}
		ftr << endl;
	}
	
}

Trajectory calCenter(Mat gray_diff, BoundingBox box, int frame_num)
{
	long long sum_x = 0, sum_y = 0, count = 0, mass = 0;
	for (int x = box.p1.x; x < box.p2.x + box.max_extend; x++) {
		for (int y = box.au * x + box.bu; y < box.ad * x + box.bd; y++) {
			mass += gray_diff.at<uchar>(y, x);
			//sum_x += x * gray_diff.at<uchar>(y, x);
			//sum_y += y * gray_diff.at<uchar>(y, x);
			if (gray_diff.at<uchar>(y, x) > 25) {
				sum_x += x;
				sum_y += y;
				count++;
			}
		}
		if (x >= gray_diff.cols - 1) break;
	}
	int cx, cy;
	/*if (mass != 0) {
		cx = sum_x / mass;
		cy = sum_y / mass;
	}
	else {
		cx = box.trajectory.back().x;
		cy = box.trajectory.back().y;
	}*/
	if (count != 0) {
		cx = sum_x / count;
		cy = sum_y / count;
	}
	else {
		cx = box.trajectory.back().x;
		cy = box.trajectory.back().y;
	}
	if (count < 200) {
		box.max_speed = 1;
	}
	else box.max_speed = 20;
	if (box.trajectory.size() > 0 && cx <= box.trajectory.back().x) cx = box.trajectory.back().x;
	if (box.trajectory.size() > 0 && cx - box.trajectory.back().x > box.max_speed) cx = box.trajectory.back().x + box.max_speed;
	return Trajectory(frame_num, cx, cy);
}
