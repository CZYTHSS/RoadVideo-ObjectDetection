#include "objectdetection.h"
#include "utils.h"
void ObjectDetect(VideoCapture cap, vector<Vec2f> edges)
{
	//计算两条边界的交点
	Vec2f perspective_point = CalPerspectivePoint(edges);
	//cout << "perspective point:" << endl;
	//cout << perspective_point[0] << "," << perspective_point[1] << endl;

	Mat frame1, frame2, diff;
	cap >> frame2;


	BoundingBox box1;
	box1.initBox(0, edges, perspective_point, frame2, "down");
	
	VideoCapture cap2 = cap;
	int frn = cap2.get(CV_CAP_PROP_FRAME_COUNT);
	int *enter_pixels = new int[frn];
	memset(enter_pixels, 0, frn);
	namedWindow("video", WINDOW_NORMAL);
	int count = 0;
	for (;;) {
		frame1 = frame2.clone();
		cap2 >> frame2;
		if (frame2.empty())
			break;
		diff = frame1 - frame2;
		Mat gray_diff, gray1, gray2;
		cvtColor(frame1, gray1, COLOR_RGB2GRAY);
		cvtColor(frame2, gray2, COLOR_RGB2GRAY);
		//cvtColor(diff, gray_diff, COLOR_RGB2GRAY);

		//threshold(gray1, gray1, 15, 255, CV_THRESH_OTSU);
		//threshold(gray2, gray2, 100, 255, CV_THRESH_BINARY);

		gray_diff = gray1 - gray2;
		//threshold(gray_diff, gray_diff, 15, 255, CV_THRESH_OTSU);
		//GaussianBlur(gray_diff, gray_diff, Size(3, 5), 0, 0);
		//threshold(gray_diff, gray_diff, 10, 255, CV_THRESH_BINARY);
		int sum = 0;
		for (int i = box1.p1.y; i < box1.p3.y; i++) {
			sum += gray_diff.at<uchar>(i, 1);
		}
		enter_pixels[count] = sum;
		count++;
		if (count % 50 == 0) cout << count << endl;

		//imshow("video", gray_diff);
		//int key_value = waitKey(15);
		//if (key_value != 255)
		//{
		//	if (key_value == 27) break;		//press ESC to break
		//	else {
		//		key_value = waitKey(100000);
		//		if (key_value == 27) break;
		//	}
		//}
		//cout << sum << endl;
	}
	ofstream fout;
	fout.open("array.txt");
	for (int i = 0; i < count; i++) {
		fout << enter_pixels[i] << endl;
	}
	fout.close();

	//计算一个合理的阈值划分enter_pixels
	int sum_array = 0;
	for (int i = 0; i < count; i++) {
		sum_array += enter_pixels[i];
	}
	float avg_array = sum_array / count;
	float thresh = 0;
	int temp_count = 0;
	for (int i = 0; i < count; i++) {
		if (enter_pixels[i] < avg_array) {
			thresh += enter_pixels[i];
			temp_count++;
		}
	}
	thresh = 3 * thresh / temp_count;

	fout.open("flag_array.txt");
	int *flag_pixels = new int[count];
	vector<Vec2i> continue_1;
	int last_flag;
	for (int i = 0; i < count; i++) {
		if (enter_pixels[i] > thresh) flag_pixels[i] = 1;
		else flag_pixels[i] = 0;
		if (i == 0) {
			last_flag = flag_pixels[i];
			if (last_flag == 1) {
				Vec2i temp(0, 1);
				continue_1.push_back(temp);
			}
		}
		else if (i == count - 1) {
			if (last_flag == 1) continue_1[continue_1.size() - 1] = count - 1;
		}
		else {
			if (last_flag == 0) {
				if (flag_pixels[i] == 1) {
					last_flag = 1;
					Vec2i temp(i, 0);
					continue_1.push_back(temp);
				}
			}
			else if (last_flag == 1 && flag_pixels[i] == 0) {
				continue_1[continue_1.size() - 1][1] = i - 1;
			}
		}
		last_flag = flag_pixels[i];
		fout << flag_pixels[i] << endl;
	}
	fout.close();
	
	for (int i = 0; i < continue_1.size() - 1; i++) {
		if (continue_1[i + 1][0] - continue_1[i][1] < 5) {
			continue_1[i][1] = continue_1[i + 1][1];
			continue_1.erase(continue_1.begin() + i + 1);
			i--;
		}
	}
	
	for (int i = 0; i < continue_1.size() - 1; i++) {
		if (continue_1[i][1] - continue_1[i][0] < 8) {
			continue_1.erase(continue_1.begin() + i);
			i--;
		}
	}
	//处理之后的continue_1里面存储的应该是侦测到的在视频中进入画面的车辆。






	//namedWindow("video", WINDOW_NORMAL);
	for (;;) {
		frame1 = frame2.clone();
		cap >> frame2;
		if (frame2.empty())
			break;
		diff = frame1 - frame2;
		Mat gray_diff, gray1, gray2;
		cvtColor(frame1, gray1, COLOR_RGB2GRAY);
		cvtColor(frame2, gray2, COLOR_RGB2GRAY);
		//cvtColor(diff, gray_diff, COLOR_RGB2GRAY);

		//threshold(gray1, gray1, 15, 255, CV_THRESH_OTSU);
		//threshold(gray2, gray2, 100, 255, CV_THRESH_BINARY);

		gray_diff = gray1 - gray2;
		//threshold(gray_diff, gray_diff, 15, 255, CV_THRESH_OTSU);
		GaussianBlur(gray_diff, gray_diff, Size(3, 5), 0, 0);
		//threshold(gray_diff, gray_diff, 10, 255, CV_THRESH_BINARY);

		DrawLine(edges[0], gray_diff);
		DrawLine(edges[1], gray_diff);


		//BoundingBox box1;
		//box1.initBox(0, edges, perspective_point, gray_diff, "down");
		//box1.drawBox(gray_diff);


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

void vehicleDetection()
{

}
