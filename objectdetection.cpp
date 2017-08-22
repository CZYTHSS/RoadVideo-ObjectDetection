#include "objectdetection.h"
#include "utils.h"


/*
min_frame_length��ʾһ����Ч�ĳ�������������������ٸ�1��
max_frame_interval��ʾ������1֮����ֶ��ٵļ������Ϊ����һ����
thresh_ratio��ʾthreshӦ�ñ����غ�ƽ��ֵ�༸��
min_total_frame��ʾһ��������1����ԭʼ��flag_pixel��Ҫ���ֶ��ٸ���1������Ϊ����Ч�ģ�ps���������10000100001000...������������������ٽ������ֵ
thresh_min ��ʾthreshֵ���ܵ��ڶ���
*/



float thresh_ratio = 3.0;
int max_frame_interval = 15;
int min_frame_length = 7;
int min_total_frame = 7;
int thresh_min = 500;
void ObjectDetect(vector<Mat> &frames, vector<Vec2f> edges, int frame_num)
{


	//���������߽�Ľ���
	Vec2f perspective_point = CalPerspectivePoint(edges);

	BoundingBox box1;
	box1.initBox(0, edges, perspective_point, frames[0], "down");
	
	int *enter_pixels = new int[frame_num];
	memset(enter_pixels, 0, frame_num);
	int count = 0;

	//��ȡ������Ƶ��ͳ��entranceλ�õ����غͣ�����count(֡��)һ�����array.txt��. enter_pixels[i]��ʾ���ǵ�i֡��i-1֡�Ĳ�ֵ����ͳ�ƽ��
	for (int i = 1; i < frame_num; i++) {
		Mat frame1, frame2, diff;
		frames[i - 1].copyTo(frame1);
		frames[i].copyTo(frame2);
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
		//imshow("gray_diff", gray_diff);
		//waitKey(0);
		//threshold(gray_diff, gray_diff, 15, 255, CV_THRESH_OTSU);
		//GaussianBlur(gray_diff, gray_diff, Size(3, 5), 0, 0);
		//threshold(gray_diff, gray_diff, 10, 255, CV_THRESH_BINARY);
		int sum = 0;
		for (int i = box1.p1.y; i < box1.p3.y; i++) {
			sum += gray_diff.at<uchar>(i, 10);
		}
		enter_pixels[count] = sum;
		count++;
		if (count % 50 == 0) cout << count << endl;

	}
	ofstream fout;
	fout.open("array.txt");
	fout << count << endl;
	for (int i = 0; i < count; i++) {
		fout << enter_pixels[i] << endl;
	}
	fout.close();

	//ofstream fout;
	ifstream fin;
	fin.open("array.txt");
	fin >> count;
	for (int i = 0; i < count; i++) {
		fin >> enter_pixels[i];
	}
	fin.close();



	//����һ���������ֵ����enter_pixels
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
	
	thresh = thresh_ratio * thresh / temp_count;
	if (thresh < thresh_min) thresh = thresh_min;

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
			else if (last_flag == 0) {
				Vec2i temp(count - 1, count - 1);
				continue_1.push_back(temp);
			}
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
		if (continue_1[i + 1][0] - continue_1[i][1] < max_frame_interval) {
			continue_1[i][1] = continue_1[i + 1][1];
			continue_1.erase(continue_1.begin() + i + 1);
			i--;
		}
	}

	for (int i = 0; i < continue_1.size(); i++) {
		if (continue_1[i][1] - continue_1[i][0] < min_frame_length) {
			continue_1.erase(continue_1.begin() + i);
			i--;
		}
	}
	
	for (int i = 0; i < continue_1.size(); i++) {
		int summation = 0;
		for (int j = continue_1[i][0]; j < continue_1[i][1]; j++) {
			summation += flag_pixels[j];
		}
		if (summation <= min_total_frame) {
			continue_1.erase(continue_1.begin() + i);
			i--;
		}
	}
	//����֮���continue_1����洢��Ӧ������⵽������Ƶ�н��뻭��ĳ�����
	//��continue_1�洢��txt�ļ�
	fout.open("vehicle_frames.txt");
	for (int i = 0; i < continue_1.size(); i++) {
		fout << continue_1[i][0] << " " << continue_1[i][1] << endl;
	}
	fout.close();

}

void vehicleDetection()
{

}
