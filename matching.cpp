/**
* @file MatchTemplate_Demo.cpp
* @brief Sample code to use the function MatchTemplate
* @author OpenCV team
*/
#include "matching.h"

//! [declare]
/// Global Variables
bool use_mask;
Mat img; Mat templ; Mat mask; Mat result;
const char* image_window = "Source Image";
const char* result_window = "Result window";

int match_method;
int max_Trackbar = 5;
//! [declare]

/// Function Headers
Point MatchingMethod(int, void*);

Mat FindWorldHomography()
{
	int template_num = 7;
	vector<Point2f> obj, scene;

	for (int i = 1; i <= template_num; i++) {
		string s = to_string(i);
		string src_path = "bgs/img_ref.png";
		src_path = "background.png";
		string temp_path = "templates/template" + s + ".png";
		Point2f matchLoc = TempMatching(src_path, temp_path);
		obj.push_back(matchLoc);
	}
	ifstream fin;
	fin.open("templates/template_location.txt");
	for (int i = 1; i <= template_num; i++) {
		Point2f t;
		fin >> t.x;
		fin >> t.y;
		scene.push_back(t);
		if (i != 1) {
			scene[i - 1].x = scene[i - 1].x - scene[0].x;
			scene[i - 1].y = scene[i - 1].y - scene[0].y;
		}
	}
	scene[0].x = 0;
	scene[0].y = 0;

	Mat H = findHomography(obj, scene, CV_RANSAC);
	cout << H << endl;

	ofstream fout;
	fout.open("homography.txt");
	int Htype = H.type();
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			fout << H.at<double>(i, j) << " ";
		}
		fout << endl;
	}
	fout.close();
	return H;
}

/**
* @function main
*/
Point TempMatching(string src_path, string temp_path)
{
	string path;
	/*for (int i = 1; i <= 15; i++) {
		string s = to_string(i);
		path = "data/clip_" + s + ".mp4";
		VideoCapture cap(path);
		Mat img;
		cap >> img;
		string dst;
		dst = "bgs/img_" + s + ".png";
		imwrite(dst, img);
	}*/

	//! [load_image]
	/// Load image and template
	img = imread(src_path, IMREAD_COLOR);
	templ = imread(temp_path, IMREAD_COLOR);


	if (img.empty() || templ.empty() || (use_mask && mask.empty()))
	{
		cout << "Can't read one of the images" << endl;
		return Point(0,0);
	}
	//! [load_image]

	//! [create_windows]
	/// Create windows
	//namedWindow(image_window, WINDOW_NORMAL);
	//namedWindow(result_window, WINDOW_NORMAL);
	//! [create_windows]

	//! [create_trackbar]
	/// Create Trackbar
	const char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
	//createTrackbar(trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod);
	//! [create_trackbar]

	Point centerLoc = MatchingMethod(0, 0);

	//! [wait_key]
	waitKey(0);
	destroyWindow(image_window);
	return centerLoc;
	//! [wait_key]
}



/**
* @function MatchingMethod
* @brief Trackbar callback
*/
Point MatchingMethod(int, void*)
{
	//! [copy_source]
	/// Source image to display
	Mat img_display;
	img.copyTo(img_display);
	//! [copy_source]

	//! [create_result_matrix]
	/// Create the result matrix
	int result_cols = img.cols - templ.cols + 1;
	int result_rows = img.rows - templ.rows + 1;

	result.create(result_rows, result_cols, CV_32FC1);
	//! [create_result_matrix]

	//! [match_template]
	/// Do the Matching and Normalize
	bool method_accepts_mask = (CV_TM_SQDIFF == match_method || match_method == CV_TM_CCORR_NORMED);
	if (use_mask && method_accepts_mask)
	{
		matchTemplate(img, templ, result, match_method, mask);
	}
	else
	{
		matchTemplate(img, templ, result, match_method);
	}
	//! [match_template]

	//! [normalize]
	normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());
	//! [normalize]

	//! [best_match]
	/// Localizing the best match with minMaxLoc
	double minVal; double maxVal; Point minLoc; Point maxLoc;
	Point matchLoc;

	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
	//! [best_match]

	//! [match_loc]
	/// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
	if (match_method == TM_SQDIFF || match_method == TM_SQDIFF_NORMED)
	{
		matchLoc = minLoc;
	}
	else
	{
		matchLoc = maxLoc;
	}
	//! [match_loc]


	//! [imshow]
	/// Show me what you got
	rectangle(img_display, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), Scalar::all(200), 2, 8, 0);
	rectangle(result, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), Scalar::all(200), 2, 8, 0);

	Point centerLoc;
	centerLoc.x = matchLoc.x + (templ.cols / 2);
	centerLoc.y = matchLoc.y + templ.rows / 2;
	circle(img_display, centerLoc, 5, Scalar(255), 2);
	
	//imshow(image_window, img_display);
	//imshow(result_window, result);
	//! [imshow]

	
	return centerLoc;
}