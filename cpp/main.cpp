#include<iostream>

#include "AGP_solver.h"

int main()
{
	string t1 = "map_outline.JPG";

	Mat img = cv::imread(t1, IMREAD_GRAYSCALE);

	//if (img.rows > 1000) {
	//	cv::resize(img, img, Size(img.rows / 4, img.cols / 4));
	//	erode(img, img, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)));
	//	dilate(img, img, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)));
	//}

	AGP_solver test(img, 230, 30);
	//cout << img.size() << endl;
	test.getBoder();


	////cout << img.size() << endl;
	//test.PrintBoder();

	//test.fullAddCluster();


	cv::namedWindow("image", WINDOW_FREERATIO);

	cv::imshow("image", img);
	////cout << "result!!";
	//cv::waitKey();
	cv::waitKey();
	//cout << img.size() << endl;
	return 0;
}
