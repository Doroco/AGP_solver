#include "AGP_solver.h"
#include<list>

void AGP_solver::BinaryFilter(Mat& img, int _BinaryThreshold)
{
	for (int h = 0; h < img.rows; ++h)
	{
		for (int w = 0; w < img.cols; ++w)
		{
			if (img.at<uchar>(h, w) <= _BinaryThreshold)
				img.at<uchar>(h, w) = 0;
			else
				img.at<uchar>(h, w) = 255;
		}
	}
}

Obstacles AGP_solver::getBoder()
{
	//cout << _BinaryMap.size() << endl;
	//장애물의 bolder를 찾는 함수 
	for (int h = 1; h < _BinaryMap.rows; h++)
	{
		for (int w = 1; w < _BinaryMap.cols; w++)
		{
			if (_BinaryMap.at<uchar>(h, w) == 0)
			{
				neighborSearch(h, w);
			}
		}
	}
	return _Obstacles;
}

void AGP_solver::neighborSearch(int w, int h)
{
	Point2i  _Op(h, w);

	list<Point2i> posGroup;

	for (int i = 0; i < 8; i++) {
		Point2i SearchPos(_Op.x + eightWay[i][0], _Op.y + eightWay[i][1]);
		if (_BinaryMap.at<uchar>(SearchPos.x, SearchPos.y) == 0)
			posGroup.emplace_back(SearchPos);
	}
	//8-Neighbor중에 하나라도 배경이 있어야함
	if (posGroup.size() < 8)
	{
		//cout << posGroup.size() << endl;
		it = find(_checkObstacle.begin(), _checkObstacle.end(), _Op);
		if (it != _checkObstacle.end())
			return;
		else
		{
			_checkObstacle.emplace_back(_Op);
			neighborSearch(posGroup.front().x, posGroup.front().y);
		}
	}
	posGroup.clear();
}

void AGP_solver::PrintBoder()
{
	for (auto iter = _checkObstacle.begin(); iter != _checkObstacle.end(); iter++) {

		Point2i p = (*iter);
		_BinaryMap.at<uchar>(p.x, p.y) = 150;
	}
	cv::namedWindow("image", WINDOW_GUI_NORMAL);

	cv::imshow("image", _BinaryMap);

	cv::waitKey();
}

