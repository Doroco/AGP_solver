#pragma once
#pragma once
#include<opencv2/opencv.hpp>
#include<iostream>
#include<utility>
#include<vector>
#include<queue>
#include <algorithm>
#include <random>
#include<math.h>

#define PI 3.14159265359
#define Deg2Rad(x) (x)*(PI / 180)

using namespace std;
using namespace cv;

typedef Point2i Opoint;
typedef vector<Opoint>      Obstacle;
typedef vector<Obstacle>    Obstacles;
typedef vector<Obstacle>    Visibility;

class AGP_solver
{
public:
	struct FovArea
	{
		Point2i center;
		vector<Point2i> _particles;
		vector<Point2i> _particlesUnion;
	};

	vector<Opoint>      _checkObstacle;
	Obstacles  _Obstacles;
	vector< FovArea>	  _areas;
	Mat		  _Map;
	Mat		  _BinaryMap;
	Mat		  _CoveredMap;
	int			  _BinaryThreshold;
	double     Fov;

	//4-neighbor Method
	int fourWay[4][2] = { {0, 1}, {1, 0}, {0, -1}, {-1, 0} };

	//8-neightbor Method;
	int eightWay[8][2] = { {0, 1}, {1, 1}, {1, 0}, {1, -1},
								{0, -1}, {-1, -1}, {-1, 0}, {-1, 1} };

	vector<Opoint>::iterator it;
public:
	AGP_solver(Mat& img, int _Threshold, double range = 10)
	{
		this->_Map = img;
		this->_CoveredMap = _Map.clone();
		this->_BinaryThreshold = _Threshold;
		BinaryFilter(_Map, _BinaryThreshold);
		this->_BinaryMap = _Map.clone();
		this->Fov = range;
	};
	Obstacles getBoder();
	void PrintBoder();

private:
	void BinaryFilter(Mat& img, int _BinaryThreshold);
	void neighborSearch(int w, int h);
};