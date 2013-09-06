#pragma once
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
class Utility
{
public:
	Utility(void);
	~Utility(void);
	void saveAsMatlab3D(Mat img, string path);
	void drawSegmentBorder(InputOutputArray imgInputOutput, Mat_<int> segment, Scalar color = Scalar(255, 255, 255));

	Mat depth2Color(Mat_<uint16_t> depth, int maxDepth, bool channals[3] = NULL);
	void depth2Color(Mat_<uint16_t> depth, Mat& colorImg, float range[2], bool discardOutPoint[2], bool channals[3] = NULL);
	vector<int> calcHist(Mat_<uint16_t> img, int segments, float range[2], int* sum = 0);
	Mat_<uchar> drawHist(vector<int> hist, int height, int cellWidth);

	// pcduino_project for depth info
	Point3f calcWeightCneter(Mat_<uint16_t> depthImg, Mat_<float>& wMat, float range[2], int minPoints = 500, float maxWeight = 3, int weightMarggin = 0);
	float getMinDepth(Mat_<uint16_t> depthImg, float depthMargin);
};
