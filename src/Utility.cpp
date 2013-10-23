#include "Utility.h"
#include <stdio.h>

Utility::Utility(void)
{
}

Utility::~Utility(void)
{
}

void Utility::saveAsMatlab3D(Mat img, string path)
{
  Mat rv;
  FILE* fp;
  fp = fopen(path.c_str(), "w");
  cvtColor(img, rv, CV_RGB2GRAY);
  assert(rv.type() == CV_8UC1);
  string content = "";
  int rows = rv.rows, cols = rv.cols;
  for (int i = 0; i < rows; i++)
  {
    uchar* pRow = rv.ptr<uchar>(i);
    for (int j = 0; j < cols; j++)
    {
      stringstream ss;
      ss << pRow[j];
      string tmp = ss.str();
      content.append(tmp).append(" ");
    }
    content.append("\n");
  }
  fwrite(content.c_str(), content.size(), 1, fp);
  fclose(fp);
}

void Utility::drawSegmentBorder(InputOutputArray imgInputOutput, Mat_<int> segment, Scalar color /*= Scalar(255, 255, 255)*/)
{
  Mat img = imgInputOutput.getMat();
  Mat_<uchar> mask(img.rows, img.cols); // ��־SuperPixel�߽�ľ���
  mask.setTo(0);
  int rows = img.rows;
  int cols = img.cols;
  for (int i = 1; i < rows; i++)
  {
    int* pCurrentPoint = segment.ptr<int>(i) + 1; // ָ��ǰ��
    int* pUpperPoint = segment.ptr<int>(i - 1) + 1; // ָ������ĵ�
    for (int j = 1; j < cols; j++)
    {
      int cPoint = *pCurrentPoint; // ��ǰ���SuperPixelID
      int lPoint = *(pCurrentPoint - 1); // ��ߵ��SuperPixelID
      int uPoint = *pUpperPoint; // ������SuperPixelID
      if (cPoint != lPoint || cPoint != uPoint)
      {
        mask(i, j) = 1;
      }
      pCurrentPoint++;
      pUpperPoint++;
    }
  }
  add(img, color, img, mask);
}

/**
 * to get gray image:
 *   Mat grayImg = util.depth2Color(depthImg, 1200);
 * to get color image:
 *   bool channals[3] = {true, false, false};
 *   Mat colorImg = util.depth2Color(depthImg, 1200, channals);
 */
Mat Utility::depth2Color(Mat_<uint16_t> depthImg, int maxDepth, bool channals[3])
{
  Mat colorImg;
  int rows = depthImg.rows;
  int cols = depthImg.cols;
  double revIntervel = 255.0 / maxDepth;
  if (NULL == channals)
  {
    colorImg.create(rows, cols, CV_8UC1);
    for (int i = 0; i < rows; i++)
    {
      uint16_t* pRow = depthImg.ptr<uint16_t>(i);
      uchar* pGrayRow = colorImg.ptr<uchar>(i);
      for (int j = 0; j < depthImg.cols; j++)
      {
        int bright = 0;
        if (pRow[j] <= maxDepth)
        {
          bright = 255 - pRow[j] * revIntervel;
        }
        pGrayRow[j] = bright;
      }
    }
  }
  else
  {
    colorImg.create(rows, cols, CV_8UC3);

      for (int i = 0; i < rows; i++)
      {
        uint16_t* pRow = depthImg.ptr<uint16_t>(i);
        Vec3b* pColorRow = colorImg.ptr<Vec3b>(i);
        for (int j = 0; j < depthImg.cols; j++)
        {
          int bright = 0;
          if (pRow[j] <= maxDepth)
          {
            bright = 255 - pRow[j] * revIntervel;
          }
          pColorRow[j] = Vec3b(channals[0] ? bright : 0, channals[1] ? bright : 0, channals[2] ? bright : 0);
        }
    }
  }
  return colorImg;
}

void Utility::depth2Color(Mat_<uint16_t> depthImg, Mat& colorImg, float range[2], bool discardOutPoint[2], bool channals[3])
{
  int rows = depthImg.rows;
  int cols = depthImg.cols;
  float minRange = range[0];
  float maxRange = range[1];
  float rangeLen = maxRange - minRange;
  double revIntervel = 255.0 / rangeLen;
  if (colorImg.type() == CV_8UC1)
  {
    for (int i = 0; i < rows; i++)
    {
      uint16_t* pRow = depthImg.ptr<uint16_t>(i);
      uchar* pGrayRow = colorImg.ptr<uchar>(i);
      for (int j = 0; j < depthImg.cols; j++)
      {
        int bright = 0;
        if (pRow[j] < minRange)
        {
          if (discardOutPoint[0])
            continue;
          bright = 255;
        }
        else if (pRow[j] <= maxRange)
        {
          bright = 255 - pRow[j] * revIntervel;
        }
        else
        {
          if (discardOutPoint[1])
            continue;
          bright = 0;
        }
        pGrayRow[j] = bright;
      }
    }
  }
  else
  {
    if (NULL == channals)
    {
      bool defChannals[3] = { true, true, true};
      channals = defChannals;
    }
    for (int i = 0; i < rows; i++)
    {
      uint16_t* pRow = depthImg.ptr<uint16_t>(i);
      Vec3b* pColorRow = colorImg.ptr<Vec3b>(i);
      for (int j = 0; j < depthImg.cols; j++)
      {
        int bright = 0;
        if (pRow[j] < minRange)
        {
          if (discardOutPoint[0]) continue;
          bright = 255;
        }
        else if(pRow[j] <= maxRange)
        {
          bright = 255 - pRow[j] * revIntervel;
        }
        else
        {
          if (discardOutPoint[1]) continue;
          bright = 0;
        }
        pColorRow[j] = Vec3b(channals[0] ? bright : 0, channals[1] ? bright : 0, channals[2] ? bright : 0);
      }
    }
  }
}

/**
 * to calc hist vector and elements sum:
 *   float range[2] = {0, 1000.0};
 *   int sum;
 *   vector<int> hist = util.calcHist(depthImg, 200, range, &sum);
 */
vector<int> Utility::calcHist(Mat_<uint16_t> img, int segments, float range[2], int* sum)
{
  vector<int> hist(segments, 0);
  if (NULL == sum)
  {
    int k;
    sum = &k;
  }
  *sum = 0;
  float minRange = range[0];
  float maxRange = range[1];
  float rangeLen = maxRange - minRange;
  double interVel = (double)rangeLen / segments;
  int rows = img.rows;
  int cols = img.cols;
  for (int i = 0; i < rows; i++)
  {
    uint16_t* pRow = img.ptr<uint16_t>(i);
    for (int j = 0; j < cols; j++)
    {
      if (pRow[j] >= minRange && pRow[j] <= maxRange)
      {
        int regionId = (pRow[j] - minRange) / interVel;
        hist[regionId]++;
        (*sum)++;
      }
    }
  }
  return hist;
}

Mat_<uchar> Utility::drawHist(vector<int> hist, int height, int cellWidth)
{

  int segments = hist.size();
  Mat_<uchar> histogram(height, segments * cellWidth);
  histogram.setTo(0);

  // norm histogram
  int maxHeight = 0;
  for (int i = 0; i < segments; i++)
  {
    if (hist[i] > maxHeight)
      maxHeight = hist[i];
  }

  // draw rectangle
  int histHeights[segments];
  for (int i = 0; i < segments; i++)
  {
    int histHeight = (double)hist[i] / maxHeight * height;
    histogram(Rect(i * cellWidth, height - histHeight, cellWidth, histHeight)).setTo(255);
  }

  return histogram;
}


Point3f Utility::calcWeightCneter(Mat_<uint16_t> depthImg, Mat_<float>& wMat, float range[2], int minPoints, float maxWeight, int weightMarggin)
{
  Point3f center;

  // param
  float minRange = range[0];
  float maxRange = range[1];
  int rows = depthImg.rows;
  int cols = depthImg.cols;
  int cameraDepthMargin = 10;

  // init
  float rangeLen = maxRange - minRange;
  assert(rangeLen > 0);
  wMat.create(rows, cols); // weight mat

  // calc weight
  float wSum = 0;
  double xSum, ySum, zSum;
  xSum = ySum = zSum = 0;
  int pointsSum = 0;// points sum which in the range
  for (int i = 0; i < rows; i++)
  {
    uint16_t* pDepthRow = depthImg.ptr<uint16_t>(i);
    float* pwMatRow = wMat.ptr<float>(i);
    for (int j = weightMarggin; j < cols - weightMarggin; j++)
    {
      int z = pDepthRow[j]; // depth of current point
      double w = 0;// weight of current point
      if (z > cameraDepthMargin && z < maxRange)// if z in the range, calc the w
      {
        if (z < minRange) z = minRange;
        w = (1 - (double)(z - minRange) / rangeLen) * maxWeight;
        wSum += w;
        xSum += j * w;
        ySum += i * w;
        zSum += z * w;
        pointsSum++;
      }
      pwMatRow[j] = w;
    }
  }

  // calc center
  if (pointsSum > minPoints)
  {
    center.x = xSum / wSum;
    center.y = ySum / wSum;
    center.z = zSum / wSum;
  }
  else
  {
    center = Point3f(0, 0, 0);
  }

  return center;
}

Point3f Utility::calcWeightCneter2(Mat_<uint16_t> depthImg, Mat_<float>& wMat, float range[2], int minPoints, float maxWeight, int weightMarggin)
{
  Point3f center;

  // param
  float minRange = range[0];
  float maxRange = range[1];
  int rows = depthImg.rows;
  int cols = depthImg.cols;
  int cameraDepthMargin = 10;

  // init
  float rangeLen = maxRange - minRange;
  assert(rangeLen > 0);
  wMat.create(rows, cols); // weight mat

  // calc weight
  float wSum = 0;
  double xSum, ySum, zSum;
  xSum = ySum = zSum = 0;
  int pointsSum = 0;// points sum which in the range
  for (int i = 0; i < rows; i++)
  {
    uint16_t* pDepthRow = depthImg.ptr<uint16_t>(i);
    float* pwMatRow = wMat.ptr<float>(i);
    for (int j = weightMarggin; j < cols - weightMarggin; j++)
    {
      int z = pDepthRow[j]; // depth of current point
      double w = 0;// weight of current point
      if (z > cameraDepthMargin && z < maxRange)// if z in the range, calc the w
      {
        if (z < minRange) z = minRange;
        w = (1 - (double)(z - minRange) / rangeLen) * maxWeight;
        wSum += w;
        xSum += j * w;
        ySum += i * w;
        zSum += z * w;
        pointsSum++;
      }
      pwMatRow[j] = w;
    }
  }

  // calc center
  if (pointsSum > minPoints)
  {
    center.x = xSum / wSum;
    center.y = ySum / wSum;
    center.z = zSum / wSum;

    center.x = (center.x - cols / 2) / (cols / 2 - weightMarggin);
    center.y = (center.y - rows / 2) / (rows / 2);
  }
  else
  {
    center = Point3f(0, 0, 0);
  }

  return center;
}

/**
 * margin: will not calculate the depth smaller than margin
 */
float Utility::getMinDepth(Mat_<uint16_t> depthImg, float depthMargin)
{
  // param
  int rows = depthImg.rows;
  int cols = depthImg.cols;

  // calc
  float minDepth = 99999;
  for (int i = 0; i < rows; i++)
  {
    uint16_t* pDepthRow = depthImg.ptr<uint16_t>(i);
    for (int j = 0; j < cols; j++)
    {
      float depth = pDepthRow[j];
      if (depth > depthMargin && depth < minDepth)
        minDepth = pDepthRow[j];
    }
  }

  return minDepth;
}
