#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <list>
#include "Extract1DEdge.h"
using namespace std;
using namespace cv;
#pragma once


class CCaliperGraphics
{
protected:
	enum class AdjustType
	{
		None=0,
		AdjustCenter,
		AdjustStart,
		AdjustEnd,
		AdjustMeasureLength,
		AdjustMeasureHeight,
		AdjustMeasureNums,
	};
	Mat		m_mInputMat;
	Point2d m_pdStart;
	Point2d m_pdEnd;
	Point2d m_pdCenter;
	double	m_dAngle;
	double	m_dLength;

	Point2d m_pdSampleLineStart;
	Point2d m_pdSampleLineEnd;


	vector<Point2d>m_vpdSampleLineEndPoints;
	vector<Point2d>m_vpdEquinoxPoints;
	vector<Point2d>m_vpTempPoints;
	vector<Point2d>m_vpdEdgePoints;
	vector<Point2d>m_vpdExcepetEdgePoints;
	vector<double>m_vdEdgeGradient;

	double	m_dMeasureAngle;
	double	m_dMeasureLength;
	double	m_dMeasureHeight;
	double	m_dSigma;
	int		m_nThreshold;
	int		m_nTranslation;
	int		m_nMeasureNums;
private:
	int				m_nCircleSize;
	bool			IsPointInCircle(Point2d pdCenter, Point2d pdPoint);
	AdjustType		m_eAdjustType;
	Extract1DEdge	extract1DEdge;
	void RansacLineFiler(const vector<Point2d>& points, vector<Point2d>& vpdExceptPoints,double sigma = 1);
public:
	CCaliperGraphics();
	void CreateCaliper(Mat& InputMat, Point2d pdStart, Point2d pdEnd, double dMeasureLength, double dMeasureHeight, double dSigma, int nThreshold, int nTranslation, int nMesureNums);
	void AdjustCaliper(Mat& InputMat, Point2d pdPoint, int nMeasureLength, int nMeasureHeight, double dSigma, int nThreshold, int nTranslation, int nMeasureNums, int nFlag = 0);
	void FindLine(Point2d& pdStart, Point2d& pdEnd, double& dAngle);
	void DisplayEdgePoints(Mat& InputMat, double dSize, Scalar color);
	void GetEdgeInfo(vector<Point2d>& vpdEdgePoints, vector<double>& vpdEdgePointsGradient);
};

