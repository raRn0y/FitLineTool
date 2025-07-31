#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

enum class Translation
{
	All=0,
	Negative,
	Poisitive
};

enum class Selection
{
	All=0,
	Fisrt,
	Last,
	Strongest,
	weakest,
};

struct Edge1D_Result
{
	Edge1D_Result(Point2d edgePoint, double gradient) :
		m_pdEdgePoint(edgePoint), m_dGradient(gradient)
	{

	}
	Point2d m_pdEdgePoint;
	double m_dGradient;
};

class Extract1DEdge
{
public:
			Extract1DEdge();

private:
	Mat		m_mInputMat;
	Mat		m_mProfieMat;
	Point2d m_pdCenter;
	double	m_dSigma;
	int		m_nThreshold;
	double	m_dAngle;//deg
	double	m_dLength;
	double	m_dHeight;
	double	m_dK;
	double	m_dB;
	vector<Point2d>m_vpCandidate;//x: xedg; y: grandient
	vector<Edge1D_Result>m_vEdgesResult;
	Point2d m_pdStart;
	Point2d m_pdEnd;
	void	GetProfieMat();
	void	FilterMat();
	void	GetGradientMat();
	void	GetEdgePoint(int threshold, Translation traslation = (Translation)0, Selection selection = (Selection)0);
public:
	vector<Edge1D_Result> Get1DEdge(Mat inputMat,Point2d pdCenter, double dMeasureLength, double dMeasureHeight,double dMeasureAngle,double sigma, int threshold, Translation traslation = (Translation)0, Selection selection = (Selection)0);

	
};

