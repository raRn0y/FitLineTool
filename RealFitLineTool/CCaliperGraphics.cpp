#include "CCaliperGraphics.h"
#include "Generic.h"


void CCaliperGraphics::RansacLineFiler(const vector<Point2d>& points,vector<Point2d>&vpdExceptPoints, double sigma)
{
	unsigned int n = points.size();

	if (n < 2)
	{
		return;
	}

	RNG random;
	double bestScore = -1.;
	vector<Point2d>vpdTemp;
	int iterations = log(1 - 0.99) / (log(1 - (1.00 / n)))*10;
	
	for (int k = 0; k < iterations; k++)
	{
		int i1 = 0, i2 = 0;
		while (i1 == i2)
		{
			i1 = random(n);
			i2 = random(n);
		}
		const cv::Point2d& p1 = points[i1];
		const cv::Point2d& p2 = points[i2];
		Point2d vectorP21 = p2 - p1;
		vectorP21 *= 1. / norm(vectorP21);
		double score = 0;
		vpdTemp.clear();
		for (int i = 0; i < n; i++)
		{
			Point2d vectorPi1 = points[i] - p1;
			double d = vectorPi1.y * vectorP21.x - vectorPi1.x * vectorP21.y;//calculate the cos¦¨ of the two vectors.
			if (fabs(d) < sigma)
			{
				score += 1;
			}
			else
			{
				vpdTemp.push_back(points[i]);
			}
		}
		if (score > bestScore)
		{
			bestScore = score;
			vpdExceptPoints = vpdTemp;
		}
	}
}

CCaliperGraphics::CCaliperGraphics()
{
	m_pdStart					= Point2d(0, 0);
	m_pdEnd						= Point2d(0, 0);
	m_pdCenter					= Point2d(0, 0);
	m_dAngle					= 0;
	m_dLength					= 0;
	m_dMeasureLength			= 0;
	m_dSigma					= 1;
	m_nThreshold				= 30;
	m_nTranslation				= 1;
	m_nMeasureNums				= 1;
	m_vpdSampleLineEndPoints	= { Point2d(0,0),Point2d(0,0) };
	m_pdSampleLineStart			= { Point2d(-100,-100) };
	m_pdSampleLineEnd			= { Point2d(-100,-100) };
	m_nCircleSize				= 0;
	m_eAdjustType = AdjustType::None;
}

bool CCaliperGraphics::IsPointInCircle(Point2d pdCenter, Point2d pdPoint)
{
	return  GetPPDistance(pdCenter, pdPoint) <= m_nCircleSize;
}

/// <summary>
/// Create a caliper
/// </summary>
/// <param name="InputMat"></param>
/// <param name="pdStart">the expected line's start endpoint</param>
/// <param name="pdEnd">the expected line's end endpoint</param>
/// <param name="dMeasureLength">measure length</param>
/// <param name="dMeasureHeight">measure length</param>
/// <param name="dSigma"></param>
/// <param name="nThreshold"></param>
/// <param name="nTranslation">1: positive, from dark to light; 0: negative, from light to dark</param>
/// <param name="nMesureNums"></param>
void CCaliperGraphics::CreateCaliper(Mat& InputMat, Point2d pdStart, Point2d pdEnd, double dMeasureLength, double dMeasureHeight, double dSigma, int nThreshold, int nTranslation, int nMesureNums)
{
	if (InputMat.empty())
	{
		return;
	}
	InputMat.copyTo(m_mInputMat);
	m_pdStart			= pdStart;
	m_pdEnd				= pdEnd;
	m_pdCenter			= GetPPCenter(pdStart, pdEnd);
	m_dAngle			= GetAngleVecWithX(pdStart, pdEnd);
	m_dLength			= GetPPDistance(pdStart, pdEnd);
	m_dMeasureLength	= dMeasureLength;
	m_dMeasureAngle		= m_dAngle - 90;
	m_dMeasureHeight	= dMeasureHeight;
	m_dSigma			= dSigma;
	m_nThreshold		= nThreshold;
	m_nTranslation		= nTranslation;
	m_nMeasureNums		= nMesureNums;
	m_nCircleSize = InputMat.cols / 150;
	//Draw expected the line.
	DrawArrow(InputMat, pdStart, pdEnd, (int)m_nCircleSize * 5, Cyan, 1);
	circle(InputMat, pdStart, m_nCircleSize, Yellow, 1, 16);
	circle(InputMat, pdEnd, m_nCircleSize, Yellow, 1, 16);
	circle(InputMat, m_pdCenter, m_nCircleSize, Yellow, 1, 16);
	//Draws the sampling line direction
	GetEndPointsOfLine(m_pdCenter, m_dAngle - 90, dMeasureLength+100, m_pdSampleLineStart, m_pdSampleLineEnd);
	DrawArrow(InputMat, m_pdSampleLineStart, m_pdSampleLineEnd, (int)m_nCircleSize * 5, Cyan, 1);
	circle(InputMat, m_pdSampleLineEnd, m_nCircleSize, Yellow, 1, 16);
	//Draw the equinox lines
	GetEquinoxPointsOfLine(m_pdStart, m_pdEnd, nMesureNums + 1, m_vpdEquinoxPoints);
	RotatedRect rRect;
	Point2d p1, p2;
	for (int i = 0; i < m_vpdEquinoxPoints.size(); i++)
	{
		GetEndPointsOfLine(m_vpdEquinoxPoints[i], m_dAngle - 90, dMeasureLength, p1, p2);
		DrawArrow(InputMat, p1, p2, (int)m_nCircleSize * 5, Cyan, 1);
		rRect = RotatedRect(m_vpdEquinoxPoints[i], Size2f(dMeasureLength, dMeasureHeight), m_dAngle - 90);
		DrawRectangle(InputMat, rRect, Blue, 1);
	}
}

void CCaliperGraphics::AdjustCaliper(Mat& InputMat, Point2d pdPoint, int nMeasureLength, int nMeasureHeight, double dSigma, int nThreshold, int nTranslation, int nMeasureNums, int nFlag)
{
	if (nFlag == 1)//drag move
	{
		m_eAdjustType = AdjustType::None;
		CreateCaliper(InputMat, m_pdStart, m_pdEnd, m_dMeasureLength, m_dMeasureHeight, dSigma, nThreshold, nTranslation, m_nMeasureNums);
		return;
	}
	if (IsPointInCircle(m_pdStart, pdPoint))
	{
		m_eAdjustType = AdjustType::AdjustStart;
	}
	else if (IsPointInCircle(m_pdEnd, pdPoint))
	{
		m_eAdjustType = AdjustType::AdjustEnd;
	}
	else if (IsPointInCircle(m_pdCenter, pdPoint))
	{
		m_eAdjustType = AdjustType::AdjustCenter;
	}
	else if (IsPointInCircle(m_pdSampleLineEnd, pdPoint))
	{
		m_eAdjustType = AdjustType::AdjustMeasureLength;
	}
	else if (pdPoint.x == -1 && pdPoint.y == -1)
	{
		CreateCaliper(InputMat, m_pdStart, m_pdEnd, m_dMeasureLength, m_dMeasureHeight, dSigma, nThreshold, nTranslation, nMeasureNums);
	}
	else if (pdPoint.x == -2 && pdPoint.y == -2)
	{
		CreateCaliper(InputMat, m_pdStart, m_pdEnd, nMeasureLength, m_dMeasureHeight, dSigma, nThreshold, nTranslation, m_nMeasureNums);
	}
	else if (pdPoint.x == -3 && pdPoint.y == -3)
	{
		CreateCaliper(InputMat, m_pdStart, m_pdEnd, m_dMeasureLength, nMeasureHeight, dSigma, nThreshold, nTranslation, m_nMeasureNums);
	}
	else
	{
		CreateCaliper(InputMat, m_pdStart, m_pdEnd, m_dMeasureLength, m_dMeasureHeight, dSigma, nThreshold, nTranslation, m_nMeasureNums);
	}

	//Adjust by UI
	if (m_eAdjustType == AdjustType::AdjustStart)
	{
		CreateCaliper(InputMat, pdPoint, m_pdEnd, m_dMeasureLength, m_dMeasureHeight, dSigma, nThreshold, nTranslation, m_nMeasureNums);
	}
	else if (m_eAdjustType == AdjustType::AdjustEnd)
	{
		CreateCaliper(InputMat, m_pdStart, pdPoint,m_dMeasureLength, m_dMeasureHeight, dSigma, nThreshold, nTranslation, m_nMeasureNums);
	}
	else if (m_eAdjustType == AdjustType::AdjustCenter)
	{		
		GetEndPointsOfLine(pdPoint, m_dAngle, m_dLength, m_pdStart, m_pdEnd);
		CreateCaliper(InputMat, m_pdStart, m_pdEnd, m_dMeasureLength, m_dMeasureHeight, dSigma, nThreshold, nTranslation, m_nMeasureNums);
	}
	else if (m_eAdjustType == AdjustType::AdjustMeasureLength)
	{
		double dMeasureLength = GetPPDistance(m_pdCenter, pdPoint) * 2 - 100;
		//make sure that the measure length would not be too short.
		if (dMeasureLength < 1)
		{
			dMeasureLength = m_dMeasureLength;
		}
		CreateCaliper(InputMat, m_pdStart, m_pdEnd, dMeasureLength, m_dMeasureHeight, dSigma, nThreshold, nTranslation, m_nMeasureNums);
	}
}

void CCaliperGraphics::FindLine(Point2d& pdStart, Point2d& pdEnd, double& dAngle)
{
	//Get ROI mat.
	RotatedRect rMaskRegion(m_pdCenter, Size2f(GetPPDistance(m_pdStart, m_pdEnd) + 10, m_dMeasureLength + 10), m_dAngle);
	Point2f rRegionPoints[4];
	rMaskRegion.points(rRegionPoints);
	Mat mask = Mat::zeros(m_mInputMat.size(), CV_8UC1);
	Point ppt[] = { rRegionPoints[0], rRegionPoints[1], rRegionPoints[2], rRegionPoints[3] };
	const Point* pts[] = { ppt };
	int npt[] = { 4 };
	fillPoly(mask, pts, npt, 1, Scalar::all(255), 8);
	Mat RoiMat;
	bitwise_and(m_mInputMat, m_mInputMat, RoiMat, mask);

	//Extract 1D edge points
	m_vpdEdgePoints.clear();
	m_vdEdgeGradient.clear();
	m_vpdExcepetEdgePoints.clear();
	for (int i = 0; i < m_vpdEquinoxPoints.size(); i++)
	{
		vector<Edge1D_Result>edges = extract1DEdge.Get1DEdge(RoiMat, m_vpdEquinoxPoints[i], m_dMeasureLength, m_dMeasureHeight, m_dAngle - 90
			, m_dSigma, m_nThreshold, m_nTranslation == 1 ? Translation::Poisitive : Translation::Negative, Selection::Strongest);
		for (int i = 0; i < edges.size(); i++)
		{
			m_vpdEdgePoints.push_back(edges[i].m_pdEdgePoint);
			m_vdEdgeGradient.push_back(edges[i].m_dGradient);
		}
	}
	Vec4f lines;
	int nSize = m_vpdEdgePoints.size() - 1;
	pdStart = Point2d(-1, -1);
	pdEnd = Point2d(-1, -1);
	if (nSize >= 0)
	{
		RansacLineFiler(m_vpdEdgePoints, m_vpdExcepetEdgePoints);
		for (Point2d point : m_vpdExcepetEdgePoints)//remove the excepted points.
		{
			for (int i = 0; i < nSize + 1; i++)
			{
				if (point == m_vpdEdgePoints[i])
				{
					m_vpdEdgePoints.erase(m_vpdEdgePoints.begin() + i);
					break;
				}
			}
		}
		nSize = m_vpdEdgePoints.size() - 1;
		//use the edge points to fit a line.
		fitLine(m_vpdEdgePoints, lines, DIST_HUBER, 0, 0.01, 0.01);
		double dK	= lines[1] / lines[0];
		double dB	= lines[3] - dK * lines[2];


		Point2d pdfitLineP1 = Point2d(m_vpdEdgePoints[0].x, dK * (m_vpdEdgePoints[0].x - lines[2]) + lines[3]);
		Point2d pdfitLineP2 = Point2d(m_vpdEdgePoints[nSize].x, dK * (m_vpdEdgePoints[nSize].x - lines[2]) + lines[3]);
		Point2d pdP11(0, 0), pdP22(0, 0);

		//calculate the intersection of fitting line and the line which passes through the edge points in the direction of the measurement angle.
		GetEndPointsOfLine(m_vpdEdgePoints[0], m_dAngle - 90, m_dMeasureLength, pdP11, pdP22);
		pdStart = GetIntersectionOfLines(pdfitLineP1, pdfitLineP2, pdP11, pdP22);

		GetEndPointsOfLine(m_vpdEdgePoints[nSize], m_dAngle - 90, m_dMeasureLength, pdP11, pdP22);
		pdEnd = GetIntersectionOfLines(pdfitLineP1, pdfitLineP2, pdP11, pdP22);

		dAngle = GetAngleVecWithX(pdStart, pdEnd);
	}


}

void CCaliperGraphics::DisplayEdgePoints(Mat& InputMat, double dSize, Scalar color)
{
	if (InputMat.empty())
	{
		return;
	}
	for (auto edge : m_vpdEdgePoints)
	{
		DrawCross(InputMat, edge, 90, dSize, color, 1);
	}
	for (auto edge : m_vpdExcepetEdgePoints)
	{
		DrawCross(InputMat, edge, 90, dSize, Red, 1);
	}
}

void CCaliperGraphics::GetEdgeInfo(vector<Point2d>& vpdEdgePoints, vector<double>& vpdEdgePointsGradient)
{
	vpdEdgePoints = m_vpdEdgePoints;
	vpdEdgePointsGradient = m_vdEdgeGradient;
}


