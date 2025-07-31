#include<opencv2/opencv.hpp>
using namespace cv;

#define PI 3.14159265358979323846
#define to_degree(x) x*180/PI
#define to_radian(x) x*PI/180
#define Green   Scalar(0,255,0)
#define Red     Scalar(0,0,255)
#define Blue    Scalar(255,0,0)
#define Cyan    Scalar(255,255,0)
#define Yellow  Scalar(0,255,255) 

static void DrawArrow(Mat& inputMat, Point2d p1, Point2d p2, int dSize, Scalar color, int nThickness = 1)
{
    if (inputMat.empty())
    {
        return;
    }
    double dK = ((double)p2.y - (double)p1.y) / ((double)p2.x - (double)p1.x);
    double dAngle = atan(dK) * 180 / PI;
    line(inputMat, p1, p2, color, nThickness, LINE_AA);
    RotatedRect rotateRect(p2, Size(dSize, dSize * 0.5), dAngle);
    Point2f rectPoints[4];
    rotateRect.points(rectPoints);
    if ((dAngle >= 0 && p1.x <= p2.x) || (dAngle < 0 && p1.x <= p2.x))
    {
        line(inputMat, p2, rectPoints[0], color, nThickness, LINE_AA);
        line(inputMat, p2, rectPoints[1], color, nThickness, LINE_AA);
    }
    else
    {
        line(inputMat, p2, rectPoints[2], color, nThickness, LINE_AA);
        line(inputMat, p2, rectPoints[3], color, nThickness, LINE_AA);
    }
}

static void DrawCross(Mat& inputMat, Point2d p, double dAngle, double dSize, Scalar color,int nThickness = 1)
{
    if (inputMat.empty())
    {
        return;
    }
    RotatedRect rotateRect(p, Size2d(dSize, dSize), dAngle);
    Point2f rectPoints[4];
    rotateRect.points(rectPoints);
    line(inputMat, rectPoints[0], rectPoints[2], color, nThickness, LINE_AA);
    line(inputMat, rectPoints[1], rectPoints[3], color, nThickness, LINE_AA);
}

static void DrawRectangle(Mat& inputMat, RotatedRect rRect,Scalar color, int nThickness = 1)
{
    Point2f pfRectPoints[4];
    rRect.points(pfRectPoints);
    for (int i = 0; i < 4; i++)
    {
        line(inputMat, pfRectPoints[i], pfRectPoints[(i + 1) % 4], color, nThickness, LINE_AA);
    }
}

static double GetPPDistance(Point2d p1, Point2d p2)
{
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}

static Point2d GetPPCenter(Point2d p1, Point2d p2)
{
    return Point2d((p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5);
}

static double GetLineSlope(Point2d p1, Point2d p2)
{
    return (p1.y - p2.y) / (p1.x - p2.x);
}

static void GetEquinoxPointsOfLine(Point2d pdStart, Point2d pdEnd, int nEquinoxNums,vector<Point2d>& vpdEquinoxPoints)
{
    if (nEquinoxNums == 0)
    {
        return;
    }
    vpdEquinoxPoints.clear();
    double dXoffset = (pdEnd.x - pdStart.x) / nEquinoxNums;
    double dYoffset = (pdEnd.y - pdStart.y) / nEquinoxNums;
    for (int i = 0; i < nEquinoxNums - 1; i++)
    {
        double dx = dXoffset * ((double)i + 1);
        double dy = dYoffset * ((double)i + 1);
        vpdEquinoxPoints.push_back(Point2d(pdStart.x + dx, pdStart.y + dy));
    }
}

/// <summary>
/// Get the endpoints of a line via the line's center point, length and angle with x-axis
/// </summary>
/// <param name="pdCenter">line's center</param>
/// <param name="dAngle">the angle between the line and the X axis. The unit is the degree</param>
/// <param name="dLength">line's length</param>
/// <param name="vpdEndPoints">line's endpoints, 1st element is the start end point.</param>
static void GetEndPointsOfLine(Point2d pdCenter, double dAngle, double dLength, Point2d& pdStart, Point2d& pdEnd)
{
    pdStart.x = pdCenter.x - cos(to_radian(dAngle)) * 0.5 * dLength;
    pdStart.y = pdCenter.y - sin(to_radian(dAngle)) * 0.5 * dLength;
    pdEnd = Point2d(pdCenter.x * 2 - pdStart.x, pdCenter.y * 2 - pdStart.y);
}

/// <summary>
///  Calculate the angle between a vector with x-axis. Formula: cos¦¨=vector.x^2/(¡Ì(vector.x^2+vector.y^2)+vector.x)
/// </summary>
/// <param name="p1">the start point of the vector</param>
/// <param name="p2">the end point of the vector</param>
/// <returns>degree</returns>
static double GetAngleVecWithX(Point2d p1,Point2d p2)
{
    if (p1 == p2)
    {
        return -1;
    }
    Point2d vector = p2 - p1;
    if (vector.x == 0)
    {
        if (vector.y > 0)
        {
            return 90;
        }
        else
        {
            return -90;
        }
    }
    double angle = to_degree(acos(pow(vector.x, 2) / (vector.x * sqrt(pow(vector.x, 2) + pow(vector.y, 2)))));
    if (p1.y > p2.y)
    {
        angle = -angle;
    }


    return  angle;
}

static double GetPointToLineDistance(Point2d p1, double dK, double dB)
{
    return abs(dK * p1.x - p1.y + dB) / sqrt(1 + dK * dK);
}

static Point2d GetIntersectionOfLines(Point2d p1,Point2d p2, Point2d p11, Point2d p22)
{
    double dK1 = (p1.y - p2.y) / (p1.x - p2.x);
    double dK2 = (p11.y - p22.y) / (p11.x - p22.x);

    if (dK1 == dK2)//parallel
    {
        return Point2d(-1, -1);
    }
    Point2d pdIntersection(0, 0);
    if (isinf(dK1) || isnan(dK1))//paralle to the y-axis
    {
        pdIntersection.x = p1.x;
        double dB2 = p11.y - dK2 * p11.x;
        pdIntersection.y = dK2 * pdIntersection.x + dB2;
    }
    else if (isinf(dK2) || isnan(dK2))
    {
        pdIntersection.x = p11.x;
        double dB1 = p1.y - dK1 * p1.x;
        pdIntersection.y = dK1 * pdIntersection.x + dB1;
    }
    else
    {
        double dB1 = p1.y - dK1 * p1.x;
        double dB2 = p11.y - dK2 * p11.x;
        pdIntersection.x = (dB2 - dB1) / (dK1 - dK2);
        pdIntersection.y = dK1 * pdIntersection.x + dB1;
    }
    return pdIntersection;
}