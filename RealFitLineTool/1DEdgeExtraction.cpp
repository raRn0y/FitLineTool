// 1DEdgeExtraction.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include "stdlib.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "CCaliperGraphics.h"
#include "Generic.h"

using namespace std;
using namespace cv;


string WindowHandle = to_string(0);
string WindowHandle2 = to_string(1);

Mat srcImage;
Mat dstImage;
int nMeasureNums = 10;
int nMeasureHeight = 10;
int nMeasureLength = 30;
int nSigma = 1;
int nThreshold = 30;
int nTranslation = 1;
int nFindLine = 0;
const int nMaxMeasureNums = 99;
//double dK = 0;
//double dB = 0;
//double dLength = 0;
//double dHeight = 100;
//double dAngle = 0;
//Point2d centerPoint(0, 0);
//RotatedRect rotateRect;
//Point2f RectPoints[4];
//Extract1DEdge extract1DEdge;
CCaliperGraphics caliperGraphics;

void InitializeWindow(Mat& matImage, string szWindowHandle, int nWindowRatio = 1)
{
    if (matImage.empty() == true || nWindowRatio == 0)
    {
        return;
    }
    int nImageWidth = matImage.size().width;
    int nImageHeight = matImage.size().height;
    namedWindow(szWindowHandle, WINDOW_KEEPRATIO);
    resizeWindow(szWindowHandle, nImageWidth / nWindowRatio, nImageHeight / nWindowRatio);
}

void On_Mouse(int event, int x, int y, int flags, void* ustc)
{
    if (event == EVENT_MOUSEMOVE && (flags & EVENT_FLAG_LBUTTON))
    {
        dstImage.copyTo(srcImage);
        caliperGraphics.AdjustCaliper(srcImage, Point2d(x, y), nMeasureLength, nMeasureHeight, nSigma, nThreshold, nTranslation, nMeasureNums);
        imshow(WindowHandle, srcImage);
    }
    else if (event == EVENT_LBUTTONUP)
    {
        dstImage.copyTo(srcImage);
        caliperGraphics.AdjustCaliper(srcImage, Point2d(x, y), nMeasureLength, nMeasureHeight, nSigma, nThreshold, nTranslation,nMeasureNums, 1);
        imshow(WindowHandle, srcImage);
    }
}

void On_AdjustMeasureNums(int, void*)
{
    dstImage.copyTo(srcImage);
    caliperGraphics.AdjustCaliper(srcImage, Point2d(-1, -1), nMeasureLength, nMeasureHeight, nSigma, nThreshold, nTranslation,nMeasureNums);
    imshow(WindowHandle, srcImage);
}

void On_AdjustMeasureHeight(int, void*)
{
    dstImage.copyTo(srcImage);
    caliperGraphics.AdjustCaliper(srcImage, Point2d(-3, -3), nMeasureLength, nMeasureHeight, nSigma, nThreshold, nTranslation,nMeasureNums);
    imshow(WindowHandle, srcImage);
}

void On_AdjustMeasureLength(int, void*)
{
    dstImage.copyTo(srcImage);
    caliperGraphics.AdjustCaliper(srcImage, Point2d(-2, -2), nMeasureLength, nMeasureHeight, nSigma, nThreshold, nTranslation,nMeasureNums);
    imshow(WindowHandle, srcImage);
}

void On_SetTranslation(int, void*)
{
    dstImage.copyTo(srcImage);
    caliperGraphics.AdjustCaliper(srcImage, Point2d(), nMeasureLength, nMeasureHeight, nSigma, nThreshold, nTranslation, nMeasureNums, 1);
}

void On_SetSigma(int, void*)
{
    dstImage.copyTo(srcImage);
    caliperGraphics.AdjustCaliper(srcImage, Point2d(), nMeasureLength, nMeasureHeight, nSigma, nThreshold, nTranslation, nMeasureNums, 1);
}

void On_SetThreshold(int, void*)
{
    dstImage.copyTo(srcImage);
    caliperGraphics.AdjustCaliper(srcImage, Point2d(), nMeasureLength, nMeasureHeight, nSigma, nThreshold, nTranslation, nMeasureNums, 1);
}

void On_FindLine(int, void*)
{
    if (nFindLine == 1)
    {
        Point2d pdLineStart(0, 0), pdLineEnd(0, 0);
        double dAngle = 0;
        caliperGraphics.FindLine(pdLineStart, pdLineEnd, dAngle);
        dstImage.copyTo(srcImage);
        line(srcImage, pdLineStart, pdLineEnd, Green, 1, 16);
        //Draw edge points
        caliperGraphics.DisplayEdgePoints(srcImage, 10, Green);
        imshow(WindowHandle, srcImage);

        //print all of edge points info.
        vector<Point2d>vpdEdgePoints;
        vector<double>vdEdgePointsGradient;
        caliperGraphics.GetEdgeInfo(vpdEdgePoints, vdEdgePointsGradient);
        cout << "Line infomation: " << pdLineStart << ", " << pdLineEnd << ", " << dAngle << endl;
        for (int i = 0; i < vpdEdgePoints.size(); i++)
        {
            cout << "Edge Point[" << i << "]: " << vpdEdgePoints[i] << ", gradient: " << vdEdgePointsGradient[i] << endl;
        }
        setTrackbarPos("FindLine", "Track", 0);
    }

}

void InitializeMeasureTrackbar()
{
    namedWindow("Track", WINDOW_KEEPRATIO);
    resizeWindow("Track", Size(500, 250));

    createTrackbar("Nums", "Track", &nMeasureNums, nMaxMeasureNums, On_AdjustMeasureNums);
    setTrackbarMin("Nums", "Track", 1);

    createTrackbar("Height", "Track", &nMeasureHeight, 500, On_AdjustMeasureHeight);
    setTrackbarMin("Height", "Track", 1);

    createTrackbar("Length", "Track", &nMeasureLength, 500, On_AdjustMeasureLength);
    setTrackbarMin("Length", "Track", 1);

    createTrackbar("Sigma", "Track", &nSigma, 1, On_SetSigma);
    setTrackbarMin("Sigma", "Track", 1);
    setTrackbarMax("Sigma", "Track", 99);

    createTrackbar("Threshold", "Track", &nThreshold, 1, On_SetThreshold);
    setTrackbarMin("Threshold", "Track", 1);
    setTrackbarMax("Threshold", "Track", 255);

    createTrackbar("Translation", "Track", &nTranslation, 1, On_SetTranslation);
    setTrackbarMax("Translation", "Track", 1);

    createTrackbar("FindLine", "Track", &nFindLine, 1, On_FindLine);
}

int main()
{
    //srcImage = imread(".\\image\\halogen_bulb_01.png");
    //srcImage = imread(".\\image\\clamp_sloped_01.png");
    //srcImage = imread(".\\image\\dip_switch_02.png");
    //srcImage = imread(".\\image\\fuse.png");
    //srcImage = imread(".\\image\\metal-part-distorted-02.png");19300536.bmp
    srcImage = imread("/Users/ezio/Documents/RealFitLineTool/1.png");
    if (srcImage.empty())
    {
        cout << "fail to load the image, please check the image's path whether is correct." << endl;
        system("pause");
        return 0;
    }
    srcImage.copyTo(dstImage);
    if (srcImage.cols >= 1500 || srcImage.rows >= 1500)
    {
        InitializeWindow(srcImage, WindowHandle, 4);
    }
    else
    {
        InitializeWindow(srcImage, WindowHandle, 1);
    }
    setMouseCallback(WindowHandle, On_Mouse, 0);
    caliperGraphics.CreateCaliper(srcImage, Point2d(600, 400), Point2d(600, 200), 100, 25, nSigma, nThreshold, nTranslation, nMeasureNums);
    InitializeMeasureTrackbar();
   

    imshow(WindowHandle, srcImage);
    waitKey(0);

}


