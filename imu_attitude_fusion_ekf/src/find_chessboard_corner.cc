/* find chessboard corners and calculate the Tcb(from board to camera)*/

#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
//#include "extended_kalman_filter.h" //only for test
//#include <opencv2/features2d.hpp>

using namespace std;
using namespace cv;
#define DEBUG

int main(int argc, char **argv)
{
    //step0: load image
    Mat imgChessboardRaw, imgCornerDetect;
    imgChessboardRaw = imread(argv[1], IMREAD_GRAYSCALE);
    imgCornerDetect = imread(argv[1]);

    if (imgChessboardRaw.data == nullptr)
    {
        cerr << "please loading an image!" << endl;
        return -1;
    }
    else
    {
        cout << "images has been loaded successfully!" << endl;
    }

    //step1:set distortion parameters and camera intrinsics
    double k1 = 0.0413, k2 = -0.0608, k3 = 0.0052;
    double p1 = -6.0677e-04, p2 = 9.7486e-05;
    double fx = 1.3278e+03, fy = 1.3275e+03;
    double cx = 1.0043e+03, cy = 466.6627;
    Mat distCoeffs = (Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
    Mat cameraIntrincis = (Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);

    //step2: find chessboard corners
    uint8_t patternRows = 8, patternCols = 11;
    Size patternSize(patternRows, patternCols), winSize(11, 11), zeroZone(-1, -1);
    TermCriteria termCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.1);
    vector<Point2f> cornerPoints2d;
    bool patternFound = findChessboardCorners(imgChessboardRaw, patternSize, cornerPoints2d, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

    //step3: calc subpixel corners
    if (patternFound)
    {
        cornerSubPix(imgChessboardRaw, cornerPoints2d, winSize, zeroZone, termCriteria); //must be gray image
    }

    //step4: calc Tcb <-solve pnp
    double chessGridScale = 0.045; //m
    vector<Point3f> cornerPoints3d;
    for (uint i = 0; i < patternCols; i++)
    {
        for (uint j = 0; j < patternRows; j++)
        {
            Point3f cornerPoint3d;
            cornerPoint3d.x = i * chessGridScale;
            cornerPoint3d.y = (patternRows - j - 1) * chessGridScale;
            cornerPoint3d.z = 0.0;
            cornerPoints3d.push_back(cornerPoint3d);
        }
    }
    Mat rvec, tvec;

    solvePnP(cornerPoints3d, cornerPoints2d, cameraIntrincis, distCoeffs, rvec, tvec);

    //step5: draw corners
    //corners detected based on raw image
    drawChessboardCorners(imgCornerDetect, patternSize, cornerPoints2d, patternFound); //must be color image, often a clone of img_chessboard
    //corners reprojected based on raw image + estimation of extrinsics + intrincisc
    vector<Point2f> cornerReprojPoints;
    projectPoints(cornerPoints3d, rvec, tvec, cameraIntrincis, distCoeffs, cornerReprojPoints);
    drawChessboardCorners(imgCornerDetect, patternSize, cornerReprojPoints, patternFound); //Less reprojected error is, better imshow matches.

    //step6: calc error
    double sum = 0.0;
    double mean = 0.0;
    for (uint i = 0; i < cornerPoints2d.size(); i++)
    {
        sum += pow((cornerPoints2d[i].x - cornerReprojPoints[i].x), 2) + pow((cornerPoints2d[i].y - cornerReprojPoints[i].y), 2);
        cout << cornerPoints2d[i] << " compare " << cornerReprojPoints[i] << endl;
    }
    mean = sqrt(sum / cornerPoints2d.size());

#ifdef DEBUG
    imshow("chessboard raw image", imgChessboardRaw);
    waitKey(0);
    imshow("chessboard corners2", imgCornerDetect);
    waitKey(0);
    cout << "cornerPoints2d: " << cornerPoints2d << endl;
    cout << "cornerPoints3d: " << cornerPoints3d << endl;
    cout << "cornerReprojPoints: " << cornerReprojPoints << endl;
    cout << "rvec is: " << rvec << endl;
    cout << "tvec is: " << tvec << endl;
    cout << "mean error is: " << mean << endl;
#endif
}