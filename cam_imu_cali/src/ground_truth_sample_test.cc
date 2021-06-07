/* find chessboard corners and calculate the Tcb(from board to camera)*/

#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
//#include "extended_kalman_filter.h" //only for test
//#include <opencv2/features2d.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;
#define DEBUG

int main(int argc, char **argv)
{
    //step0: read data
    ifstream fin3f("../data/points/3D-jmw.txt");
    ifstream fin2f("../data/points/2D-jmw.txt");
    if (!fin3f || !fin2f)
    {
        cerr << "Points data is not found!" << endl;
        return 1;
    }

    vector<Point3f> cornerPoints3d;
    vector<Point2f> cornerPoints2d;
    //vector<Vector3d> vwm, vam;
    while (!fin3f.eof() && !fin2f.eof())
    {
        Point3f point_3f;
        Point2f point_2f;
        fin3f >> point_3f.x;
        fin3f >> point_3f.y;
        fin3f >> point_3f.z;
        fin2f >> point_2f.x;
        fin2f >> point_2f.y;

        cornerPoints3d.push_back(point_3f);
        cornerPoints2d.push_back(point_2f);
        cout << point_3f << endl;
        cout << point_2f << endl;
    }

    //step1:set distortion parameters and camera intrinsics
    double k1 = 0.1397, k2 = -0.3424, k3 = 0.2832;
    double p1 = 0.0017, p2 = -0.00020309;
    double fx = 1389.8, fy = 1389.8;
    double cx = 968.196716, cy = 540.533875;
    Mat distCoeffs = (Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
    Mat cameraIntrincis = (Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);

    //step4: calc Tcb <-solve pnp

    Mat rvec, tvec;

    solvePnP(cornerPoints3d, cornerPoints2d, cameraIntrincis, distCoeffs, rvec, tvec);
    cout << rvec << endl;
    cout << tvec << endl;

    Mat Rotation;
    Rodrigues(rvec, Rotation);
    cout << Rotation << endl;
    //step5: draw corners
    //corners detected based on raw image
    //drawChessboardCorners(imgCornerDetect, patternSize, cornerPoints2d, patternFound); //must be color image, often a clone of img_chessboard
    //corners reprojected based on raw image + estimation of extrinsics + intrincisc
    vector<Point2f> cornerReprojPoints;
    projectPoints(cornerPoints3d, rvec, tvec, cameraIntrincis, distCoeffs, cornerReprojPoints);
    //drawChessboardCorners(imgCornerDetect, patternSize, cornerReprojPoints, patternFound); //Less reprojected error is, better imshow matches.

    //step6: calc error

    double sum = 0.0;
    double mean = 0.0;
    for (uint i = 0; i < cornerPoints2d.size(); i++)
    {
        sum += pow((cornerPoints2d[i].x - cornerReprojPoints[i].x), 2) + pow((cornerPoints2d[i].y - cornerReprojPoints[i].y), 2);
        cout << cornerPoints2d[i] << " compare " << cornerReprojPoints[i] << endl;
    }
    mean = sqrt(sum / cornerPoints2d.size());
    cout << mean << endl;
    /*
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
*/
}