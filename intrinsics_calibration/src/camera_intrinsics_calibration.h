#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace CamIntrinsicsCali {
class FunctionSelector {
public:
  bool undistort_image_save_enable_ =
      false; // 如果需要获得去畸变后的图片，则使能此开关
  bool result_visual_enable_ =
      false; // 预留用于显示debug过程的开关，代码里暂未使用
};

// ImageProcessor: 读取图像，保存降采样后的图像
class ImageProcessor {
public:
  explicit ImageProcessor();
  ~ImageProcessor() = default;
  void ImagesReaderFromPathOrFile(const cv::String &input_path_or_file,
                                  std::vector<cv::Mat> &output_images);

  void ImagePyrDownRewriter(
      const std::vector<cv::Mat> &input_images, const double &scale,
      const std::string &
          output_file_path); // 对输入图像降采样到object_image_size，并保存成新的图像
};

// CameraIntrinsicsCalibrator: 内参标定，去畸变
class CameraIntrinsicsCalibrator {
public:
  explicit CameraIntrinsicsCalibrator();
  ~CameraIntrinsicsCalibrator() = default;
  cv::Mat cameraMatrix_;       // [fx,0,cx,0,fy,cy,0,0,1]
  cv::Mat distCoeffs_;         // [k1,k2,p1,p2,k3]
  std::vector<cv::Mat> rvecs_; // rodrigues vectors
  std::vector<cv::Mat> tvecs_; // translation vectors
  cv::Mat perViewErrors_;
  double err_rms_;

  // 输入各个view下的corners3d-2d坐标，迭代计算内参及每个view下的camera位姿,重投影误差
  bool CalibrateCameraIntrinsics(
      const std::vector<std::vector<cv::Point3f>> &input_world_corner_points,
      const std::vector<std::vector<cv::Point2f>> &input_image_corner_points);

  void CameraIntrinscisOutput(const std::string &output_file_path);

  void ImageUndistort(const cv::Mat &input_distort_image,
                      cv::Mat &output_undistort_image);

  // 根据读取的第一张图片像素调整imagesize
  void setImageSize(const int &width, const int &length);

private:
  cv::TermCriteria termCriteria_ =
      cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 0.01);
  cv::Size imageSize_;
};

// ChessboardCornersExtractor：角点提取并显示，提取到亚像素级别
class ChessboardCornersExtractor {
public:
  explicit ChessboardCornersExtractor(); // sets patternSize_, termCriteria_,
                                         // etc.
  ~ChessboardCornersExtractor() = default;

  // finds the chessboard's corners in a subpixel level.
  // The input image should be gray image.
  bool
  FindChessboardCornersSubPix(const cv::Mat &input_chessboard_image,
                              std::vector<cv::Point2f> &output_corner_points);

  void ChessboardCornersPositionCalculator(
      std::vector<cv::Point3f> &output_corner_points);

private:
  // Chessboard corners size and the extractor configuration.
  cv::Size patternSize_;
  cv::Size winSize_ = cv::Size(11, 11);
  cv::Size zeroZone_ = cv::Size(-1, -1);
  cv::TermCriteria termCriteria_ =
      cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 0.01);
  double size_ = 0.045;
  int patternRows_ = 8;
  int patternCols_ = 11;
};

} // namespace CamIntrinsicsCali