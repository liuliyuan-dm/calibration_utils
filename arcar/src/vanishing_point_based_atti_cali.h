#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace ArCar {

class ImageProcessor {
public:
  explicit ImageProcessor();
  ~ImageProcessor() = default;
  void ImagesReaderFromPathOrFile(
      const cv::String &
          input_path_or_file, //读取指定路径下的所有文件，或者某一个文件，文件格式为.jpg/.png
      std::vector<cv::Mat> &output_images);
};

class VanishingPointBasedAttitudeCalculator {
public:
  explicit VanishingPointBasedAttitudeCalculator();
  ~VanishingPointBasedAttitudeCalculator() = default;
  void setCameraMatrix(cv::Mat &input_camera_matrix);
  void setVanishingPoint(cv::Point2f &input_vanishing_point);
  void setCameraHeight(double &input_camera_height);

  // 手动标记灭点像素坐标，作为函数输入，根据已知内参解算外参Pitch\yaw角度（unit:rad）
  void CalculateAttitudeBasedVanishingPoint(
      cv::Point2f &output_pitch_yaw_attitude_in_rad);

  // 根据解算得到的外参角度，进行鸟瞰视角投影，输出结果中车道线若为平行线，则代表计算准确
  void BirdViewReproject(cv::Mat &input_undistort_image,
                         const cv::Point2f &input_pitch_yaw_attitude_in_rad,
                         cv::Mat &output_bird_view_reproject_image);

private:
  cv::Mat camera_matrix_;       // camera_matrix_ = [fx,0,cx;0,fy,cy;0,0,1]
  cv::Point2f vanishing_point_; // vanishing_point_ = [u0,v0]
  double camera_height_;
};

} // namespace ArCar