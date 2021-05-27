#include "camera_intrinsics_calibration.h"

namespace CamIntrinsicsCali {
ImageProcessor::ImageProcessor() {}

void ImageProcessor::ImagesReaderFromPathOrFile(
    const cv::String &input_path_or_file, std::vector<cv::Mat> &output_images) {
  std::vector<cv::String> file_names;
  cv::glob(input_path_or_file, file_names);
  for (int i = 0; i < file_names.size(); i++) {
    cv::Mat image;
    image = cv::imread(file_names[i]);
    if (image.data) {
      output_images.emplace_back(std::move(image));
      std::cout << "INFO: load succeed!" << std::endl;
    } else {
      continue;
    }
  }
}

void ImageProcessor::ImagePyrDownRewriter(
    const std::vector<cv::Mat> &input_images, const double &scale,
    const std::string &output_file_path) {
  cv::Size object_image_size =
      cv::Size(input_images[0].cols / scale,
               input_images[0].rows / scale); // scale是想把图片缩小的倍数
  for (int i = 0; i < input_images.size(); i++) {
    cv::Mat object_image;
    cv::pyrDown(input_images[i], object_image, object_image_size);
    cv::imwrite(output_file_path + std::to_string(i) + ".jpg", object_image);
    std::cout << "INFO: downsample and rewrite succeed!" << std::endl;
  }
}

CameraIntrinsicsCalibrator::CameraIntrinsicsCalibrator() {}

bool CameraIntrinsicsCalibrator::CalibrateCameraIntrinsics(
    const std::vector<std::vector<cv::Point3f>> &input_world_corner_points,
    const std::vector<std::vector<cv::Point2f>> &input_image_corner_points) {
  cv::Mat stdDeviationsIntrinsics;
  cv::Mat stdDeviationsExtrinsics;
  err_rms_ = cv::calibrateCamera(
      input_world_corner_points, input_image_corner_points, imageSize_,
      cameraMatrix_, distCoeffs_, rvecs_, tvecs_, stdDeviationsIntrinsics,
      stdDeviationsExtrinsics, perViewErrors_, 0, termCriteria_);
  std::cout << "camera matrix is: " << cameraMatrix_ << std::endl;
  std::cout << "distortion coeffs is: " << distCoeffs_ << std::endl;
  std::cout << "RMS is: " << err_rms_ << std::endl;
  std::cout << "err for each image is: " << perViewErrors_ << std::endl;
}

void CameraIntrinsicsCalibrator::CameraIntrinscisOutput(
    const std::string &output_file_path) {
  std::fstream output_txt;
  output_txt.open(output_file_path + "/camera_**.intrinsics.proto.txt",
                  std::ios::out);
  output_txt
      << "fx: " << cameraMatrix_.at<double>(0, 0) << "\n"
      << "fy: " << cameraMatrix_.at<double>(1, 1) << "\n"
      << "cx: " << cameraMatrix_.at<double>(0, 2) << "\n"
      << "cy: " << cameraMatrix_.at<double>(1, 2) << "\n"
      << "distortion_coef: " << distCoeffs_.at<double>(0) << "\n"
      << "distortion_coef: " << distCoeffs_.at<double>(1) << "\n"
      << "distortion_coef: " << distCoeffs_.at<double>(2) << "\n"
      << "distortion_coef: " << distCoeffs_.at<double>(3) << "\n"
      << "distortion_coef: " << distCoeffs_.at<double>(4) << "\n"
      << "distortion_coef: "
      << "0"
      << "\n"
      << "distortion_coef: "
      << "0"
      << "\n"
      << "distortion_coef: "
      << "0"; // TODO:
              // 因为标定的时候直接用了五参数模型，所以这里只是依照八参数模型的模板，把剩下的三个参数填充了0，
              // 五个参数依次是k1,k2,p1,p2,k3，opencv里distortion_coef的默认保存顺序也是这样的
  output_txt.close();
}

void CameraIntrinsicsCalibrator::ImageUndistort(
    const cv::Mat &input_distort_image, cv::Mat &output_undistort_image) {
  cv::undistort(input_distort_image, output_undistort_image, cameraMatrix_,
                distCoeffs_);
  std::cout << "INFO: undistort succeed!" << std::endl;
}

void CameraIntrinsicsCalibrator::setImageSize(const int &width,
                                              const int &length) {
  imageSize_ = cv::Size(width, length);
}

ChessboardCornersExtractor::ChessboardCornersExtractor() {
  patternSize_ = cv::Size(patternRows_, patternCols_);
}

bool ChessboardCornersExtractor::FindChessboardCornersSubPix(
    const cv::Mat &input_chessboard_image,
    std::vector<cv::Point2f> &output_corner_points) {
  cv::Mat gray_image_for_cali;
  cv::cvtColor(input_chessboard_image, gray_image_for_cali, cv::COLOR_RGB2GRAY);
  if (cv::findChessboardCorners(
          gray_image_for_cali, patternSize_, output_corner_points,
          cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
              cv::CALIB_CB_FAST_CHECK)) {
    cv::cornerSubPix(gray_image_for_cali, output_corner_points, winSize_,
                     zeroZone_, termCriteria_);
    std::cout << "INFO: Corners extracted succeed!" << std::endl;
    cv::drawChessboardCorners(input_chessboard_image, patternSize_,
                              output_corner_points, true);
    cv::imshow("corners", input_chessboard_image);
    cv::waitKey(0);
  } else {
    std::cout << "WARNING: Corners extracted failed!" << std::endl;
    return false;
  }
  return true;
}

void ChessboardCornersExtractor::ChessboardCornersPositionCalculator(
    std::vector<cv::Point3f> &output_world_corner_points) {
  for (int i = 0; i < patternCols_; i++) {
    for (int j = 0; j < patternRows_; j++) {
      cv::Point3f world_corner_point;
      world_corner_point.x = i * size_;
      world_corner_point.y = (patternRows_ - j - 1) * size_;
      world_corner_point.z = 0.0;
      output_world_corner_points.emplace_back(std::move(world_corner_point));
    }
  }
}

} // namespace CamIntrinsicsCali