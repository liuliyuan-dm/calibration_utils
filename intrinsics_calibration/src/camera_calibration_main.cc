// calibrates camera intrinsics.

#include "camera_intrinsics_calibration.h"

int main(int argc, char **argv) {
  CamIntrinsicsCali::FunctionSelector function_selector;
  CamIntrinsicsCali::ImageProcessor image_processor;
  CamIntrinsicsCali::CameraIntrinsicsCalibrator camera_intrinsics_calibrator;
  std::string input_file_path = argv[1];
  std::string output_file_path = argv[2];
  std::vector<cv::Mat> raw_images_for_cali; //"../data/cali/xavier003/back/"
  image_processor.ImagesReaderFromPathOrFile(input_file_path,
                                             raw_images_for_cali);
  // extracts chessboard corner points in camera frame.
  // calculates chessboard corner points' position in wolrd frame.
  CamIntrinsicsCali::ChessboardCornersExtractor chessboard_corners_extractor;
  std::vector<std::vector<cv::Point2f>> image_corner_points_all_views;
  std::vector<std::vector<cv::Point3f>> world_corner_points_all_views;
  bool flag_chessboard_corners_found;
  for (int i = 0; i < raw_images_for_cali.size(); i++) {
    std::vector<cv::Point2f> image_corner_points_each_view;
    std::vector<cv::Point3f> world_corner_points_each_view;
    if (chessboard_corners_extractor.FindChessboardCornersSubPix(
            raw_images_for_cali[i], image_corner_points_each_view)) {
      chessboard_corners_extractor.ChessboardCornersPositionCalculator(
          world_corner_points_each_view);
      image_corner_points_all_views.emplace_back(
          std::move(image_corner_points_each_view));
      world_corner_points_all_views.emplace_back(
          std::move(world_corner_points_each_view));
    }
  }

  // calibrates camera intrinsics.
  camera_intrinsics_calibrator.setImageSize(raw_images_for_cali[0].cols,
                                            raw_images_for_cali[0].rows);
  camera_intrinsics_calibrator.CalibrateCameraIntrinsics(
      world_corner_points_all_views, image_corner_points_all_views);
  camera_intrinsics_calibrator.CameraIntrinscisOutput(output_file_path);

  // undistort
  if (function_selector.undistort_image_save_enable_) {
    for (int i = 0; i < raw_images_for_cali.size(); i++) {
      cv::Mat undistort_image;
      camera_intrinsics_calibrator.ImageUndistort(raw_images_for_cali[i],
                                                  undistort_image);
      cv::imshow("test", undistort_image);
      cv::waitKey(0);
      cv::imwrite(input_file_path + "undistort_" + std::to_string(i) + ".jpg",
                  undistort_image);
    }
  }
}