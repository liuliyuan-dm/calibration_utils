#include "vanishing_point_based_atti_cali.h"

int main(int argc, char **argv) {
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 2867.479919277094 / 2, 0,
                           2000.271542457706 / 2, 0, 2872.841268187542 / 2,
                           1499.805437689291 / 2, 0, 0, 1);
  cv::Point2f vanishing_point(1469, 362);
  double camera_height = 1.44354;
  std::string filename = "/home/liyuanliu/code/project/arcar/data/"
                         "vanishing_point_data/5(1469,362).jpg";

  ArCar::VanishingPointBasedAttitudeCalculator
      vanishing_point_based_attitude_calculator;
  vanishing_point_based_attitude_calculator.setCameraMatrix(camera_matrix);
  vanishing_point_based_attitude_calculator.setVanishingPoint(vanishing_point);
  vanishing_point_based_attitude_calculator.setCameraHeight(camera_height);

  cv::Point2f output_attitude_in_rad;
  vanishing_point_based_attitude_calculator
      .CalculateAttitudeBasedVanishingPoint(output_attitude_in_rad);

  cv::Mat input_undistort_image;
  int rows = input_undistort_image.rows;
  int cols = input_undistort_image.cols;

  input_undistort_image = cv::imread(filename);
  cv::Mat bird_view_reproject_image = input_undistort_image.clone();
  bird_view_reproject_image.setTo((0, 0, 0));
  vanishing_point_based_attitude_calculator.BirdViewReproject(
      input_undistort_image, output_attitude_in_rad, bird_view_reproject_image);
}

/*
http://metapicz.com/#landing
_________________________________________________________________________
|NUM.     |VANISHING POINT|REAL (PITCH,YAW)|CAL (PITCH,YAW)|CAL HEIGHT
|
|DJI0770  |(1015,718)
|DJI0771  |(975, 749)     (+15.2,-7.1)     (15.5585,-1.04464)
|DJI0772  |
|DJI0773  |
|DJI0774  |
|DJI0775  |(1469, 362)     (+14.90,+16.80)  (15.1876,17.5872)
|DJI0776  |
|DJI0777  |
|DJI0778  |
|DJI0779  |(1615,643)
|DJI0780  |
|________________________________________________________________________
*/
// cv::Mat dist_coeffs =
//    (cv::Mat_<double>(1, 5) << 0.04741695414837928, -0.09082422276205709,
//     0.0001845408682387943, -0.001211580708377683, 0.03341932052257165);