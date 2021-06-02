#include "vanishing_point_based_atti_cali.h"

namespace ArCar {
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
      std::cout << "load succeed!" << std::endl;
    } else {
      continue;
    }
  }
}

VanishingPointBasedAttitudeCalculator::VanishingPointBasedAttitudeCalculator() {
}

void VanishingPointBasedAttitudeCalculator::setCameraMatrix(
    cv::Mat &input_camera_matrix) {
  camera_matrix_ = input_camera_matrix;
}

void VanishingPointBasedAttitudeCalculator::setVanishingPoint(
    cv::Point2f &input_vanishing_point) {
  vanishing_point_ = input_vanishing_point;
}

void VanishingPointBasedAttitudeCalculator::setCameraHeight(
    double &input_camera_height) {
  camera_height_ = input_camera_height;
}

// 参考文档：基于消失点的相机俯仰角偏航角计算方法.md
// 直接调用公式（3）即可求解pitch和yaw角度，
// 注意一下文档中Pitch和yaw代表哪两个角度。
void VanishingPointBasedAttitudeCalculator::
    CalculateAttitudeBasedVanishingPoint(
        cv::Point2f &output_pitch_yaw_attitude_in_rad) {
  // 计算俯仰角output_pitch_yaw_attitude_in_rad.y和航向角output_pitch_yaw_attitude_in_rad.x，单位rad
  output_pitch_yaw_attitude_in_rad.y =
      atan2(camera_matrix_.at<double>(1, 2) - vanishing_point_.y,
            camera_matrix_.at<double>(1, 1));
  output_pitch_yaw_attitude_in_rad.x =
      atan2((vanishing_point_.x - camera_matrix_.at<double>(0, 2)) *
                cos(output_pitch_yaw_attitude_in_rad.y),
            camera_matrix_.at<double>(0, 0));

  // 以车道线宽度为先验信息（比如此处假定宽度3.25m），根据两条车道线在画面水平中心处的像素距离，反推计算得到相机高度。
  // 实际车辆上相机高度比较稳定，可以直接通过量取的方法获得。
  /*
  double w = 3.25 * 2;
  double delta_u = 1765;
  double output_camera_height = 0;
  output_camera_height = cameraMatrix.at<double>(0, 0) * w *
                         sin(output_pitch_yaw_attitude_in_rad.y) /
                         cos(output_pitch_yaw_attitude_in_rad.x) / delta_u;
  */
  std::cout << "yaw is: " << output_pitch_yaw_attitude_in_rad.x * 57.3
            << " deg,"
            << " pitch is: " << output_pitch_yaw_attitude_in_rad.y * 57.3
            << " deg" << std::endl;
}

// 参考文档：基于消失点的相机俯仰角偏航角计算方法.md  鸟瞰图投影参见公式（4）
void VanishingPointBasedAttitudeCalculator::BirdViewReproject(
    cv::Mat &input_undistort_image,
    const cv::Point2f &input_pitch_yaw_attitude_in_rad,
    cv::Mat &output_bird_view_reproject_image) {
  double fx = camera_matrix_.at<double>(0, 0);
  double fy = camera_matrix_.at<double>(1, 1);
  double cx = camera_matrix_.at<double>(0, 2);
  double cy = camera_matrix_.at<double>(1, 2);
  double x, y, u, v;
  double scale =
      0.02; // 鸟瞰图的缩放因子，为了看到更大的视野，此处调参确定用0.02

  for (size_t vc = vanishing_point_.y; vc < input_undistort_image.rows; vc++) {
    for (size_t uc = 0; uc < input_undistort_image.cols; uc++) {
      // 把像素坐标(uc,vc)转换到以光轴中心为原点的像素坐标系，方便计算，参考文档：基于消失点的相机俯仰角偏航角计算方法.md
      double ucd = uc - cx;
      double vcd = cy - vc;
      double vc0 = cy - vanishing_point_.y; // 把灭点坐标也转换到同一个坐标系下

      // 根据公式（4）反推对应点在世界坐标系下的坐标（x,y）
      x = vc0 * ucd * camera_height_ * scale / (vc0 - vcd) / fx /
          sin(input_pitch_yaw_attitude_in_rad.y);
      y = vc0 * vcd * camera_height_ * scale / (vc0 - vcd) / fx /
          sin(input_pitch_yaw_attitude_in_rad.y) /
          sin(input_pitch_yaw_attitude_in_rad.y);

      // 假定新相机在道路平面正上方俯视，世界坐标系下的点投影到俯视的成像平面上，得到(u,v)
      u = fx * x;
      v = fy * y;

      u = u + cx;
      v = -v + cy +
          vanishing_point_.y; //+vanishing_point_.y仅仅是为了对画面进行整体平移

      // 完成从原始图像到新的鸟瞰图之间的转换
      // 为了方便，这里直接对计算的结果进行了强制类型转换，实际上可以插值，使得得到的图像过度更平滑
      int ui = (int)(u);
      int vi = (int)(v);

      unsigned char *row_ptr_c = input_undistort_image.ptr<unsigned char>(vc);
      unsigned char *data_ptr_c =
          &row_ptr_c[uc * input_undistort_image.channels()];
      unsigned char *row_ptr =
          output_bird_view_reproject_image.ptr<unsigned char>(vi);
      unsigned char *data_ptr =
          &row_ptr[ui * output_bird_view_reproject_image.channels()];

      if (v > 0 && u > 0 && v < input_undistort_image.rows &&
          u < input_undistort_image.cols) {
        for (int c = 0; c != input_undistort_image.channels(); c++) {
          data_ptr[c] = data_ptr_c[c];
        }
      }
    }
  }

  cv::imshow("output_bird_view_reproject_image",
             output_bird_view_reproject_image);
  cv::waitKey(0);
  // cv::imwrite("/home/liyuanliu/code/project/arcar/data/vanishing_point_data/"
  //            "output_bird_view_project_DJI0779.jpg",
  //            output_bird_view_reproject_image);
}

} // namespace ArCar
