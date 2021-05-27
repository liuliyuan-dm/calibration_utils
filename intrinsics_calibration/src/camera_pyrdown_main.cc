// 对于过大的图片，可能会遇到需要降采样缩小图片，再对缩小后的图片进行标定的情况

#include "camera_intrinsics_calibration.h"

int main(int argc, char **argv) {
  CamIntrinsicsCali::FunctionSelector function_selector;
  CamIntrinsicsCali::ImageProcessor image_processor;
  // 降采样处理（if needed）
  std::string input_file_path = argv[1];
  std::string output_file_path = argv[2];
  std::vector<cv::Mat> pyrdown_images;
  image_processor.ImagesReaderFromPathOrFile(input_file_path, pyrdown_images);
  image_processor.ImagePyrDownRewriter(pyrdown_images, 2.0, output_file_path);
}