#!/usr/bin/env bash

# intrinsics calibrate
# ./src/camera_calibration_main input_file_path output_file_path
./src/camera_calibration_main ../data/cali/xavier003/right/ ../data/cali/xavier003/right/

# image downsample
# ./src/camera_pyrdown_main input_file_path output_file_path
# ./src/camera_pyrdown_main ../data/cali/xavier003/back/ ../data/cali/xavier003/back/