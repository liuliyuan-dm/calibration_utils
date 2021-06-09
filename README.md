# calibration_utils

Utils for sensors calibration.

```bash
└── work_handover    # An overview of handover work. Please read in detail.
    ├── work handover.assets
    │   ├── image-20210603150008862.png
    │   ├── image-20210603170545255.png
    │   └── image-20210603170627693.png
    └── work handover.md
    
├── arcar   # ArCar attitude estimation based on vanishing point.
│   ├── build
│   │   ├── CMakeCache.txt
│   │   ├── CMakeFiles
│   │   ├── cmake_install.cmake
│   │   ├── Makefile
│   │   └── src
│   ├── CMakeLists.txt
│   ├── data
│   │   └── vanishing_point_data
│   ├── readme.md
│   ├── reference  # docs
│   │   ├── [2007] Research on Lane-Marking Line Based Camera Calibration.pdf
│   │   ├── bird_view_reference.jpg
│   │   ├── 基于消失点的相机俯仰角偏航角计算方法.assets
│   │   └── 基于消失点的相机俯仰角偏航角计算方法.md
│   └── src
│       ├── CMakeLists.txt
│       ├── process_main.cc
│       ├── run.sh
│       ├── vanishing_point_based_atti_cali.cc
│       └── vanishing_point_based_atti_cali.h

├── cam_imu_cali # Extrinsic calibration between camera and imu.
│   ├── build
│   │   ├── clang-tidy-config-errors.txt
│   │   ├── CMakeCache.txt
│   │   ├── CMakeFiles
│   │   ├── cmake_install.cmake
│   │   ├── Makefile
│   │   └── src
│   ├── clang_tidy.sh
│   ├── CMakeLists.txt
│   ├── data  # raw data was uploaded on baiduyun. @hongxiang
│   │   ├── cam
│   │   ├── imu
│   │   └── points
│   ├── include
│   ├── reference  # docs
│   │   ├── hand eye calibration.assets
│   │   └── hand eye calibration.md
│   └── src
│       ├── CMakeLists.txt
│       ├── extended_kalman_filter.cc
│       ├── extended_kalman_filter.h
│       ├── find_chessboard_corner.cc
│       ├── ground_truth_sample_test.cc
│       ├── hand_eye_calibration.cc
│       ├── hand_eye_calibration.h
│       ├── imu.cc
│       ├── imu_copy.h
│       ├── imu_data_generator.cc
│       ├── imu_fusion_main.cc
│       ├── imu_fusion_main_copy.cc
│       ├── imu.h
│       └── run.sh

├── imu_attitude_fusion_ekf # Imu attitude fusion based on extended kalman filter. (only gyroscope and accelerator used.)
│   ├── build
│   │   ├── clang-tidy-config-errors.txt
│   │   ├── CMakeCache.txt
│   │   ├── CMakeFiles
│   │   ├── cmake_install.cmake
│   │   ├── Makefile
│   │   └── src
│   ├── clang_tidy.sh
│   ├── CMakeLists.txt
│   ├── data
│   │   ├── cam
│   │   ├── imu
│   │   └── points
│   ├── include
│   ├── reference  # docs
│   │   └── imu-sensor-fusion.pdf
│   └── src
│       ├── CMakeLists.txt
│       ├── extended_kalman_filter.cc
│       ├── extended_kalman_filter.h
│       ├── find_chessboard_corner.cc
│       ├── ground_truth_sample_test.cc
│       ├── hand_eye_calibration.cc
│       ├── hand_eye_calibration.h
│       ├── imu.cc
│       ├── imu_copy.h
│       ├── imu_data_generator.cc
│       ├── imu_fusion_main.cc
│       ├── imu_fusion_main_copy.cc
│       ├── imu.h
│       └── run.sh

├── intrinsics_calibration # Camera intrinsics calibration.
│   ├── build
│   │   ├── CMakeCache.txt
│   │   ├── CMakeFiles
│   │   ├── cmake_install.cmake
│   │   ├── Makefile
│   │   └── src
│   ├── CMakeLists.txt
│   ├── data
│   │   ├── cali
│   │   ├── pyrDown
│   │   └── undistort
│   ├── How to calibrate camera   #  An instruction of camera intrinsics calibration.
│   ├── include
│   └── src
│       ├── camera_calibration_main.cc
│       ├── camera_intrinsics_calibration.cc
│       ├── camera_intrinsics_calibration.h
│       ├── camera_pyrdown_main.cc
│       ├── CMakeLists.txt
│       └── run.sh
```

