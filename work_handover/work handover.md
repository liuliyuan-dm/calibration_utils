### Software

* INS (IMU+RTK) setup
* Attitude fusion using IMU sensor (EKF)
* Camera intrinsics calibration
* Camera-IMU calibration
* Camera-Lidar calibration
* Lidar-Lidar calibration
* INS-Lidar calibration
* Calibration result evaluation （Oppo,TODO）
* Arcar calibration (based on vanishing point)
* Arcar calibration (Hand-eye) （TODO）

### Hardware

* Calibration supported tool design (TODO)



### Description

#### INS setup

配合鸿翔完成INS组合导航相关的配置、调试和校验工作，并共同整理了一些文档。@hongxiang.lan

#### Attitude fusion using IMU sensor (EKF)

在最初的IMU-Camera手眼标定方法中，曾尝试用gyroscope和accelerator数据进行融合，计算IMU对地姿态，但因为此方法要求保证标定过程中线性加速度接近0，对于用户来说，此操作要求较难满足，因此最终没有使用此方案。但对于低线性加速度的情况，此方法估算出的姿态精度还是足够的（仿真效果误差0.05度以内，但对于真实数据的融合精度需要用更高精度的设备来校验）。

Reference: imu-sensor-fusion.pdf

#### Camera intrinsics calibration

1. 先在对焦环上打UV胶，调焦，以清晰对焦marker为准，紫外线集中光照10s，待UV胶固化

2. 推荐使用玻璃基漫反射的marker板子（角点数目11*9，边长45mm）

3. 操作标定：大致将整个画面分成约9个区域，使得拍摄的每组图片能够分别覆盖这九个区域至少一次。建议拍摄15-25张。（例如下图，第一个图属于marker板覆盖了中间区域，第二个图属于覆盖了左下角区域）

4. 拍摄过程中可以通过web前端的可视化界面，直接点击download按钮保存图片，默认像素1920*1080。

   ![image-20210603150008862](work%20handover.assets/image-20210603150008862.png)

![image-20210603170627693](work%20handover.assets/image-20210603170627693.png)

Reference: 角点提取，张正友标定法  https://github.com/liuliyuan-dm/calibration_utils/tree/main/intrinsics_calibration

#### Camera-IMU calibration

Marker固定不动，转动Camera-IMU，IMU三个方向都要充分转动，但要保证摄像头采样时刻Marker在画面中（目前的方案是录制视频，然后marker到画面外的点会判定角点提取失败，不参与计算，但实际上只有部分角点在画面里，也是可以参与计算的）。

要考虑到光照（背光、过曝）、运动模糊的影响，尽量在光照良好，高快门的环境下实验。

参考手眼标定算法，计算相机帧i和帧j之间的位姿A，IMU依靠陀螺仪在i,j两帧之间进行纯积分，计算i,j两帧之间IMU的位姿B。最终构建AX=XB问题（X为相机和IMU之间外参），进行求解。

Reference: Kalibr（一个很有名的工具）,  hand eye calibration.md

Code: https://github.com/liuliyuan-dm/calibration_utils/tree/main/cam_imu_cali

#### Camera-Lidar calibration

#### Lidar-Lidar calibration

这两部分算法是@taoran.chen来完成的（本质是PnP问题），此标定项目对数据采集质量有比较高的要求：

1. 希望一次性标定尽可能多的传感器，所以**理想状态**是一次性围绕设备摆放8个不同尺寸的板子（格子尺寸相同，格子数目不同，且相互之间有明显区分），四个摄像头，每个摄像头视野里能够同时看到至少两个板子（摄像头之间有重叠视野更好），注意板子在画面中占比要尽可能大，角点区分度明显。理想状态下待标定的Lidar能够尽可能看到所有的板子，以保证Lidar和每个摄像头都有共视。
2. Camera视野里板子的棋盘格角点必须是完整的，不能有部分遮挡。Lidar视野里板子至少有一对对角能够被完整识别（可以据此回归出一个完整的板子尺寸），要求Lidar点云视野里识别的不同板子有明显的区分度，两块板子的摆放不可以平行（奇异）。
3. 如果1中所要求的理想条件不能完全满足，可以退一步地选择，比如每次只采集两个摄像头和一个Lidar的数据，然后保持板子位置不变，设备多变换几个角度，直到所有摄像头和所有Lidar之间均完成数据采集。
4. 即便1中所要求的理想条件能够完全满足，也希望能够收集更多组数据进行联合标定，板子位置保持不变，设备变换不同的姿态、调整距离板子的远近（如2m, 3m, 5m, 10m等），采集多组数据，联合优化。

Reference：@taoran.chen  https://github.com/deepmirrorinc/Alpha/tree/calibration-final

### INS-Lidar calibration

这部分也是通过类似于手眼标定的方法来完成的，用Ins和Lidar生成的两条轨迹做匹配计算，此方法要求设备有半径约5m的八字环绕动作，环绕要求尽可能快，但是因为RTK本身航向角度误差就有1度，Lidar生成的轨迹又依赖于RTK，所以最终求得的外参精度并不理想，但后续Ins-Lidar的外参可以用来校验，与Ins-Camera, Camera-Lidar的外参形成三角闭环。

Reference：Baidu Apollo标定手段 https://gitbook.cn/books/5cd8e067d7a0dd4c0f47391a/index.html

###  Calibration result evaluation

为了评定Lidar-Camera外参标定的误差，我们会选取不同距离处的点云投影到像平面上，根据像素误差来定义精度（e.g. 5pixel@5m）。和Oppo合作的项目中也有此项要求，Oppo希望我们输出一个sample来验证标定精度。

Reference: https://deepmirror.atlassian.net/wiki/spaces/MTD/pages/866255018/2021-01-20%2BLidar-Camera%2BFront%2BCalibration

Code: *TODO*

### Arcar calibration (based on vanishing point)

目的是基于平行的车道线在远方汇聚于灭点的假设，来计算相机与车道线之前的夹角（俯仰角、偏航角）。

Reference: https://github.com/liuliyuan-dm/calibration_utils/tree/main/arcar

### Arcar calibration (Hand-eye)

TODO

这块还没有做，已知的问题是需要做比较大的Marker，现在的Marker板，对于车载标定来说太小了。同样可以借鉴Apollo的标定方法。

Reference: https://gitbook.cn/books/5cd8e067d7a0dd4c0f47391a/index.html

### Calibration supported tool design

TODO

后续如果想让标定变得更方便，建议做一个可以手动调节旋转可升降的平台。比如结合一个液压升降台+一个机加工用的可旋转的平台，固定在一起使用。

Reference: [可旋转平台示例](https://item.taobao.com/item.htm?spm=a230r.1.14.19.170424a7crCaHe&id=608721762259&ns=1&abbucket=18#detail) [可升降平台示例](https://item.taobao.com/item.htm?spm=a230r.1.14.192.21c27519H7hPbe&id=642656937167&ns=1&abbucket=18#detail)

