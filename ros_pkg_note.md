#  ros中常用的好用的pkg笔记

* [pointcloud_to_laserscan](https://wiki.ros.org/pointcloud_to_laserscan)：雷达数据3D转2D，用于接入navigate框架
* [depthimage_to_laserscan](https://wiki.ros.org/depthimage_to_laserscan)：RGBD深度图转2D雷达数据：用于接入navigate框架
* [robot_upstart](https://wiki.ros.org/robot_upstart)：ROS的开机自启动pkg
* [imu_tools](https://wiki.ros.org/imu_tools)：提供了imu滤波解决方案，可以用于将六轴imu数据转换成九轴imu数据
* linefit-ground-segmentation：提供了地面分割解决方案，能够在2D导航中实现上坡功能
* image_transport：提供了图像的压缩与解压缩，用来录制bag，或者在带宽有限时通信。以下是解压命令：`<node pkg="image_transport" type="republish" name="republish" args="compressed in:=/camera/image_raw raw out:=/camera/image_raw" output="screen" respawn="true" />`，以下是压缩命令：`rosrun image_transport republish raw in:=/camera/color/image_raw compressed out:=/camera/color/image_raw`
