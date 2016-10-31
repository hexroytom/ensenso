#ifndef ENSENSO_ROS_DRIVER_H
#define ENSENSO_ROS_DRIVER_H

// ROS headers
#include <ros/ros.h>
#include <ros/service.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

// Conversions
#include <eigen_conversions/eigen_msg.h>

// Image transport
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL headers
//#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>

// Ensenso grabber
#include <ensenso/ensenso_grabber.h>
// Services
#include <ensenso/Lights.h>
#include <ensenso/CapturePattern.h>
#include <ensenso/ComputeCalibration.h>
#include <ensenso/ConfigureStreaming.h>
#include <ensenso/GridSpacing.h>
#include <ensenso/InitCalibration.h>
#include <ensenso/SetBool.h>
#include <std_srvs/Trigger.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

class ensenso_ros_driver
{
private:
  // ROS
  ros::NodeHandle                   nh_, nh_private_;
  ros::ServiceServer                calibrate_srv_;
  ros::ServiceServer                capture_srv_;
  ros::ServiceServer                grid_spacing_srv_;
  ros::ServiceServer                init_cal_srv_;
  ros::ServiceServer                ligths_srv_;
  ros::ServiceServer                start_srv_;
  ros::ServiceServer                configure_srv_;
  // Images
  image_transport::CameraPublisher  l_raw_pub_;
  image_transport::CameraPublisher  r_raw_pub_;
  image_transport::Publisher        l_rectified_pub_;
  image_transport::Publisher        r_rectified_pub_;
  // Point cloud
  bool                              point_cloud_;
  ros::Publisher                    cloud_pub_;
  // Camera info
  ros::Publisher                    linfo_pub_;
  ros::Publisher                    rinfo_pub_;
  // TF
  std::string                       camera_frame_id_;
  // Ensenso grabber
  boost::signals2::connection       connection_;
  pcl::EnsensoGrabber::Ptr          ensenso_ptr_;

public:
    ensenso_ros_driver();
    void singleCloud(PointCloudXYZ& pts);

};

#endif // ENSENSO_ROS_DRIVER_H
