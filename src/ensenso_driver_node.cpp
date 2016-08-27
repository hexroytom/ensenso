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
#include <pcl/common/colors.h>
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
    ensenso_ros_driver():
        nh_private_("~")
    {
        // Read parameters
        std::string serial;
        nh_private_.param(std::string("serial"), serial, std::string("160824"));
        if (!nh_private_.hasParam("serial"))
          ROS_WARN_STREAM("Parameter [~serial] not found, using default: " << serial);
        nh_private_.param("camera_frame_id", camera_frame_id_, std::string("ensenso_optical_frame"));
        if (!nh_private_.hasParam("camera_frame_id"))
          ROS_WARN_STREAM("Parameter [~camera_frame_id] not found, using default: " << camera_frame_id_);
        // Booleans
        bool front_light, projector;
        nh_private_.param("front_light", front_light, false);
        if (!nh_private_.hasParam("front_light"))
          ROS_WARN_STREAM("Parameter [~front_light] not found, using default: " << front_light);
        nh_private_.param("projector", projector, true);
        if (!nh_private_.hasParam("projector"))
          ROS_WARN_STREAM("Parameter [~projector] not found, using default: " << projector);
        nh_private_.param("point_cloud", point_cloud_, false);
        if (!nh_private_.hasParam("point_cloud"))
          ROS_WARN_STREAM("Parameter [~point_cloud] not found, using default: " << point_cloud_);
        // Advertise topics
        image_transport::ImageTransport it(nh_);
        l_raw_pub_ = it.advertiseCamera("left/image_raw", 1);
        r_raw_pub_ = it.advertiseCamera("right/image_raw", 1);
        l_rectified_pub_ = it.advertise("left/image_rect", 1);
        r_rectified_pub_ = it.advertise("right/image_rect", 1);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2 >("depth/points", 1, true); // Latched
        linfo_pub_=nh_.advertise<sensor_msgs::CameraInfo> ("left/camera_info", 1, true);
        rinfo_pub_=nh_.advertise<sensor_msgs::CameraInfo> ("right/camera_info", 1, true);
        // Initialize Ensenso
        ensenso_ptr_.reset(new pcl::EnsensoGrabber);
        ensenso_ptr_->openDevice(serial);
        ensenso_ptr_->openTcpPort();
        ensenso_ptr_->configureCapture();
        ensenso_ptr_->enableProjector(projector);
        ensenso_ptr_->enableFrontLight(front_light);
    }
    ~ensenso_ros_driver()
    {
        ensenso_ptr_->closeTcpPort();
        ensenso_ptr_->closeDevice();
    }
    void singleCloud(PointCloudXYZ& pts)
    {
        bool was_running = ensenso_ptr_->isRunning();
        if(was_running)
            ensenso_ptr_->stop();

        int res=ensenso_ptr_->grabSingleCloud(pts);
        if(!res)
            ROS_ERROR("Device not open!");
        if(was_running)
            ensenso_ptr_->start();
    }
};

int main(int argc, char **argv)
{
  ros::init (argc, argv, "ensenso");
  ros::NodeHandle nh;
  ros::Publisher pub=nh.advertise<sensor_msgs::PointCloud2>("/ensenso_single_pointcloud",1);
  ros::Rate loop(3);
  ensenso_ros_driver ensensoNode;

  PointCloudXYZ pts;
  sensor_msgs::PointCloud2Ptr pts_msg(new sensor_msgs::PointCloud2);

  while(ros::ok())
  {
      ensensoNode.singleCloud(pts);
      pts_msg->header.stamp=ros::Time::now();
      pcl::toROSMsg(pts, *pts_msg);
      pts_msg->header.frame_id = "/camera_link";
      pub.publish(*pts_msg);
      loop.sleep();
  }
  return 0;
}
