#include "ensenso/ensenso_ros_driver.h"
#include "ensenso/ensenso_grabber.h"

ensenso_ros_driver::ensenso_ros_driver():
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
    nh_private_.param("projector", projector, false);
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

void ensenso_ros_driver::singleCloud(PointCloudXYZ &pts)
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
