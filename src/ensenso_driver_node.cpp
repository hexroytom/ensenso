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
#include <dynamic_reconfigure/server.h>
#include <ensenso/CameraParametersConfig.h>

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
#include <ensenso/CaptureSinglePointCloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef std::pair<pcl::PCLImage, pcl::PCLImage> PairOfImages;

class ensenso_ros_driver
{
private:
  // ROS
  ros::NodeHandle                   nh_, nh_private_;
//  ros::ServiceServer                calibrate_srv_;
//  ros::ServiceServer                capture_srv_;
//  ros::ServiceServer                grid_spacing_srv_;
//  ros::ServiceServer                init_cal_srv_;
//  ros::ServiceServer                ligths_srv_;
  ros::ServiceServer                start_srv_;
  ros::ServiceServer                configure_srv_;
  ros::ServiceServer                capture_single_pc_srv;
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
  //indicators of cloud and images streaming
  bool is_streaming_cloud_;
  bool is_streaming_images_;
  //dynamic reconfigure server
  dynamic_reconfigure::Server<ensenso::CameraParametersConfig> server;

public:
    ensenso_ros_driver():
        nh_private_("~"),
        is_streaming_cloud_(false),
        is_streaming_images_(false)
    {
        // Read parameters
        std::string serial;
        nh_private_.param(std::string("serial"), serial, std::string("160824"));
        if (!nh_private_.hasParam("serial"))
          ROS_WARN_STREAM("Parameter [~serial] not found, using default: " << serial);
        nh_private_.param("camera_frame_id", camera_frame_id_, std::string("camera_link"));
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
        //Advertise services
        capture_single_pc_srv=nh_.advertiseService("capture_single_point_cloud",&ensenso_ros_driver::CaptureSingleCloudSrvCB,this);
        configure_srv_ = nh_.advertiseService("configure_streaming", &ensenso_ros_driver::configureStreamingCB, this);
        start_srv_ = nh_.advertiseService("start_streaming", &ensenso_ros_driver::startStreamingCB, this);
        // Initialize Ensenso
        ensenso_ptr_.reset(new pcl::EnsensoGrabber);
        ensenso_ptr_->openDevice(serial);
        ensenso_ptr_->openTcpPort();
        ensenso_ptr_->configureCapture();
        ensenso_ptr_->enableProjector(projector);
        ensenso_ptr_->enableFrontLight(front_light);
        // Initialize dynamic reconfigure server
//        dynamic_reconfigure::Server<ensenso::CameraParametersConfig>::CallbackType f;
//        f=boost::bind(&ensenso_ros_driver::CameraParametersCallback,this,_1,_2);
//        server.setCallback(f);
    }
    ~ensenso_ros_driver()
    {
        ensenso_ptr_->closeTcpPort();
        ensenso_ptr_->closeDevice();
    }
    int singleCloud(PointCloudXYZ& pts)
    {
        bool was_running = ensenso_ptr_->isRunning();
        if(was_running)
            ensenso_ptr_->stop();

        int res=ensenso_ptr_->grabSingleCloud(pts);
        if(!res)
        {
            ROS_ERROR("Device not open!");
            return (false);
        }
        if(was_running)
            ensenso_ptr_->start();
        return (true);
    }

    bool CaptureSingleCloudSrvCB(ensenso::CaptureSinglePointCloudRequest &req,
                                 ensenso::CaptureSinglePointCloudResponse &res)
    {
        PointCloudXYZ pc;
        if(req.req)
        {
            int result=singleCloud(pc);
            if(result)
            {
                pcl::toROSMsg(pc,res.pc);
                res.pc.header.frame_id=camera_frame_id_;
                res.pc.header.stamp=ros::Time::now();
                return true;
            }
            else
                return false;
        }
        else
            return false;
    }

    bool configureStreamingCB(ensenso::ConfigureStreaming::Request& req, ensenso::ConfigureStreaming::Response &res)
    {
      is_streaming_cloud_=req.cloud;
      is_streaming_images_=req.images;

      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      // Disconnect previous connection
      connection_.disconnect();
      // Connect new signals
      if (req.cloud && req.images)
      {
        boost::function<void(
          const boost::shared_ptr<PointCloudXYZ>&,
          const boost::shared_ptr<PairOfImages>&,
          const boost::shared_ptr<PairOfImages>&)> f = boost::bind (&ensenso_ros_driver::grabberCallback, this, _1, _2, _3);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      else if (req.images)
      {
        boost::function<void(
          const boost::shared_ptr<PairOfImages>&,
          const boost::shared_ptr<PairOfImages>&)> f = boost::bind (&ensenso_ros_driver::grabberCallback, this, _1, _2);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      else if (req.cloud)
      {
        boost::function<void(
            const boost::shared_ptr<PointCloudXYZ>&)> f = boost::bind (&ensenso_ros_driver::grabberCallback, this, _1);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      if (was_running)
        ensenso_ptr_->start();
      res.success = true;
      return true;
    }

    bool startStreamingCB(ensenso::SetBool::Request& req, ensenso::SetBool::Response &res)
    {
      if (req.data)
        ensenso_ptr_->start();
      else
        ensenso_ptr_->stop();
      res.success = true;
      return true;
    }

    void grabberCallback( const boost::shared_ptr<PointCloudXYZ>& cloud)
    {
      // Point cloud
      cloud->header.frame_id = camera_frame_id_;
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(*cloud, cloud_msg);
      cloud_pub_.publish(cloud_msg);
    }

    void grabberCallback( const boost::shared_ptr<PairOfImages>& rawimages, const boost::shared_ptr<PairOfImages>& rectifiedimages)
    {
      // Get cameras info
      sensor_msgs::CameraInfo linfo, rinfo;
      ensenso_ptr_->getCameraInfo("Left", linfo);
      ensenso_ptr_->getCameraInfo("Right", rinfo);
      linfo.header.frame_id = camera_frame_id_;
      rinfo.header.frame_id = camera_frame_id_;
      // Images
      l_raw_pub_.publish(*toImageMsg(rawimages->first), linfo, ros::Time::now());
      r_raw_pub_.publish(*toImageMsg(rawimages->second), rinfo, ros::Time::now());
      l_rectified_pub_.publish(toImageMsg(rectifiedimages->first));
      r_rectified_pub_.publish(toImageMsg(rectifiedimages->second));
    }

    void grabberCallback( const boost::shared_ptr<PointCloudXYZ>& cloud,
                          const boost::shared_ptr<PairOfImages>& rawimages, const boost::shared_ptr<PairOfImages>& rectifiedimages)
    {
      // Get cameras info
      sensor_msgs::CameraInfo linfo, rinfo;
      ensenso_ptr_->getCameraInfo("Left", linfo);
      ensenso_ptr_->getCameraInfo("Right", rinfo);
      linfo.header.frame_id = camera_frame_id_;
      rinfo.header.frame_id = camera_frame_id_;
      // Images
      l_raw_pub_.publish(*toImageMsg(rawimages->first), linfo, ros::Time::now());
      r_raw_pub_.publish(*toImageMsg(rawimages->second), rinfo, ros::Time::now());
      l_rectified_pub_.publish(toImageMsg(rectifiedimages->first));
      r_rectified_pub_.publish(toImageMsg(rectifiedimages->second));
      // Camera_info
      linfo_pub_.publish(linfo);
      rinfo_pub_.publish(rinfo);
      // Point cloud
      cloud->header.frame_id = camera_frame_id_;
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(*cloud, cloud_msg);
      cloud_pub_.publish(cloud_msg);
    }

    sensor_msgs::ImagePtr toImageMsg(pcl::PCLImage pcl_image)
    {
      unsigned char *image_array = reinterpret_cast<unsigned char *>(&pcl_image.data[0]);
      int type(CV_8UC1);
      std::string encoding("mono8");
      if (pcl_image.encoding == "CV_8UC3")
      {
        type = CV_8UC3;
        encoding = "bgr8";
      }
      cv::Mat image_mat(pcl_image.height, pcl_image.width, type, image_array);
      std_msgs::Header header;
      header.frame_id = "world";
      header.stamp = ros::Time::now();
      return cv_bridge::CvImage(header, encoding, image_mat).toImageMsg();
    }

    void CameraParametersCallback(ensenso::CameraParametersConfig &config, uint32_t level)
    {
      // Process enumerators
      std::string trigger_mode, profile;
      switch (config.groups.capture.TriggerMode)
      {
        case 0:
          trigger_mode = "Software";
          break;
        case 1:
          trigger_mode = "FallingEdge";
          break;
        case 2:
          trigger_mode = "RisingEdge";
          break;
        default:
          trigger_mode = "Software";
      }
      switch (config.groups.stereo.OptimizationProfile)
      {
        case 0:
          profile = "Aligned";
          break;
        case 1:
          profile = "Diagonal";
          break;
        case 2:
          profile = "AlignedAndDiagonal";
          break;
        default:
          profile = "AlignedAndDiagonal";
      }
      ROS_DEBUG("---");
      ROS_DEBUG("Capture Parameters");
      ROS_DEBUG_STREAM("AutoBlackLevel: "   << std::boolalpha << config.groups.capture.AutoBlackLevel);
      ROS_DEBUG_STREAM("AutoExposure: "     << std::boolalpha << config.groups.capture.AutoExposure);
      ROS_DEBUG_STREAM("AutoGain: "         << std::boolalpha << config.groups.capture.AutoGain);
      ROS_DEBUG_STREAM("Binning: "          << config.groups.capture.Binning);
      ROS_DEBUG_STREAM("BlackLevelOffset: " << config.groups.capture.BlackLevelOffset);
      ROS_DEBUG_STREAM("Exposure: "         << config.groups.capture.Exposure);
      ROS_DEBUG_STREAM("FlexView: "         << std::boolalpha << config.groups.capture.FlexView);
      ROS_DEBUG_STREAM("FlexViewImages: "   << config.groups.capture.FlexViewImages);
      ROS_DEBUG_STREAM("FrontLight: "       << std::boolalpha << config.groups.capture.FrontLight);
      ROS_DEBUG_STREAM("Gain: "             << config.groups.capture.Gain);
      ROS_DEBUG_STREAM("GainBoost: "        << std::boolalpha << config.groups.capture.GainBoost);
      ROS_DEBUG_STREAM("HardwareGamma: "    << std::boolalpha << config.groups.capture.HardwareGamma);
      ROS_DEBUG_STREAM("Hdr: "              << std::boolalpha << config.groups.capture.Hdr);
      ROS_DEBUG_STREAM("PixelClock: "       << config.groups.capture.PixelClock);
      ROS_DEBUG_STREAM("Projector: "        << std::boolalpha << config.groups.capture.Projector);
      ROS_DEBUG_STREAM("TargetBrightness: " << config.groups.capture.TargetBrightness);
      ROS_DEBUG_STREAM("TriggerMode: "      << trigger_mode);
      ROS_DEBUG_STREAM("DisparityMapAOI: "  << std::boolalpha << config.groups.capture.DisparityMapAOI);
      ROS_DEBUG("Stereo Matching Parameters");
      ROS_DEBUG_STREAM("MinimumDisparity: "     << config.groups.stereo.MinimumDisparity);
      ROS_DEBUG_STREAM("NumberOfDisparities: "  << config.groups.stereo.NumberOfDisparities);
      ROS_DEBUG_STREAM("OptimizationProfile: "  << profile);
      ROS_DEBUG_STREAM("Scaling: "              << config.groups.stereo.Scaling);
      ROS_DEBUG("Advanced Matching Parameters");
      ROS_DEBUG_STREAM("DepthChangeCost: " << config.groups.stereo.DepthChangeCost);
      ROS_DEBUG_STREAM("DepthStepCost: " << config.groups.stereo.DepthStepCost);
      ROS_DEBUG_STREAM("ShadowingThreshold: " << config.groups.stereo.ShadowingThreshold);
      ROS_DEBUG("Postprocessing Parameters");
      ROS_DEBUG_STREAM("UniquenessRatio: " << config.groups.postproc.UniquenessRatio);
      ROS_DEBUG_STREAM("MedianFilterRadius: "<< config.groups.postproc.MedianFilterRadius);
      ROS_DEBUG_STREAM("SpeckleComponentThreshold: "<< config.groups.postproc.SpeckleComponentThreshold);
      ROS_DEBUG_STREAM("SpeckleRegionSize: "<< config.groups.postproc.SpeckleRegionSize);
      ROS_DEBUG_STREAM("FillBorderSpread: "<< config.groups.postproc.FillBorderSpread);
      ROS_DEBUG_STREAM("FillRegionSize: " << config.groups.postproc.FillRegionSize);
      ROS_DEBUG("Stream Parameters");
      ROS_DEBUG_STREAM("Cloud: "   << std::boolalpha << config.groups.stream.Cloud);
      ROS_DEBUG_STREAM("Images: "   << std::boolalpha << config.groups.stream.Images);
      ROS_DEBUG("---");
      // Capture parameters
      ensenso_ptr_->setAutoBlackLevel(config.groups.capture.AutoBlackLevel);
      ensenso_ptr_->setAutoExposure(config.groups.capture.AutoExposure);
      ensenso_ptr_->setAutoGain(config.groups.capture.AutoGain);
      ensenso_ptr_->setBlackLevelOffset(config.groups.capture.BlackLevelOffset);
      ensenso_ptr_->setExposure(config.groups.capture.Exposure);
      ensenso_ptr_->setFrontLight(config.groups.capture.FrontLight);
      ensenso_ptr_->setGain(config.groups.capture.Gain);
      ensenso_ptr_->setGainBoost(config.groups.capture.GainBoost);
      ensenso_ptr_->setHardwareGamma(config.groups.capture.HardwareGamma);
      ensenso_ptr_->setHdr(config.groups.capture.Hdr);
      ensenso_ptr_->setPixelClock(config.groups.capture.PixelClock);
      ensenso_ptr_->setProjector(config.groups.capture.Projector);
      ensenso_ptr_->setTargetBrightness(config.groups.capture.TargetBrightness);
      ensenso_ptr_->setTriggerMode(trigger_mode);
      ensenso_ptr_->setUseDisparityMapAreaOfInterest(config.groups.capture.DisparityMapAOI);
      // Flexview and binning only work in 'Software' trigger mode and with the projector on
      if (trigger_mode.compare("Software") == 0 && config.groups.capture.Projector)
      {
        ensenso_ptr_->setBinning(config.groups.capture.Binning);
      }
      // Stereo parameters
      ensenso_ptr_->setMinimumDisparity(config.groups.stereo.MinimumDisparity);
      ensenso_ptr_->setNumberOfDisparities(config.groups.stereo.NumberOfDisparities);
      ensenso_ptr_->setOptimizationProfile(profile);
      ensenso_ptr_->setScaling(config.groups.stereo.Scaling);
      ensenso_ptr_->setDepthChangeCost(config.groups.stereo.DepthChangeCost);
      ensenso_ptr_->setDepthStepCost(config.groups.stereo.DepthStepCost);
      ensenso_ptr_->setShadowingThreshold(config.groups.stereo.ShadowingThreshold);
      //Postprocessing parameters
      ensenso_ptr_->setUniquenessRatio(config.groups.postproc.UniquenessRatio);
      ensenso_ptr_->setMedianFilterRadius(config.groups.postproc.MedianFilterRadius);
      ensenso_ptr_->setSpeckleComponentThreshold(config.groups.postproc.SpeckleComponentThreshold);
      ensenso_ptr_->setSpeckleRegionSize(config.groups.postproc.SpeckleRegionSize);
      ensenso_ptr_->setFillBorderSpread(config.groups.postproc.FillBorderSpread);
      ensenso_ptr_->setFillRegionSize(config.groups.postproc.FillRegionSize);
      // Streaming parameters
      configureStreaming(config.groups.stream.Cloud, config.groups.stream.Images);
    }

    bool configureStreaming(const bool cloud, const bool images=true)
    {
      if ((is_streaming_cloud_ == cloud) && (is_streaming_images_ == images))
        return true;  // Nothing to be done here
      is_streaming_cloud_ = cloud;
      is_streaming_images_ = images;
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      // Disconnect previous connection
      connection_.disconnect();
      // Connect new signals
      if (cloud && images)
      {
        boost::function<void(
          const boost::shared_ptr<PointCloudXYZ>&,
          const boost::shared_ptr<PairOfImages>&,
          const boost::shared_ptr<PairOfImages>&)> f = boost::bind (&ensenso_ros_driver::grabberCallback, this, _1, _2, _3);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      else if (images)
      {
        boost::function<void(
          const boost::shared_ptr<PairOfImages>&,
          const boost::shared_ptr<PairOfImages>&)> f = boost::bind (&ensenso_ros_driver::grabberCallback, this, _1, _2);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      else if (cloud)
      {
        boost::function<void(
            const boost::shared_ptr<PointCloudXYZ>&)> f = boost::bind (&ensenso_ros_driver::grabberCallback, this, _1);
        connection_ = ensenso_ptr_->registerCallback(f);
      }
      if (was_running)
        ensenso_ptr_->start();
      return true;
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

//  while(ros::ok())
//  {
//      ensensoNode.singleCloud(pts);
//      pts_msg->header.stamp=ros::Time::now();
//      pcl::toROSMsg(pts, *pts_msg);
//      pts_msg->header.frame_id = "/camera_link";
//      pub.publish(*pts_msg);
//      loop.sleep();
//  }
  ros::spin();
  return 0;
}
