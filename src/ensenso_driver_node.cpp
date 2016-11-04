// ROS headers
#include <ros/ros.h>
#include <ros/service.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <ensenso/CameraParametersConfig.h>
#include <pcl_conversions/pcl_conversions.h>

// Conversions
#include <eigen_conversions/eigen_msg.h>

// Image transport
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL headers
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

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
#include <ensenso/RegistImage.h>

//std_lib
#include <iostream>

using namespace std;
using namespace cv;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef std::pair<pcl::PCLImage, pcl::PCLImage> PairOfImages;

class ensenso_ros_driver
{
private:
  // ROS
  ros::NodeHandle                   nh_, nh_private_;
  ros::ServiceServer                calibrate_srv_;
//  ros::ServiceServer                capture_srv_;
//  ros::ServiceServer                grid_spacing_srv_;
//  ros::ServiceServer                init_cal_srv_;
//  ros::ServiceServer                ligths_srv_;
  ros::ServiceServer                start_srv_;
  ros::ServiceServer                configure_srv_;
  ros::ServiceServer                capture_single_pc_srv;
  ros::ServiceServer                capture_pattern_srv_;
  ros::ServiceServer                init_cal_srv_;
  ros::ServiceServer                regist_image_srv_;
  // Images
  image_transport::CameraPublisher  l_raw_pub_;
  image_transport::CameraPublisher  r_raw_pub_;
  image_transport::Publisher        l_rectified_pub_;
  image_transport::Publisher        r_rectified_pub_;
  image_transport::Publisher        rgb_pub_;
  // Point cloud
  bool                              point_cloud_;
  ros::Publisher                    cloud_pub_;
  // Camera info
  ros::Publisher                    linfo_pub_;
  ros::Publisher                    rinfo_pub_;
  ros::Publisher                    registPC_pub_;
  // TF
  std::string                       camera_frame_id_;
  // Ensenso grabber
  boost::signals2::connection       connection_;
  pcl::EnsensoGrabber::Ptr          ensenso_ptr_;
  //indicators of cloud and images streaming
  bool is_streaming_cloud_;
  bool is_streaming_images_;
  bool is_streaming_rgb_;
  //dynamic reconfigure server
  dynamic_reconfigure::Server<ensenso::CameraParametersConfig> server;

  int pattern_existed_count;
  //indicator of connection of monocular camera
  bool connect_monocular_;
  std::string json_;

public:
    ensenso_ros_driver(bool connect_monocular=false):
        nh_private_("~"),
        is_streaming_cloud_(false),
        is_streaming_images_(false),
        is_streaming_rgb_(false),
        connect_monocular_(connect_monocular)
    {
        // Read parameters
        std::string depCamSerial;
        std::string rgbCamSerial;
        nh_private_.param(std::string("depCamSerial"), depCamSerial, std::string("160824"));
        if (!nh_private_.hasParam("depCamSerial"))
          ROS_WARN_STREAM("Parameter [~depCamSerial] not found, using default: " << depCamSerial);
        nh_private_.param(std::string("rgbCamSerial"), rgbCamSerial, std::string("4102849778"));
        if (!nh_private_.hasParam("rgbCamSerial"))
          ROS_WARN_STREAM("Parameter [~rgbCamSerial] not found, using default: " << rgbCamSerial);
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
        rgb_pub_ = it.advertise("rgb/image_raw",1);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2 >("depth/points", 1, true); // Latched
        linfo_pub_=nh_.advertise<sensor_msgs::CameraInfo> ("left/camera_info", 1, true);
        rinfo_pub_=nh_.advertise<sensor_msgs::CameraInfo> ("right/camera_info", 1, true);
        registPC_pub_=nh_.advertise<sensor_msgs::PointCloud2>("registered_pointcloud",1,true);
        //Advertise services
        capture_single_pc_srv=nh_.advertiseService("capture_single_point_cloud",&ensenso_ros_driver::CaptureSingleCloudSrvCB,this);
        configure_srv_ = nh_.advertiseService("configure_streaming", &ensenso_ros_driver::configureStreamingCB, this);
        start_srv_ = nh_.advertiseService("start_streaming", &ensenso_ros_driver::startStreamingCB, this);
        capture_pattern_srv_ = nh_.advertiseService("capture_pattern", &ensenso_ros_driver::capturePatternCB,this);
        init_cal_srv_ = nh_.advertiseService("init_calibration",&ensenso_ros_driver::initCalibrationCB,this);
        calibrate_srv_ = nh_.advertiseService("compute_calibration", &ensenso_ros_driver::computeCalibrationCB, this);
        regist_image_srv_ = nh_.advertiseService("grab_registered_image", &ensenso_ros_driver::registImageCB, this);
        // Initialize Ensenso
        if(!connect_monocular_){
            ensenso_ptr_.reset(new pcl::EnsensoGrabber);
            ensenso_ptr_->openDevice(depCamSerial);
            ensenso_ptr_->openTcpPort();
            ensenso_ptr_->configureCapture();
            ensenso_ptr_->enableProjector(projector);
            ensenso_ptr_->enableFrontLight(front_light);
            ensenso_ptr_->start();
        }else{
            ensenso_ptr_.reset(new pcl::EnsensoGrabber);
            ensenso_ptr_->openDevice(depCamSerial,rgbCamSerial);
            ensenso_ptr_->openTcpPort();
            ensenso_ptr_->configureCapture(); //Only configure depth camera
            ensenso_ptr_->enableProjector(projector);
            ensenso_ptr_->enableFrontLight(front_light);
            ensenso_ptr_->start();
        }
        // Initialize dynamic reconfigure server
        dynamic_reconfigure::Server<ensenso::CameraParametersConfig>::CallbackType f;
        f=boost::bind(&ensenso_ros_driver::CameraParametersCallback,this,_1,_2);
        server.setCallback(f);
    }
    ~ensenso_ros_driver()
    {
        ensenso_ptr_->closeTcpPort();
        ensenso_ptr_->closeDevice();
    }

    bool registImageCB(ensenso::RegistImage::Request& req,ensenso::RegistImage::Response& res)
    {
        bool was_running = ensenso_ptr_->isRunning();
        if (was_running)
          ensenso_ptr_->stop();
        //retrive rgb image and registered pointcloud
        if(req.is_rgb){
            cv::Mat image;
            PointCloudXYZ::Ptr pc(new PointCloudXYZ);
            ensenso_ptr_->grabRegistImages(image,pc);
            //Transfer to ROS msg
            std_msgs::Header header;
            cv_bridge::CvImage cv_img(header,sensor_msgs::image_encodings::RGB8,image);
            res.image=*(cv_img.toImageMsg());

            pcl::toROSMsg(*pc,res.pointcloud);

        }//retrive image of left camera and pointcloud
        else
        {

        }

        if (was_running)
          ensenso_ptr_->start();
        return true;
    }

    bool computeCalibrationCB(ensenso::ComputeCalibration::Request& req, ensenso::ComputeCalibration::Response &res)
    {
      // Very important to stop the camera before performing the calibration
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > poses;
      for (size_t i = 0; i < req.robotposes.poses.size(); i++) {
        Eigen::Affine3d pose;
        tf::poseMsgToEigen(req.robotposes.poses[i], pose);
        poses.push_back(pose);
      }
      Eigen::Affine3d seed;
      tf::poseMsgToEigen(req.seed, seed);
      std::string result;
      double error;
      int iters;
      if (!ensenso_ptr_->computeCalibrationMatrix(poses, result, iters, error, "Moving", "Hand", seed))
        res.success = false;
      else {
        ROS_INFO("Calibration computation finished");
        // Populate the response
        res.success = true;
        Eigen::Affine3d eigen_result;
        ensenso_ptr_->jsonTransformationToMatrix(result, eigen_result);
        eigen_result.translation () /= 1000.0;  // Convert translation to meters (Ensenso API returns milimeters)
        tf::poseEigenToMsg(eigen_result, res.result);
        res.reprojection_error = error;
        res.iterations = iters;
        if (req.store_to_eeprom)
        {
          if (!ensenso_ptr_->clearEEPROMExtrinsicCalibration())
            ROS_WARN("Could not reset extrinsic calibration");
          ensenso_ptr_->storeEEPROMExtrinsicCalibration();
          ROS_INFO("Calibration stored into the EEPROM");
        }
      }
      if (was_running)
        ensenso_ptr_->start();
      return true;
    }

    //CapturePattern call back function
    bool capturePatternCB(ensenso::CapturePattern::Request& req, ensenso::CapturePattern::Response &res)
    {
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      //Set grid space and close projector
      //ensenso_ptr_->setGridSpacing((double)req.grid_spacing);
      ensenso_ptr_->setProjector(false);
      ensenso_ptr_->setFrontLight(true);
      //Capture an image and search for the pattern in the image
        //if pattern found, it will be put in the pattern buffer
        //if not, the function will return -1
      int pattern_exist_count = ensenso_ptr_->patternExistedtCount();
      res.pattern_count = ensenso_ptr_->captureCalibrationPattern();
      res.success = (res.pattern_count - pattern_exist_count==1 && res.pattern_count != -1 ? true:false);
      if (res.success)
      {
        // Pattern pose
        Eigen::Affine3d pattern_pose;
        ensenso_ptr_->estimateCalibrationPatternPose(pattern_pose);
        tf::poseEigenToMsg(pattern_pose, res.pose);
      }
      else{
          ROS_ERROR("Failed to capture pattern");
          //Open projector again even if it failed
          ensenso_ptr_->setProjector(true);
          ensenso_ptr_->setFrontLight(false);
          if (was_running)
            ensenso_ptr_->start();
          return false;
      }
      //Open projector again
      ensenso_ptr_->setProjector(true);
      ensenso_ptr_->setFrontLight(false);
      if (was_running)
        ensenso_ptr_->start();
      return true;
    }

    bool initCalibrationCB(ensenso::InitCalibration::Request& req, ensenso::InitCalibration::Response &res)
    {
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      // The grid_spacing value in the request has preference over the decode one
      double spacing;
      if (req.grid_spacing > 0)
        spacing = req.grid_spacing;
      else
        spacing = ensenso_ptr_->getPatternGridSpacing();

      if (spacing > 0)
      {
        res.success = ensenso_ptr_->initExtrinsicCalibration(spacing);
        if(!res.success){
                if (was_running)
                    ensenso_ptr_->start();
                return false;
            }
        res.used_grid_spacing = spacing;
      }
      if (was_running)
        ensenso_ptr_->start();
      return true;
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

    int singleDepthMap(cv::Mat& depMap)
    {
        PointCloudXYZ pts;
        singleCloud(pts);
        int height=pts.height;int width=pts.width;
        depMap.create(pts.height,pts.width,CV_32FC1);
        for(int i=0;i<pts.height;i++)
            for(int j=0;j<pts.width;j++)
                depMap.at<float>(i,j)=pts.points[i*width+j].z;
    }

    int singleRGBImage(cv::Mat& RGBimg)
    {
        bool was_running = ensenso_ptr_->isRunning();
        if(was_running)
            ensenso_ptr_->stop();

        int res=ensenso_ptr_->grabRGBImage(RGBimg);
        if(!res)
        {
            ROS_ERROR("Device not open!");
            return (false);
        }
        if(was_running)
            ensenso_ptr_->start();
        return (true);
    }

    int singleRegistPC(cv::Mat& img, PointCloudXYZ::Ptr pc,bool is_pub)
    {
        bool was_running = ensenso_ptr_->isRunning();
        if(was_running)
            ensenso_ptr_->stop();

        int res=ensenso_ptr_->grabRegistImages(img,pc);
        if(!res)
        {
            ROS_ERROR("Device not open!");
            return (false);
        }
//        if(is_pub)
//        {
//            pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbPc(new pcl::PointCloud<pcl::PointXYZRGB>);
//            rgbPc->header.frame_id=pc->header.frame_id;
//            rgbPc->header.stamp=pc->header.stamp;
//            rgbPc->width=pc->width;
//            rgbPc->height=pc->height;
//            rgbPc->resize(rgbPc->width*rgbPc->height);

//            //Copy pointcloud
//            pcl::copyPointCloud(*pc,*rgbPc);
//            //Copy RGB
//            for(int i=0;i<rgbPc->height;++i)
//                for(int j=0;j<rgbPc->width;++j){
//                    rgbPc->points[i*rgbPc->width+j].r=img.at<cv::Vec3b>(i,j)[2];
//                    rgbPc->points[i*rgbPc->width+j].g=img.at<cv::Vec3b>(i,j)[1];
//                    rgbPc->points[i*rgbPc->width+j].b=img.at<cv::Vec3b>(i,j)[0];
//                }
//            sensor_msgs::PointCloud2 cloud_msg;
//            pcl::toROSMsg(*rgbPc,cloud_msg);
//            registPC_pub_.publish(cloud_msg);
//        }

//        std_msgs::Header header;
//        cv_bridge::CvImage cv_img(header,sensor_msgs::image_encodings::BGR8,img);

//        if(is_pub)
//        {
//            rgb_pub_.publish(cv_img.toImageMsg());
//        }
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

    void grabberCallback( const boost::shared_ptr<PointCloudXYZ>& cloud,
                          const boost::shared_ptr<PairOfImages>& rawimages, const boost::shared_ptr<PairOfImages>& rectifiedimages,
                          const boost::shared_ptr<cv::Mat>& rgb)
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
      //RGB
      std_msgs::Header header;
      cv_bridge::CvImage cv_img(header,sensor_msgs::image_encodings::RGB8,*rgb);
      rgb_pub_.publish(cv_img.toImageMsg());
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
      switch (config.TriggerMode)
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
      switch (config.OptimizationProfile)
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
      ROS_DEBUG_STREAM("AutoBlackLevel: "   << std::boolalpha << config.AutoBlackLevel);
      ROS_DEBUG_STREAM("AutoExposure: "     << std::boolalpha << config.AutoExposure);
      ROS_DEBUG_STREAM("AutoGain: "         << std::boolalpha << config.AutoGain);
      ROS_DEBUG_STREAM("Binning: "          << config.Binning);
      ROS_DEBUG_STREAM("BlackLevelOffset: " << config.BlackLevelOffset);
      ROS_DEBUG_STREAM("Exposure: "         << config.Exposure);
      ROS_DEBUG_STREAM("FrontLight: "       << std::boolalpha << config.FrontLight);
      ROS_DEBUG_STREAM("Gain: "             << config.Gain);
      ROS_DEBUG_STREAM("GainBoost: "        << std::boolalpha << config.GainBoost);
      ROS_DEBUG_STREAM("HardwareGamma: "    << std::boolalpha << config.HardwareGamma);
      ROS_DEBUG_STREAM("Hdr: "              << std::boolalpha << config.Hdr);
      ROS_DEBUG_STREAM("PixelClock: "       << config.PixelClock);
      ROS_DEBUG_STREAM("Projector: "        << std::boolalpha << config.Projector);
      ROS_DEBUG_STREAM("TargetBrightness: " << config.TargetBrightness);
      ROS_DEBUG_STREAM("TriggerMode: "      << trigger_mode);
      ROS_DEBUG_STREAM("DisparityMapAOI: "  << std::boolalpha << config.DisparityMapAOI);
      ROS_DEBUG("Stereo Matching Parameters");
      ROS_DEBUG_STREAM("MinimumDisparity: "     << config.MinimumDisparity);
      ROS_DEBUG_STREAM("NumberOfDisparities: "  << config.NumberOfDisparities);
      ROS_DEBUG_STREAM("OptimizationProfile: "  << profile);
      ROS_DEBUG_STREAM("Scaling: "              << config.Scaling);
      ROS_DEBUG("Advanced Matching Parameters");
      ROS_DEBUG_STREAM("DepthChangeCost: " << config.DepthChangeCost);
      ROS_DEBUG_STREAM("DepthStepCost: " << config.DepthStepCost);
      ROS_DEBUG_STREAM("ShadowingThreshold: " << config.ShadowingThreshold);
      ROS_DEBUG("Postprocessing Parameters");
      ROS_DEBUG_STREAM("UniquenessRatio: " << config.UniquenessRatio);
      ROS_DEBUG_STREAM("MedianFilterRadius: "<< config.MedianFilterRadius);
      ROS_DEBUG_STREAM("SpeckleComponentThreshold: "<< config.SpeckleComponentThreshold);
      ROS_DEBUG_STREAM("SpeckleRegionSize: "<< config.SpeckleRegionSize);
      ROS_DEBUG_STREAM("FillBorderSpread: "<< config.FillBorderSpread);
      ROS_DEBUG_STREAM("FillRegionSize: " << config.FillRegionSize);
      ROS_DEBUG("Stream Parameters");
      ROS_DEBUG_STREAM("Cloud: "   << std::boolalpha << config.Cloud);
      ROS_DEBUG_STREAM("Images: "   << std::boolalpha << config.Images);
      ROS_DEBUG("---");
      // Capture parameters
      ensenso_ptr_->setAutoBlackLevel(config.AutoBlackLevel);
      ensenso_ptr_->setAutoExposure(config.AutoExposure);
      ensenso_ptr_->setAutoGain(config.AutoGain);
      ensenso_ptr_->setBlackLevelOffset(config.BlackLevelOffset);
      ensenso_ptr_->setExposure(config.Exposure);
      ensenso_ptr_->setFrontLight(config.FrontLight);
      ensenso_ptr_->setGain(config.Gain);
      ensenso_ptr_->setGainBoost(config.GainBoost);
      ensenso_ptr_->setHardwareGamma(config.HardwareGamma);
      ensenso_ptr_->setHdr(config.Hdr);
      ensenso_ptr_->setPixelClock(config.PixelClock);
      ensenso_ptr_->setProjector(config.Projector);
      ensenso_ptr_->setTargetBrightness(config.TargetBrightness);
      ensenso_ptr_->setTriggerMode(trigger_mode);
      ensenso_ptr_->setUseDisparityMapAreaOfInterest(config.DisparityMapAOI);
      // Flexview and binning only work in 'Software' trigger mode and with the projector on
      if (trigger_mode.compare("Software") == 0 && config.Projector)
      {
        ensenso_ptr_->setBinning(config.Binning);
      }
      // Stereo parameters
      ensenso_ptr_->setMinimumDisparity(config.MinimumDisparity);
      ensenso_ptr_->setNumberOfDisparities(config.NumberOfDisparities);
      ensenso_ptr_->setOptimizationProfile(profile);
      ensenso_ptr_->setScaling(config.Scaling);
      ensenso_ptr_->setDepthChangeCost(config.DepthChangeCost);
      ensenso_ptr_->setDepthStepCost(config.DepthStepCost);
      ensenso_ptr_->setShadowingThreshold(config.ShadowingThreshold);
      //Postprocessing parameters
      ensenso_ptr_->setUniquenessRatio(config.UniquenessRatio);
      ensenso_ptr_->setMedianFilterRadius(config.MedianFilterRadius);
      ensenso_ptr_->setSpeckleComponentThreshold(config.SpeckleComponentThreshold);
      ensenso_ptr_->setSpeckleRegionSize(config.SpeckleRegionSize);
      ensenso_ptr_->setFillBorderSpread(config.FillBorderSpread);
      ensenso_ptr_->setFillRegionSize(config.FillRegionSize);
      // Streaming parameters
      configureStreaming(config.Cloud, config.Images,connect_monocular_);
    }

    bool configureStreaming(const bool cloud, const bool images=true, const bool rgb=false)
    {
      if ((is_streaming_cloud_ == cloud) && (is_streaming_images_ == images) && (is_streaming_rgb_ == rgb))
        return true;  // Nothing to be done here
      is_streaming_cloud_ = cloud;
      is_streaming_images_ = images;
      is_streaming_rgb_=rgb;
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      // Disconnect previous connection
      connection_.disconnect();
      // Connect new signals
      if(cloud && images && rgb)
      {
          boost::function<void(
            const boost::shared_ptr<PointCloudXYZ>&,
            const boost::shared_ptr<PairOfImages>&,
            const boost::shared_ptr<PairOfImages>&,
            const boost::shared_ptr<cv::Mat>&)> f = boost::bind (&ensenso_ros_driver::grabberCallback, this, _1, _2, _3,_4);
          connection_ = ensenso_ptr_->registerCallback(f);
      }
      else if (cloud && images)
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

    bool loadPCD2Mat(const string path,Mat& depMap)
    {
        PointCloudXYZ pts;
        int result=pcl::io::loadPCDFile(path,pts);
        depMap.create(pts.height,pts.width,CV_32FC1);
        int width=pts.width;
        for(int i=0;i<pts.height;i++)
            for(int j=0;j<pts.width;j++)
                depMap.at<float>(i,j)=pts.points[i*width+j].z;

        return result;
    }

    bool setParamsByJson(const string& json)
    {
        json_=json;
        if(!ensenso_ptr_->setParamsByJson(json_))
        {
            ROS_ERROR("Failed to set camera parameters by JSON file!");
            return false;
        }
    }

    bool setParamsByJson(const string& depCamJson,const string& rgbCamJson)
    {
        if(!ensenso_ptr_->setParamsByJson("Depth",depCamJson) || !ensenso_ptr_->setParamsByJson("Color",rgbCamJson))
        {
            ROS_ERROR("Failed to set camera parameters by JSON file!");
            return false;
        }
    }

};


int main(int argc, char **argv)
{
  ros::init (argc, argv, "ensenso");
  bool connect_mono=true;
  ensenso_ros_driver ensensoNode(connect_mono);
  ensensoNode.setParamsByJson("/home/yake/dep.json");
//  Mat img;
//  PointCloudXYZ::Ptr pc(new PointCloudXYZ);
//  ros::Rate loop(1);
//  while(1){
//    ensensoNode.singleRegistPC(img,pc,true);
//    loop.sleep();
//  }

  ros::spin();
  return 0;
}
