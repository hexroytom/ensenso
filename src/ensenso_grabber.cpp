#include <pcl/pcl_config.h>
#include <pcl/exceptions.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <boost/format.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include "ensenso/ensenso_grabber.h"

void ensensoExceptionHandling (const NxLibException &ex,
                 std::string func_nam)
{
  PCL_ERROR ("%s: NxLib error %s (%d) occurred while accessing item %s.\n", func_nam.c_str (), ex.getErrorText ().c_str (), ex.getErrorCode (),
         ex.getItemPath ().c_str ());
  if (ex.getErrorCode () == NxLibExecutionFailed)
  {
    NxLibCommand cmd ("");
    PCL_WARN ("\n%s\n", cmd.result ().asJson (true, 4, false).c_str ());
  }
}


pcl::EnsensoGrabber::EnsensoGrabber () :
  device_open_ (false),
  tcp_open_ (false),
  running_ (false),
  connect_monocular_(false)
{
  point_cloud_signal_ = createSignal<sig_cb_ensenso_point_cloud> ();
  images_signal_ = createSignal<sig_cb_ensenso_images> ();
  point_cloud_images_signal_ = createSignal<sig_cb_ensenso_point_cloud_images> ();
  point_cloud_images_rgb_signal_ = createSignal<sig_cb_ensenso_point_cloud_images_rgb> ();
  PCL_INFO ("Initialising nxLib\n");

  try
  {
    nxLibInitialize ();
    root_.reset (new NxLibItem);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "EnsensoGrabber");
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not initialise NxLib.");  // If constructor fails; throw exception
  }
}

pcl::EnsensoGrabber::~EnsensoGrabber () throw ()
{
  try
  {
    stop ();
    root_.reset ();

    disconnect_all_slots<sig_cb_ensenso_point_cloud> ();
    disconnect_all_slots<sig_cb_ensenso_images> ();
    disconnect_all_slots<sig_cb_ensenso_point_cloud_images> ();
    disconnect_all_slots<sig_cb_ensenso_point_cloud_images_rgb> ();

    if (tcp_open_)
      closeTcpPort ();
    nxLibFinalize ();
  }
  catch (...)
  {
    // destructor never throws
  }
}

int pcl::EnsensoGrabber::enumDevices () const
{
  int camera_count = 0;

  try
  {
    NxLibItem cams = NxLibItem ("/Cameras/BySerialNo");
    camera_count = cams.count ();

    // Print information for all cameras in the tree
    PCL_INFO ("Number of connected cameras: %d\n", camera_count);
    PCL_INFO ("Serial No    Model   Status\n");

    for (int n = 0; n < cams.count (); ++n)
    {
      PCL_INFO ("%s   %s   %s\n", cams[n][itmSerialNumber].asString ().c_str (),
            cams[n][itmModelName].asString ().c_str (),
            cams[n][itmStatus].asString ().c_str ());
    }
    PCL_INFO ("\n");
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "enumDevices");
  }
  return (camera_count);
}

bool pcl::EnsensoGrabber::openDevice (std::string serial_no)
{
  if (device_open_)
    PCL_THROW_EXCEPTION (pcl::IOException, "Cannot open multiple devices!");
  PCL_INFO ("Opening Ensenso stereo camera S/N: %s\n", serial_no.c_str());
  try
  {
    // Create a pointer referencing the camera's tree item, for easier access:
    camera_ = (*root_)[itmCameras][itmBySerialNo][serial_no];

    if (!camera_.exists () || camera_[itmType] != valStereo)
    {
      PCL_THROW_EXCEPTION (pcl::IOException, "Please connect a single stereo camera to your computer!");
    }

    NxLibCommand open (cmdOpen);
    open.parameters ()[itmCameras] = camera_[itmSerialNumber].asString ();
    open.execute ();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "openDevice");
    return (false);
  }

  device_open_ = true;
  return (true);
}

bool pcl::EnsensoGrabber::openDevice (std::string dep_serial_no,std::string rgb_serial_no)
{
  if (device_open_)
    PCL_THROW_EXCEPTION (pcl::IOException, "Cannot open multiple devices!");
  PCL_INFO ("Opening Ensenso stereo camera S/N: %s\n", dep_serial_no.c_str());
  try
  {
    // Create a pointer referencing the camera's tree item, for easier access:
    camera_ = (*root_)[itmCameras][itmBySerialNo][dep_serial_no];
    rgb_camera_ = (*root_)[itmCameras][itmBySerialNo][rgb_serial_no];

    if (!camera_.exists () || camera_[itmType] != valStereo)
    {
      PCL_THROW_EXCEPTION (pcl::IOException, "Please connect a single stereo camera to your computer!");
    }

    if (!rgb_camera_.exists () || rgb_camera_[itmType] != valMonocular)
    {
      PCL_THROW_EXCEPTION (pcl::IOException, "Please connect a single color camera to your computer!");
    }

    NxLibCommand open (cmdOpen);
    std::string camString = "[\""+camera_[itmSerialNumber].asString()+"\",\""+rgb_camera_[itmSerialNumber].asString()+"\"]";
    open.parameters()[itmCameras].setJson(camString,true);
    open.execute ();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "openDevice");
    return (false);
  }

  device_open_ = true;
  connect_monocular_ = true;
  return (true);
}

bool pcl::EnsensoGrabber::closeDevice ()
{
  if (!device_open_)
    return (false);

  stop ();
  PCL_INFO ("Closing Ensenso stereo camera\n");

  try
  {
    NxLibCommand (cmdClose).execute ();
    device_open_ = false;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "closeDevice");
    return (false);
  }
  return (true);
}

void pcl::EnsensoGrabber::start ()
{
  if (isRunning ())
    return;

  if (!device_open_)
    openDevice (0);

  //frequency_.reset ();
  running_ = true;
  grabber_thread_ = boost::thread (&pcl::EnsensoGrabber::processGrabbing, this);
}

void pcl::EnsensoGrabber::stop ()
{
  if (running_)
  {
    running_ = false;  // Stop processGrabbing () callback
    grabber_thread_.join ();
  }
}

bool pcl::EnsensoGrabber::isRunning () const
{
  return (running_);
}

bool pcl::EnsensoGrabber::isTcpPortOpen () const
{
  return (tcp_open_);
}

std::string pcl::EnsensoGrabber::getName () const
{
  return ("EnsensoGrabber");
}

bool pcl::EnsensoGrabber::configureCapture (const bool auto_exposure,
                      const bool auto_gain,
                      const int bining,
                      const float exposure,
                      const bool front_light,
                      const int gain,
                      const bool gain_boost,
                      const bool hardware_gamma,
                      const bool hdr,
                      const int pixel_clock,
                      const bool projector,
                      const int target_brightness,
                      const std::string trigger_mode,
                      const bool use_disparity_map_area_of_interest) const
{
  if (!device_open_)
    return (false);

  try
  {
    NxLibItem captureParams = camera_[itmParameters][itmCapture];
    captureParams[itmAutoExposure].set (auto_exposure);
    captureParams[itmAutoGain].set (auto_gain);
    captureParams[itmBinning].set (bining);
    captureParams[itmExposure].set (exposure);
    captureParams[itmFrontLight].set (front_light);
    captureParams[itmGain].set (gain);
    captureParams[itmGainBoost].set (gain_boost);
    captureParams[itmHardwareGamma].set (hardware_gamma);
    captureParams[itmHdr].set (hdr);
    captureParams[itmPixelClock].set (pixel_clock);
    captureParams[itmProjector].set (projector);
    captureParams[itmTargetBrightness].set (target_brightness);
    captureParams[itmTriggerMode].set (trigger_mode);
    captureParams[itmUseDisparityMapAreaOfInterest].set (use_disparity_map_area_of_interest);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "configureCapture");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::grabSingleCloud (pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  if (!device_open_)
    return (false);

  if (running_)
    return (false);

  try
  {
    NxLibCommand (cmdCapture).execute ();
    // Stereo matching task
    NxLibCommand (cmdComputeDisparityMap).execute ();
    // Convert disparity map into XYZ data for each pixel
    NxLibCommand (cmdComputePointMap).execute ();
    // Get info about the computed point map and copy it into a std::vector
    double timestamp;
    std::vector<float> pointMap;
    int width, height;
    camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);  // Get raw image timestamp
    camera_[itmImages][itmPointMap].getBinaryDataInfo (&width, &height, 0, 0, 0, 0);
    camera_[itmImages][itmPointMap].getBinaryData (pointMap, 0);
    // Copy point cloud and convert in meters
    cloud.header.stamp = getPCLStamp (timestamp);
    cloud.resize (height * width);
    cloud.width = width;
    cloud.height = height;
    cloud.is_dense = false;
    // Copy data in point cloud (and convert milimeters in meters)
    for (size_t i = 0; i < pointMap.size (); i += 3)
    {
      cloud.points[i / 3].x = pointMap[i] / 1000.0;
      cloud.points[i / 3].y = pointMap[i + 1] / 1000.0;
      cloud.points[i / 3].z = pointMap[i + 2] / 1000.0;
    }
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "grabSingleCloud");
    return (false);
  }
}

bool pcl::EnsensoGrabber::grabRGBImage(cv::Mat& image)
{
    if (!device_open_)
      return (false);

    if (running_)
      return (false);

    if(!connect_monocular_)
      return (false);

    try
    {
      NxLibCommand (cmdCapture).execute ();
      //Retrive color image
      int error=0;
      std::vector<unsigned char> color_list;
      double timeStamp;
      int width,height,channels,element_step;
      bool is_float;
      rgb_camera_[itmImages][itmRaw].getBinaryDataInfo(&width,&height,&channels,&element_step,&is_float,&timeStamp);
      rgb_camera_[itmImages][itmRaw].getBinaryData(&error,color_list,&timeStamp);
      image=cv::Mat::zeros(height,width,CV_8UC3);
      for(int i=0;i<color_list.size();i+=3){
          int r=i/(3*width);
          int c=i/3-r*width;
          image.at<cv::Vec3b>(r,c)[0]=color_list[i+2]; //B
          image.at<cv::Vec3b>(r,c)[1]=color_list[i+1]; //G
          image.at<cv::Vec3b>(r,c)[2]=color_list[i]; //R
      }
      return (true);

    }
    catch (NxLibException &ex)
    {
      ensensoExceptionHandling (ex, "grabRGBImage");
      return (false);
    }
}

bool pcl::EnsensoGrabber::grabRegistImages(cv::Mat& image,pcl::PointCloud<pcl::PointXYZ>::Ptr pc,bool rgb_or_stereo)
{
    if (!device_open_)
      return (false);

    if (running_)
      return (false);

    if(rgb_or_stereo){
    try
    {
      //Create capture object and capture images
      NxLibCommand capture(cmdCapture);
      capture.execute();
      NxLibCommand disparityMap(cmdComputeDisparityMap);
      disparityMap.execute();
      NxLibCommand pointMap(cmdComputePointMap);
      pointMap.execute();

      //Create render point map object
      NxLibCommand renderPointMap(cmdRenderPointMap);
      renderPointMap.parameters()[itmCamera]=rgb_camera_[itmSerialNumber].asString();
      renderPointMap.parameters()[itmNear]=1;
      //Use GPU to render
      (*root_)[itmParameters][itmRenderPointMap][itmUseOpenGL]=false;
      (*root_)[itmParameters][itmRenderPointMap][itmScaling]=0.6;
      renderPointMap.execute();

      //Retrive color image
      int error=0;
      double timeStamp;
      int width,height,channels,element_step;
      bool is_float;
      //Retrive color image
      try
      {
       rgb_camera_[itmImages][itmRaw].getBinaryDataInfo(&width,&height,&channels,&element_step,&is_float,&timeStamp);
       image=cv::Mat::zeros(height,width,CV_8UC3);
       rgb_camera_[itmImages][itmRaw].getBinaryData(image.data,height*width*3,0,0);
      }
      catch (NxLibException &ex)
      {
         ensensoExceptionHandling (ex, "grabColorImage");
      }

      try{
        //Retrive perspective transform depth image
        std::vector<float> depth_list;

        //get depth image and its data info
        (*root_)[itmImages][itmRenderPointMap].getBinaryDataInfo(&error,&width,&height,&channels,&element_step,&is_float,&timeStamp);
        (*root_)[itmImages][itmRenderPointMap].getBinaryData(depth_list,0);

        pc->header.stamp = getPCLStamp (timeStamp);
        pc->header.frame_id = "/camera_link";
        pc->width           = width;
        pc->height          = height;
        pc->is_dense        = false;
        pc->resize(height * width);

        // Copy data in point cloud (and convert milimeters in meters)
        for (size_t i = 0; i < depth_list.size (); i += 3) {
            pc->points[i / 3].x = depth_list[i] / 1000.0;
            pc->points[i / 3].y = depth_list[i + 1] / 1000.0;
            pc->points[i / 3].z = depth_list[i + 2] / 1000.0;
            }
        return (true);
      }
      catch(NxLibException& ex){
          ensensoExceptionHandling (ex, "grabPerspectiveTransformedPointcloud");
          return (false);
      }

    }
    catch (NxLibException &ex)
    {
      ensensoExceptionHandling (ex, "grabRegistImage");
      return (false);
    }
    }else{
        //Retrive point cloud
        try
        {
          NxLibCommand (cmdCapture).execute ();
          // Stereo matching task
          NxLibCommand (cmdComputeDisparityMap).execute ();
          // Convert disparity map into XYZ data for each pixel
          NxLibCommand (cmdComputePointMap).execute ();
          // Get info about the computed point map and copy it into a std::vector
          double timestamp;
          std::vector<float> pointMap;
          int width, height;
          camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);  // Get raw image timestamp
          camera_[itmImages][itmPointMap].getBinaryDataInfo (&width, &height, 0, 0, 0, 0);
          camera_[itmImages][itmPointMap].getBinaryData (pointMap, 0);
          // Copy point cloud and convert in meters
          pc->header.stamp = getPCLStamp (timestamp);
          pc->header.frame_id = "/camera_link";
          pc->resize (height * width);
          pc->width = width;
          pc->height = height;
          pc->is_dense = false;
          // Copy data in point cloud (and convert milimeters in meters)
          for (size_t i = 0; i < pointMap.size (); i += 3)
          {
            pc->points[i / 3].x = pointMap[i] / 1000.0;
            pc->points[i / 3].y = pointMap[i + 1] / 1000.0;
            pc->points[i / 3].z = pointMap[i + 2] / 1000.0;
          }
        }
        catch (NxLibException &ex)
        {
          ensensoExceptionHandling (ex, "grabPointCloud");
          return (false);
        }

        //Retrive left camera image
        try{
            setProjector(false);
            setFrontLight(true);
            NxLibCommand (cmdCapture).execute ();
            //Rectify raw image
            NxLibCommand rectify(cmdRectifyImages);
            rectify.execute();

            double timeStamp;
            int width,height,channels,element_step;
            bool is_float;
            camera_[itmImages][itmRectified][itmLeft].getBinaryDataInfo(&width,&height,&channels,&element_step,&is_float,&timeStamp);
            image=cv::Mat::zeros(height,width,CV_8UC1);
            camera_[itmImages][itmRectified][itmLeft].getBinaryData(image.data,height*width,0,0);
            //Restore setting
            setProjector(true);
            setFrontLight(false);
            return (true);
        }
        catch(NxLibException &ex)
        {
            ensensoExceptionHandling (ex, "grabLeftCameraImage");
            return (false);
        }
    }
}

bool pcl::EnsensoGrabber::initExtrinsicCalibration (const double grid_spacing) const
{
  if (!device_open_)
    return (false);

  if (running_)
    return (false);

  try
  {
    if (!clearCalibrationPatternBuffer ())
      return (false);
    (*root_)[itmParameters][itmPattern][itmGridSpacing].set (grid_spacing);  // GridSize can't be changed, it's protected in the tree
    // With the speckle projector on it is nearly impossible to recognize the pattern
    // (the 3D calibration is based on stereo images, not on 3D depth map)
    
    // Most important parameters are: projector=off, front_light=on
    //configureCapture (true, true, 1, 0.32, true, 1, false, false, false, 10, false, 80, "Software", false);
    setProjector(false);
    setFrontLight(true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "initExtrinsicCalibration");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::clearCalibrationPatternBuffer () const
{
  if (!device_open_)
    return (false);

  if (running_)
    return (false);
  try
  {
    NxLibCommand (cmdDiscardPatterns).execute ();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "clearCalibrationPatternBuffer");
    return (false);
  }
  return (true);
}

int pcl::EnsensoGrabber::captureCalibrationPattern () const
{
  if (!device_open_)
    return (-1);

  if (running_)
    return (-1);

  try
  {
    NxLibCommand (cmdCapture).execute ();
    NxLibCommand collect_pattern (cmdCollectPattern);
    collect_pattern.parameters ()[itmBuffer].set (true);  // Store the pattern into the buffer
    collect_pattern.execute ();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "captureCalibrationPattern");
    return (-1);
  }

  return ( (*root_)[itmParameters][itmPatternCount].asInt ());
}

bool pcl::EnsensoGrabber::estimateCalibrationPatternPose (Eigen::Affine3d &pattern_pose, const bool average) const
{
  if (!device_open_)
    return (false);

  if (running_)
    return (false);

  try
  {
    NxLibCommand estimate_pattern_pose (cmdEstimatePatternPose);
    estimate_pattern_pose.parameters ()[itmAverage].set (average);
    
    estimate_pattern_pose.execute ();
    NxLibItem tf = estimate_pattern_pose.result ()[itmPatternPose];
    // Convert tf into a matrix
    if (!jsonTransformationToMatrix (tf.asJson (), pattern_pose))
      return (false);
    pattern_pose.translation () /= 1000.0;  // Convert translation in meters (Ensenso API returns milimeters)
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "estimateCalibrationPatternPoses");
    return (false);
  }
}

bool pcl::EnsensoGrabber::computeCalibrationMatrix (const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &robot_poses,
                          std::string &json,
                          int &iterations,
                          double &reprojection_error,
                          const std::string setup,
                          const std::string target,
                          const Eigen::Affine3d &guess_tf,
                          const bool pretty_format
                          ) const
{
  if ( (*root_)[itmVersion][itmMajor] < 2 && (*root_)[itmVersion][itmMinor] < 3)
    PCL_WARN ("EnsensoSDK 1.3.x fixes bugs into extrinsic calibration matrix optimization, please update your SDK!\n"
          "http://www.ensenso.de/support/sdk-download/\n");
  
  NxLibCommand calibrate (cmdCalibrateHandEye);
  try
  {
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > robot_poses_mm (robot_poses);
    std::vector<std::string> robot_poses_json;
    robot_poses_json.resize (robot_poses.size ());
    for (uint i = 0; i < robot_poses_json.size (); ++i)
    {
      robot_poses_mm[i].translation () *= 1000.0; // Convert meters in millimeters
      if (!matrixTransformationToJson (robot_poses_mm[i], robot_poses_json[i]))
        return (false);
    }
    // Set Hand-Eye calibration parameters
    if (boost::iequals (setup, "Fixed"))
      calibrate.parameters ()[itmSetup].set (valFixed);
    else
      calibrate.parameters ()[itmSetup].set (valMoving);
    calibrate.parameters ()[itmTarget].set (target);

    // Set guess matrix
    if (guess_tf.matrix () != Eigen::Matrix4d::Identity ())
    {
      // Matrix > JSON > Angle axis
      NxLibItem tf ("/tmpTF");
      if (!matrixTransformationToJson (guess_tf, json))
        return (false);
      tf.setJson (json);

      // Rotation
      double theta = tf[itmRotation][itmAngle].asDouble ();  // Angle of rotation
      double x = tf[itmRotation][itmAxis][0].asDouble ();   // X component of Euler vector
      double y = tf[itmRotation][itmAxis][1].asDouble ();   // Y component of Euler vector
      double z = tf[itmRotation][itmAxis][2].asDouble ();   // Z component of Euler vector
      tf.erase(); // Delete tmpTF node
      
      calibrate.parameters ()[itmLink][itmRotation][itmAngle].set (theta);
      calibrate.parameters ()[itmLink][itmRotation][itmAxis][0].set (x);
      calibrate.parameters ()[itmLink][itmRotation][itmAxis][1].set (y);
      calibrate.parameters ()[itmLink][itmRotation][itmAxis][2].set (z);
      // Translation
      calibrate.parameters ()[itmLink][itmTranslation][0].set (guess_tf.translation ()[0] * 1000.0);
      calibrate.parameters ()[itmLink][itmTranslation][1].set (guess_tf.translation ()[1] * 1000.0);
      calibrate.parameters ()[itmLink][itmTranslation][2].set (guess_tf.translation ()[2] * 1000.0);
    }

    // Feed all robot poses into the calibration command
    for (uint i = 0; i < robot_poses_json.size (); ++i)
    {
      // Very weird behavior here:
      // If you modify this loop, check that all the transformations are still here in the [itmExecute][itmParameters] node
      // because for an unknown reason sometimes the old transformations are erased in the tree ("null" in the tree)
      // Ensenso SDK 2.3.348: If not moved after guess calibration matrix, the vector is empty.
      calibrate.parameters ()[itmTransformations][i].setJson (robot_poses_json[i], false);
    }

    calibrate.execute ();  // Might take up to 120 sec (maximum allowed by Ensenso API)

    if (calibrate.successful())
    {
      json = calibrate.result()[itmLink].asJson (pretty_format);
      iterations = calibrate.result()[itmIterations].asInt();
      reprojection_error = calibrate.result()[itmReprojectionError].asDouble();
      ROS_INFO("computeCalibrationMatrix succeeded. Iterations: %d, Reprojection error: %.2f", iterations, reprojection_error);
      ROS_INFO_STREAM("Result: " << std::endl << json);
      return (true);
    }
    else
    {
      json.clear ();
      return (false);
    }
  }
  catch (NxLibException &ex)
  {
    try
    {
      int iters = calibrate.result()[itmIterations].asInt();
      double error = calibrate.result()[itmReprojectionError].asDouble();
      ROS_WARN("computeCalibrationMatrix Failed. Iterations: %d, Reprojection error: %.2f", iters, error);
      ROS_WARN_STREAM("Result: " << std::endl << calibrate.result()[itmLink].asJson(true));
      return (false);
    }
    catch (...) {
      ensensoExceptionHandling (ex, "computeCalibrationMatrix");
    } 
    
  }
}

bool pcl::EnsensoGrabber::storeEEPROMExtrinsicCalibration () const
{
  try
  {
    NxLibCommand store (cmdStoreCalibration);
    store.execute ();
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "storeEEPROMExtrinsicCalibration");
    return (false);
  }
}

bool pcl::EnsensoGrabber::loadEEPROMExtrinsicCalibration () const
{
  try
  {
    NxLibCommand load (cmdLoadCalibration);
    load.parameters ()[itmCameras] = camera_[itmSerialNumber].asString ();
    load.execute ();
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "loadEEPROMExtrinsicCalibration");
    return (false);
  }
}


bool pcl::EnsensoGrabber::clearEEPROMExtrinsicCalibration ()
{
  try
  {
    setExtrinsicCalibration("");
    NxLibCommand store (cmdStoreCalibration);
    store.execute ();
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "clearEEPROMExtrinsicCalibration");
    return (false);
  }
}

bool pcl::EnsensoGrabber::setAutoBlackLevel (const bool enable) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmAutoBlackLevel].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setAutoBlackLevel");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setAutoExposure (const bool enable) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmAutoExposure].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setAutoExposure");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setAutoGain (const bool enable) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmAutoGain].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setAutoGain");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setBinning (const int binning) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmBinning].set (binning);
  }
  catch (NxLibException &ex)
  {
    // TODO: Handle better setBinning exceptions
    //~ ensensoExceptionHandling (ex, "setBinning");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setBlackLevelOffset (const float offset) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmBlackLevelOffset].set (offset);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setBlackLevelOffset");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setExposure (const float exposure) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmExposure].set (exposure);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setExposure");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setFrontLight (const bool enable) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmFrontLight].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setFrontLight");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setGain (const float gain) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmGain].set (gain);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setGain");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setGainBoost (const bool enable) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmGainBoost].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setGainBoost");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setGridSpacing (const double grid_spacing) const
{
if (!device_open_)
    return (false);
  try
  {
    (*root_)[itmParameters][itmPattern][itmGridSpacing].set (grid_spacing);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setGridSpacing");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setHardwareGamma (const bool enable) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmHardwareGamma].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setHardwareGamma");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setHdr (const bool enable) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmHdr].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setHdr");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setMinimumDisparity (const int disparity) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmMinimumDisparity].set (disparity);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setMinimumDisparity");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setNumberOfDisparities (const int number) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmNumberOfDisparities].set (number);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "NumberOfDisparities");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setOptimizationProfile (const std::string profile) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmOptimizationProfile].set (profile);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setOptimizationProfile");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setPixelClock (const int pixel_clock) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmPixelClock].set (pixel_clock);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setPixelClock");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setProjector (const bool enable) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmProjector].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setProjector");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setTargetBrightness (const int target) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmTargetBrightness].set (target);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setTargetBrightness");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setScaling (const float scaling) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmScaling].set (scaling);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setScaling");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setTriggerMode (const std::string mode) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmTriggerMode].set (mode);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setTriggerMode");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setUseDisparityMapAreaOfInterest (const bool enable) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmUseDisparityMapAreaOfInterest].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setUseDisparityMapAreaOfInterest");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setDepthChangeCost(const int changecost) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmDepthChangeCost].set (changecost);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setDepthChangeCost");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setDepthStepCost(const int stepcost) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmDepthStepCost].set (stepcost);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setDepthStepCost");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setShadowingThreshold(const int shadowingthreshold) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmShadowingThreshold].set (shadowingthreshold);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setShadowingThreshold");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setUniquenessRatio(const int ratio) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmPostProcessing][itmUniquenessRatio].set (ratio);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setUniquenessRatio");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setMedianFilterRadius(const int radius) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmPostProcessing][itmMedianFilterRadius].set (radius);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setMedianFilterRadius");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setSpeckleComponentThreshold(const int threshold) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmPostProcessing][itmSpeckleRemoval][itmComponentThreshold].set (threshold);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setComponentThreshold");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setSpeckleRegionSize(const int regionsize) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmPostProcessing][itmSpeckleRemoval][itmRegionSize].set (regionsize);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setRegionSize");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setFillBorderSpread(const int maximumspread) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmPostProcessing][itmFilling][itmBorderSpread].set (maximumspread);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setBorderSpread");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setFillRegionSize(const int regionsize) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmPostProcessing][itmFilling][itmRegionSize].set (regionsize);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setRegionSize");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setExtrinsicCalibration (const double euler_angle,
                           Eigen::Vector3d &rotation_axis,
                           const Eigen::Vector3d &translation,
                           const std::string target)
{
  if (!device_open_)
    return (false);

  if (rotation_axis != Eigen::Vector3d (0, 0, 0))
    rotation_axis.normalize ();

  try
  {
    NxLibItem calibParams = camera_[itmLink];
    calibParams[itmTarget].set (target);
    calibParams[itmRotation][itmAngle].set (euler_angle);
    
    calibParams[itmRotation][itmAxis][0].set (rotation_axis[0]);
    calibParams[itmRotation][itmAxis][1].set (rotation_axis[1]);
    calibParams[itmRotation][itmAxis][2].set (rotation_axis[2]);
    
    calibParams[itmTranslation][0].set (translation[0] * 1000.0);  // Convert in millimeters
    calibParams[itmTranslation][1].set (translation[1] * 1000.0);
    calibParams[itmTranslation][2].set (translation[2] * 1000.0);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setExtrinsicCalibration");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setExtrinsicCalibration (const std::string target)
{
  if (!device_open_)
    return (false);
  
  Eigen::Vector3d rotation (Eigen::Vector3d::Zero ());
  Eigen::Vector3d translation (Eigen::Vector3d::Zero ());
  return (setExtrinsicCalibration (0.0, rotation, translation, target));
}

bool pcl::EnsensoGrabber::setExtrinsicCalibration (const Eigen::Affine3d &transformation,
                           const std::string target)
{
  std::string json;
  if (!matrixTransformationToJson (transformation, json))
    return (false);
  
  double euler_angle;
  Eigen::Vector3d rotation_axis;
  Eigen::Vector3d translation;

  if (!jsonTransformationToAngleAxis (json, euler_angle, rotation_axis, translation))
    return (false);

  return (setExtrinsicCalibration (euler_angle, rotation_axis, translation, target));
}

float pcl::EnsensoGrabber::getFramesPerSecond () const
{
  boost::mutex::scoped_lock lock (fps_mutex_);
  //return (frequency_.getFrequency ());
  return (0);
}

bool pcl::EnsensoGrabber::openTcpPort (const int port)
{
  try
  {
    nxLibOpenTcpPort (port);
    tcp_open_ = true;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "openTcpPort");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::closeTcpPort ()
{
  try
  {
    nxLibCloseTcpPort ();
    tcp_open_ = false;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "closeTcpPort");
    return (false);
  }
  return (true);
}

std::string pcl::EnsensoGrabber::getTreeAsJson (const bool pretty_format) const
{
  try
  {
    return (root_->asJson (pretty_format));
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "getTreeAsJson");
    return ("");
  }
}

std::string pcl::EnsensoGrabber::getResultAsJson (const bool pretty_format) const
{
  try
  {
    NxLibCommand cmd ("");
    return (cmd.result ().asJson (pretty_format));
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "getResultAsJson");
    return ("");
  }
}

double pcl::EnsensoGrabber::getPatternGridSpacing () const
{
  if (!device_open_)
    return (-1);
  if (running_)
    return (-1);
  NxLibCommand collect_pattern (cmdCollectPattern);
  try
  {
    NxLibCommand (cmdCapture).execute ();
    collect_pattern.parameters ()[itmBuffer].set (false);
    collect_pattern.parameters ()[itmDecodeData].set (true);
    collect_pattern.execute ();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "getPatternGridSpacing");
    return (-1);
  }

  return collect_pattern.result()[itmGridSpacing].asDouble();
}

bool pcl::EnsensoGrabber::enableFrontLight(const bool enable) const
{
  try
  {
    camera_[itmParameters][itmCapture][itmFrontLight].set(enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling(ex, "enableFrontLight");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::enableProjector(const bool enable) const
{
  try
  {
    camera_[itmParameters][itmCapture][itmProjector].set(enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling(ex, "enableProjector");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::getCameraInfo(std::string cam, sensor_msgs::CameraInfo &cam_info) const
{
  try
  {
    cam_info.width = camera_[itmSensor][itmSize][0].asInt();
    cam_info.height = camera_[itmSensor][itmSize][1].asInt();
    cam_info.distortion_model = "plumb_bob";
    // Distorsion factors
    cam_info.D.resize(5);
    for(std::size_t i = 0; i < cam_info.D.size(); ++i)
      cam_info.D[i] = camera_[itmCalibration][itmMonocular][cam][itmDistortion][i].asDouble();
    // K and R matrices
    for(std::size_t i = 0; i < 3; ++i)
    {
      for(std::size_t j = 0; j < 3; ++j)
      {
        cam_info.K[3*i+j] = camera_[itmCalibration][itmMonocular][cam][itmCamera][j][i].asDouble();
        cam_info.R[3*i+j] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmRotation][j][i].asDouble();
      }
    }
    cam_info.P[0] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][0][0].asDouble();
    cam_info.P[1] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][1][0].asDouble();
    cam_info.P[2] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][2][0].asDouble();
    cam_info.P[3] = 0.0;
    cam_info.P[4] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][0][1].asDouble();
    cam_info.P[5] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][1][1].asDouble();
    cam_info.P[6] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][2][1].asDouble();
    cam_info.P[7] = 0.0;
    cam_info.P[10] = 1.0;
    if (cam == "Right")
    {
      double B = camera_[itmCalibration][itmStereo][itmBaseline].asDouble() / 1000.0;
      double fx = cam_info.P[0];
      cam_info.P[3] = (-fx * B);
    }
    return true;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "getCameraInfo");
    return false;
  }
}



bool pcl::EnsensoGrabber::jsonTransformationToEulerAngles (const std::string &json,
                               double &x,
                               double &y,
                               double &z,
                               double &w,
                               double &p,
                               double &r) const
{
  try
  {
    NxLibCommand convert (cmdConvertTransformation);
    convert.parameters ()[itmTransformation].setJson (json, false);
    convert.parameters ()[itmSplitRotation].set (valXYZ);
    convert.execute ();
    
    NxLibItem tf = convert.result ()[itmTransformations];
    x = tf[0][itmTranslation][0].asDouble ();
    y = tf[0][itmTranslation][1].asDouble ();
    z = tf[0][itmTranslation][2].asDouble ();
    r = tf[0][itmRotation][itmAngle].asDouble ();  // Roll
    p = tf[1][itmRotation][itmAngle].asDouble ();  // Pitch
    w = tf[2][itmRotation][itmAngle].asDouble ();  // yaW
    return (true);
  }

  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "jsonTransformationToEulerAngles");
    return (false);
  }
}

bool pcl::EnsensoGrabber::jsonTransformationToAngleAxis (const std::string json,
                             double &alpha,
                             Eigen::Vector3d &axis,
                             Eigen::Vector3d &translation) const
{
  try
  {
    NxLibItem tf ("/tmpTF");
    tf.setJson(json);
    translation[0] = tf[itmTranslation][0].asDouble ();
    translation[1] = tf[itmTranslation][1].asDouble ();
    translation[2] = tf[itmTranslation][2].asDouble ();
    
    alpha = tf[itmRotation][itmAngle].asDouble ();  // Angle of rotation
    axis[0] = tf[itmRotation][itmAxis][0].asDouble ();  // X component of Euler vector
    axis[1] = tf[itmRotation][itmAxis][1].asDouble ();  // Y component of Euler vector
    axis[2] = tf[itmRotation][itmAxis][2].asDouble ();  // Z component of Euler vector
    tf.erase(); // Delete tmpTF node
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "jsonTransformationToAngleAxis");
    return (false);
  }
}

bool pcl::EnsensoGrabber::jsonTransformationToMatrix (const std::string transformation,
                            Eigen::Affine3d &matrix) const
{
  try
  {
    NxLibCommand convert_transformation (cmdConvertTransformation);
    convert_transformation.parameters ()[itmTransformation].setJson (transformation);
    convert_transformation.execute ();
    Eigen::Affine3d tmp (Eigen::Affine3d::Identity ());
    
    // Rotation
    tmp.linear ().col (0) = Eigen::Vector3d (convert_transformation.result ()[itmTransformation][0][0].asDouble (),
                         convert_transformation.result ()[itmTransformation][0][1].asDouble (),
                         convert_transformation.result ()[itmTransformation][0][2].asDouble ());

    tmp.linear ().col (1) = Eigen::Vector3d (convert_transformation.result ()[itmTransformation][1][0].asDouble (),
                         convert_transformation.result ()[itmTransformation][1][1].asDouble (),
                         convert_transformation.result ()[itmTransformation][1][2].asDouble ());

    tmp.linear ().col (2) = Eigen::Vector3d (convert_transformation.result ()[itmTransformation][2][0].asDouble (),
                         convert_transformation.result ()[itmTransformation][2][1].asDouble (),
                         convert_transformation.result ()[itmTransformation][2][2].asDouble ());

    // Translation
    tmp.translation () = Eigen::Vector3d (convert_transformation.result ()[itmTransformation][3][0].asDouble (),
                        convert_transformation.result ()[itmTransformation][3][1].asDouble (),
                        convert_transformation.result ()[itmTransformation][3][2].asDouble ());
    matrix = tmp;
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "jsonTransformationToMatrix");
    return (false);
  }
}

bool pcl::EnsensoGrabber::eulerAnglesTransformationToJson (const double x,
                               const double y,
                               const double z,
                               const double w,
                               const double p,
                               const double r,
                               std::string &json,
                               const bool pretty_format) const
{
  try
  {
    NxLibCommand chain (cmdChainTransformations);
    NxLibItem tf = chain.parameters ()[itmTransformations];
    
    if (!angleAxisTransformationToJson (x, y, z, 0, 0, 1, r, json))
      return (false);
    tf[0].setJson (json, false);  // Roll
    
    if (!angleAxisTransformationToJson (0, 0, 0, 0, 1, 0, p, json))
      return (false);
    tf[1].setJson (json, false);  // Pitch
    
    if (!angleAxisTransformationToJson (0, 0, 0, 1, 0, 0, w, json))
      return (false);
    tf[2].setJson (json, false);  // yaW
    
    chain.execute ();
    json = chain.result ()[itmTransformation].asJson (pretty_format);
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "eulerAnglesTransformationToJson");
    return (false);
  }
}

bool pcl::EnsensoGrabber::angleAxisTransformationToJson (const double x,
                             const double y,
                             const double z,
                             const double rx,
                             const double ry,
                             const double rz,
                             const double alpha,
                             std::string &json,
                             const bool pretty_format) const
{
  try
  {
    NxLibItem tf ("/tmpTF");
    tf[itmTranslation][0].set (x);
    tf[itmTranslation][1].set (y);
    tf[itmTranslation][2].set (z);
    
    tf[itmRotation][itmAngle].set (alpha);  // Angle of rotation
    tf[itmRotation][itmAxis][0].set (rx);  // X component of Euler vector
    tf[itmRotation][itmAxis][1].set (ry);  // Y component of Euler vector
    tf[itmRotation][itmAxis][2].set (rz);  // Z component of Euler vector
    
    json = tf.asJson (pretty_format);
    tf.erase ();
    return (true);
  }

  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "angleAxisTransformationToJson");
    return (false);
  }
}

bool pcl::EnsensoGrabber::matrixTransformationToJson (const Eigen::Affine3d &matrix,
                            std::string &json,
                            const bool pretty_format) const
{
  try
  {
    NxLibCommand convert (cmdConvertTransformation);
    
    // Rotation
    convert.parameters ()[itmTransformation][0][0].set (matrix.linear ().col (0)[0]);
    convert.parameters ()[itmTransformation][0][1].set (matrix.linear ().col (0)[1]);
    convert.parameters ()[itmTransformation][0][2].set (matrix.linear ().col (0)[2]);
    convert.parameters ()[itmTransformation][0][3].set (0.0);
    
    convert.parameters ()[itmTransformation][1][0].set (matrix.linear ().col (1)[0]);
    convert.parameters ()[itmTransformation][1][1].set (matrix.linear ().col (1)[1]);
    convert.parameters ()[itmTransformation][1][2].set (matrix.linear ().col (1)[2]);
    convert.parameters ()[itmTransformation][1][3].set (0.0);
    
    convert.parameters ()[itmTransformation][2][0].set (matrix.linear ().col (2)[0]);
    convert.parameters ()[itmTransformation][2][1].set (matrix.linear ().col (2)[1]);
    convert.parameters ()[itmTransformation][2][2].set (matrix.linear ().col (2)[2]);
    convert.parameters ()[itmTransformation][2][3].set (0.0);
    
    // Translation
    convert.parameters ()[itmTransformation][3][0].set (matrix.translation ()[0]);
    convert.parameters ()[itmTransformation][3][1].set (matrix.translation ()[1]);
    convert.parameters ()[itmTransformation][3][2].set (matrix.translation ()[2]);
    convert.parameters ()[itmTransformation][3][3].set (1.0);
    
    convert.execute ();
    json = convert.result ()[itmTransformation].asJson (pretty_format);
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "matrixTransformationToJson");
    return (false);
  }
}

pcl::uint64_t pcl::EnsensoGrabber::getPCLStamp (const double ensenso_stamp)
{
#if defined _WIN32 || defined _WIN64
  return (ensenso_stamp * 1000000.0);
#else
  return ( (ensenso_stamp - 11644473600.0) * 1000000.0);
#endif
}

std::string pcl::EnsensoGrabber::getOpenCVType (const int channels,
                        const int bpe,
                        const bool isFlt)
{
  int bits = bpe * 8;
  char type = isFlt ? 'F' : (bpe > 3 ? 'S' : 'U');
  return (boost::str (boost::format ("CV_%i%cC%i") % bits % type % channels));
}

void pcl::EnsensoGrabber::processGrabbing ()
{
  bool continue_grabbing = running_;
  while (continue_grabbing)
  {
    try
    {
      // Publish cloud / images
      if (num_slots<sig_cb_ensenso_point_cloud> () > 0 || num_slots<sig_cb_ensenso_images> () > 0 || num_slots<sig_cb_ensenso_point_cloud_images> () > 0 || num_slots<sig_cb_ensenso_point_cloud_images_rgb> () > 0)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        boost::shared_ptr<PairOfImages> rawimages (new PairOfImages);
        boost::shared_ptr<PairOfImages> rectifiedimages (new PairOfImages);
        boost::shared_ptr<cv::Mat> rgb (new cv::Mat);
        fps_mutex_.lock ();
        //frequency_.event ();
        fps_mutex_.unlock ();
        
        NxLibCommand (cmdCapture).execute ();
        double timestamp;
        camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);

        // Gather images
        if (num_slots<sig_cb_ensenso_images> () > 0 || num_slots<sig_cb_ensenso_point_cloud_images> () > 0 ||  num_slots<sig_cb_ensenso_point_cloud_images_rgb> () > 0)
        {
          // Rectify images
          NxLibCommand (cmdRectifyImages).execute ();
          int width, height, channels, bpe;
          bool isFlt, collected_pattern = false;
          
          try  // Try to collect calibration pattern, if not possible, publish RAW images and Rectified images instead
          {
            NxLibCommand collect_pattern (cmdCollectPattern);
            collect_pattern.parameters ()[itmBuffer].set (false);  // Do NOT store the pattern into the buffer!
            collect_pattern.execute ();
            collected_pattern = true;
          }
          catch (const NxLibException &ex)
          {
            // We failed to collect the pattern but the RAW images are available!
          }

          if (collected_pattern)
          {
            camera_[itmImages][itmWithOverlay][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
            rawimages->first.header.stamp = rawimages->second.header.stamp = getPCLStamp (timestamp);
            rawimages->first.width = rawimages->second.width = width;
            rawimages->first.height = rawimages->second.height = height;
            rawimages->first.data.resize (width * height * sizeof(float));
            rawimages->second.data.resize (width * height * sizeof(float));
            rawimages->first.encoding = rawimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
            camera_[itmImages][itmWithOverlay][itmLeft].getBinaryData (rawimages->first.data.data (), rawimages->first.data.size (), 0, 0);
            camera_[itmImages][itmWithOverlay][itmRight].getBinaryData (rawimages->second.data.data (), rawimages->second.data.size (), 0, 0);
            // rectifiedimages
            camera_[itmImages][itmRectified][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
            rectifiedimages->first.width = rectifiedimages->second.width = width;
            rectifiedimages->first.height = rectifiedimages->second.height = height;
            rectifiedimages->first.data.resize (width * height * sizeof(float));
            rectifiedimages->second.data.resize (width * height * sizeof(float));
            rectifiedimages->first.encoding = rectifiedimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
            camera_[itmImages][itmRectified][itmLeft].getBinaryData (rectifiedimages->first.data.data (), rectifiedimages->first.data.size (), 0, 0);
            camera_[itmImages][itmRectified][itmRight].getBinaryData (rectifiedimages->second.data.data (), rectifiedimages->second.data.size (), 0, 0);
          }
          else
          {
            camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
            rawimages->first.header.stamp = rawimages->second.header.stamp = getPCLStamp (timestamp);
            rawimages->first.width = rawimages->second.width = width;
            rawimages->first.height = rawimages->second.height = height;
            rawimages->first.data.resize (width * height * sizeof(float));
            rawimages->second.data.resize (width * height * sizeof(float));
            rawimages->first.encoding = rawimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
            camera_[itmImages][itmRaw][itmLeft].getBinaryData (rawimages->first.data.data (), rawimages->first.data.size (), 0, 0);
            camera_[itmImages][itmRaw][itmRight].getBinaryData (rawimages->second.data.data (), rawimages->second.data.size (), 0, 0);
            // rectifiedimages
            camera_[itmImages][itmRectified][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
            rectifiedimages->first.width = rectifiedimages->second.width = width;
            rectifiedimages->first.height = rectifiedimages->second.height = height;
            rectifiedimages->first.data.resize (width * height * sizeof(float));
            rectifiedimages->second.data.resize (width * height * sizeof(float));
            rectifiedimages->first.encoding = rectifiedimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
            camera_[itmImages][itmRectified][itmLeft].getBinaryData (rectifiedimages->first.data.data (), rectifiedimages->first.data.size (), 0, 0);
            camera_[itmImages][itmRectified][itmRight].getBinaryData (rectifiedimages->second.data.data (), rectifiedimages->second.data.size (), 0, 0);
          }

        // Gather point cloud
        if (num_slots<sig_cb_ensenso_point_cloud> () > 0 || num_slots<sig_cb_ensenso_point_cloud_images> () > 0 || num_slots<sig_cb_ensenso_point_cloud_images_rgb> () >0)
        {
          // Stereo matching task
          NxLibCommand (cmdComputeDisparityMap).execute ();
          // Convert disparity map into XYZ data for each pixel
          NxLibCommand (cmdComputePointMap).execute ();
          // Get info about the computed point map and copy it into a std::vector
          std::vector<float> pointMap;
          int width, height;
          camera_[itmImages][itmPointMap].getBinaryDataInfo (&width, &height, 0, 0, 0, 0);
          camera_[itmImages][itmPointMap].getBinaryData (pointMap, 0);
          // Copy point cloud and convert in meters
          cloud->header.stamp = getPCLStamp (timestamp);
          cloud->points.resize (height * width);
          cloud->width = width;
          cloud->height = height;
          cloud->is_dense = false;
          // Copy data in point cloud (and convert milimeters in meters)
          for (size_t i = 0; i < pointMap.size (); i += 3)
          {
            cloud->points[i / 3].x = pointMap[i] / 1000.0;
            cloud->points[i / 3].y = pointMap[i + 1] / 1000.0;
            cloud->points[i / 3].z = pointMap[i + 2] / 1000.0;
          }
        }

        //Gather RGB image of monocular camera
        if(num_slots<sig_cb_ensenso_point_cloud_images_rgb> () >0)
        {   //Retrive color image
            try
            {
              int error=0;
              std::vector<unsigned char> color_list;
              double timeStamp;
              rgb_camera_[itmImages][itmRaw].getBinaryDataInfo(&width,&height,&channels,&bpe,&isFlt,&timeStamp);
              *rgb=cv::Mat::zeros(height,width,CV_8UC3);
              rgb_camera_[itmImages][itmRaw].getBinaryData(rgb->data,height*width*3,0,0);

            }
            catch (NxLibException &ex)
            {
              ensensoExceptionHandling (ex, "grabRGBImage");
            }
          }
            //Retreive perspective transformed point cloud
                //...
        }
        // Publish signals
        if (num_slots<sig_cb_ensenso_point_cloud_images_rgb> () > 0)
          point_cloud_images_rgb_signal_->operator () (cloud, rawimages, rectifiedimages,rgb);
        else if (num_slots<sig_cb_ensenso_point_cloud_images> () > 0)
          point_cloud_images_signal_->operator () (cloud, rawimages, rectifiedimages);
        else if (num_slots<sig_cb_ensenso_point_cloud> () > 0)
          point_cloud_signal_->operator () (cloud);
        else if (num_slots<sig_cb_ensenso_images> () > 0)
          images_signal_->operator () (rawimages,rectifiedimages);
      }
      continue_grabbing = running_;
    }
    catch (NxLibException &ex)
    {
      ensensoExceptionHandling (ex, "processGrabbing");
    }
  }
}

bool pcl::EnsensoGrabber::setParamsByJson(const std::string json)
{
    std::ifstream file(json.c_str(),std::ifstream::in);
    if(file.is_open()&&file.rdbuf()){
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string const& filecontent = buffer.str();

        NxLibItem tmp("/home/yake/Temp");
        tmp.setJson(filecontent);
        if(tmp[itmParameters].exists()){
            camera_[itmParameters].setJson(tmp[itmParameters].asJson(),true);
            return true;
        }else{
            camera_[itmParameters].setJson(tmp.asJson(),true);
            return true;
        }

    }else{
        PCL_ERROR("Can not read JSON file!");
        return false;
    }


}

bool pcl::EnsensoGrabber::setParamsByJson(const std::string& camType,const std::string& json)
{
    if(camType == "Depth"){
    std::ifstream file(json.c_str(),std::ifstream::in);
    if(file.is_open()&&file.rdbuf()){
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string const& filecontent = buffer.str();

        NxLibItem tmp("/home/yake/Temp");
        tmp.setJson(filecontent);
        if(tmp[itmParameters].exists()){
            camera_[itmParameters].setJson(tmp[itmParameters].asJson(),true);
            return true;
        }else{
            camera_[itmParameters].setJson(tmp.asJson(),true);
            return true;
        }

    }else{
        PCL_ERROR("Can not read JSON file!");
        return false;
    }
    }else if(camType == "Color"){
        std::ifstream file(json.c_str(),std::ifstream::in);
        if(file.is_open()&&file.rdbuf()){
            std::stringstream buffer;
            buffer << file.rdbuf();
            std::string const& filecontent = buffer.str();

            NxLibItem tmp("/home/yake/Temp");
            tmp.setJson(filecontent);
            if(tmp[itmParameters].exists()){
                rgb_camera_[itmParameters].setJson(tmp[itmParameters].asJson(),true);
                return true;
            }else{
                rgb_camera_[itmParameters].setJson(tmp.asJson(),true);
                return true;
            }

        }else{
            PCL_ERROR("Can not read JSON file!");
            return false;
        }
    }


}

int pcl::EnsensoGrabber::patternExistedtCount()
{
    return ((*root_)[itmParameters][itmPatternCount].asInt());
}

void pcl::EnsensoGrabber::getMonoCalParams(std::string& path)
{
    std::ofstream file(path.c_str());
    try{
        if(file.is_open()){
            file<<rgb_camera_[itmCalibration][itmCamera].asJson(true);
            file<<rgb_camera_[itmCalibration][itmDistortion].asJson(true);
        }
        file.close();
    }
    catch(NxLibException &ex)
    {
        ensensoExceptionHandling(ex,"getMonoCalParams");
    }
}
