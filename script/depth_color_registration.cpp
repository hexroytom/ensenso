#include <ensenso/ensenso_grabber.h>
#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;



int main()
{
    nxLibInitialize();
    nxLibOpenTcpPort(24000);

    //NxLibItem cams = NxLibItem("/Cameras/BySerialNo");
    NxLibItem root;
    NxLibItem depCam = root[itmCameras][itmBySerialNo]["160824"];
    NxLibItem rgbCam = root[itmCameras][itmBySerialNo]["4102849778"];

    //Open both cameras
    string camString = "[\""+depCam[itmSerialNumber].asString()+"\",\""+rgbCam[itmSerialNumber].asString()+"\"]";
    NxLibCommand open(cmdOpen);
    open.parameters()[itmCameras].setJson(camString,true);
    open.execute();

    //read json file
    std::ifstream file("/home/yake/rgb.json");
    if (file.is_open() && file.rdbuf()) {
       // You can use the std::stringstream class to read a textfile:
       std::stringstream buffer;
       buffer << file.rdbuf();
       std::string const& fileContent = buffer.str();

       // We now have the entire json representation of the parameter set
       // in the string variable 'fileContent'

       // In order to detect if the file contains the parameters only, or the entire camera node we simply write it into
       // a temporary NxLib node "/tmp" and check if it contains a subitem 'Parameters'; if it does, we only use this
       // subitem to rewrite the camera's parameter tree, otherwise we assume that the file contained the parameter subtree only

       NxLibItem tmp("/home/yake/Temp");
       tmp.setJson(fileContent); // Parse json file content into temporary item
       if (tmp[itmParameters].exists()) {
          // Looks like we had a file containing the full camera node; let's just use the parameters subtree to overwrite
          // the camera's parameter subtree
          rgbCam[itmParameters].setJson(tmp[itmParameters].asJson(), true); // writeableNodesOnly = true silently skips
                                                                            // read-only nodes instead of failing when
                                                                            // encountering a read-only node
       } else {
          // It seems that our file only contained the parameters node, because we don't have Parameters as a subnode;
          // We then use the entire content of the temporary node to rewrite the camera's parameters node
          rgbCam[itmParameters].setJson(tmp.asJson(), true); // with writebleNodesOnly = true, see comment above
       }
    }

    //read json file
    std::ifstream file2("/home/yake/dep.json");
    if (file2.is_open() && file2.rdbuf()) {
       // You can use the std::stringstream class to read a textfile:
       std::stringstream buffer2;
       buffer2 << file2.rdbuf();
       std::string const& fileContent2 = buffer2.str();

       // We now have the entire json representation of the parameter set
       // in the string variable 'fileContent'

       // In order to detect if the file contains the parameters only, or the entire camera node we simply write it into
       // a temporary NxLib node "/tmp" and check if it contains a subitem 'Parameters'; if it does, we only use this
       // subitem to rewrite the camera's parameter tree, otherwise we assume that the file contained the parameter subtree only

       NxLibItem tmp2("/home/yake/Temp");
       tmp2.setJson(fileContent2); // Parse json file content into temporary item
       if (tmp2[itmParameters].exists()) {
          // Looks like we had a file containing the full camera node; let's just use the parameters subtree to overwrite
          // the camera's parameter subtree
          depCam[itmParameters].setJson(tmp2[itmParameters].asJson(), true); // writeableNodesOnly = true silently skips
                                                                            // read-only nodes instead of failing when
                                                                            // encountering a read-only node
       } else {
          // It seems that our file only contained the parameters node, because we don't have Parameters as a subnode;
          // We then use the entire content of the temporary node to rewrite the camera's parameters node
          depCam[itmParameters].setJson(tmp2.asJson(), true); // with writebleNodesOnly = true, see comment above
       }
    }

    //Chage monocular camera format
//    std::ifstream file3("/home/yake/size.json");
//     std::stringstream buffer3;
//     buffer3 << file3.rdbuf();
//     std::string const& fileContent3 = buffer3.str();

//    NxLibItem tmp("/home/yake/Temp");
//    tmp.setJson(fileContent3);
//    rgbCam[itmSensor][itmSize].setJson(tmp.asJson(),true);
//    file3.close();

//    std::ofstream file3("/home/yake/size.json");
//    file3<<rgbCam[itmSensor][itmSize].asJson(true);
//    file3.close();


    //Create capture object and capture images
    NxLibCommand capture(cmdCapture);
    capture.execute();
    NxLibCommand disparityMap(cmdComputeDisparityMap);
    disparityMap.execute();
    NxLibCommand pointMap(cmdComputePointMap);
    pointMap.execute();


    //Create render point map object
    NxLibCommand renderPointMap(cmdRenderPointMap);
    renderPointMap.parameters()[itmCamera]=rgbCam[itmSerialNumber].asString();
    renderPointMap.parameters()[itmNear]=1;
    //Use GPU to render
    root[itmParameters][itmRenderPointMap][itmUseOpenGL]=false;

    try {
            renderPointMap.execute();
       // Code block using NxLib's C++ classes NxLibItem and NxLibCommand. These might throw and
       // exception of type NxLibException.
    } catch (NxLibException& e) {
       // Errors related to accessing the parameter tree are directly contained as return codes.
       // NxLibCommand can return the special return code 'NxLibExecutionFailed' to indicate that
       // the executed command returned more error information in the ErrorSymbol and ErrorText nodes.

       // Create an item referencing the result node
       NxLibItem result = NxLibItem()[itmExecute][itmResult];
       if (e.getErrorCode() == NxLibExecutionFailed) {
          // A command has failed to execute successfully. Print all information to identify what happened.
          printf("The execution of a command failed with error %s. Message: %s.\n",
             result[itmErrorSymbol].asString().c_str(),
             result[itmErrorText].asString().c_str());
          printf("The full command and all parameters in JSON format of the failed command were:\n%s\n",
             result[itmExecute].asJson(true).c_str());
       } else {
          // Access to some tree node failed. Print the error code and corresponding text.
          printf("An NxLib API error %d occurred when accessing the item %s. Message: %s.\n",
             e.getErrorCode(),
             e.getItemPath().c_str(),
             e.getErrorText().c_str());
       }
    }

    //Retrive perspective transform depth image
    std::vector<float> ppDepImg;

    //get depth image and its data info
    int width,height,channels,element_step;
    int error =0;
    bool is_float;
    double timeStamp;
    root[itmImages][itmRenderPointMap].getBinaryDataInfo(&error,&width,&height,&channels,&element_step,&is_float,&timeStamp);
    root[itmImages][itmRenderPointMap].getBinaryData(ppDepImg,0);

    pcl::PointCloud<pcl::PointXYZRGB> ppCloud;
    ppCloud.header.frame_id = "/camera_link";
    ppCloud.width           = width;
    ppCloud.height          = height;
    ppCloud.is_dense        = false;
    ppCloud.resize(height * width);

    // Copy data in point cloud (and convert milimeters in meters)
    for (size_t i = 0; i < ppDepImg.size (); i += 3) {
        ppCloud.points[i / 3].x = ppDepImg[i] / 1000.0;
        ppCloud.points[i / 3].y = ppDepImg[i + 1] / 1000.0;
        ppCloud.points[i / 3].z = ppDepImg[i + 2] / 1000.0;
    }

    //Retrive normal depth image
    std::vector<float> norDepImg;
    int width2,height2;
    depCam[itmImages][itmPointMap].getBinaryDataInfo(&width2,&height2,0,0,0,0);
    depCam[itmImages][itmPointMap].getBinaryData(norDepImg,0);

    pcl::PointCloud<pcl::PointXYZ> norCloud;
    norCloud.header.frame_id="/camera_link";
    norCloud.width=width2;
    norCloud.height=height2;
    norCloud.is_dense=false;
    norCloud.resize(width2*height2);

    for(int i =0 ;i<norDepImg.size();i+=3){
        norCloud.points[i/3].x=norDepImg[i]/1000.0;
        norCloud.points[i/3].y=norDepImg[i+1]/1000.0;
        norCloud.points[i/3].z=norDepImg[i+2]/1000.0;
    }

    //Retrive color image
    int error2=0;
    std::vector<unsigned char> color_list;
    double timeStamp2;
    int width3,height3,channels3,element_step3;
    bool is_float3;
    double timeStamp3;
    rgbCam[itmImages][itmRaw].getBinaryDataInfo(&width3,&height3,&channels3,&element_step3,&is_float3,&timeStamp3);
    rgbCam[itmImages][itmRaw].getBinaryData(&error2,color_list,&timeStamp2);

    //Mat colorImg= Mat::zeros(height3,width3,CV_8UC3);
    for(int i = 0; i<color_list.size();i+=3){
        ppCloud.points[i / 3].b=color_list[i+2];//B
        ppCloud.points[i / 3].g=color_list[i+1];//G
        ppCloud.points[i / 3].r=color_list[i];//R
    }



    //visualize two depth images and rgb image
    pcl::visualization::PCLVisualizer view1("ppDep");
    view1.addPointCloud(ppCloud.makeShared(),"pp point cloud");
    view1.spinOnce();

    pcl::visualization::PCLVisualizer view2("norDep");
    view2.addPointCloud(norCloud.makeShared(),"nor point cloud");
    view2.spin();



//    //display image
//    Mat rgbMat=Mat::zeros(480,752,CV_8UC3);
//    for(int i=0;i<rgbImg.size();i++)
//        {
//        cout<<rgbImg[i]<<endl;
//    }
//    for(int i=0;i<rgbImg.size();i+=4)
//    {
//        int r=i/(752*4); int c =(i-r*752*4)/4;
//        rgbMat.at<cv::Vec3b>(r,c)[0]=rgbImg[i+2];
//        rgbMat.at<cv::Vec3b>(r,c)[1]=rgbImg[i+1];
//        rgbMat.at<cv::Vec3b>(r,c)[2]=rgbImg[i];
//    }
//    imshow("view",rgbMat);
//    cv::waitKey(0);

    //close cameras
    NxLibCommand close(cmdClose);
    close.parameters()[itmCameras].setJson(camString,true);
    close.execute();

    return 0;
}
