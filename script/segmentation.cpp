//opencv
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
//pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

class pc_seg
{

friend void display(const pc_seg& seg,const string& window);

public:
    pc_seg(const string PCD_file):
        PCD_file_(PCD_file),
        is_init_(false)
    {
        if(read_PCD2Mat(PCD_file_,depMap_,pts_)==0)
            is_init_=true;
        else
        {is_init_=false;PCL_ERROR("Initiate object failed!");}
    }

    void morphology_proc(const int& size)
    {
        if(is_init_)
        {
            Mat eleStruct=getStructuringElement(MORPH_CROSS,Size(size,size));
            morphologyEx(depMap_,depMap_,MORPH_CLOSE,eleStruct);
        }
    }
private:
    string PCD_file_;
    PointCloudXYZ pts_;
    Mat depMap_;
    bool is_init_;

private:

    int read_PCD(const std::string path,pcl::PointCloud<pcl::PointXYZ> &pts)
    {
        int result=pcl::io::loadPCDFile(path,pts);
        if (result == -1)
            PCL_ERROR("Could not load PCD file!");
        return result;
    }

    int read_PCD2Mat(const string& path,Mat& img,PointCloudXYZ& pts)
    {
        if(read_PCD(path,pts)== -1)
         {
            PCL_ERROR("Could not load PCD file!");
            return (-1);
         }
        else
        {
            img.create(pts.height,pts.width,CV_32FC1);
            int width=pts.width;
            for(int i=0;i<pts.height;i++)
                for(int j=0;j<pts.width;j++)
                    img.at<float>(i,j)=pts.points[i*width+j].z;
            return 0;
        }
    }
};

void display(const pc_seg& seg,const string& window)
{
      if(seg.is_init_)
      {
          imshow(window,seg.depMap_);
          waitKey(0);
      }
      else
          PCL_ERROR("Object not initiated!");
}

int main()
{
//    pc_seg seg("/home/yake/catkin_ws/src/ensenso/pcd/1473864320_pc.pcd");
//    display(seg,"before closing");
//    seg.morphology_proc(3);
//    display(seg,"after closing");
    PointCloudXYZ pts;
    pcl::io::loadPLYFile("/home/yake/depth.ply",pts);

    return 0;
}
