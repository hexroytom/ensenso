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
    Mat depMap_;
    Mat depMap_morph;
    Mat depMap_HE; //increase contrast by histogram equalization
    Mat depMap_boundary;
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

    void morphology_proc(const Mat& input, Mat& output,const int& morphType,const int& morphShape,const int& size)
    {
        if(is_init_)
        {
            Mat eleStruct=getStructuringElement(morphShape,Size(size,size));
            morphologyEx(input,output,morphType,eleStruct);
        }
    }

    void find_boundary(Mat& input,Mat& output,const int& thresh)
    {
        Mat input_;
        if(input.depth()==CV_32F)
            input.convertTo(input_,CV_8U,255);
        else
            input.copyTo(input_);
        Canny(input_,output,thresh,thresh*2,3,true);
    }

    void histoEqual_proc(const Mat& input, Mat& output)
    {
        Mat input_;
        if(input.depth()==CV_32F)
            input.convertTo(input_,CV_8U,255);
        else
            input.copyTo(input_);
        equalizeHist(input_,output);
    }

private:
    string PCD_file_;
    PointCloudXYZ pts_;
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
    pc_seg seg("/home/yake/catkin_ws/src/ensenso/pcd/1473930054_pc.pcd");
    seg.morphology_proc(seg.depMap_,seg.depMap_morph,cv::MORPH_TOPHAT,cv::MORPH_RECT,7);
    //seg.find_boundary(seg.depMap_HE,seg.depMap_boundary,5);
    imshow("ori",seg.depMap_);
    imshow("morph",seg.depMap_morph);
    //imshow("boundary",seg.depMap_boundary);
    waitKey(0);
    return 0;
}
