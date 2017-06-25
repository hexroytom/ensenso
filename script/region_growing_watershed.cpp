//opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <iostream>
#include <vector>
#include <string>

//#include "persistence1d.hpp"

#include <algorithm>

using namespace cv;
using namespace std;
//using namespace p1d;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;

struct Point1D{
	Point1D(const int& index_ = 0, const int& var_ = 0) :index(index_), var(var_){};
	int index;
	int var;
};

struct Point1DCluster{
	Point1DCluster(const int& median_ = 0) : median(median_){};
	vector<Point1D> pts;
	int median;
};

class pc_seg
{

	friend void display(const pc_seg& seg, const string& window);

public:
	Mat depMap_;
	Mat depMap_U;
	Mat depMap_stretch;
	Mat depMap_morph;
	Mat depMap_HE; //increase contrast by histogram equalization
	Mat depMap_boundary;
	Mat depMap_binary;
	Mat depMap_DT;
	Mat depMap_gradie;
	PointCloudXYZ pts_;
	PointCloudNormal::Ptr normal_;
public:
	pc_seg(const string PCD_file) :
		PCD_file_(PCD_file),
		is_init_(false),
		normal_(new PointCloudNormal)
	{
		if (read_PCD2Mat(PCD_file_, depMap_, pts_) == 0)
			is_init_ = true;
		else
		{
			is_init_ = false; PCL_ERROR("Initiate object failed!");
		}
	}

	void morphology_proc(const Mat& input, Mat& output, const int& morphType, const int& morphShape, const int& size, const int iterations)
	{
		Mat output_;
		if (is_init_)
		{
			Mat eleStruct = getStructuringElement(morphShape, Size(size, size));
			morphologyEx(input, output_, morphType, eleStruct, cv::Point(-1, -1), iterations);
			output_.copyTo(output);
		}
	}

	void find_boundary(Mat& input, Mat& output, const int& thresh)
	{
		Mat input_;
		if (input.depth() == CV_32F)
			input.convertTo(input_, CV_8U, 255);
		else
			input.copyTo(input_);
		Canny(input_, output, thresh, thresh * 2, 3, true);
	}

	void histoEqual_proc(const Mat& input, Mat& output)
	{
		Mat input_;
		if (input.depth() == CV_32F)
			input.convertTo(input_, CV_8U, 255);
		else
			input.copyTo(input_);
		equalizeHist(input_, output);
	}

	void contrast_stretching(const Mat& input, Mat& output, int lowOut, int highOut)
	{
		Mat input_;
		if (input.depth() == CV_32F)
			input.convertTo(input_, CV_8U, 255);
		else
			input.copyTo(input_);
		//CV_Assert(input.type()==CV_8UC1);
		//calculate histogram
		Mat hist;
		int histSize = 256;
		float range[] = { 0, 256 };
		const float* histRange = { range };
		calcHist(&input_, 1, 0, Mat(), hist, 1, &histSize, &histRange, true, false);

		vector<Point1D> pts;
		for (int i = 0; i < hist.rows; ++i)
		{
			int cnt = hist.at<float>(i, 0);
			if (cnt>0)
			{
				pts.push_back(Point1D(i, cnt));
			}
		}

		vector<Point1DCluster> clusters;
		Point1DCluster tmp_single_cluster;

		for (auto it = pts.begin(); it != pts.end(); it++){
			if (tmp_single_cluster.pts.empty()){
				tmp_single_cluster.pts.push_back(*it);
				//if this is the last element, then create a cluster and break
				if (it == pts.end() - 1){
					clusters.push_back(tmp_single_cluster);
					break;
				}
			}


			if (((it + 1)->index) - (it->index) <= 1)
			{
				tmp_single_cluster.pts.push_back(*(it + 1));
				//obvious that the last element has been processed, just quit the for loop
				if (it + 1 == pts.end() - 1){
					clusters.push_back(tmp_single_cluster);
					break;
				}

			}
			else
			{
				clusters.push_back(tmp_single_cluster);
				tmp_single_cluster.pts.clear();
			}
		}

		//not necessary for now
		//for (auto cluster : clusters){
		//	std::sort(cluster.pts.begin(), cluster.pts.end(),sortPoint1DCluster);
		//}
		//for (auto cluster : clusters)
		//{
		//	int mid_index = cluster.pts.size() / 2;
		//	cluster.median = cluster.pts[mid_index].index;
		//}

		clusters.erase(clusters.begin());
		clusters.erase(clusters.end() - 1);

		vector<Point1D> filtered_pts;
		for (auto single_cluster : clusters)
		{
			for (auto point : single_cluster.pts)
			{
				filtered_pts.push_back(point);
			}
		}

		std::sort(filtered_pts.begin(), filtered_pts.end(), sortPoint1DCluster);

		//vector<float> hist_array;
		//for (int i = 0; i<hist.rows; i++)
		//	hist_array.push_back(hist.at<float>(i, 0));
		//not necessary 
		//Persistence1D p;
		//vector<int> minIndex, maxIndex;
		//p.RunPersistence(hist_array);
		//vector<TPairedExtrema> extrema;
		//p.GetPairedExtrema(extrema);
		//p.GetExtremaIndices(minIndex, maxIndex);
		//std::sort(minIndex.begin(), minIndex.end());
		//std::sort(extrema.begin(), extrema.end(), sortPersistence);
		////define contrast stretch params: lowIn & highIn


		int lowIn = (filtered_pts.begin())->index; int highIn = (filtered_pts.end() - 1)->index;
		output.create(input_.rows, input_.cols, CV_8UC1);
		for (int i = 0; i<input_.rows; i++)
		for (int j = 0; j<input_.cols; j++){
			float stretch = computeContrastStretch(input_.at<uchar>(i, j), lowIn, highIn, lowOut, highOut);
			output.at<uchar>(i, j) = saturate_cast<uchar>(stretch);
		}
	}

	void watershed_segmentation(Mat& criterion, Mat& marker, vector<vector<cv::Point>> contours, Mat& dst, Mat& label)
	{
		Mat dep_3CH, marker_;
		marker.copyTo(marker_);
		cvtColor(criterion, dep_3CH, COLOR_GRAY2BGR);
		watershed(dep_3CH, marker_);

		Mat mark = Mat::zeros(marker_.size(), CV_8UC1);
		marker_.convertTo(mark, CV_8UC1);
		bitwise_not(mark, mark);

		// Generate random colors
		vector<Vec3b> colors;
		for (size_t i = 0; i < contours.size(); i++)
		{
			int b = theRNG().uniform(0, 255);
			int g = theRNG().uniform(0, 255);
			int r = theRNG().uniform(0, 255);
			colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
		}

		// Create the result image
		dst = Mat::zeros(marker_.size(), CV_8UC3);
		// Fill labeled objects with random colors
		for (int i = 0; i < marker_.rows; i++)
		{
			for (int j = 0; j < marker_.cols; j++)
			{
				int index = marker_.at<int>(i, j);
				if (index > 0 && index <= static_cast<int>(contours.size()))
					dst.at<Vec3b>(i, j) = colors[index - 1];
				else
					dst.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
			}
		}

		//std::vector<int> indice;
		//for (int i = 0; i < marker_.rows; i++)
		//{
		//	for (int j = 0; j < marker_.cols; j++)
		//	{
		//		int label = marker_.at<int>(i, j);
		//		if (label > 0 && label <= static_cast<int>(contours.size()))
		//		{
		//			indice.push_back(i*marker_.cols + j);
		//		}
		//	}
		//}
		mark.copyTo(label);

	}

	void distanceTransform_proc(const Mat& input, Mat& distance_transform, Mat& markers, const int& num_iter)
	{
		if (num_iter <= 0)
		{
			PCL_ERROR("The number of iterations must be larger than 01");
			return;
		}

		Mat distanceTransfrom_, binary_;

		for (int i = 0; i < num_iter; i++)
		{
			Mat tmp_input;
			if (i + 1 == 1){	//if this is the first time to process the image
				input.copyTo(tmp_input);
				distanceTransform(tmp_input, distanceTransfrom_, CV_DIST_L2, 3);
				cv::normalize(distanceTransfrom_, distanceTransfrom_, 0, 255, NORM_MINMAX);
				distanceTransfrom_.convertTo(distanceTransfrom_, CV_8U);
				//binarize image to gain MARKERS
				threshold(distanceTransfrom_, binary_, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
			}
			else{
				binary_.copyTo(tmp_input);
				distanceTransform(tmp_input, distanceTransfrom_, CV_DIST_L2, 3);
				cv::normalize(distanceTransfrom_, distanceTransfrom_, 0, 255, NORM_MINMAX);
				distanceTransfrom_.convertTo(distanceTransfrom_, CV_8U);
				//binarize image to gain MARKERS
				threshold(distanceTransfrom_, binary_, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
			}
		}
		distanceTransfrom_.copyTo(distance_transform);
		binary_.copyTo(markers);
	}

	void drawConvexHull(vector<vector<cv::Point>> contours, Mat& img)
	{
		for (auto single_contour : contours)
		{
			if (single_contour.size() > 100)
			{
				vector<cv::Point> convex;
				convexHull(single_contour, convex);
				polylines(img, convex, true, Scalar(255, 255, 255));
			}
		}
	}

	void findAndDrawContours(Mat& edge_img, Mat& contour_img)
	{
		vector<vector<cv::Point>> contours;
		findContours(edge_img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		for (auto single_contour : contours){
			if (single_contour.size() > 50){
				polylines(contour_img, single_contour, true, Scalar::all(255));
			}
		}
	}

	void extractPointsByIndices(pcl::PointIndices::Ptr indices, const PointCloudXYZ::Ptr ref_pts, PointCloudXYZ::Ptr extracted_pts, bool is_negative)
	{
		//init
		//indices = boost::make_shared<pcl::PointIndices>();
		//extracted_pts = boost::make_shared<PointCloudXYZ>();

		pcl::ExtractIndices<pcl::PointXYZ> tmp_extractor;
		tmp_extractor.setInputCloud(ref_pts);
		tmp_extractor.setNegative(is_negative);
		tmp_extractor.setIndices(indices);
		tmp_extractor.filter(*extracted_pts);
	}

	void extractPointsByIndices(pcl::PointIndices::Ptr indices, const PointCloudNormal::Ptr ref_pts, PointCloudNormal::Ptr extracted_pts)
	{
		pcl::ExtractIndices<pcl::Normal> tmp_extractor;
		tmp_extractor.setInputCloud(ref_pts);
		tmp_extractor.setNegative(false);
		tmp_extractor.setIndices(indices);
		tmp_extractor.filter(*extracted_pts);
	}

	void integralImgNormalEstimation()
	{
		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
		ne.setMaxDepthChangeFactor(0.02f);
		ne.setNormalSmoothingSize(10.0f);
		ne.setInputCloud(pts_.makeShared());
		ne.compute(*normal_);

	}

	void integralImgNormalEstimation(const pcl::PointIndices::Ptr indices, PointCloudNormal::Ptr normal)
	{
		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
		ne.setMaxDepthChangeFactor(0.02f);
		ne.setNormalSmoothingSize(10.0f);
		ne.setInputCloud(pts_.makeShared());
		//ne.setIndices(indices);
		ne.compute(*normal);
	}

	void plane_segment_proc(PointCloudXYZ::Ptr input_pts, float distance_thresh, pcl::PointIndices::Ptr inliers)
	{
		//plane segmentation
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
		// Optional
		plane_seg.setOptimizeCoefficients(true);
		// Mandatory
		plane_seg.setModelType(pcl::SACMODEL_PLANE);
		plane_seg.setMethodType(pcl::SAC_RANSAC);
		plane_seg.setDistanceThreshold(distance_thresh);
		plane_seg.setInputCloud(input_pts);
		plane_seg.segment(*inliers, *coefficients);
	}

	int read_PCD(const std::string path, pcl::PointCloud<pcl::PointXYZ> &pts)
	{
		int result = pcl::io::loadPCDFile(path, pts);
		if (result == -1)
			PCL_ERROR("Could not load PCD file!");
		return result;
	}

	int read_PCD2Mat(const string& path, Mat& img, PointCloudXYZ& pts)
	{
		if (read_PCD(path, pts) == -1)
		{
			PCL_ERROR("Could not load PCD file!");
			return (-1);
		}
		else
		{
			img.create(pts.height, pts.width, CV_32F);
			int width = pts.width;
			for (int i = 0; i<pts.height; i++)
			for (int j = 0; j<pts.width; j++)
				img.at<float>(i, j) = pts.points[i*width + j].z;
			img.convertTo(depMap_U, CV_8UC1, 255);
			return 0;
		}
	}

	void read_PCD_save_Mat(const string& PCD_path, const string& Mat_path)
	{
		//read PCD to Mat
		Mat img, img_UCHAR;
		PointCloudXYZ pts;
		read_PCD2Mat(PCD_path, img, pts);
		//convert the image from CV_32F to CV_8U
		img.convertTo(img_UCHAR, CV_8U, 255);
		imwrite(Mat_path, img_UCHAR);
	}

private:
	string PCD_file_;
	bool is_init_;

private:
	float computeContrastStretch(const int& input, int lowIn, int highIn, int lowOut, int highOut)
	{
		float result;
		//origin contrast stretching
		if (0 <= input && input<lowIn){
			result = lowOut / lowIn*input;
		}
		else if (lowIn <= input && input <= highIn){
			result = ((highOut - lowOut) / (highIn - lowIn))*(input - lowIn) + lowOut;
		}
		else if (input>highIn && input <= 255){
			result = 0.0;
		}

		//only strectch the target
		//        if(lowIn<=input && input<=highIn){
		//           result=((highOut-lowOut)/(highIn-lowIn))*(input-lowIn)+lowOut;
		//        }else{
		//            result=0.0;
		//        }

		return result;
	}

//	static  bool sortPersistence(const TPairedExtrema& i, const TPairedExtrema& j)
//	{
//		return(i.Persistence>j.Persistence);
//	}

	static bool sortPoint1DCluster(const Point1D& i, const Point1D& j)
	{
		return(i.index < j.index);
	}

	static bool sortPoint1DClusterByVar(const Point1D& i, const Point1D& j)
	{
		return(i.var > j.var);
	}

};

void display(const pc_seg& seg, const string& window)
{
	if (seg.is_init_)
	{
		imshow(window, seg.depMap_);
		waitKey(0);
	}
	else
		PCL_ERROR("Object not initiated!");
}

void display(const Mat& img)
{
	imshow("test", img);
	waitKey(0);
}




int main()
{
	//define images
	Mat depMap_;
	Mat depMap_U;
	Mat depMap_stretch;
	Mat depMap_morph;
	Mat depMap_HE; //increase contrast by histogram equalization
	Mat depMap_boundary;
	Mat depMap_binary;
	Mat depMap_DT;
	Mat depMap_gradie;


	pc_seg seg("F:/Project/Segmentation/pcd/1476274997_pc.pcd");
	//using contrast stretch to increase contrast
	seg.depMap_U.copyTo(depMap_U);
	PointCloudXYZ::Ptr pts(new PointCloudXYZ);
	pcl::copyPointCloud(seg.pts_,*pts);

	//plane extraction
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	PointCloudXYZ::Ptr unorganised_pts(new PointCloudXYZ);
	seg.plane_segment_proc(seg.pts_.makeShared(), 0.01, inliers);
	pcl::ExtractIndices<pcl::PointXYZ> extractor;
	extractor.setInputCloud(seg.pts_.makeShared());
	extractor.setNegative(true);
	extractor.setIndices(inliers);
	extractor.filter(*unorganised_pts);
	//extractor.filterDirectly(pts);//pts still organised but without plane

	//normal estimation
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(pts);
	ne.compute(*normal);

	//extract normal
	PointCloudNormal::Ptr unorganised_normal(new PointCloudNormal);
	pcl::ExtractIndices<pcl::Normal> nor_extractor;
	nor_extractor.setInputCloud(normal);
	nor_extractor.setNegative(true);
	nor_extractor.setIndices(inliers);
	nor_extractor.filter(*unorganised_normal);

	//pcl::visualization::PCLVisualizer view("debug");
	//view.addPointCloud(unorganised_pts, "pts");
	//view.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(unorganised_pts, unorganised_normal, 10);
	//view.spin();

	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	//variables for time recording
	double t1, t2, t;

	//region growing segmentation
	t1 = (double)cv::getTickCount();
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMaxClusterSize(20000);
	reg.setMinClusterSize(100);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(unorganised_pts);
	reg.setInputNormals(unorganised_normal);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(0.5);
	std::vector<pcl::PointIndices> clusters;
	
	reg.extract(clusters);
	t1 = ((double)cv::getTickCount() - t1) / cv::getTickFrequency();
	cout << "Time Consumed by Region Growing : " << t1 << " s" << endl;

	return 0;
}
