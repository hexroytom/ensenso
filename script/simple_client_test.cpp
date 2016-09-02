//ros
#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//ros service
#include <ensenso/CaptureSinglePointCloud.h>
#include <ensenso/ConfigureStreaming.h>
#include <ensenso/SetBool.h>
//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


class simple_client
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub;
    ros::Rate loop;
    ros::ServiceClient capture_client;
    ros::ServiceClient configure_stream_client;
    std::string store_path_prefix_;
    //defined as a static member so that it can be called during static function OnShutDown()
    static ros::ServiceClient start_stream_client;

public:
    std::string default_path;
public:
    //define as a statics function so that it can be passed to function signal()
    //define what should be done before the node is shutdown
    static void OnShutDownCb(int sig)
    {
        //stop device
        ensenso::SetBool stop_srv;
        stop_srv.request.data=false;
        if(simple_client::start_stream_client.call(stop_srv))
            ROS_INFO("stop device successfully!");
        ros::shutdown();
    }

    simple_client():
        loop(2),
        default_path("/home/yake/catkin_ws/src/ensenso/pcd/")
    {
        //define shutdown callback
        signal(SIGINT,simple_client::OnShutDownCb);
        //create clients
        capture_client=nh_.serviceClient<ensenso::CaptureSinglePointCloud>("capture_single_point_cloud");
        configure_stream_client=nh_.serviceClient<ensenso::ConfigureStreaming>("configure_streaming");
        simple_client::start_stream_client=nh_.serviceClient<ensenso::SetBool>("start_streaming");
        //create publisher
        pub=nh_.advertise<sensor_msgs::PointCloud2>("single_point_cloud",1);
        //get store path
        nh_.param("store_path_prefix",store_path_prefix_,default_path);

    }

    ~simple_client()
    {}

    void run()
    {
        //initiate srv message
        ensenso::CaptureSinglePointCloud capture_srv;
        capture_srv.request.req=true;
        ensenso::ConfigureStreaming conf_srv;
        conf_srv.request.cloud=true; conf_srv.request.images=true;
        ensenso::SetBool start_srv;
        start_srv.request.data=false;// true--start device, false--stop device

        //call configure srv and start srv
        if(configure_stream_client.call(conf_srv))
                ROS_INFO("configuration successful\n");
        if(simple_client::start_stream_client.call(start_srv));
                ROS_INFO("start device successfully.");

        //capture single point cloud in the while loop
                while(ros::ok())
                {
                    //call srv
                    if(capture_client.call(capture_srv))
                    {
                        ROS_INFO("Capture single pointcloud succeed!");
                        pub.publish(capture_srv.response.pc);
                    }
                    else
                        ROS_INFO("Error: capture failed!");

                    loop.sleep();
                }
    }

    void save_pcd()
    {
        //initiate capture srv message
        ensenso::CaptureSinglePointCloud capture_srv;
        capture_srv.request.req=true;
        //create empty point cloud
        pcl::PointCloud<pcl::PointXYZ> pcl_pc;

        if(capture_client.call(capture_srv))
        {
            pcl::fromROSMsg(capture_srv.response.pc,pcl_pc);
            int time=(int)ros::Time::now().toSec();
            store_path_prefix_.append(std::to_string(time));
            store_path_prefix_.append("_pc.pcd");
            pcl::io::savePCDFileBinary(store_path_prefix_,pcl_pc);
        }


    }


};

ros::ServiceClient simple_client::start_stream_client;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"simple_client",ros::init_options::NoSigintHandler);
    simple_client client;
    client.save_pcd();
    ros::spin();
    return 0;

//--------------------------------------basic service call for debug--------------------------------------
//    ros::NodeHandle nh_;
//    ros::Publisher pub=nh_.advertise<sensor_msgs::PointCloud2>("single_point_cloud",1);
//    ros::Rate loop(2);

//    ros::ServiceClient capture_client=nh_.serviceClient<ensenso::CaptureSinglePointCloud>("capture_single_point_cloud");
//    ros::ServiceClient configure_stream_client=nh_.serviceClient<ensenso::ConfigureStreaming>("configure_streaming");
//    ros::ServiceClient start_stream_client=nh_.serviceClient<ensenso::SetBool>("start_streaming");

//    //initiate srv message
//    ensenso::CaptureSinglePointCloud capture_srv;
//    capture_srv.request.req=true;
//    ensenso::ConfigureStreaming conf_srv;
//    conf_srv.request.cloud=true; conf_srv.request.images=true;
//    ensenso::SetBool start_srv;
//    start_srv.request.data=true;// true--start device, false--stop device

//    //call configure srv and start srv
//    if(configure_stream_client.call(conf_srv))
//            ROS_INFO("configuration successful\n");
//    if(start_stream_client.call(start_srv));
//            ROS_INFO("start device successfully.");

//    //capture single point cloud in while loop
//            while(ros::ok())
//            {
//                //call srv
//                if(capture_client.call(capture_srv))
//                {
//                    ROS_INFO("Capture single pointcloud succeed!");
//                    pub.publish(capture_srv.response.pc);
//                }
//                else
//                    ROS_INFO("Error: capture failed!");

//                loop.sleep();
//            }
//--------------------------------------basic service call for debug--------------------------------------
}
