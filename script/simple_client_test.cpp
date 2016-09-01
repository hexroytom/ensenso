#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <ensenso/CaptureSinglePointCloud.h>
#include <ensenso/ConfigureStreaming.h>
#include <ensenso/SetBool.h>


class simple_client
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub;
    ros::Rate loop;
    ros::ServiceClient capture_client;
    ros::ServiceClient configure_stream_client;
    //defined as a static member so that it can be called during static function OnShutDown()
    static ros::ServiceClient start_stream_client;
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
        loop(2)
    {
        //define shutdown callback
        signal(SIGINT,simple_client::OnShutDownCb);
        //create clients
        capture_client=nh_.serviceClient<ensenso::CaptureSinglePointCloud>("capture_single_point_cloud");
        configure_stream_client=nh_.serviceClient<ensenso::ConfigureStreaming>("configure_streaming");
        simple_client::start_stream_client=nh_.serviceClient<ensenso::SetBool>("start_streaming");
        //create publisher
        pub=nh_.advertise<sensor_msgs::PointCloud2>("single_point_cloud",1);

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


};

ros::ServiceClient simple_client::start_stream_client;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"simple_client",ros::init_options::NoSigintHandler);
    simple_client client;
    client.run();
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
