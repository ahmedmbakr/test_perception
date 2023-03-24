// SYSTEM
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"

using namespace std::chrono_literals;


class MinimalPublisher : public ros::NodeHandle
{
public:
    MinimalPublisher()
            : ros::NodeHandle("minimal_publisher"), count_(0)
    {
        publisher_ = this->advertise<std_msgs::String>("topic", 10); //create a publisher
        timer_ = this->createWallTimer(
                ros::WallDuration(0.5), std::bind(&MinimalPublisher::timer_callback, this));
    }

    void timer_callback()
    {
        auto message = std_msgs::String();
        message.data = "Hello, world! " + std::to_string(count_++); 
        ROS_INFO_STREAM("Publishing: " + message.data);
        publisher_.publish(message);
    }

private:
    ros::WallTimer timer_;
    ros::Publisher publisher_;
    size_t count_;
};

class IntensityFilter : public  ros::NodeHandle
{
public:
    IntensityFilter()
        : NodeHandle("intensity_filter")
    {
        this->param("min_intensity_threshold", 0); // declare a parameter
        this->param("max_intensity_threshold", 255); // declare a parameter
        _subscription = this->subscribe<sensor_msgs::PointCloud2>(
                "/velodyne_points", 10, std::bind(&IntensityFilter::topic_callback, this, std::placeholders::_1));
        _publisher = this->advertise<sensor_msgs::PointCloud2>("intensity_filtered_cloud", 10);
    }
    void topic_callback(const sensor_msgs::PointCloud2::ConstPtr& rosCloud)
    {
        int min_intensity_threshold, max_intensity_threshold;
        this->getParam("min_intensity_threshold", min_intensity_threshold);
        this->getParam("max_intensity_threshold", max_intensity_threshold);

        ROS_INFO_STREAM("Number of points in the cloud: " + rosCloud->data.size());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*rosCloud, *pclCloud);

        pcl::PassThrough<pcl::PointXYZI> intensityFilter;
        intensityFilter.setInputCloud(pclCloud);
        intensityFilter.setFilterFieldName("intensity");
        intensityFilter.setFilterLimits(min_intensity_threshold, max_intensity_threshold);
        intensityFilter.filter(*pclCloud);


        sensor_msgs::PointCloud2 rosCloud2;

        pcl::toROSMsg(*pclCloud, rosCloud2);
        rosCloud2.header = rosCloud->header;
        _publisher.publish(rosCloud2);
    }

private:
    ros::Subscriber _subscription;
    ros::Publisher _publisher;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "intensity_filter");
    //rclcpp::spin(std::make_shared<MinimalPublisher>());
    auto intensityFilter = IntensityFilter();
    ros::spin();
    ros::shutdown();
    return 0;
}

