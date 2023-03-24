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
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"

using namespace std::chrono_literals;

class MinimalPublisher : public ros::NodeHandle
{
public:
    MinimalPublisher()
            : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->createWallTimer(
                500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        ROS_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    ros::Timer::SharedPtr timer_;
    ros::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

class IntensityFilter : public  ros::NodeHandle
{
public:
    IntensityFilter()
        : NodeHandle("intensity_filter")
    {
        this->declare_parameter<int>("min_intensity_threshold", 0);
        this->declare_parameter<int>("max_intensity_threshold", 255);
        _subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/velodyne_points", 10, std::bind(&IntensityFilter::topic_callback, this, std::placeholders::_1));
        _publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("intensity_filtered_cloud", 10);
    }
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr rosCloud)
    {
        int min_intensity_threshold, max_intensity_threshold;
        this->get_parameter("min_intensity_threshold", min_intensity_threshold);
        this->get_parameter("max_intensity_threshold", max_intensity_threshold);

        ROS_INFO(this->get_logger(), "There are %zu points in the cloud", rosCloud->data.size());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*rosCloud, *pclCloud);

        pcl::PassThrough<pcl::PointXYZI> intensityFilter;
        intensityFilter.setInputCloud(pclCloud);
        intensityFilter.setFilterFieldName("intensity");
        intensityFilter.setFilterLimits(min_intensity_threshold, max_intensity_threshold);
        intensityFilter.filter(*pclCloud);


        sensor_msgs::msg::PointCloud2 rosCloud2;

        pcl::toROSMsg(*pclCloud, rosCloud2);
        rosCloud2.header = rosCloud->header;
        _publisher->publish(rosCloud2);
    }

private:
    ros::Subscriber<sensor_msgs::msg::PointCloud2>::SharedPtr _subscription;
    ros::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv);
    //rclcpp::spin(std::make_shared<MinimalPublisher>());
    ros::spin(std::make_shared<IntensityFilter>());
    ros::shutdown();
    return 0;
}

