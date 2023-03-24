// SYSTEM
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"

#include <tf2_ros/create_timer_ros.h>
#include "message_filters/subscriber.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#ifdef TF2_CPP_HEADERS
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif


using namespace std::chrono_literals;

class PlanarFilter : public  ros::NodeHandle
{
public:
    PlanarFilter()
        : NodeHandle("polygon_filter"),
          _targetFrame("kart")
    {
        this->declare_parameter<float>("lookahead_distance", 50.0);
        this->declare_parameter<std::string>("target_frame", "kart");
        this->get_parameter("target_frame", _targetFrame);

        std::chrono::duration<int> buffer_timeout(1);

        _buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                this->get_node_base_interface(),
                this->get_node_timers_interface());
        _buffer->setCreateTimerInterface(timer_interface);
        _tf2Listener = std::make_shared<tf2_ros::TransformListener>(*_buffer);

        _pointCloudSub.subscribe(this, "/point_cloud_topic");
        _tf2Filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
                _pointCloudSub, *_buffer, _targetFrame, 100, this->get_node_logging_interface(),
                this->get_node_clock_interface(), buffer_timeout);

        _tf2Filter->registerCallback(&PlanarFilter::_topic_callback, this);


//        _subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//                "/point_cloud_topic", 10, std::bind(&PlanarFilter::_topic_callback, this, std::placeholders::_1));

        _publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("polygon_filtered_cloud", 10);

        _boundryPoints = {
                {0.f, -1.f},
//                {-7.5f, 0.f},
                {2.f, -1.f},
                {2.f, 1.f},
                {0.f, 1.f}
        };
    }

private:
    ros::Subscriber<sensor_msgs::msg::PointCloud2>::SharedPtr _subscription;
    ros::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> _pointCloudSub;

    std::string _targetFrame;
    std::shared_ptr<tf2_ros::Buffer> _buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf2Listener;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> _tf2Filter;


    // Points in the whole track // Might need a third number of how far they are along the track
    std::vector<std::pair<float, float>> _boundryPoints;

    // Points in the current polygon to search // pair is x, y
    std::vector<std::pair<float, float>> _polygonPoints;

    float _lookahead_distance;

    void _topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr rosCloud)
    {
        float voxel_size;
        this->get_parameter("lookahead_distance", _lookahead_distance);
        this->get_parameter("voxel_size", voxel_size);

        ROS_INFO(this->get_logger(), "Point cloud received with  %zu points", rosCloud->data.size());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*rosCloud, *pclCloud);

        geometry_msgs::msg::TransformStamped transform = _buffer->lookupTransform(_targetFrame, "velodyne", rosCloud->header.stamp);

        auto t = transform.transform.translation;
        auto r = transform.transform.rotation;

        Eigen::Affine3d transformEigenAffine;

        transformEigenAffine = Eigen::Translation3d(t.x,
                                                    t.y,
                                                    t.z) *
                               Eigen::Quaterniond(r.w,
                                                  r.x,
                                                  r.y,
                                                  r.z);

        pcl::transformPointCloud(*pclCloud, *pclCloud, transformEigenAffine);

//        _buffer->transform()
//        transform.transform.rotation.x;

        pcl::IndicesPtr polygonInliers(new pcl::Indices);

        for(int i = 0; i < (int)pclCloud->points.size();i++)
        {
            if(test_point(pclCloud->points.at(i)))
                polygonInliers->push_back(i);
        }

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(pclCloud);
        extract.setIndices(polygonInliers);
        extract.setNegative(false);
        extract.filter(*pclCloud);

        sensor_msgs::msg::PointCloud2 rosCloud2;

        pcl::toROSMsg(*pclCloud, rosCloud2);
        rosCloud2.header = rosCloud->header;
        rosCloud2.header.frame_id = _targetFrame;
        _publisher->publish(rosCloud2);
    }

    bool test_point(const pcl::PointXYZI& point){
        float testX = point.x;
        float testY = point.y;

        int nVertices = _boundryPoints.size();
        int c = 0;
        // i = first vertex, j = last. When stepping, move i to second, the j to first. post increment i++ essential
        for (int i = 0, j = nVertices - 1; i < nVertices; j = i++)
        {
            float iVertexX = _boundryPoints.at(i).first;
            float iVertexY = _boundryPoints.at(i).second;
            float jVertexX = _boundryPoints.at(j).first;
            float jVertexY = _boundryPoints.at(j).second;
            if (((iVertexY > testY) != (jVertexY > testY)) &&
                (testX < (jVertexX-iVertexX) * (testY-iVertexY) / (jVertexY - iVertexY) + iVertexX))
            {
                c = !c;
            }
        }
        return c;
        // This is copied from https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
        /*int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
        {
            int i, j, c = 0;
            for (i = 0, j = nvert-1; i < nvert; j = i++) {
                if ( ((verty[i]>testy) != (verty[j]>testy)) &&
                     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
                    c = !c;
            }
            return c;
        }*/
    }
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv);
    //rclcpp::spin(std::make_shared<MinimalPublisher>());
    ros::spin(std::make_shared<PlanarFilter>());
    ros::shutdown();
    return 0;
}

