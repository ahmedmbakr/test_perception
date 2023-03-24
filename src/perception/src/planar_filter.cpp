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
#include "roscpp/roscpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pcl_conversions/pcl_conversions.h"

using namespace std::chrono_literals;

class PlanarFilter : public  roscpp::NodeHandle
{
public:
    PlanarFilter()
        : NodeHandle("planar_filter")
    {
        this->declare_parameter<float>("plane_thickness_m", 0.05);
        this->declare_parameter<float>("voxel_size", 0.05);
        this->declare_parameter<float>("box_height", 0.3);
        this->declare_parameter<float>("box_width", 0.91);
        this->declare_parameter<float>("height_tolerence", 0.4);
        this->declare_parameter<float>("width_tolerence", 1.0);

        _subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/point_cloud_topic", 10, std::bind(&PlanarFilter::_topic_callback, this, std::placeholders::_1));

        _publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("plane_filtered_cloud", 10);
        _obstacle_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacle_filtered_cloud", 10);

        _pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("box_center_pose", 10);
    }


private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscription;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _obstacle_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_publisher;

    float _box_height, _box_width, _height_tolerence, _width_tolerence;

    void _topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr rosCloud)
    {
        float plane_thickness_m, voxel_size;
        this->get_parameter("plane_thickness_m", plane_thickness_m);
        this->get_parameter("voxel_size", voxel_size);
        this->get_parameter("box_height", _box_height);
        this->get_parameter("box_width", _box_width);
        this->get_parameter("height_tolerence", _height_tolerence);
        this->get_parameter("width_tolerence", _width_tolerence);

        ROS_INFO(this->get_logger(), "There are %zu points in the cloud", rosCloud->data.size());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*rosCloud, *pclCloud);

        pcl::IndicesPtr indices(new pcl::Indices);

        pcl::removeNaNFromPointCloud(*pclCloud, *indices);
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(pclCloud);
        extract.setIndices(indices);
//        extract.setNegative(true);
        extract.filter(*pclCloud);
        ROS_INFO(this->get_logger(), "There are %zu points in the cloud after removing nans", pclCloud->points.size());

        downsampleCloud(pclCloud, pclCloud, voxel_size);

        extractGroundPlane(pclCloud, planeCloud, obstacleCloud, plane_thickness_m,
                           1000);

        auto test = extractObstacles(obstacleCloud, obstacleCloud, 0.1f, 50,
                         65536);

        ROS_INFO(this->get_logger(), "The box is at %f, %f", test.first, test.second);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "kart";
        pose.header.stamp = rosCloud->header.stamp;
        pose.pose.position.x = test.first;
        pose.pose.position.y = test.second;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        _pose_publisher->publish(pose);

        sensor_msgs::msg::PointCloud2 rosCloud2;

        pcl::toROSMsg(*planeCloud, rosCloud2);
        rosCloud2.header = rosCloud->header;
        _publisher->publish(rosCloud2);

        sensor_msgs::msg::PointCloud2 rosCloud3;
        pcl::toROSMsg(*obstacleCloud, rosCloud3);
        rosCloud3.header = rosCloud->header;
        _obstacle_publisher->publish(rosCloud3);
    }

    bool downsampleCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr inputCloud,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr outputCloud, float voxelSize) {
        pcl::VoxelGrid<pcl::PointXYZI> vox;
        vox.setLeafSize(voxelSize, voxelSize, voxelSize);
        vox.setInputCloud(inputCloud);
        vox.filter(*outputCloud);

        ROS_INFO(this->get_logger(), "Downsampled cloud has %zu points", outputCloud->points.size());

        return true;
    }

    bool extractGroundPlane(pcl::PointCloud<pcl::PointXYZI>::ConstPtr inputCloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr groundPlane,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr filteredPoints,
                            float ransacDistanceThreshold,
                            int minimumPlanePointCount) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr firstPlaneCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr intermediatePlaneCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr removedPoints(new pcl::PointCloud<pcl::PointXYZI>);

        if (inputCloud->empty())
            return false;
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(ransacDistanceThreshold);

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        // Initial segmentation uses the filtered cloud
        seg.setInputCloud(inputCloud);
        extract.setInputCloud(inputCloud);

        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            std::cout << "Inliers were empty" << std::endl;
            return false;
        }

        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*firstPlaneCloud);

        extract.setNegative(true);
        extract.filter(*removedPoints);

        *planeCloud += *firstPlaneCloud;

        while (!inliers->indices.empty() && inliers->indices.size() > minimumPlanePointCount) {
            // Extract out the points that were not part of the plane
            extract.setNegative(true);
            extract.filter(*removedPoints);
            seg.setInputCloud(removedPoints);
            extract.setInputCloud(removedPoints);

            seg.segment(*inliers, *coefficients);

            if (inliers->indices.empty()) {
                std::cout << "Inliers were empty" << std::endl;
                return false;
            }
            // TODO onlu add the plane to the plane cloud if they are approximatly in the same direction.
            // TODO Also check to make sure they are within XX degrees of where the ground should be, so it can't detect a wall first
            //   It should still remove them from the obstacle cloud, just not add them to the planeCloud.

            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*intermediatePlaneCloud);

            if (inliers->indices.size() > 2000) {
                *planeCloud += *intermediatePlaneCloud;
            }
        }
        *filteredPoints = *removedPoints;

        *groundPlane = *planeCloud;

        return true;
    }

    std::pair<float, float> extractObstacles(pcl::PointCloud<pcl::PointXYZI>::ConstPtr inputCloud,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr obstaclePoints,
                          float clusteringTolerance,
                          int minimumObstaclePoints,
                          int maximumObstaclePoints) {
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(inputCloud);

        // Cluster with a 10 cm tolerance, and 30 < cluster.size < 300
        std::vector<pcl::PointIndices> clusters;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(clusteringTolerance);
        ec.setMinClusterSize(minimumObstaclePoints);
        ec.setMaxClusterSize(maximumObstaclePoints);
        ec.setSearchMethod(tree);
        ec.setInputCloud(inputCloud);
        ec.extract(clusters);

        std::sort(clusters.begin(), clusters.end(), [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
            return a.indices.size() > b.indices.size();
        });

        Eigen::Vector3f boxCenterKartFrame;

        pcl::PCA<pcl::PointXYZI> pca;

        // TODO: make this extraction better.

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(obstaclePoints);
        extract.setNegative(false);

        // Extract indices from obstacle clusters
        for (auto cluster: clusters) {
            if (cluster.indices.size() < minimumObstaclePoints) {
                ROS_INFO(this->get_logger(), "No Cludster of sufficient size found");
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCluster(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr projection(new pcl::PointCloud<pcl::PointXYZI>);
            for (const auto &index: cluster.indices) {
                pcl::PointXYZI point;
                point.x = (inputCloud->at(index).x);
                point.y = (inputCloud->at(index).y);
                point.z = (inputCloud->at(index).z);
                obstacleCluster->push_back(point);
            }

            pca.setInputCloud(obstacleCluster);
            pca.project(*obstacleCluster, *projection);

            Eigen::Vector2f min(1e6, 1e6);
            Eigen::Vector2f max(-1e6, -1e6);

            for (auto& point : projection->points)
            {
                min[0] = std::min(point.x, min[0]);
                min[1] = std::min(point.y, min[1]);

                max[0] = std::max(point.x, max[0]);
                max[1] = std::max(point.y, max[1]);
            }
            ROS_ERROR(this->get_logger(), "Min = %f, %f, Width = %f", min[0], min[1], max[0] - min[0]);
            ROS_ERROR(this->get_logger(), "Max = %f, %f, Height = %f", max[0], max[1], max[1] - min[1]);

            // Bounds check the size of the box
            if (std::abs((max[0] - min[0]) - _box_width) > _width_tolerence &&
                std::abs((max[1] - min[1]) - _box_height) > _height_tolerence) {
                ROS_ERROR(this->get_logger(), "Obstacle Failed Bounds Check, Continuing to next cluster");
                continue;
            }

            *obstaclePoints = *obstacleCluster;

            Eigen::Vector3f center = pca.getMean().head(3);
            ROS_ERROR(this->get_logger(), "Center = %f, %f, %f", center[0], center[1], center[2]);

            boxCenterKartFrame = center;

            break;
        }
        if (obstaclePoints->empty()) {
            std::cout << "No points in obstacle cloud" << std::endl;
            return std::make_pair(0, 0);
        }
        return std::make_pair(boxCenterKartFrame[0], boxCenterKartFrame[1]);
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

