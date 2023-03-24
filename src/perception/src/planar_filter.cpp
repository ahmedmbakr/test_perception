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
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std::chrono_literals;

class PlanarFilter : public  ros::NodeHandle
{
public:
    PlanarFilter()
        : ros::NodeHandle("planar_filter")
    {
        this->param("plane_thickness_m", 0.05);
        this->param("voxel_size", 0.05);
        this->param("box_height", 0.3);
        this->param("box_width", 0.91);
        this->param("height_tolerence", 0.4);
        this->param("width_tolerence", 1.0);

        _subscription = this->subscribe<sensor_msgs::PointCloud2>(
                "/point_cloud_topic", 10, std::bind(&PlanarFilter::_topic_callback, this, std::placeholders::_1));

        _publisher = this->advertise<sensor_msgs::PointCloud2>("plane_filtered_cloud", 10);
        _obstacle_publisher = this->advertise<sensor_msgs::PointCloud2>("obstacle_filtered_cloud", 10);

        _pose_publisher = this->advertise<geometry_msgs::PoseStamped>("box_center_pose", 10);
    }


private:
    ros::Subscriber _subscription;
    ros::Publisher _publisher;
    ros::Publisher _obstacle_publisher;
    ros::Publisher _pose_publisher;

    float _box_height, _box_width, _height_tolerence, _width_tolerence;

    void _topic_callback(const sensor_msgs::PointCloud2::ConstPtr& rosCloud)
    {
        float plane_thickness_m, voxel_size;
        this->getParam("plane_thickness_m", plane_thickness_m);
        this->getParam("voxel_size", voxel_size);
        this->getParam("box_height", _box_height);
        this->getParam("box_width", _box_width);
        this->getParam("height_tolerence", _height_tolerence);
        this->getParam("width_tolerence", _width_tolerence);

        ROS_INFO_STREAM("Number of points in the cloud: "+ rosCloud->data.size());
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
        ROS_INFO_STREAM("Number of points in the cloud after removing nans: " + pclCloud->points.size());

        downsampleCloud(pclCloud, pclCloud, voxel_size);

        extractGroundPlane(pclCloud, planeCloud, obstacleCloud, plane_thickness_m,
                           1000);

        auto test = extractObstacles(obstacleCloud, obstacleCloud, 0.1f, 50,
                         65536);

        ROS_INFO_STREAM("The box is at:" + std::to_string(test.first) + ", " + std::to_string(test.second));

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "kart";
        pose.header.stamp = rosCloud->header.stamp;
        pose.pose.position.x = test.first;
        pose.pose.position.y = test.second;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        _pose_publisher.publish(pose);

        sensor_msgs::PointCloud2 rosCloud2;

        pcl::toROSMsg(*planeCloud, rosCloud2);
        rosCloud2.header = rosCloud->header;
        _publisher.publish(rosCloud2);

        sensor_msgs::PointCloud2 rosCloud3;
        pcl::toROSMsg(*obstacleCloud, rosCloud3);
        rosCloud3.header = rosCloud->header;
        _obstacle_publisher.publish(rosCloud3);
    }

    bool downsampleCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr inputCloud,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr outputCloud, float voxelSize) {
        pcl::VoxelGrid<pcl::PointXYZI> vox;
        vox.setLeafSize(voxelSize, voxelSize, voxelSize);
        vox.setInputCloud(inputCloud);
        vox.filter(*outputCloud);

        ROS_INFO_STREAM("Downsampled cloud numb points: " + outputCloud->points.size());

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
                ROS_INFO_STREAM("No Cludster of sufficient size found");
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
            ROS_ERROR_STREAM("Min = "+ std::to_string(min[0]) +", " + std::to_string(min[1]) + ", Width = " + std::to_string(max[0] - min[0]));
            ROS_ERROR_STREAM("Max = " + std::to_string(max[0]) + ", " + std::to_string(max[1]) + ", Height = " + std::to_string(max[1] - min[1]));

            // Bounds check the size of the box
            if (std::abs((max[0] - min[0]) - _box_width) > _width_tolerence &&
                std::abs((max[1] - min[1]) - _box_height) > _height_tolerence) {
                ROS_ERROR_STREAM("Obstacle Failed Bounds Check, Continuing to next cluster");
                continue;
            }

            *obstaclePoints = *obstacleCluster;

            Eigen::Vector3f center = pca.getMean().head(3);
            ROS_ERROR_STREAM("Center = "+ std::to_string(center[0]) +", " + std::to_string(center[1])+", " + std::to_string(center[2]));

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
    ros::init(argc, argv, "planar_filter");
    //rclcpp::spin(std::make_shared<MinimalPublisher>());
    auto planarFilter = PlanarFilter();
    ros::spin();
    ros::shutdown();
    return 0;
}

