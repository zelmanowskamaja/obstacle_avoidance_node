#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


class ObstacleDetection : public rclcpp::Node {
public:
    ObstacleDetection()
    : Node("obstacle_detection") {
        // Inicjalizacja subskrybentów
        auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/sensing/lidar/scan", custom_qos,
            std::bind(&ObstacleDetection::lidar_callback, this, std::placeholders::_1));
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&ObstacleDetection::map_callback, this, std::placeholders::_1));
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/localization/cartographer/pose", 10,
            std::bind(&ObstacleDetection::pose_callback, this, std::placeholders::_1));

        // Inicjalizacja publisherów
        scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_lidar_scan", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        obstacle_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/obstackle_detected", 10);
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/modified_map", rclcpp::SensorDataQoS());

        // Ustawienia początkowe
        current_map_ = nullptr;
        current_pose_ = nullptr;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
    float map_resolution_;
    geometry_msgs::msg::Pose map_origin_;

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!current_map_ || !current_pose_)
            return;

        auto filtered_scan = *msg; // Direct copy of the message

        double pose_yaw = get_yaw_from_quaternion(
            current_pose_->pose.orientation.x,
            current_pose_->pose.orientation.y,
            current_pose_->pose.orientation.z,
            current_pose_->pose.orientation.w);

        double lidar_x = current_pose_->pose.position.x + 0.35 * cos(pose_yaw);
        double lidar_y = current_pose_->pose.position.y + 0.35 * sin(pose_yaw);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.type = marker.POINTS;
        marker.action = marker.ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.04;
        marker.scale.y = 0.04;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        for (size_t i = 0; i < filtered_scan.ranges.size(); ++i) {
            float range = filtered_scan.ranges[i];
            if (std::isinf(range)) continue;

            double angle = msg->angle_min + i * msg->angle_increment;
            double x = lidar_x + range * cos(angle + pose_yaw);
            double y = lidar_y + range * sin(angle + pose_yaw);

            int map_x = static_cast<int>((x - map_origin_.position.x) / map_resolution_);
            int map_y = static_cast<int>((y - map_origin_.position.y) / map_resolution_);

            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = y;
            p.z = 0.0;
            marker.points.push_back(p);

            bool is_occupied = false;
            if (map_x >= 0 && map_x < current_map_->info.width &&
                map_y >= 0 && map_y < current_map_->info.height) {
                int idx = map_y * current_map_->info.width + map_x;
                is_occupied = (current_map_->data[idx] > 10);
            }

            if (is_occupied) {
                filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
            }
        }

        scan_publisher_->publish(filtered_scan);
        marker_publisher_->publish(marker);

        std_msgs::msg::Bool msg_bool;
        msg_bool.data = std::any_of(filtered_scan.ranges.begin(), filtered_scan.ranges.end(),
                                    [](float range) { return range < 1.7; });

        obstacle_publisher_->publish(msg_bool);
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        process_occupancy_grid(*msg);
        current_map_ = msg;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        
        current_pose_ = msg;
    }

    void process_occupancy_grid(const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
        // Get grid dimensions
        int width = occupancy_grid.info.width;
        int height = occupancy_grid.info.height;

        // Convert data to a cv::Mat
        cv::Mat grid(height, width, CV_8SC1, const_cast<int8_t*>(occupancy_grid.data.data()));

        // Label connected components using 8-connectivity
        cv::Mat labels, stats, centroids;
        int num_features = cv::connectedComponentsWithStats(grid, labels, stats, centroids, 8, CV_32S);

        // Remove clusters with size between 10 and 20
        for (int i = 1; i < num_features; ++i) {
            int cluster_size = stats.at<int>(i, cv::CC_STAT_AREA);
            if (cluster_size > 10 && cluster_size < 20) {
                // Set label areas to 0
                grid.setTo(0, labels == i);
            }
        }

        // Convert cv::Mat back to vector
        std::vector<int8_t> modified_data(grid.begin<int8_t>(), grid.end<int8_t>());

        // Publish modified occupancy grid
        auto occupancy_grid_new = std::make_shared<nav_msgs::msg::OccupancyGrid>(occupancy_grid);
        occupancy_grid_new->data = modified_data;
        map_publisher_->publish(*occupancy_grid_new);
    }

    double get_yaw_from_quaternion(double x, double y, double z, double w) {
        tf2::Quaternion quat(x, y, z, w);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        return yaw;
    }


};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
