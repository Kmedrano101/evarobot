// Author: Kevin Medrano Ayala
// Contact: kevin.ejem18@gmail.com

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_ros/transforms.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/qos.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <mutex>
#include <pcl/filters/passthrough.h>

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer()
    : Node("lidar_transformer"),
      timer_(nullptr)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
                .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        point_cloud_pub_1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/myrobot/lidar/points", qos_profile);
        RCLCPP_INFO(this->get_logger(), "Publisher initialized on topic /myrobot/lidar/points");

        point_cloud_sub_1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/gazebo_sim/lidar/points", qos_profile, std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));

        // Create a timer to publish at 100 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), // 10 ms interval for 100 Hz
            std::bind(&PointCloudTransformer::timerCallback, this));
    }

private:

    std::mutex callback_mutex_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_msg_;

    // Callback function for handling point clouds from camera 1
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(callback_mutex_); // Ensure thread-safety
        latest_msg_ = msg;
    }

    // Timer callback function to publish at 100 Hz
    void timerCallback()
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        if (latest_msg_)
        {
            RCLCPP_INFO(this->get_logger(), "Publishing transformed point cloud at 100 Hz");
            std::string target_frame = "world";
            std::string current_frame = "lidar";

            try
            {
                // Transform the point cloud to the 'map' frame
                geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                    target_frame, current_frame, tf2::TimePointZero);

                sensor_msgs::msg::PointCloud2 transformed_msg;
                pcl_ros::transformPointCloud(target_frame, transform_stamped, *latest_msg_, transformed_msg);

                // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
                pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(transformed_msg, *pcl_cloud);
                
                // Apply PassThrough filter to remove ground points
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud(pcl_cloud);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(0.2, std::numeric_limits<float>::max()); // Keep points with z >= 0.2
                pass.filter(*filtered_cloud); 

                // Apply VoxelGrid filter for downsampling
                pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
                voxel_filter.setInputCloud(filtered_cloud);
                voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);  // Adjust the leaf size as needed
                voxel_filter.filter(*downsampled_cloud);

                // Convert filtered pcl::PointCloud back to sensor_msgs::PointCloud2
                sensor_msgs::msg::PointCloud2 output_msg;
                pcl::toROSMsg(*downsampled_cloud, output_msg);
                output_msg.header.frame_id = target_frame;
                output_msg.header.stamp = latest_msg_->header.stamp;

                // Publish the filtered point cloud
                point_cloud_pub_1_->publish(output_msg);
            }
            catch (tf2::TransformException & ex)
            {
                RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
            }
        }
    }

    // Declare the publishers, subscribers, and timer as private member variables
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_1_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_1_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudTransformer>();

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
